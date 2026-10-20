# SPDX-License-Identifier: MulanPSL-2.0
"""Per-engine map operations behind one interface.

`map_ops` owns the engine-independent half of save / load / reset: the map
directory layout, the occupancy + cloud preview, metadata, atomic publish and
the lifecycle announcements. What differs per SLAM engine is only how its pose
graph is flushed to a file, how a saved graph is put back, and how a reset is
requested. Those three operations live here, one implementation per `algo`, so
adding an engine does not touch the callers.

Artifacts stay engine-tagged but the directory contract does not change: every
saved map is `{MAPPING_MAPS_DIR}/<map_id>/` with `occupancy.{pgm,yaml,png}`,
`cloud.pcd`, `meta.yaml` plus the engine's own graph file(s) — `rtabmap.db` for
RTAB-Map, `<map_id>.posegraph` + `<map_id>.data` for slam_toolbox.
"""
from __future__ import annotations

import logging
import math
import os
import signal
import subprocess
import time
from typing import Callable, Optional, Protocol

log = logging.getLogger("mapping.engines")

# `localizers` imports nothing from here and `map_ops` imports both; rather than
# add a cycle, the two calls an engine needs are handed in at import time.
localizers_enabled: Callable[[], bool] = lambda: False
start_localizer: Callable[..., tuple[bool, str]] = (
    lambda map_dir, map_id, owns_tf=False, initial_pose=None: (False, "no localizer bound"))
stop_localizer: Callable[[], None] = lambda: None


def bind_localizer(enabled, start, stop) -> None:
    """Give the engines the localizer they share with `map_ops`."""
    global localizers_enabled, start_localizer, stop_localizer
    localizers_enabled, start_localizer, stop_localizer = enabled, start, stop

# slam_toolbox's serialization writes `<stem>.posegraph` and `<stem>.data`; the
# stem is fixed rather than derived from map_id so a directory copied under a
# new name still loads.
SLAM_TOOLBOX_STEM = "posegraph"


class EngineOps(Protocol):
    """Engine-specific half of the map operations."""

    name: str
    graph_files: tuple[str, ...]

    def graph_ready(self, map_dir: str) -> tuple[bool, str]:
        """Is the saved graph in `map_dir` present and loadable?"""

    def snapshot(self, node, staging_dir: str, timeout_s: float) -> tuple[bool, str]:
        """Write the live graph into `staging_dir` (engine artifact names)."""

    def activate(self, node, map_dir: str, map_id: str, timeout_s: float,
                 pose: Optional[tuple[float, float, float]] = None) -> tuple[bool, str]:
        """Put the saved graph back and switch to localization, at `pose` when
        one is given and at the graph's first node otherwise."""

    def reset(self, node, timeout_s: float) -> tuple[bool, str]:
        """Discard the live graph and start a fresh mapping session."""

    def switch_mode(self, node, mode: str, map_dir: str, map_id: str,
                    timeout_s: float, pose=None) -> tuple[bool, str]:
        """Flip between mapping and localization on the map already loaded,
        standing the robot at `pose` when one has been recovered."""

    def freeze(self, node, timeout_s: float) -> tuple[bool, str]:
        """Stop editing the loaded map, verifying that it actually stopped."""

    def stop_engine(self) -> tuple[bool, str]:
        """Take the engine off `map -> odom` entirely, so nothing claims to know
        where the robot is until something has checked."""

    def hold(self, node, timeout_s: float) -> tuple[bool, str]:
        """Freeze the engine while a localizer recovers the robot's pose."""

    def resume(self, node, timeout_s: float) -> tuple[bool, str]:
        """Carry on from the pose `activate` was given."""


def _engine_process(match: str):
    """(pid, argv) of the running engine launch matching `match`, or None.

    The SLAM node is started outside this service and reparented to init, so
    there is no handle to it; /proc is where its command line still is, and
    that command line is what a restart has to reproduce.
    """
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                argv = [a for a in f.read().split(b"\0") if a]
        except OSError:
            continue
        text = b" ".join(argv).decode("utf-8", "replace")
        if match in text and "ros2" in text and "launch" in text:
            return int(entry), [a.decode("utf-8", "replace") for a in argv]
    return None


def _children_of(pid: int) -> list[int]:
    """Direct children of `pid`, read from /proc."""
    out = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/stat", "rb") as f:
                fields = f.read().rsplit(b")", 1)[1].split()
            if int(fields[1]) == pid:
                out.append(int(entry))
        except (OSError, IndexError, ValueError):
            continue
    return out


def _stop_process(pid: int, timeout_s: float = 12.0) -> bool:
    """Stop a launch and the nodes it started, and nothing else.

    Signals go to the process and its children individually rather than to its
    process group: in a container the group can reach back to PID 1, and killing
    that takes the whole service down with it — which is exactly what happened
    the first time this was written with `killpg`.
    """
    if pid <= 1 or pid == os.getpid():
        return False
    targets = [pid] + _children_of(pid)
    # The engine is started under `setsid`, so it leads its own process group
    # and signalling that group reaches the launch's own children without
    # reaching anything else. Only take that route when the group really is
    # someone else's: in a container the default group reaches PID 1, and
    # signalling it ends the container.
    try:
        pgid = os.getpgid(pid)
        if pgid > 1 and pgid != os.getpgid(0):
            targets.append(-pgid)
    except (ProcessLookupError, PermissionError):
        pass
    targets = [t for t in targets if t != os.getpid() and (t < 0 or t > 1)]
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for t in targets:
            try:
                os.kill(t, sig)
            except ProcessLookupError:
                continue
            except PermissionError:
                log.warning("[engine] not allowed to signal pid %d", t)
        deadline = time.monotonic() + (timeout_s if sig == signal.SIGTERM else 4.0)
        while time.monotonic() < deadline:
            alive = []
            for t in targets:
                if t < 0:
                    continue        # a group has no liveness of its own
                try:
                    os.kill(t, 0)
                    alive.append(t)
                except ProcessLookupError:
                    pass
            if not alive:
                return True
            targets = alive
            time.sleep(0.2)
    return False


def _call_service(node, srv_type, name: str, request, timeout_s: float):
    """Call a ROS service, returning (ok, response_or_detail). Kept local so
    this module does not import map_ops (which imports this one)."""
    try:
        client = node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=min(timeout_s, 10.0)):
            return False, f"service {name} unavailable"
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, f"service {name} timed out after {timeout_s:.0f}s"
        return True, future.result()
    except Exception as e:  # noqa: BLE001
        return False, f"service {name} raised {e}"


def _pgm_shape(path: str):
    """(width, height) from a binary PGM header, or None if it is not one."""
    try:
        with open(path, "rb") as f:
            if f.readline().strip() != b"P5":
                return None
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            w, h = line.split()[:2]
            return int(w), int(h)
    except Exception:  # noqa: BLE001
        return None


def _await_map_shape(node, topic: str, want, timeout_s: float, differs_from=None,
                     newer_than=None):
    """Wait for a published OccupancyGrid whose (width, height) is `want`.

    Returns (ok, observed_shape_or_None, stamp_ns_or_None). `deserialize_map`
    answers with an empty message, so the map the engine publishes afterwards
    is the only evidence that the request was accepted rather than silently
    refused; `differs_from` (a shape) or `newer_than` (a stamp) lets the caller
    ignore the grid that was already latched.
    """
    import threading

    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)

    seen = {"shape": None, "stamp": None}
    hit = threading.Event()
    # Re-rasterizing a graph can land a cell or two either way, so the match is
    # a tolerance rather than equality; it still separates the saved map from
    # the session that was live before it.
    slack = 4

    def on_map(msg):
        shape = (int(msg.info.width), int(msg.info.height))
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if differs_from is not None and shape == differs_from:
            return
        if newer_than is not None and stamp <= newer_than:
            return
        seen["shape"], seen["stamp"] = shape, stamp
        if want is None or (abs(shape[0] - want[0]) <= slack
                            and abs(shape[1] - want[1]) <= slack):
            hit.set()

    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    sub = node.create_subscription(OccupancyGrid, topic, on_map, qos)
    try:
        hit.wait(timeout_s)
    finally:
        node.destroy_subscription(sub)
    return hit.is_set(), seen["shape"], seen["stamp"]


class SlamToolboxOps:
    """slam_toolbox (Karto scan matching + pose-graph, ROS 2 reference 2D SLAM).

    Serialization is `slam_toolbox/srv/SerializePoseGraph` (writes
    `<filename>.posegraph` + `<filename>.data`) and restoring is
    `slam_toolbox/srv/DeserializePoseGraph`, whose `match_type` decides how the
    robot re-enters the map: `LOCALIZE_AT_POSE` starts localization at a given
    pose, which is what `load_map` means here. A reset is
    `slam_toolbox/srv/Reset`, falling back to deserializing nothing when the
    running version predates that service.
    """

    name = "slam_toolbox"
    graph_files = (f"{SLAM_TOOLBOX_STEM}.posegraph", f"{SLAM_TOOLBOX_STEM}.data")

    def __init__(self, namespace: str = "") -> None:
        self._ns = (namespace or os.environ.get("MAPPING_SLAM_TOOLBOX_NS", "/slam_toolbox")).rstrip("/")
        # `pause_new_measurements` is a toggle, not a setter, so the caller has
        # to remember which way it left the engine.
        self._paused = False
        # The mapping node's command line, remembered when it is stopped so the
        # deployment's own settings come back when mapping resumes.
        self._mapping_argv: list[str] = []
        self._scan_topic = os.environ.get("MAPPING_SCAN_TOPIC", "/scan")
        self._base_frame = os.environ.get("MAPPING_BASE_FRAME", "base_link")
        self._odom_frame = os.environ.get("MAPPING_ODOM_FRAME", "odom")
        self._map_frame = os.environ.get("MAPPING_MAP_FRAME", "map")
        self._use_sim_time = os.environ.get("MAPPING_USE_SIM_TIME", "").lower() in ("1", "true", "yes")

    def configure(self, *, scan_topic: str = "", base_frame: str = "",
                  odom_frame: str = "", map_frame: str = "",
                  use_sim_time: Optional[bool] = None) -> None:
        """Take the deployment's frames and topic, which the localization node
        has to be started with because it is a different process."""
        self._scan_topic = scan_topic or self._scan_topic
        self._base_frame = base_frame or self._base_frame
        self._odom_frame = odom_frame or self._odom_frame
        self._map_frame = map_frame or self._map_frame
        if use_sim_time is not None:
            self._use_sim_time = bool(use_sim_time)

    def _srv(self, leaf: str) -> str:
        return f"{self._ns}/{leaf}"

    def graph_ready(self, map_dir: str) -> tuple[bool, str]:
        """Both serialization files must be there; slam_toolbox fails opaquely
        when only one is (the `.data` holds the scans, the `.posegraph` the
        graph)."""
        missing = [f for f in self.graph_files if not os.path.isfile(os.path.join(map_dir, f))]
        if missing:
            return False, f"missing {', '.join(missing)} in {map_dir}"
        size = sum(os.path.getsize(os.path.join(map_dir, f)) for f in self.graph_files)
        if size <= 0:
            return False, f"empty pose graph in {map_dir}"
        return True, f"pose graph ok ({size} bytes)"

    def snapshot(self, node, staging_dir: str, timeout_s: float) -> tuple[bool, str]:
        """Ask slam_toolbox to serialize into the staging directory and wait for
        both files to appear (the service returns before the write completes)."""
        try:
            from slam_toolbox.srv import SerializePoseGraph
        except Exception as e:  # noqa: BLE001
            return False, f"slam_toolbox service types unavailable: {e}"
        stem = os.path.join(staging_dir, SLAM_TOOLBOX_STEM)
        req = SerializePoseGraph.Request()
        req.filename = stem
        ok, resp = _call_service(node, SerializePoseGraph, self._srv("serialize_map"), req, timeout_s)
        if not ok:
            return False, str(resp)
        deadline = time.monotonic() + max(5.0, timeout_s / 4.0)
        while time.monotonic() < deadline:
            ready, detail = self.graph_ready(staging_dir)
            if ready:
                return True, f"serialized pose graph ({detail})"
            time.sleep(0.2)
        return False, f"serialize_map returned but {stem}.posegraph/.data did not appear"

    def activate(self, node, map_dir: str, map_id: str, timeout_s: float,
                 pose: Optional[tuple[float, float, float]] = None) -> tuple[bool, str]:
        """Put the saved graph back and stand the robot in it at `pose`.

        A saved map is loaded into a fresh localization node, never into the
        running mapping node. `deserialize_map` on the asynchronous mapping
        node merges the saved graph with the session in memory, clears any
        pause, and folds new scans in from wherever the match type placed the
        robot (the graph's first node when no pose is given), so the grid it
        publishes grows past the saved one within a second or two, and whether
        a reader saw the saved grid or the grown one was a race. The
        localization node holds only the saved graph, does not add to it, and
        is the only publisher of `/map` and `map -> odom` while it runs, which
        is what `load_map` promises.
        """
        ready, detail = self.graph_ready(map_dir)
        if not ready:
            return False, detail
        topic = os.environ.get("MAPPING_OCCUPANCY_TOPIC", "/map")
        want = _pgm_shape(os.path.join(map_dir, "occupancy.pgm"))
        _, _, before_stamp = _await_map_shape(node, topic, None, 2.0)
        ok, detail = self._start_localization_node(map_dir, map_id, pose)
        if not ok:
            return False, detail
        loaded, shape, _ = _await_map_shape(node, topic, want, min(timeout_s, 60.0),
                                            newer_than=before_stamp)
        if not loaded:
            return False, (f"the localization node started on {map_id} published "
                           f"{shape}, not the saved {want}")
        return True, detail

    def freeze(self, node, timeout_s: float) -> tuple[bool, str]:
        """Stop the engine editing its graph, and check that it stopped.

        `pause_new_measurements` is a toggle whose response says `status: true`
        whichever way it flipped, so the state cannot be read back, and
        `deserialize_map` clears it — a pause taken before loading a map is
        gone by the time the map is loaded. The only reliable readout is the
        engine's own behaviour: a frozen slam_toolbox stops republishing a
        changing grid. So this toggles, watches, and toggles again if the map
        is still moving.
        """
        topic = os.environ.get("MAPPING_OCCUPANCY_TOPIC", "/map")
        for attempt in (1, 2):
            ok, detail = self._toggle_pause(node, timeout_s, "held")
            if not ok:
                return False, detail
            _, first, _ = _await_map_shape(node, topic, None, 3.0)
            changed, second, _ = _await_map_shape(node, topic, None, 4.0,
                                                  differs_from=first)
            if not changed:
                return True, f"engine frozen (grid steady at {first})"
            log.info("[engine] pause toggle %d left the map moving (%s -> %s); "
                     "flipping again", attempt, first, second)
        return False, ("slam_toolbox keeps editing its graph after two pause "
                       "toggles — the saved map would be modified")

    def reset(self, node, timeout_s: float) -> tuple[bool, str]:
        """Clear the live graph. `Reset` exists from slam_toolbox 2.6; older
        builds are told so rather than silently continuing on a stale map."""
        try:
            from slam_toolbox.srv import Reset
        except Exception as e:  # noqa: BLE001
            return False, (f"slam_toolbox Reset service type unavailable ({e}); "
                           "upgrade slam_toolbox to reset in place")
        ok, resp = _call_service(node, Reset, self._srv("reset"), Reset.Request(), timeout_s)
        if not ok:
            return False, str(resp)
        return True, "slam_toolbox reset"

    def hold(self, node, timeout_s: float) -> tuple[bool, str]:
        """Freeze the pose graph while a localizer relocalizes.

        `pause_new_measurements` stops slam_toolbox consuming scans, so its
        `map -> odom` stops moving; it still publishes the frozen value, which
        is why the view is unsettled until the localizer converges and the
        engine is resumed at the recovered pose. Toggling the same service
        resumes it (see `resume`).
        """
        return self._toggle_pause(node, timeout_s, "held")

    def resume(self, node, timeout_s: float) -> tuple[bool, str]:
        """Start consuming scans again, from wherever `activate` put us."""
        return self._toggle_pause(node, timeout_s, "resumed")

    def _toggle_pause(self, node, timeout_s: float, word: str) -> tuple[bool, str]:
        """Toggle slam_toolbox's measurement pause and remember the new state."""
        try:
            from slam_toolbox.srv import Pause
        except Exception as e:  # noqa: BLE001
            return False, f"slam_toolbox Pause service type unavailable ({e})"
        ok, resp = _call_service(node, Pause, self._srv("pause_new_measurements"),
                                 Pause.Request(), timeout_s)
        if not ok:
            return False, str(resp)
        self._paused = word == "held"
        return True, f"slam_toolbox {word}"

    MAPPING_LAUNCH_MATCH = "slam_toolbox_2d.launch.py"
    LOCALIZATION_LAUNCH = "/mapping/launch/slam_toolbox_localization.launch.py"

    def switch_mode(self, node, mode: str, map_dir: str, map_id: str,
                    timeout_s: float, pose=None) -> tuple[bool, str]:
        """Swap slam_toolbox between its mapping node and its localization node.

        Which of the two slam_toolbox executables runs is decided when the
        process starts, so this is a process swap rather than a service call.
        Everything that made the earlier attempts misbehave follows from that
        one fact. The asynchronous mapping node folds every scan into whatever
        graph it holds, so a saved map loaded into it is merged with the session
        already in memory and then edited further as the robot drives -- which
        is why a loaded map came back bigger than the one that was saved and
        kept moving. Pausing it stops the editing but not its `map -> odom`, so
        a filter publishing a corrected transform fights the frozen one and the
        robot jumps between the two answers.

        A fresh localization node has neither problem: it holds only the saved
        graph, it does not add to it, and it is the sole publisher of `/map` and
        of `map -> odom` for as long as it runs. Going back to mapping restarts
        the node this deployment booted with, from the command line it booted
        with.
        """
        mode = (mode or "").strip().lower()
        if mode not in ("mapping", "localization"):
            return False, f"mode={mode!r} invalid (mapping|localization)"
        if mode == "localization":
            return self._start_localization_node(map_dir, map_id, pose)
        return self._start_mapping_node()

    ENGINE_REQUEST = "/tmp/mapping_engine_request"
    ENGINE_ARGS = "/tmp/mapping_engine_args"
    ENGINE_PID = "/tmp/mapping_engine_pid"

    def stop_engine(self) -> tuple[bool, str]:
        """Take slam_toolbox off `map -> odom` until a fix has been checked.

        Between a load and a confirmed relocalization nobody should publish the
        map frame: the engine's answer is the session's, not the saved map's,
        and a consumer that plans or records in that frame would be acting on a
        pose no one has checked. The supervisor is asked for `idle` rather than
        killed, because the container's entrypoint waits on the supervisor and
        would end with it.
        """
        try:
            with open(self.ENGINE_REQUEST, "w", encoding="utf-8") as f:
                f.write("idle")
        except OSError as e:
            return False, f"could not write the engine request: {e}"
        pid = self._supervised_pid()
        if pid is None:
            return True, "no supervised engine launch was running"
        if not _stop_process(pid):
            return False, "the engine launch would not stop"
        return True, "engine stood down; nothing owns map -> odom"

    def wait_ready(self, node, timeout_s: float = 60.0) -> tuple[bool, str]:
        """Block until the running slam_toolbox node answers its services.

        After a swap the new node spends several seconds deserializing the
        graph before it processes a scan; a pose handed to it before then is
        applied to a robot that has since moved on.
        """
        try:
            from slam_toolbox.srv import SerializePoseGraph
        except Exception as e:  # noqa: BLE001
            return False, f"slam_toolbox service types unavailable: {e}"
        client = node.create_client(SerializePoseGraph, self._srv("serialize_map"))
        try:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=1.0):
                    return True, "slam_toolbox is up"
            return False, f"slam_toolbox did not come up within {timeout_s:.0f}s"
        finally:
            try:
                node.destroy_client(client)
            except Exception:  # noqa: BLE001
                pass

    def seed_pose(self, node, pose, frame: str = "") -> tuple[bool, str]:
        """Tell the running localization node where the robot is right now.

        `map_start_pose` is read once at startup and describes where the robot
        WAS when the swap was requested; this is how the pose it has NOW gets
        in, over the same `/initialpose` topic RViz's tool uses.
        """
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
        except Exception as e:  # noqa: BLE001
            return False, f"geometry_msgs unavailable: {e}"
        pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        try:
            x, y, yaw = pose
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = frame or self._map_frame
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            # A tight prior: the filter that produced this had converged.
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.02
            # Give discovery a moment; a message published before the node has
            # matched the publisher is dropped without a trace.
            deadline = time.monotonic() + 5.0
            while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.1)
            for _ in range(3):
                pub.publish(msg)
                time.sleep(0.1)
            return True, f"seeded slam_toolbox at ({x:.2f}, {y:.2f}, {yaw:.2f})"
        finally:
            try:
                node.destroy_publisher(pub)
            except Exception:  # noqa: BLE001
                pass

    def _start_localization_node(self, map_dir: str, map_id: str, pose) -> tuple[bool, str]:
        """Ask the engine supervisor for the localization node, at `pose`."""
        ready, detail = self.graph_ready(map_dir)
        if not ready:
            return False, detail
        x, y, yaw = pose if pose is not None else (0.0, 0.0, 0.0)
        args = [
            f"map_file_name:={os.path.join(map_dir, SLAM_TOOLBOX_STEM)}",
            f"scan_topic:={self._scan_topic}",
            f"base_frame:={self._base_frame}",
            f"odom_frame:={self._odom_frame}",
            f"map_frame:={self._map_frame}",
            f"use_sim_time:={'true' if self._use_sim_time else 'false'}",
            f"start_x:={float(x)}", f"start_y:={float(y)}", f"start_yaw:={float(yaw)}",
        ]
        ok, detail = self._request_engine("localization", args)
        if not ok:
            return False, detail
        return True, (f"slam_toolbox localizing on {map_id} at "
                      f"({float(x):.2f}, {float(y):.2f}, {float(yaw):.2f}); "
                      f"the saved map is not modified")

    def _start_mapping_node(self) -> tuple[bool, str]:
        """Ask the supervisor for the deployment's own mapping node back."""
        ok, detail = self._request_engine("mapping", [])
        if not ok:
            return False, detail
        self._paused = False
        return True, "slam_toolbox mapping again"

    def _request_engine(self, mode: str, args: list[str]) -> tuple[bool, str]:
        """Write the request and stop the current launch so it is picked up.

        The container's entrypoint waits on the supervisor script, not on the
        launch, so stopping the launch swaps the engine; stopping the
        supervisor would end the container.
        """
        try:
            with open(self.ENGINE_ARGS, "w", encoding="utf-8") as f:
                f.write("\n".join(args))
            with open(self.ENGINE_REQUEST, "w", encoding="utf-8") as f:
                f.write(mode)
        except OSError as e:
            return False, f"could not write the engine request: {e}"
        pid = self._supervised_pid()
        if pid is None:
            # Nothing to stop: the supervisor is idling because the engine was
            # stood down for a relocalization, and it picks the request up on
            # its next turn round the loop. On a localization cold start the
            # supervisor may still be initializing; in that race it used to
            # overwrite this request with its initial `idle`. Check after the
            # startup grace period and restore only that initialization value.
            time.sleep(3.0)
            try:
                with open(self.ENGINE_REQUEST, encoding="utf-8") as f:
                    observed = f.read().strip()
                if observed == "idle" and mode != "idle":
                    with open(self.ENGINE_REQUEST, "w", encoding="utf-8") as f:
                        f.write(mode)
                    time.sleep(2.0)
                elif observed != mode:
                    return False, (f"engine request changed from {mode} to "
                                   f"{observed or '<empty>'}")
            except OSError as e:
                return False, f"could not verify the engine request: {e}"
            return True, f"engine supervisor asked for the {mode} node"
        if not _stop_process(pid):
            return False, "the running engine launch would not stop"
        # The supervisor waits two seconds before starting the replacement.
        time.sleep(4.0)
        return True, f"engine supervisor asked for the {mode} node"

    def _supervised_pid(self) -> Optional[int]:
        """The launch pid the supervisor recorded, if it is still alive."""
        try:
            pid = int(open(self.ENGINE_PID, encoding="utf-8").read().strip())
        except (OSError, ValueError):
            return None
        try:
            os.kill(pid, 0)
        except (ProcessLookupError, PermissionError):
            return None
        return pid

    def _spawn(self, cmd, what: str) -> tuple[bool, str]:
        """Start a launch in its own session and leave it running."""
        log_path = os.path.join("/tmp", f"slam_toolbox_{what}.log")
        try:
            out = open(log_path, "ab", buffering=0)
            subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT,
                             start_new_session=True)
        except Exception as e:  # noqa: BLE001
            return False, f"failed to start the {what} node: {e}"
        log.info("[engine] started slam_toolbox %s node -> %s", what, log_path)
        return True, f"started the {what} node"


class RtabmapOps:
    """RTAB-Map, delegating to the implementations that live in `map_ops`.

    The functions are injected rather than imported so this module stays free of
    a circular import; `map_ops` registers them at import time.
    """

    name = "rtabmap"
    graph_files = ("rtabmap.db",)

    def __init__(self, *, graph_ready: Callable, snapshot: Callable,
                 activate: Callable, reset: Callable, set_mode: Callable) -> None:
        self._graph_ready = graph_ready
        self._snapshot = snapshot
        self._activate = activate
        self._reset = reset
        self._set_mode = set_mode

    def graph_ready(self, map_dir: str) -> tuple[bool, str]:
        return self._graph_ready(map_dir)

    def snapshot(self, node, staging_dir: str, timeout_s: float) -> tuple[bool, str]:
        return self._snapshot(node, staging_dir, timeout_s)

    def switch_mode(self, node, mode: str, map_dir: str, map_id: str,
                    timeout_s: float, pose=None) -> tuple[bool, str]:
        """RTAB-Map flips in place through its own service; it localizes on a
        loaded database without editing it, so no process has to be swapped."""
        ok, detail = self._set_mode(node, mode)
        return ok, (detail if isinstance(detail, str) else str(detail))

    def freeze(self, node, timeout_s: float) -> tuple[bool, str]:
        """RTAB-Map's localization mode already leaves the database alone."""
        return self._set_mode(node, "localization")

    def stop_engine(self) -> tuple[bool, str]:
        """RTAB-Map keeps running; `load_map` opens the database in localization
        mode, so its transform already comes from the saved map rather than from
        a session the caller has not checked."""
        return True, "rtabmap needs no stop"

    def activate(self, node, map_dir: str, map_id: str, timeout_s: float,
                 pose: Optional[tuple[float, float, float]] = None) -> tuple[bool, str]:
        return self._activate(node, map_dir, map_id, timeout_s, pose)

    def reset(self, node, timeout_s: float) -> tuple[bool, str]:
        return self._reset(node, timeout_s)

    def hold(self, node, timeout_s: float) -> tuple[bool, str]:
        """RTAB-Map has no equivalent pause; `load_map` switches it to
        localization mode instead, so there is nothing to freeze."""
        return True, "rtabmap needs no hold"

    def resume(self, node, timeout_s: float) -> tuple[bool, str]:
        return True, "rtabmap needs no resume"


_REGISTRY: dict[str, EngineOps] = {}


def register(ops: EngineOps) -> None:
    """Make `ops` the implementation for its `name`."""
    _REGISTRY[ops.name] = ops


def engine_for(algo: str) -> Optional[EngineOps]:
    """Return the operations for `algo`, or None when the engine does not
    implement map persistence (dlio / fastlio2 are odometry-only)."""
    return _REGISTRY.get((algo or "").strip().lower())


def graph_files_for(algo: str) -> tuple[str, ...]:
    """Artifact filenames the engine writes inside a saved map directory."""
    ops = engine_for(algo)
    return tuple(getattr(ops, "graph_files", ())) if ops else ()


register(SlamToolboxOps())
