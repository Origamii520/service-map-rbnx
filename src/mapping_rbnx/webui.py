# SPDX-License-Identifier: MulanPSL-2.0
"""Lightweight map web UI for the mapping service.

A dependency-light (stdlib http.server + Pillow) page that lets an operator
*see* the live SLAM map and pose, and drive the runtime map operations:
  - live 2D occupancy + robot pose preview (auto-refresh)
  - Save  → save_map(map_id)
  - Library → list saved maps with thumbnails; Load → load_map(map_id, mode)
  - Pose Estimate → click the map to seed pose_estimate(x, y, theta)

Runs inside the mapping bridge process, so its buttons call the same
map_ops impls the gRPC/MCP capabilities use — no extra round trip. It reads
the live map/pose off its own dedicated rclpy node (see _ensure_subscriptions).

The Mapping bridge enables this on port 8091 by default; deployment config may
set ``webui_port: 0`` to disable it. The server binds 127.0.0.1 by default
because it exposes unauthenticated map operations; an authenticated deployment
may explicitly set MAPPING_WEBUI_HOST otherwise.
"""
from __future__ import annotations

import collections
import io
import json
import logging
import math
import os
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from . import lifecycle, localizers, map_ops

log = logging.getLogger("mapping_rbnx.webui")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
MAP_TOPIC = os.environ.get("MAPPING_MAP_TOPIC", "/map")
MAP_FRAME = os.environ.get("MAPPING_MAP_FRAME", "map")
_tf_buffer = None
POSE_TOPIC = os.environ.get("MAPPING_POSE_TOPIC", "/robonix/map/pose")

# Latest map / pose / range data, filled by ROS subscriptions on the webui's
# own node. "scan" and "cloud" hold points already projected into the map
# frame, so the page can draw them straight onto the occupancy canvas and an
# operator can see whether the live returns line up with the saved map.
_latest = {"grid": None, "pose": None, "scan": None, "cloud": None,
           "localizer_pose": None, "localizer_pose_at": 0.0,
           "scan_at": 0.0, "pose_at": 0.0}
# Topics for the range sensors, resolved through Atlas by the bridge and
# injected with set_sensor_topics(). Empty means the deployment has no such
# capability bound, and the page simply has nothing to draw.
_sensor_topics = {"scan": "", "cloud": ""}


# A deployment whose 2-D scan is not an Atlas capability can name it here (or
# through the deployment config key webui_scan_topic). The Ranger is exactly
# that case: its mid360 declares only robonix/primitive/lidar/lidar3d, and the
# 2-D scan is derived downstream by the navigation service, which does not
# declare it. Without this the overlay could only draw that robot's cloud.
SCAN_TOPIC_OVERRIDE = os.environ.get("MAPPING_WEBUI_SCAN_TOPIC", "")
# Names that are a projection someone else already rejected: the raw output of
# pointcloud_to_laserscan before speckle filtering. Preferring the filtered one
# keeps the overlay agreeing with what navigation actually consumes.
_SCAN_NAME_PENALTY = ("_raw", "/raw")
_scan_discovery_logged = False
# Which range subscriptions exist. Discovery can call the subscriber a second
# time once a late-starting scan appears, and subscribing twice to the same
# topic doubles the callback work for nothing.
_range_subscribed = {"scan": False, "cloud": False}


def set_sensor_topics(scan: str = "", cloud: str = "") -> None:
    """Bind the range-sensor topics the page overlays on the map.

    Called by atlas_bridge with whatever it resolved from Atlas for
    robonix/primitive/lidar/lidar (2-D scan) and .../lidar3d (point cloud), so
    the UI follows the deployment's capability bindings instead of hardcoding
    topic names. Must be called before the first request creates the
    subscriptions; later calls are ignored for topics already subscribed.
    """
    _sensor_topics["scan"] = SCAN_TOPIC_OVERRIDE or scan or ""
    _sensor_topics["cloud"] = cloud or ""
    log.info("webui range overlay topics: scan=%s cloud=%s",
             _sensor_topics["scan"] or "<none>", _sensor_topics["cloud"] or "<none>")
_subscribed = False
_sub_lock = threading.Lock()
# Keep the dedicated node + executor alive for the process lifetime.
_webui_node = None
_webui_exec = None

# ── activity log ──────────────────────────────────────────────────────────────
# A small in-memory ring of timestamped action records so the operator can see,
# in the page, what each button did and how a pose_estimate converged. Surfaced
# at GET /api/log; also mirrored to the Python logger.
_LOG = collections.deque(maxlen=200)
_log_lock = threading.Lock()
# Last pose_estimate seed, so convergence can be measured against where the
# robot actually settled after relocalizing.
_seed = {"x": None, "y": None, "theta": None, "t": 0.0}


def _log_add(kind: str, msg: str) -> None:
    """Append one timestamped entry (kind ∈ save|load|switch|pose|info) to the
    UI activity log and mirror it to the service logger."""
    with _log_lock:
        _LOG.append({"t": time.time(), "kind": kind, "msg": msg})
    log.info("[webui:%s] %s", kind, msg)


def _live_pose_xytheta():
    """Current map-frame pose as (x, y, yaw), or None.

    In localization mode the engine is frozen on the saved map and the particle
    filter is what tracks the robot, so its estimate is the live pose; the
    engine's own topic still publishes, but it publishes the pose it was frozen
    at.
    """
    ps = _latest.get("localizer_pose") if _localizer_is_live() else None
    if ps is None:
        ps = _latest.get("pose")
    if ps is None:
        return None
    pp = ps.pose.pose
    q = pp.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return pp.position.x, pp.position.y, yaw


def _track_convergence(sx: float, sy: float, stheta: float, settle_s: float = 4.0) -> None:
    """After seeding a pose, wait settle_s for rtabmap to relocalize, then log
    where the robot actually settled and its offset from the clicked estimate
    (distance in metres + heading delta in degrees)."""
    def _check():
        time.sleep(settle_s)
        cur = _live_pose_xytheta()
        if cur is None:
            _log_add("pose", "no live pose — cannot measure convergence")
            return
        cx, cy, cyaw = cur
        dist = math.hypot(cx - sx, cy - sy)
        dth = abs((cyaw - stheta + math.pi) % (2 * math.pi) - math.pi)
        _log_add("pose", f"pose after seed → ({cx:.2f}, {cy:.2f}, {math.degrees(cyaw):.0f}°)  "
                         f"Δ from estimate = {dist:.2f} m / {math.degrees(dth):.0f}°")
    threading.Thread(target=_check, daemon=True).start()


def _ensure_subscriptions() -> None:
    """Subscribe (once) to the live occupancy grid + pose so the UI can render
    them. Best-effort — if ROS isn't up yet the preview stays empty and we
    retry on the next request.

    Uses a DEDICATED node + executor rather than map_ops' shared node. The
    shared node's SingleThreadedExecutor is already spinning by the time the
    UI is first hit, and a subscription created on an already-spinning
    SingleThreadedExecutor is not reliably serviced — the callback never fires
    and the map stays blank even though /map is publishing. Here the
    subscriptions are created BEFORE this node's executor starts spinning, so
    they are always serviced.
    """
    global _subscribed, _webui_node, _webui_exec
    with _sub_lock:
        if _subscribed:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from nav_msgs.msg import OccupancyGrid
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

            # The bridge process initialises rclpy lazily (map_ops._get_node on
            # the first map op). The UI may be hit before any map op, so init
            # the context here if needed — mirrors map_ops' own guard. Without
            # this the UI returned before ever subscribing and stayed blank.
            if not rclpy.ok():
                rclpy.init(args=None)

            # Match rtabmap's /map publisher (RELIABLE + TRANSIENT_LOCAL) so the
            # last latched grid is delivered immediately on subscribe.
            latched = QoSProfile(depth=1)
            latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            latched.reliability = QoSReliabilityPolicy.RELIABLE

            def _on_grid(msg):
                _latest["grid"] = msg

            def _on_pose(msg):
                _latest["pose"] = msg
                _latest["pose_at"] = time.time()

            def _on_localizer_pose(msg):
                # The particle filter's own estimate, kept for its covariance:
                # that is what says whether it has converged yet.
                _latest["localizer_pose"] = msg
                _latest["localizer_pose_at"] = time.time()

            node = Node("mapping_webui")
            if map_ops.active_algo() == "slam_toolbox":
                from .grid_relay import Relay
                node._grid_relay = Relay(node, MAP_TOPIC, MAPS_DIR, lifecycle.current, _on_grid, latched)
            else:
                node.create_subscription(OccupancyGrid, MAP_TOPIC, _on_grid, latched)
            node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _on_pose, 10)
            if localizers.enabled():
                node.create_subscription(PoseWithCovarianceStamped, localizers.POSE_TOPIC,
                                         _on_localizer_pose, 10)
            _subscribe_range_sensors(node)
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            threading.Thread(target=ex.spin, daemon=True).start()
            _webui_node, _webui_exec = node, ex
            _subscribed = True
            log.info("webui subscribed (dedicated node): map=%s pose=%s", MAP_TOPIC, POSE_TOPIC)
        except Exception as e:  # noqa: BLE001
            log.warning("webui subscriptions failed: %s", e)


# Cap on points sent to the page. A 2-D scan is a few hundred; a 3-D cloud can
# be hundreds of thousands, which no browser will draw at 1 Hz. Subsampling is
# uniform so the shape of the return stays honest.
MAX_OVERLAY_POINTS = int(os.environ.get("MAPPING_WEBUI_MAX_POINTS", "1200"))
# Cloud points further than this above or below the sensor are dropped: the
# overlay exists to be compared against a 2-D occupancy grid, and ceiling or
# floor returns only obscure that.
CLOUD_Z_BAND_M = float(os.environ.get("MAPPING_WEBUI_CLOUD_Z_BAND", "0.35"))


def _scan_points(msg) -> dict:
    """Project a LaserScan into (x, y) pairs in the sensor's own frame.

    Range readings outside [range_min, range_max], and the inf/NaN a lidar
    emits for "no return", are dropped rather than drawn at range_max, which
    would paint a fake wall around the robot.
    """
    pts = []
    ang = msg.angle_min
    lo, hi = msg.range_min, msg.range_max
    step = max(1, math.ceil(len(msg.ranges) / MAX_OVERLAY_POINTS))
    for i, r in enumerate(msg.ranges):
        if i % step == 0 and lo <= r <= hi and math.isfinite(r):
            a = ang + i * msg.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
    return {"frame": msg.header.frame_id, "pts": pts, "t": time.time()}


def _cloud_points(msg) -> dict:
    """Project a PointCloud2 into (x, y) pairs in the sensor's own frame,
    keeping only returns within CLOUD_Z_BAND_M of the sensor plane and
    subsampling to MAX_OVERLAY_POINTS."""
    try:
        from sensor_msgs_py import point_cloud2
    except Exception:  # noqa: BLE001
        return {"frame": msg.header.frame_id, "pts": [], "t": time.time()}
    pts = []
    total = max(1, msg.width * msg.height)
    step = max(1, total // (MAX_OVERLAY_POINTS * 4))
    for i, pt in enumerate(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)):
        if i % step:
            continue
        x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
        if abs(z) <= CLOUD_Z_BAND_M:
            pts.append((x, y))
            if len(pts) >= MAX_OVERLAY_POINTS:
                break
    return {"frame": msg.header.frame_id, "pts": pts, "t": time.time()}


def _frame_to_map(frame: str):
    """Latest (tx, ty, yaw) taking `frame` into the map frame, or None.

    Looked up once per request and applied to every point, rather than
    transforming each point through tf2 — the overlay is a visual check, and a
    single planar transform is what the canvas needs anyway.
    """
    if not frame or _webui_node is None:
        return None
    if _tf_buffer is None:
        return None
    try:
        import rclpy
        tr = _tf_buffer.lookup_transform(
            MAP_FRAME, frame.lstrip("/"), rclpy.time.Time())
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw
    except Exception as e:  # noqa: BLE001
        log.debug("webui overlay: no transform %s -> %s: %s", frame, MAP_FRAME, e)
        return None


def _overlay_in_map(kind: str) -> dict:
    """Range points for `kind` ("scan"|"cloud") expressed in the map frame.

    Returns {"pts", "frame", "stale", "why"}. When there is nothing to draw the
    reason is named, because the two reasons call for opposite responses: no
    messages means the sensor or its topic is wrong, while messages that cannot
    be transformed means localization is not publishing the map frame -- the
    robot's problem, not the overlay's. Reading that off the page is the point
    of the overlay on a robot with no RViz.
    """
    topic = _sensor_topics.get(kind) or ""
    data = _latest.get(kind)
    if not topic:
        return {"pts": [], "frame": "", "stale": True, "why": "no capability bound"}
    if not data or not data.get("pts"):
        return {"pts": [], "frame": "", "stale": True,
                "why": "no data on %s" % topic}
    age = time.time() - data.get("t", 0)
    tf = _frame_to_map(data.get("frame", ""))
    if tf is None:
        return {"pts": [], "frame": data.get("frame", ""), "stale": True,
                "why": "no %s -> %s transform: localization is not publishing "
                       "the map frame" % (data.get("frame", "?"), MAP_FRAME)}
    tx, ty, yaw = tf
    c, s_ = math.cos(yaw), math.sin(yaw)
    pts = [[round(tx + x * c - y * s_, 3), round(ty + x * s_ + y * c, 3)]
           for x, y in data["pts"]]
    stale = age > 3.0
    return {"pts": pts, "frame": data.get("frame", ""), "stale": stale,
            "age_s": round(age, 1),
            "why": "last message %.1fs old" % age if stale else ""}


def pick_scan_topic(topics: list[tuple[str, list[str]]]) -> str:
    """Choose a LaserScan topic from a ROS graph listing, or "".

    Used when no 2-D scan capability is bound. A deployment can still have a
    scan -- the Ranger's is produced by the navigation service from the point
    cloud and never declared -- and an operator checking localization needs to
    see it. Prefer the shortest name so a filtered scan wins over a longer
    intermediate one, and rank raw projections last.
    """
    scans = [name for name, types in topics
             if "sensor_msgs/msg/LaserScan" in types]
    if not scans:
        return ""
    return sorted(scans, key=lambda n: (any(p in n for p in _SCAN_NAME_PENALTY),
                                        len(n), n))[0]


def _discover_scan_topic(node) -> str:
    """Look for a LaserScan on the graph and subscribe to it if found.

    Retried from the range endpoint rather than done once at startup: the
    navigation service that publishes the derived scan may come up after
    mapping does, and an overlay that gave up at boot would stay blank for the
    rest of the session.
    """
    global _scan_discovery_logged
    try:
        found = pick_scan_topic(node.get_topic_names_and_types())
    except Exception as e:  # noqa: BLE001
        log.debug("webui overlay: scan discovery failed: %s", e)
        return ""
    if not found:
        return ""
    _sensor_topics["scan"] = found
    if not _scan_discovery_logged:
        _scan_discovery_logged = True
        log.info("webui overlay: no 2-D scan capability bound; using %s "
                 "found on the graph (set webui_scan_topic to pin one)", found)
    return found


_overlay_log = {"why": "", "t": 0.0}


def _log_overlay_state(out: dict) -> None:
    """Log why the overlay is empty, once per distinct reason per minute.

    The page shows this, but the page is not what gets sent back when someone
    asks for help -- the log is. "no scanner_normalized -> map transform" in a
    log file is the difference between chasing a sensor and chasing
    localization.
    """
    reasons = [
        "%s: %s" % (kind, out[kind]["why"])
        for kind in ("scan", "cloud")
        if kind in out and out[kind].get("why")
    ]
    why = "; ".join(reasons)
    if why == _overlay_log["why"] and time.time() - _overlay_log["t"] < 60.0:
        return
    _overlay_log["why"], _overlay_log["t"] = why, time.time()
    if why:
        log.warning("overlay has nothing to draw — %s", why)
    else:
        log.info("overlay drawing live returns in the %s frame", MAP_FRAME)


def _subscribe_range_sensors(node) -> None:
    """Subscribe to the range topics Atlas resolved, if any.

    Sensor data is best-effort published, so both use a SENSOR_DATA profile;
    subscribing RELIABLE to a BEST_EFFORT publisher silently receives nothing.
    A missing message type or topic disables that overlay and is logged once —
    the map preview itself must keep working either way.
    """
    from rclpy.qos import qos_profile_sensor_data

    global _tf_buffer
    if _tf_buffer is None:
        try:
            import tf2_ros
            _tf_buffer = tf2_ros.Buffer()
            # The listener's /tf subscriptions must exist before this node's
            # executor starts spinning, for the same reason the map and pose
            # subscriptions do -- see _ensure_subscriptions. Created here, not
            # lazily on the first lookup, or they are never serviced and every
            # transform lookup fails forever.
            tf2_ros.TransformListener(_tf_buffer, node, spin_thread=False)
        except Exception as e:  # noqa: BLE001
            log.warning("webui overlay: tf2 unavailable, no range overlay: %s", e)
            _tf_buffer = None

    def _on_scan(msg) -> None:
        _latest["scan"] = _scan_points(msg)
        _latest["scan_msg"] = msg
        _latest["scan_at"] = time.time()

    scan_topic = _sensor_topics.get("scan") or ""
    cloud_topic = _sensor_topics.get("cloud") or ""
    if _range_subscribed["scan"]:
        scan_topic = ""
    if _range_subscribed["cloud"]:
        cloud_topic = ""
    if scan_topic:
        try:
            from sensor_msgs.msg import LaserScan
            node.create_subscription(
                LaserScan, scan_topic,
                # The projected points draw the overlay; the message itself is
                # what the map check needs, and re-projecting it back would
                # throw away the ranges that check depends on.
                _on_scan,
                qos_profile_sensor_data)
            _range_subscribed["scan"] = True
            log.info("webui overlay: subscribed scan %s", scan_topic)
        except Exception as e:  # noqa: BLE001
            log.warning("webui overlay: scan %s unavailable: %s", scan_topic, e)
    if cloud_topic:
        try:
            from sensor_msgs.msg import PointCloud2
            node.create_subscription(
                PointCloud2, cloud_topic,
                lambda m: _latest.__setitem__("cloud", _cloud_points(m)),
                qos_profile_sensor_data)
            _range_subscribed["cloud"] = True
            log.info("webui overlay: subscribed cloud %s", cloud_topic)
        except Exception as e:  # noqa: BLE001
            log.warning("webui overlay: cloud %s unavailable: %s", cloud_topic, e)


def _grid_to_png(grid, pose=None) -> bytes:
    """Render a nav_msgs/OccupancyGrid to a PNG (free=white, occ=black,
    unknown=grey), origin bottom-left, with an optional robot pose marker.

    The conventional palette, deliberately. An occupancy grid carries three
    states and the whole of its meaning is telling them apart; a dark-theme
    inversion tried here put free and unknown within twenty greys of each other
    and left only wall outlines -- one state gone, and the map no longer read as
    an occupancy grid at all."""
    from PIL import Image, ImageDraw
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = grid.data
    img = Image.new("RGB", (w, h), (128, 128, 128))
    px = img.load()
    for j in range(h):
        row = j * w
        for i in range(w):
            v = data[row + i]
            if v < 0:
                continue  # unknown → grey
            c = 255 - int(v * 255 / 100)  # 0→white(free), 100→black(occ)
            px[i, h - 1 - j] = (c, c, c)  # flip Y so up = +y
    if pose is not None and res > 0:
        try:
            p = pose.pose.pose
            mx = int((p.position.x - ox) / res)
            my = h - 1 - int((p.position.y - oy) / res)
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            d = ImageDraw.Draw(img)
            r = 4
            d.ellipse([mx - r, my - r, mx + r, my + r], fill=(220, 30, 30))
            d.line([mx, my, mx + int(10 * math.cos(yaw)), my - int(10 * math.sin(yaw))],
                   fill=(220, 30, 30), width=2)
        except Exception:  # noqa: BLE001
            pass
    # Upscale small grids so the page isn't a postage stamp.
    scale = max(1, min(4, 700 // max(1, w)))
    if scale > 1:
        img = img.resize((w * scale, h * scale), Image.NEAREST)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


# "Is my localization right?" is the one question this page must answer without
# being asked, because acting on a wrong one is what makes a robot dangerous.
# A particle filter cannot answer it — its covariance says how much the
# particles agree with each other, not with the world — so the answer comes
# from the map: the fraction of the current laser's beams that land on a wall
# the saved map already has. The threshold is the same one relocalization is
# gated on, so the page and the recovery agree about what "right" means.
_trust: dict = {"grid_for": "", "grid": None, "fit": None, "at": 0.0, "detail": "",
                "history": [], "bad_since": 0.0, "withdrawn": False}
# Publishing `map -> odom` is a claim to know where the robot is. When the laser
# has disagreed with the map for this long, the claim is withdrawn: the engine
# stands down and a relocalization starts. Everything downstream then loses the
# map frame and stops on its own -- nav2 reports `Localization: inactive` and
# refuses goals, Scene's tf lookups return nothing and it records nothing --
# without mapping having to reach into either of them. A page that only says
# "do not send goals" is advice, and a robot driving on a pose known to be wrong
# is exactly what this exists to prevent.
WITHDRAW_AFTER_S = float(os.environ.get("MAPPING_TRUST_WITHDRAW_AFTER_S", "12"))
TRUST_PERIOD_S = 2.0
# One scan is a noisy witness: a person walking past, or a moment facing a wall
# the map only half covers, drops a single reading well below the gate while the
# robot is perfectly localized. Reporting the median of the last few readings
# says what the last quarter-minute looked like, which is what an operator
# glancing at the page needs to know.
TRUST_WINDOW = 7


def _localizer_is_live(max_age_s: float = 5.0) -> bool:
    """Is the particle filter currently the thing tracking the robot?"""
    if not localizers.enabled():
        return False
    at = float(_latest.get("localizer_pose_at") or 0.0)
    return at > 0.0 and (time.time() - at) <= max_age_s


# How long either input may go missing before the check refuses to speak for it.
# Longer than any real gap between scans, short enough that a robot cannot cross
# a room on a pose nothing is updating.
STALE_AFTER_S = float(os.environ.get("MAPPING_TRUST_STALE_AFTER_S", "5"))


def _stale_inputs(now: float) -> str:
    """Why the check cannot run right now, or "" when both inputs are current."""
    scan_age = now - float(_latest.get("scan_at") or 0.0)
    pose_at = max(float(_latest.get("pose_at") or 0.0),
                  float(_latest.get("localizer_pose_at") or 0.0))
    pose_age = now - pose_at
    if not _latest.get("scan_at"):
        return "no laser has arrived yet"
    if scan_age > STALE_AFTER_S:
        return f"no laser for {scan_age:.0f}s"
    if not pose_at:
        return "no pose has arrived yet"
    if pose_age > STALE_AFTER_S:
        return f"no pose for {pose_age:.0f}s"
    return ""


def _localization_trust() -> dict:
    """Whether the live pose is supported by the map, recomputed periodically.

    Returns {} outside localization mode; otherwise a verdict of `ok`,
    `suspect` or `unknown` with the measured fraction. `unknown` means the
    laser cannot speak — the robot is looking at ground the map never covered —
    which is not the same as a bad fix and must not be shown as one.
    """
    # The lifecycle is what knows which map is loaded; get_mode_impl reports the
    # engine's mode and carries no map id, so reading the id from there returned
    # nothing and the check silently never ran.
    live = lifecycle.current()
    if (live.get("mode") or "") != "localization":
        return {}
    map_id = live.get("map_id") or ""
    if not map_id:
        return {}
    map_dir = os.path.join(MAPS_DIR, map_id)
    if _trust["grid_for"] != map_id:
        _trust.update(grid_for=map_id, grid=localizers.load_grid(map_dir),
                      fit=None, at=0.0, detail="")
    run_id = (map_ops._LOCALIZATION.snapshot()["run_id"]
              if map_ops.active_algo() == "slam_toolbox" else None)
    if _trust.get("run_id") != run_id:
        _trust.update(run_id=run_id, fit=None, at=0.0, detail="", history=[],
                      bad_since=0.0, withdrawn=False)
    now = time.time()
    if now - float(_trust["at"] or 0.0) >= TRUST_PERIOD_S:
        # Comparing a stale scan against a stale pose measures nothing: the two
        # were captured together and agree with each other for as long as both
        # sit still in memory. A run that carried the robot for real read
        # `ok 100%` for two minutes after the laser stopped arriving, while the
        # true error was 2.97 m -- a system that has stopped listening reporting
        # perfect confidence. So silence is its own verdict, and it withdraws
        # the map frame on the same timer a wrong pose does: a robot nobody is
        # tracking must not keep being navigated.
        stale = _stale_inputs(now)
        pose = None if stale else _live_pose_xytheta()
        scan = None if stale else _latest.get("scan_msg")
        if stale:
            _trust.update(fit=None, at=now, history=[], detail=stale)
        elif pose is not None and scan is not None and _trust["grid"] is not None:
            fit, detail = localizers.scan_fit_of(_trust["grid"], scan, pose)
            history = _trust["history"]
            if fit >= 0.0:
                history.append(fit)
                del history[:-TRUST_WINDOW]
            _trust.update(fit=fit, detail=detail, at=now)
        else:
            _trust.update(fit=None, at=now, history=[],
                          detail="waiting for a pose, a scan and the saved map")
    history = _trust["history"]
    if not history:
        stale = _stale_inputs(time.time())
        if stale:
            _enforce_trust(False, map_id, 0.0)
            out = {"verdict": "stale", "threshold": localizers.SCAN_FIT_MIN,
                   "detail": stale}
            if _trust["withdrawn"]:
                out["withdrawn"] = True
            return out
        return {"verdict": "unknown", "threshold": localizers.SCAN_FIT_MIN,
                "detail": _trust["detail"] or "no reading yet"}
    ordered = sorted(history)
    fit = ordered[len(ordered) // 2]
    good = fit >= localizers.SCAN_FIT_MIN
    _enforce_trust(good, map_id, fit)
    out = {"verdict": "ok" if good else "suspect",
           "threshold": localizers.SCAN_FIT_MIN,
           "fit": round(fit, 3),
           "latest": round(history[-1], 3),
           "samples": len(history),
           "detail": _trust["detail"]}
    if _trust["withdrawn"]:
        out["withdrawn"] = True
    return out


def _enforce_trust(good: bool, map_id: str, fit: float) -> None:
    """Withdraw the map frame while the laser says the pose is wrong.

    Hysteresis, not a hair trigger: a single bad reading is a person walking
    past, and pulling the frame for that would stop a healthy robot. Sustained
    disagreement is a robot that has been moved or has slipped, and driving on
    is the dangerous option.
    """
    if map_ops.active_algo() != "slam_toolbox":
        return
    session = map_ops._LOCALIZATION.snapshot()
    if session["map_id"] == map_id and session["state"] == "running":
        # Initial localization already withdrew the engine. Never restart its
        # search just because it has not produced a verified pose yet.
        _trust["bad_since"] = 0.0
        return
    now = time.time()
    if good:
        _trust["bad_since"] = 0.0
        if _trust["withdrawn"]:
            _trust["withdrawn"] = False
        return
    if not _trust["bad_since"]:
        _trust["bad_since"] = now
        return
    if _trust["withdrawn"] or now - _trust["bad_since"] < WITHDRAW_AFTER_S:
        return
    _trust["withdrawn"] = True
    _log_add("load", f"✗ the laser has disagreed with '{map_id}' for "
                     f"{WITHDRAW_AFTER_S:.0f}s ({fit:.0%}); withdrawing the map frame "
                     f"and relocalizing")
    out = map_ops.relocalize_impl()
    if not out.get("ok"):
        _log_add("load", f"✗ automatic relocalization refused: {out.get('detail','')}")


def _localization_state() -> dict:
    """How the particle filter is doing, for the page to say so.

    `off` when no localizer is configured, `waiting` when one is running but has
    not published an estimate yet (it has just been asked to relocalize), then
    `converging` while the particle cloud is still spread out and `converged`
    once it is tight enough to act on. The spread is reported too, so the page
    can show it shrinking rather than just flipping between two words.
    """
    if map_ops.active_algo() != "slam_toolbox" or not localizers.enabled():
        return {"state": "off"}
    # Configured is not running: in mapping mode the SLAM engine owns the pose
    # and the particle filter is not up, so there is nothing to wait for.
    if map_ops.get_mode_impl().get("mode", "") != "localization":
        return {"state": "off"}
    session = map_ops._LOCALIZATION.snapshot()
    live = lifecycle.current()
    if session["map_id"] != live.get("map_id") or session["generation"] != live.get("generation"):
        return {"state": "waiting", "phase": "idle", "localizer": localizers.name(), "candidates": []}
    state = {"done": "converged", "running": "converging", "failed": "failed"}.get(session["state"], "waiting")
    return {**session, "state": state, "localizer": localizers.name()}


def _list_saved_maps() -> list[dict]:
    """Library rows for the page, taken from the capability's own listing.

    `map_ops.list_maps_impl` already knows which engine wrote each map and which
    files are that engine's graph, so the page reads the same rows every other
    consumer does instead of re-deriving them from `rtabmap.db`. The `has_db` /
    `db_size` key names stay as they were so the page's rendering is unchanged.
    """
    out = []
    res = map_ops.list_maps_impl()
    if not res.get("ok"):
        log.warning("map library listing failed: %s", res.get("detail", ""))
        return out
    for row in json.loads(res.get("maps_json") or "[]"):
        out.append({
            "map_id": row["map_id"],
            "engine": row.get("engine", ""),
            "loadable_here": bool(row.get("loadable_here", True)),
            "detail": row.get("artifact_detail", ""),
            "has_db": bool(row.get("has_spatial_artifact")),
            "has_preview": bool(row.get("has_preview")),
            "db_size": int(row.get("artifact_size", 0)),
            "updated": int(row.get("updated", 0)),
            "meta": row.get("meta", {}),
        })
    return out


# The operator page is three ordinary files under webui_static/. It used to be
# one Python string holding minified HTML, CSS and JavaScript, which made every
# change a merge hazard and silently ate escapes -- a "\n" written for a JS
# string literal became a real newline and broke the whole script. Served from
# disk, the JavaScript is JavaScript: formatted, diffable, and checkable with
# any JS tool.
STATIC_DIR = Path(__file__).resolve().parent / "webui_static"
_STATIC_TYPES = {".html": "text/html; charset=utf-8",
                 ".js": "application/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8"}


def _static(name: str) -> tuple[int, str, bytes]:
    """Read one asset. Returns (status, content_type, body).

    Path traversal is refused rather than sanitised: every asset this server
    has is a fixed name in one directory, so anything else is a mistake or an
    attack and neither deserves a file.
    """
    if name not in {"index.html", "app.js", "style.css",
                    "vendor/bootstrap.min.css", "editor.html", "editor.js", "editor.css"}:
        return 404, "text/plain", b"not found"
    path = STATIC_DIR / name
    try:
        return 200, _STATIC_TYPES[path.suffix], path.read_bytes()
    except OSError as e:
        log.error("webui asset %s unreadable: %s", path, e)
        return 500, "text/plain", str(e).encode()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
        return

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, "application/json", json.dumps(obj).encode())

    def _editor_store(self):
        from .grid_edits import Store, map_directory
        query = parse_qs(urlparse(self.path).query)
        return Store(map_directory(MAPS_DIR, query.get("map_id", [""])[0]))

    def _editor_get(self, path):
        from .grid_edits import Conflict
        try:
            store = self._editor_store()
            query = parse_qs(urlparse(self.path).query)
            revision = query.get("revision", [None])[0]
            if path == "/api/editor/meta":
                return self._json(dict(store.metadata(), auto_apply=map_ops.map_engine(str(store.path)) == "slam_toolbox"))
            for suffix, kind, content_type in (("map.png", "png", "image/png"),
                    ("export.pgm", "pgm", "image/x-portable-graymap"),
                    ("export.yaml", "yaml", "application/yaml")):
                if path == "/api/editor/" + suffix:
                    return self._send(200, content_type, store.export(kind, revision))
            return self._json({"ok": False, "detail": "not found"}, 404)
        except Conflict as exc:
            return self._json({"ok": False, "detail": str(exc)}, 409)
        except (ValueError, OSError, KeyError) as exc:
            return self._json({"ok": False, "detail": str(exc)}, 400)

    def _editor_post(self, path):
        import base64
        from .grid_edits import Conflict, limits
        try:
            # Only same-origin JSON writes: a cross-origin HTML form cannot
            # change a local map. This is not an authentication boundary.
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ValueError("expected application/json")
            n = int(self.headers.get("Content-Length", 0))
            if not 0 < n <= limits()[1]:
                return self._json({"ok": False, "detail": "invalid request size"}, 413)
            body = json.loads(self.rfile.read(n))
            if path not in ("/api/editor/save", "/api/editor/restore"):
                return self._json({"ok": False, "detail": "not found"}, 404)
            store = self._editor_store()
            restore = path.endswith("/restore")
            png = b"" if restore else base64.b64decode(body["png"].removeprefix("data:image/png;base64,"), validate=True)
            out = store.save(png, body["version"], body["base_sha256"], restore=restore)
            return self._json(out)
        except Conflict as exc:
            return self._json({"ok": False, "detail": str(exc)}, 409)
        except (ValueError, OSError, KeyError, TypeError) as exc:
            return self._json({"ok": False, "detail": str(exc)}, 400)

    def do_GET(self):
        p = urlparse(self.path).path
        try:
            if p == "/editor":
                return self._send(*_static("editor.html"))
            if p.startswith("/api/editor/"):
                return self._editor_get(p)
            if p == "/" or p == "/index.html":
                return self._send(*_static("index.html"))
            if p.startswith("/static/"):
                return self._send(*_static(p[len("/static/"):]))
            if p == "/api/map.png":
                _ensure_subscriptions()
                g = _latest["grid"]
                if g is None:
                    return self._send(503, "text/plain", b"no map yet")
                # Pure occupancy only — the live pose marker is drawn by the
                # canvas on top, so don't bake a second (scaled) one into the PNG.
                return self._send(200, "image/png", _grid_to_png(g))
            if p == "/api/state":
                _ensure_subscriptions()
                g = _latest["grid"]
                # Read the mode from map_ops rather than remembering one
                # here: a mode set by config, MCP or gRPC must show up in this
                # page too, and a load or reset changes it without the page
                # being involved at all.
                live = lifecycle.current()
                st = {"has_map": g is not None,
                      "mode": map_ops.get_mode_impl().get("mode", ""),
                      # Which SLAM engine is running decides what a saved map
                      # means, so the page names it instead of leaving the
                      # operator to guess from the deployment config.
                      "engine": map_ops.active_algo(),
                      "localizer": localizers.name(),
                      "localization": _localization_state(),
                      "trust": _localization_trust(),
                      "map_id": live.get("map_id", ""),
                      "generation": live.get("generation", 0)}
                relay = getattr(_webui_node, "_grid_relay", None)
                if relay is not None:
                    st["navigation_grid"] = dict(relay.status)
                if g is not None:
                    st.update(width=g.info.width, height=g.info.height,
                              resolution=round(g.info.resolution, 4),
                              origin_x=g.info.origin.position.x,
                              origin_y=g.info.origin.position.y)
                cur = _live_pose_xytheta()
                if cur is not None:
                    st["pose"] = {"x": cur[0], "y": cur[1], "theta": cur[2]}
                    if _seed["x"] is not None:
                        st["seed"] = {"x": _seed["x"], "y": _seed["y"], "theta": _seed["theta"]}
                        st["dist_from_seed"] = round(math.hypot(cur[0] - _seed["x"],
                                                                cur[1] - _seed["y"]), 3)
                return self._json(st)
            if p == "/api/pose_candidates":
                return self._json(map_ops.get_pose_candidates_impl())
            if p == "/api/range":
                # One request for every bound overlay: the page draws them
                # together and a second round trip would let them drift apart
                # on screen. A sensor the deployment has not bound is absent
                # from the payload rather than present and empty, so the page
                # has nothing to report about a capability that is not there.
                _ensure_subscriptions()
                if not _sensor_topics.get("scan") and _webui_node is not None:
                    if _discover_scan_topic(_webui_node):
                        _subscribe_range_sensors(_webui_node)
                out = {"bound": {k: v for k, v in _sensor_topics.items() if v}}
                for kind in ("scan", "cloud"):
                    if _sensor_topics.get(kind):
                        out[kind] = _overlay_in_map(kind)
                _log_overlay_state(out)
                return self._json(out)
            if p == "/api/log":
                with _log_lock:
                    return self._json(list(_LOG))
            if p == "/api/maps":
                return self._json(_list_saved_maps())
            if p.startswith("/api/maps/") and p.endswith("/preview.png"):
                mid = p[len("/api/maps/"):-len("/preview.png")]
                fp = os.path.join(MAPS_DIR, map_ops._sanitize_map_id(mid), "occupancy.png")
                if os.path.isfile(os.path.join(os.path.dirname(fp), ".grid-edits", "active")):
                    from .grid_edits import Store, map_directory
                    store = Store(map_directory(MAPS_DIR, mid))
                    return self._send(200, "image/png", store.export("png"))
                if os.path.isfile(fp):
                    return self._send(200, "image/png", open(fp, "rb").read())
                return self._send(404, "text/plain", b"no preview")
            return self._send(404, "text/plain", b"not found")
        except Exception as e:  # noqa: BLE001
            log.exception("webui GET %s failed", p)
            return self._send(500, "text/plain", str(e).encode())

    def do_POST(self):
        p = urlparse(self.path).path
        try:
            if p.startswith("/api/editor/"):
                return self._editor_post(p)
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
            if p == "/api/save":
                mid = body.get("map_id", "")
                out = map_ops.save_map_impl(mid, body.get("note", ""))
                _log_add("save", out.get("detail") or (f"saved {mid}" if out.get("ok") else "save failed"))
                return self._json(out)
            if p == "/api/load":
                mid, mode = body.get("map_id", ""), body.get("mode", "localization")
                out = map_ops.load_map_impl(
                    mid, mode, bool(body.get("has_initial_pose", False)),
                    float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                    float(body.get("theta", 0.0)))
                _log_add("load", f"{'✓' if out.get('ok') else '✗'} load {mid} ({mode}): {out.get('detail','')}")
                return self._json(out)
            if p == "/api/delete":
                mid = body.get("map_id", "")
                out = map_ops.delete_map_impl(mid)
                _log_add("delete", out.get("detail") or (f"deleted {mid}" if out.get("ok") else "delete failed"))
                return self._json(out)
            if p == "/api/reset":
                out = map_ops.reset_map_impl()
                _log_add("reset", out.get("detail") or ("map cleared" if out.get("ok") else "reset failed"))
                return self._json(out)
            if p == "/api/pose_estimate":
                x, y, th = (float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                            float(body.get("theta", 0.0)))
                if map_ops.active_algo() == "slam_toolbox" and "run_id" in body:
                    live = lifecycle.current()
                    if body.get("map_id") != live.get("map_id") or body.get("generation") != live.get("generation"):
                        return self._json({"ok": False, "detail": "map changed; refresh before choosing a pose"})
                    out = map_ops.pose_estimate_impl(x, y, th, run_id=int(body["run_id"]))
                else:
                    out = map_ops.pose_estimate_impl(x, y, th)
                if out.get("ok"):
                    _seed.update(x=x, y=y, theta=th, t=time.time())
                    _log_add("pose", f"estimate seeded → ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}°); "
                                     "waiting for relocalization…")
                    _track_convergence(x, y, th)
                else:
                    _log_add("pose", f"✗ pose estimate: {out.get('detail','')}")
                return self._json(out)
            if p == "/api/relocalize":
                out = map_ops.relocalize_impl()
                _log_add("load", f"{'✓' if out.get('ok') else '✗'} relocalize: {out.get('detail','')}")
                return self._json(out)
            if p == "/api/switch_mode":
                mode = body.get("mode", "")
                out = map_ops.switch_mode_impl(mode)
                _log_add("switch", f"{'✓' if out.get('ok') else '✗'} switch to {mode}: {out.get('detail','')}")
                return self._json(out)
            return self._send(404, "text/plain", b"not found")
        except Exception as e:  # noqa: BLE001
            log.exception("webui POST %s failed", p)
            return self._json({"ok": False, "detail": str(e)}, code=500)




_server = None


_supervisor = None


def start_supervisor() -> None:
    """Keep checking the localization with nobody watching the page.

    The reading and the withdrawal it drives are a property of the running
    system, not a feature of the status page: a robot whose pose has gone wrong
    is dangerous whether or not an operator has a browser open. Started
    unconditionally by atlas_bridge, so a deployment that disables the web UI
    still gets the check. The page reads the same state this loop maintains.
    """
    global _supervisor
    if _supervisor is not None:
        return

    def _loop() -> None:
        while True:
            time.sleep(TRUST_PERIOD_S)
            try:
                _ensure_subscriptions()
                _localization_trust()
            except Exception as e:  # noqa: BLE001
                log.warning("localization supervisor: %s", e)

    _supervisor = threading.Thread(target=_loop, daemon=True, name="loc-supervisor")
    _supervisor.start()
    log.info("localization supervisor running every %.1fs", TRUST_PERIOD_S)


def maybe_start() -> None:
    """Start the web UI iff MAPPING_WEBUI_PORT is set. Idempotent; non-fatal."""
    global _server
    port = os.environ.get("MAPPING_WEBUI_PORT", "").strip()
    if not port or _server is not None:
        return
    # This is an unauthenticated admin plane (delete/reset/load/pose seed), so
    # fail safe on loopback. A deployment that has an authenticated overlay
    # network may opt into another host explicitly through `webui_host`.
    host = os.environ.get("MAPPING_WEBUI_HOST", "127.0.0.1").strip() or "127.0.0.1"
    try:
        srv = ThreadingHTTPServer((host, int(port)), _Handler)
    except Exception as e:  # noqa: BLE001
        log.warning("webui: cannot bind %s:%s: %s", host, port, e)
        return
    _server = srv
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log.info("map web UI on http://%s:%s", host, port)
