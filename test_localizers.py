# SPDX-License-Identifier: MulanPSL-2.0
"""Localizer slot: configuration validation and the launch-argument contract.

Everything here runs without ROS: the pieces that need a live node (starting the
stack, the global-localization service call) are exercised in the Webots
benchmark, not in unit tests.
"""
import os
from concurrent.futures import Future
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from mapping_rbnx import localizers  # noqa: E402


class LocalizerClientTests(unittest.TestCase):
    def setUp(self):
        empty = SimpleNamespace(Request=lambda: object())
        modules = {'std_srvs': SimpleNamespace(), 'std_srvs.srv': SimpleNamespace(Empty=empty)}
        self.imports = mock.patch.dict(sys.modules, modules)
        self.imports.start()
        self.addCleanup(self.imports.stop)
        self.node = mock.Mock()
        self.client = self.node.create_client.return_value

    def test_readiness_releases_client_on_all_exit_paths(self):
        for state in ('ready', 'expired', 'exited', 'error'):
            with self.subTest(state=state):
                self.node.reset_mock()
                self.client.wait_for_service.side_effect = RuntimeError('discovery failed') if state == 'error' else None
                self.client.wait_for_service.return_value = state == 'ready'
                proc = SimpleNamespace(poll=lambda: 1, returncode=1) if state == 'exited' else None
                with mock.patch.object(localizers, '_PROC', proc):
                    if state == 'error':
                        with self.assertRaisesRegex(RuntimeError, 'discovery failed'):
                            localizers.wait_ready(self.node)
                    else:
                        ok, _ = localizers.wait_ready(self.node, timeout_s=0 if state == 'expired' else 1)
                        self.assertEqual(ok, state == 'ready')
                self.node.destroy_client.assert_called_once_with(self.client)

    def test_scatter_releases_client_and_reports_response_failures(self):
        for state in ('success', 'unavailable', 'pending', 'failed', 'error'):
            with self.subTest(state=state):
                self.node.reset_mock()
                self.client.wait_for_service.return_value = state != 'unavailable'
                future = Future()
                if state == 'success':
                    future.set_result(object())
                elif state == 'failed':
                    future.set_exception(RuntimeError('response failed'))
                self.client.call_async.return_value = future
                self.client.call_async.side_effect = RuntimeError('request failed') if state == 'error' else None
                if state == 'error':
                    with self.assertRaisesRegex(RuntimeError, 'request failed'):
                        localizers.global_localize(self.node)
                else:
                    ok, detail = localizers.global_localize(self.node, timeout_s=0)
                    self.assertEqual(ok, state == 'success')
                    if state == 'failed':
                        self.assertIn('response failed', detail)
                    if state == 'pending':
                        self.assertTrue(future.cancelled())
                self.node.destroy_client.assert_called_once_with(self.client)


class LocalizerConfigTests(unittest.TestCase):
    def tearDown(self):
        localizers.configure({})

    def test_default_is_off_so_existing_deployments_do_not_change(self):
        self.assertEqual(localizers.configure({}), "none")
        self.assertFalse(localizers.enabled())

    def test_named_localizers_enable_the_slot(self):
        for name in ("amcl", "beluga", " AMCL "):
            self.assertEqual(localizers.configure({"localizer": name}), name.strip().lower())
            self.assertTrue(localizers.enabled())

    def test_unknown_localizer_fails_at_configuration(self):
        with self.assertRaises(ValueError) as ctx:
            localizers.configure({"localizer": "amcl2"})
        self.assertIn("none, amcl, beluga", str(ctx.exception))

    def test_scan_topic_and_frames_reach_the_launch(self):
        localizers.configure({
            "localizer": "amcl", "scan_topic": "/tiago/scan",
            "base_frame": "base_footprint", "odom_frame": "odom_combined",
            "use_sim_time": True, "min_particles": 300, "max_particles": 900,
        })
        self.assertEqual(localizers._CONFIG["scan_topic"], "/tiago/scan")
        self.assertEqual(localizers._CONFIG["base_frame"], "base_footprint")
        self.assertEqual(localizers._CONFIG["odom_frame"], "odom_combined")
        self.assertTrue(localizers._CONFIG["use_sim_time"])
        self.assertEqual((localizers._CONFIG["min_particles"], localizers._CONFIG["max_particles"]), (300, 900))

    def test_start_without_a_saved_grid_is_refused_by_name(self):
        localizers.configure({"localizer": "amcl"})
        ok, detail = localizers.start("/nonexistent/map/dir", "nowhere")
        self.assertFalse(ok)
        self.assertIn("occupancy.yaml", detail)

    def test_start_is_refused_when_the_slot_is_off(self):
        localizers.configure({"localizer": "none"})
        ok, detail = localizers.start("/tmp", "any")
        self.assertFalse(ok)
        self.assertIn("localizer: none", detail)


class RelocalizationHandoverTests(unittest.TestCase):
    """The localizer recovers a pose and hands it back; the engine keeps the frame."""

    def _ops(self, algo):
        from mapping_rbnx import engines, map_ops  # noqa: F401  (map_ops registers rtabmap)
        return engines.engine_for(algo)

    def test_every_engine_can_be_held_and_resumed(self):
        for algo in ("slam_toolbox", "rtabmap"):
            ops = self._ops(algo)
            self.assertIsNotNone(ops, algo)
            for verb in ("hold", "resume", "activate"):
                self.assertTrue(callable(getattr(ops, verb, None)), f"{algo}.{verb}")

    def test_rtabmap_needs_no_hold_because_load_map_switches_its_mode(self):
        ops = self._ops("rtabmap")
        for verb in ("hold", "resume"):
            ok, detail = getattr(ops, verb)(None, 1.0)
            self.assertTrue(ok)
        self.assertIn("rtabmap", detail)

    def test_cold_start_idle_does_not_erase_localization_request(self):
        from mapping_rbnx import engines

        ops = engines.SlamToolboxOps()
        with tempfile.TemporaryDirectory() as tmp:
            ops.ENGINE_REQUEST = os.path.join(tmp, "request")
            ops.ENGINE_ARGS = os.path.join(tmp, "args")
            ops.ENGINE_PID = os.path.join(tmp, "pid")
            sleeps = 0

            def supervisor_initializes_idle(_delay):
                nonlocal sleeps
                sleeps += 1
                if sleeps == 1:
                    with open(ops.ENGINE_REQUEST, "w", encoding="utf-8") as f:
                        f.write("idle")

            with (mock.patch.object(ops, "_supervised_pid", return_value=None),
                  mock.patch.object(engines.time, "sleep",
                                    side_effect=supervisor_initializes_idle)):
                ok, detail = ops._request_engine("localization", ["map:=saved"])

            self.assertTrue(ok, detail)
            with open(ops.ENGINE_REQUEST, encoding="utf-8") as f:
                self.assertEqual(f.read(), "localization")
            self.assertEqual(sleeps, 2)

    def test_slam_toolbox_reports_a_missing_service_rather_than_pretending(self):
        # No ROS here, so the service type import fails — which must be an
        # error, not a silent success that leaves the engine running.
        ok, detail = self._ops("slam_toolbox").hold(None, 0.1)
        self.assertFalse(ok)
        self.assertIn("slam_toolbox", detail)

    def test_the_localizer_does_not_broadcast_the_frame_unless_asked(self):
        # One owner for map -> odom at every instant. The filter is silent by
        # default and speaks only when a caller has taken the engine off the
        # frame first; two publishers make the robot teleport between their
        # answers, which is what this guards.
        with open(os.path.join(os.path.dirname(__file__), "launch",
                               "localization_2d.launch.py"), encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn('DeclareLaunchArgument("tf_broadcast", default_value="false")', src)
        self.assertIn('"tf_broadcast": tf_broadcast', src)

    def test_a_loaded_map_is_localized_on_not_mapped_into(self):
        # The asynchronous node folds every scan into whatever graph it holds,
        # so localizing with it edits the map that was just loaded. The engine
        # swaps to slam_toolbox's localization executable instead, and that one
        # must not close loops: a closure against a fixed map has nothing to
        # correct and wrenched a correctly localized robot three metres sideways.
        with open(os.path.join(os.path.dirname(__file__), "launch",
                               "slam_toolbox_localization.launch.py"), encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn("localization_slam_toolbox_node", src)
        self.assertIn('"do_loop_closing": False', src)
        self.assertIn('"map_start_pose"', src)

    def test_a_static_match_has_to_beat_the_alternatives(self):
        # Any pose that explains the scan scores highly; what makes an answer
        # trustworthy is that no other place explains it nearly as well. Without
        # a margin the search returns the first saturated candidate, which was
        # six metres from the robot and reported no doubt at all.
        self.assertGreater(localizers.STATIC_MATCH_MIN, 0.5)
        self.assertGreater(localizers.STATIC_MATCH_MARGIN, 0.0)

    def test_convergence_needs_travel_not_just_a_tight_cloud(self):
        # A filter standing still in a symmetric room collapses tightly onto
        # the wrong hypothesis: measured at ±0.2 m while 4 m from the truth.
        self.assertGreaterEqual(localizers.MIN_TRAVEL_M, 1.0)
        import inspect
        sig = inspect.signature(localizers.wait_for_convergence)
        self.assertIn("min_travel_m", sig.parameters)

    def test_activate_takes_the_recovered_pose(self):
        import inspect
        sig = inspect.signature(self._ops("slam_toolbox").activate)
        self.assertIn("pose", sig.parameters)


class ConvergenceTests(unittest.TestCase):
    """The spread the UI reports, read straight off AMCL's covariance."""

    @staticmethod
    def _msg(var_x, var_y, var_yaw):
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = var_x, var_y, var_yaw

        class Msg:
            class pose:
                covariance = cov
        return Msg()

    def test_spread_reads_position_and_heading_from_the_covariance(self):
        pos, yaw = localizers.spread(self._msg(0.09, 0.16, 0.04))
        self.assertAlmostEqual(pos, 0.5)        # sqrt(0.09 + 0.16)
        self.assertAlmostEqual(yaw, 0.2)

    def test_a_filter_spread_over_the_map_is_not_converged(self):
        pos, yaw = localizers.spread(self._msg(9.0, 9.0, 1.0))
        self.assertEqual(localizers.convergence_state(pos, yaw), "converging")

    def test_a_tight_filter_is_converged(self):
        pos, yaw = localizers.spread(self._msg(0.01, 0.01, 0.004))
        self.assertEqual(localizers.convergence_state(pos, yaw), "converged")

    def test_heading_alone_can_hold_it_back(self):
        # Position is tight but the robot does not know which way it faces —
        # driving on that is how you end up mapping into a wall.
        self.assertEqual(localizers.convergence_state(0.05, 0.9), "converging")

    def test_negative_variance_does_not_raise(self):
        pos, yaw = localizers.spread(self._msg(-1.0, 0.04, -1.0))
        self.assertAlmostEqual(pos, 0.2)
        self.assertAlmostEqual(yaw, 0.0)


class LaunchFileTests(unittest.TestCase):
    def test_launch_file_ships_with_the_package(self):
        self.assertTrue(os.path.isfile(os.path.join(os.path.dirname(__file__),
                                                    "launch", "localization_2d.launch.py")))

    def test_launch_declares_every_argument_the_module_passes(self):
        with open(os.path.join(os.path.dirname(__file__), "launch",
                               "localization_2d.launch.py"), encoding="utf-8") as fh:
            src = fh.read()
        for arg in ("localizer", "map_yaml", "scan_topic", "base_frame", "odom_frame",
                    "global_frame", "use_sim_time", "min_particles", "max_particles"):
            self.assertIn(f'DeclareLaunchArgument("{arg}"', src, f"{arg} is passed but not declared")

    def test_beluga_uses_its_packaged_executable_and_stable_pose_topic(self):
        with open(os.path.join(os.path.dirname(__file__), "launch",
                               "localization_2d.launch.py"), encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn('"amcl_node" if localizer == "beluga"', src)
        self.assertIn('[("pose", "/amcl_pose")]', src)
        self.assertNotIn('package.replace("_", "-")', src)

    def test_localization_boot_starts_engine_supervisor_idle(self):
        with open(os.path.join(os.path.dirname(__file__), "scripts",
                               "start_engine.sh"), encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn('if [ "${MAP_MODE:-mapping}" = "localization" ]', src)
        self.assertIn("printf '%s\\n' idle > \"$ENGINE_REQUEST\"", src)

    def test_non_rtabmap_localization_runs_the_staged_startup_load(self):
        with open(os.path.join(os.path.dirname(__file__), "src", "mapping_rbnx",
                               "atlas_bridge.py"), encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn('active_mode == "localization" and algo != "rtabmap"', src)
        self.assertIn('map_ops.load_map_impl(map_id, "localization")', src)



class EngineRegistryTests(unittest.TestCase):
    """Every engine that can persist maps must name its artifacts."""

    def test_slam_toolbox_declares_its_graph_files(self):
        from mapping_rbnx import engines
        ops = engines.engine_for("slam_toolbox")
        self.assertIsNotNone(ops)
        self.assertEqual(set(ops.graph_files), {"posegraph.posegraph", "posegraph.data"})

    def test_missing_graph_is_reported_by_name(self):
        from mapping_rbnx import engines
        ok, detail = engines.engine_for("slam_toolbox").graph_ready("/nonexistent")
        self.assertFalse(ok)
        self.assertIn("posegraph", detail)

    def test_engines_without_persistence_are_absent_rather_than_broken(self):
        from mapping_rbnx import engines
        self.assertIsNone(engines.engine_for("dlio"))
        self.assertEqual(engines.graph_files_for("dlio"), ())

class MapPersistenceTests(unittest.TestCase):
    """Backend tagging: a saved map names its engine, and a map from another
    engine is refused by name instead of failing inside the load."""

    def setUp(self):
        import tempfile
        from mapping_rbnx import map_ops
        self.map_ops = map_ops
        self.tmp = tempfile.mkdtemp()
        self._saved_maps_dir = map_ops.MAPS_DIR
        map_ops.MAPS_DIR = self.tmp
        self._saved_algo = os.environ.get("MAPPING_ALGO")

    def tearDown(self):
        import shutil
        self.map_ops.MAPS_DIR = self._saved_maps_dir
        if self._saved_algo is None:
            os.environ.pop("MAPPING_ALGO", None)
        else:
            os.environ["MAPPING_ALGO"] = self._saved_algo
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _map(self, name, files, meta=""):
        d = os.path.join(self.tmp, name)
        os.makedirs(d, exist_ok=True)
        for f in files:
            with open(os.path.join(d, f), "wb") as fh:
                fh.write(b"x" * 16)
        if meta:
            with open(os.path.join(d, "meta.yaml"), "w", encoding="utf-8") as fh:
                fh.write(meta)
        return d

    def test_meta_records_the_engine_and_keeps_the_other_fields(self):
        d = self._map("m", (), "map_id: m\nnote: hallway\n")
        self.map_ops._write_meta_fields(d, {"engine": "slam_toolbox"})
        meta = self.map_ops.read_meta(d)
        self.assertEqual(meta["engine"], "slam_toolbox")
        self.assertEqual((meta["map_id"], meta["note"]), ("m", "hallway"))

    def test_maps_saved_before_the_field_existed_read_as_rtabmap(self):
        d = self._map("old", ("rtabmap.db",), "map_id: old\n")
        self.assertEqual(self.map_ops.map_engine(d), "rtabmap")
        self.assertEqual(self.map_ops.map_engine(self._map("empty", ())), "")

    def test_load_refuses_a_map_built_by_another_engine(self):
        self._map("stmap", ("posegraph.posegraph", "posegraph.data"),
                  "map_id: stmap\nengine: slam_toolbox\n")
        os.environ["MAPPING_ALGO"] = "rtabmap"
        out = self.map_ops.load_map_impl("stmap")
        self.assertFalse(out["ok"])
        self.assertIn("slam_toolbox", out["detail"])
        self.assertIn("rtabmap", out["detail"])

    def test_listing_names_the_engine_and_which_maps_load_here(self):
        self._map("stmap", ("posegraph.posegraph", "posegraph.data"),
                  "map_id: stmap\nengine: slam_toolbox\n")
        self._map("rtmap", ("rtabmap.db",), "map_id: rtmap\nengine: rtabmap\n")
        os.environ["MAPPING_ALGO"] = "slam_toolbox"
        import json
        rows = {r["map_id"]: r for r in json.loads(self.map_ops.list_maps_impl()["maps_json"])}
        self.assertEqual(rows["stmap"]["engine"], "slam_toolbox")
        self.assertTrue(rows["stmap"]["loadable_here"])
        self.assertEqual(rows["stmap"]["artifact_size"], 32)   # both graph files
        self.assertFalse(rows["rtmap"]["loadable_here"])
        self.assertIn("this deployment runs slam_toolbox", rows["rtmap"]["artifact_detail"])

    def test_missing_graph_file_is_not_advertised_as_loadable(self):
        self._map("half", ("posegraph.posegraph",), "map_id: half\nengine: slam_toolbox\n")
        os.environ["MAPPING_ALGO"] = "slam_toolbox"
        import json
        row = json.loads(self.map_ops.list_maps_impl()["maps_json"])[0]
        self.assertFalse(row["has_spatial_artifact"])
        self.assertFalse(row["loadable_here"])


class StartupPersistenceResolutionTests(unittest.TestCase):
    """CMD_INIT validates the artifact owned by the selected mapping engine."""

    def setUp(self):
        from mapping_rbnx import atlas_bridge, map_ops
        self.bridge = atlas_bridge
        self.map_ops = map_ops
        self.tmp = tempfile.TemporaryDirectory()
        self.maps_dir = os.path.join(self.tmp.name, "maps")
        self.runtime_dir = os.path.join(self.tmp.name, "runtime")
        os.makedirs(self.maps_dir)
        self.saved_maps_dir = self.bridge.MAPS_DIR
        self.saved_runtime_dir = self.map_ops.RUNTIME_DB_DIR
        self.bridge.MAPS_DIR = self.maps_dir
        self.map_ops.RUNTIME_DB_DIR = self.runtime_dir

    def tearDown(self):
        self.bridge.MAPS_DIR = self.saved_maps_dir
        self.map_ops.RUNTIME_DB_DIR = self.saved_runtime_dir
        self.tmp.cleanup()

    def _map(self, map_id, engine, files):
        map_dir = os.path.join(self.maps_dir, map_id)
        os.makedirs(map_dir)
        with open(os.path.join(map_dir, "meta.yaml"), "w", encoding="utf-8") as fh:
            fh.write(f"map_id: {map_id}\nengine: {engine}\n")
        for name in files:
            with open(os.path.join(map_dir, name), "wb") as fh:
                fh.write(b"saved-map-artifact")
        return map_dir

    def test_complete_slam_toolbox_graph_is_accepted_without_runtime_db(self):
        self._map("lab", "slam_toolbox", ("posegraph.posegraph", "posegraph.data"))
        out = self.bridge._resolve_persistence({
            "algo": "slam_toolbox", "map_mode": "localization", "map_id": "lab",
        })
        self.assertEqual(out["map_id"], "lab")
        self.assertNotIn("database_path", out)

    def test_incomplete_slam_toolbox_graph_names_the_missing_file(self):
        self._map("half", "slam_toolbox", ("posegraph.posegraph",))
        with self.assertRaisesRegex(RuntimeError, "posegraph.data"):
            self.bridge._resolve_persistence({
                "algo": "slam_toolbox", "map_mode": "localization", "map_id": "half",
            })

    def test_saved_engine_must_match_selected_engine(self):
        self._map("other", "rtabmap", ("rtabmap.db",))
        with self.assertRaisesRegex(RuntimeError, "built by rtabmap"):
            self.bridge._resolve_persistence({
                "algo": "slam_toolbox", "map_mode": "localization", "map_id": "other",
            })

    def test_rtabmap_localization_uses_a_private_runtime_copy(self):
        map_dir = self._map("rt", "rtabmap", ("rtabmap.db",))
        runtime_db = os.path.join(self.runtime_dir, "rt-runtime.db")
        fake_ops = mock.Mock()
        fake_ops.graph_ready.return_value = (True, "database ok")
        with mock.patch.object(self.bridge.engines, "engine_for", return_value=fake_ops), \
                mock.patch.object(self.map_ops, "_runtime_db_copy", return_value=runtime_db) as copy:
            out = self.bridge._resolve_persistence({
                "algo": "rtabmap", "map_mode": "localization", "map_id": "rt",
            })
        copy.assert_called_once_with(os.path.join(map_dir, "rtabmap.db"), "rt")
        self.assertEqual(out["database_path"], runtime_db)

    def test_non_rtabmap_mapping_does_not_create_a_sqlite_runtime_db(self):
        out = self.bridge._resolve_persistence({
            "algo": "slam_toolbox", "map_mode": "mapping", "map_id": "new-map",
        })
        self.assertEqual(out, {"map_mode": "mapping", "reset_map": "true"})
        self.assertFalse(os.path.exists(self.runtime_dir))


class SlamToolboxParamTests(unittest.TestCase):
    """A mistyped scan-matching knob must fail at init, not at map time."""

    def test_shutdown_reaps_the_recorded_slam_toolbox_process_group(self):
        root = os.path.dirname(__file__)
        with open(os.path.join(root, "scripts", "stop.sh"), encoding="utf-8") as f:
            stop_source = f.read()
        with open(os.path.join(root, "scripts", "start_engine.sh"), encoding="utf-8") as f:
            engine_source = f.read()
        self.assertIn('kill -TERM -- "-$pgid"', stop_source)
        self.assertIn('kill -KILL -- "-$pgid"', stop_source)
        self.assertIn('terminate_recorded_engine_group', stop_source)
        self.assertIn('stale slam_toolbox process group', engine_source)
        self.assertIn('trap cleanup_slam_toolbox EXIT', engine_source)

    def test_native_launch_directory_is_used_for_both_slam_toolbox_nodes(self):
        root = os.path.dirname(__file__)
        with open(os.path.join(root, "scripts", "start_engine.sh"), encoding="utf-8") as f:
            source = f.read()
        self.assertIn(
            '${MAPPING_LAUNCH_DIR:-/mapping/launch}/slam_toolbox_2d.launch.py',
            source,
        )
        self.assertIn(
            '${MAPPING_LAUNCH_DIR:-/mapping/launch}/slam_toolbox_localization.launch.py',
            source,
        )

    def setUp(self):
        from mapping_rbnx import profiles
        self.bridge = profiles

    def test_unset_params_keep_the_launch_defaults(self):
        self.assertEqual(self.bridge.resolve_slam_toolbox_overrides(None), {})
        self.assertEqual(self.bridge.resolve_slam_toolbox_overrides({}), {})

    def test_known_keys_reach_the_resolved_file(self):
        out = self.bridge.resolve_slam_toolbox_overrides({"min_travel_m": 0.15, "scan_buffer": 40})
        self.assertEqual(out, {"slam_toolbox_min_travel_m": "0.15",
                               "slam_toolbox_scan_buffer": "40.0"})

    def test_typo_and_bad_values_are_refused_by_name(self):
        for bad, expect in (({"min_travel": 0.1}, "unknown slam_toolbox_params"),
                            ({"scan_buffer": "many"}, "is not a number"),
                            ({"min_travel_m": 0}, "must be positive"),
                            ([("min_travel_m", 1)], "must be a mapping")):
            with self.assertRaises(RuntimeError) as ctx:
                self.bridge.resolve_slam_toolbox_overrides(bad)
            self.assertIn(expect, str(ctx.exception))

    def test_start_engine_forwards_every_key_the_bridge_writes(self):
        with open(os.path.join(os.path.dirname(__file__), "scripts", "start_engine.sh"),
                  encoding="utf-8") as fh:
            src = fh.read()
        for key in self.bridge.SLAM_TOOLBOX_KEYS.values():
            self.assertIn(key, src, f"{key} is written but start_engine.sh never reads it")


if __name__ == "__main__":
    unittest.main()


class TrustEnforcementTests(unittest.TestCase):
    """Withdrawing the map frame — the part that makes the verdict binding.

    The status line saying "suspect" is advice; nav2 accepted a goal and drove
    on a pose 7 m wrong while it said so. Publishing `map -> odom` is a claim to
    know where the robot is, so the claim is withdrawn when the evidence stops
    supporting it — including when the evidence stops arriving at all.
    """

    def setUp(self):
        from mapping_rbnx import webui, map_ops
        self.webui, self.map_ops = webui, map_ops
        from unittest.mock import patch
        algo = patch.object(map_ops, "active_algo", return_value="slam_toolbox")
        algo.start()
        self.addCleanup(algo.stop)
        self._relocalize = map_ops.relocalize_impl
        self.calls = []
        map_ops.relocalize_impl = lambda: (self.calls.append(1), {"ok": True})[1]
        self._after = webui.WITHDRAW_AFTER_S
        webui.WITHDRAW_AFTER_S = 0.0     # the timer itself is not what is under test
        webui._trust.update(bad_since=0.0, withdrawn=False)

    def tearDown(self):
        self.map_ops.relocalize_impl = self._relocalize
        self.webui.WITHDRAW_AFTER_S = self._after
        self.webui._trust.update(bad_since=0.0, withdrawn=False)

    def test_one_bad_reading_does_not_stand_the_robot_down(self):
        # Somebody walking past the laser is not a lost robot.
        self.webui._enforce_trust(False, "m", 0.3)
        self.assertFalse(self.webui._trust["withdrawn"])
        self.assertEqual(self.calls, [])

    def test_sustained_disagreement_withdraws_once_and_relocalizes(self):
        self.webui._enforce_trust(False, "m", 0.3)   # arms the timer
        self.webui._enforce_trust(False, "m", 0.3)   # expires it (WITHDRAW_AFTER_S = 0)
        self.assertTrue(self.webui._trust["withdrawn"])
        self.webui._enforce_trust(False, "m", 0.3)   # must not fire again
        self.assertEqual(len(self.calls), 1)

    def test_recovery_re_arms_the_check(self):
        self.webui._enforce_trust(False, "m", 0.3)
        self.webui._enforce_trust(False, "m", 0.3)
        self.webui._enforce_trust(True, "m", 0.9)
        self.assertFalse(self.webui._trust["withdrawn"])
        self.webui._enforce_trust(False, "m", 0.3)
        self.webui._enforce_trust(False, "m", 0.3)
        self.assertEqual(len(self.calls), 2)

    def test_a_silent_sensor_is_not_confidence(self):
        # A carried robot froze mapping outright, and the check compared the
        # cached scan against the cached pose: they were captured together, so
        # they agreed perfectly. It read "ok 100%" for two minutes while the
        # true error was 2.97 m. Silence has to be its own verdict.
        import time
        now = time.time()
        self.webui._latest["scan_at"] = now
        self.webui._latest["pose_at"] = now
        self.assertEqual(self.webui._stale_inputs(now), "")
        self.webui._latest["scan_at"] = now - (self.webui.STALE_AFTER_S + 1)
        self.assertIn("no laser", self.webui._stale_inputs(now))
        self.webui._latest["scan_at"] = now
        self.webui._latest["pose_at"] = now - (self.webui.STALE_AFTER_S + 1)
        self.webui._latest["localizer_pose_at"] = 0.0
        self.assertIn("no pose", self.webui._stale_inputs(now))


class SavedMapThresholdTests(unittest.TestCase):
    """The saved map must still have three states when it is read back.

    save_map writes nav2's pixel convention (254 free, 205 unknown, 0 occupied)
    and a yaml beside it; the two have to agree. They did not. map_server reads
    a pixel as occ = (255 - pixel) / 255, so unknown came out at 0.196, under a
    free_thresh of 0.25 -- every cell the robot had never seen was handed to
    nav2 as free floor to plan across, and a loaded map rendered pure black and
    white with the unknown state gone. It only showed up on a loaded map:
    while mapping, `/map` comes straight from slam_toolbox, where unknown is -1
    and no threshold is involved.
    """

    UNKNOWN_PX, FREE_PX, OCCUPIED_PX = 205, 254, 0

    @staticmethod
    def _trinary(pixel, free_thresh, occupied_thresh=0.65):
        """map_server's own reading of one pixel, negate: 0."""
        occ = (255 - pixel) / 255.0
        if occ > occupied_thresh:
            return 100
        return 0 if occ < free_thresh else -1

    def _written_free_thresh(self):
        import re
        here = os.path.dirname(__file__)
        src = open(os.path.join(here, "scripts", "save_map.py"), encoding="utf-8").read()
        m = re.search(r"free_thresh: ([0-9.]+)", src)
        self.assertIsNotNone(m, "save_map.py no longer writes free_thresh")
        return float(m.group(1))

    def test_the_three_states_survive_a_save_and_load(self):
        ft = self._written_free_thresh()
        self.assertEqual(self._trinary(self.FREE_PX, ft), 0)
        self.assertEqual(self._trinary(self.UNKNOWN_PX, ft), -1,
                         "unknown is being served as free floor")
        self.assertEqual(self._trinary(self.OCCUPIED_PX, ft), 100)

    def test_the_threshold_is_not_a_coincidence(self):
        # ROS's default 0.196 separates unknown from free by 0.00008, which is
        # close enough that a float or a rounded yaml turns the map back into
        # two states. Insist on real clearance.
        ft = self._written_free_thresh()
        self.assertLess(ft, (255 - self.UNKNOWN_PX) / 255.0 - 0.05,
                        "free_thresh sits too close to unknown to be safe")
        self.assertGreater(ft, (255 - self.FREE_PX) / 255.0 + 0.05,
                           "free_thresh sits too close to free to be safe")


class ScanFitEvidenceTests(unittest.TestCase):
    """A fit is a verdict on the pose only when the scan can deliver one.

    Beams ending where the map never looked are excluded as evidence, which is
    right: a robot facing an unmapped corridor should not be marked wrong for
    it. But a pose that has drifted off the map excludes almost every beam for
    the same reason, and the few that graze a wall then read 70% while the
    overlay shows the scan nowhere near a wall. Below a floor on how much of
    the scan was scored at all, the answer is "unknown", not a score.
    """

    class _Scan:
        angle_min = -1.57
        angle_increment = 0.0157      # 200 beams over 180 degrees
        range_min = 0.05
        range_max = 12.0

        def __init__(self, ranges):
            self.ranges = ranges

    def _map(self):
        """A 2 x 2 m map: one wall, and observed ground only near it."""
        from mapping_rbnx import localizers
        w = h = 40
        occ, known = bytearray(w * h), bytearray(w * h)
        for c in range(w):
            occ[2 * w + c] = 1
            for r in range(5):
                known[r * w + c] = 1
        return localizers._Grid(w, h, bytes(occ), bytes(known), 0.05, (0.0, 0.0))

    def test_a_pose_that_has_left_the_map_reads_unknown_not_high(self):
        from mapping_rbnx import localizers
        # Facing off the edge: most beams leave the grid entirely and are no
        # evidence, so only a fraction of the scan can be scored. Without the
        # floor those few would be reported as the pose's quality.
        fit, detail = localizers.scan_fit_of(self._map(), self._Scan([1.0] * 200),
                                             (1.0, 0.15, -1.5708))
        self.assertEqual(fit, -1.0, detail)
        self.assertIn("too little of the scan", detail)
        self.assertIn("25 of 200", detail)

    def test_the_floor_is_a_share_of_the_scan_not_a_count(self):
        from mapping_rbnx import localizers
        self.assertGreater(localizers.SCAN_FIT_MIN_SCORED, 0.0)
        self.assertLess(localizers.SCAN_FIT_MIN_SCORED, 1.0)
