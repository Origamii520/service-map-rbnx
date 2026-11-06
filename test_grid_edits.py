# SPDX-License-Identifier: MulanPSL-2.0
"""Storage, HTTP, and localization consumers without a robot or ROS graph."""
import base64
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import numpy as np
from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).parent / 'src'))
from mapping_rbnx.grid_edits import Store, Conflict, effective_directory, map_directory
from mapping_rbnx.grid_relay import Selection


def png(pixels):
    buf = io.BytesIO()
    Image.fromarray(pixels).save(buf, format='PNG')
    return buf.getvalue()


class LimitsTests(unittest.TestCase):
    def setUp(self):
        from mapping_rbnx import grid_edits
        self.edits = grid_edits
        grid_edits.configure({})
        self.addCleanup(grid_edits.configure, {})

    def test_defaults_override_and_reset(self):
        self.assertEqual(self.edits.limits(), (16_000_000, 24_000_000))
        self.edits.configure({'editor_max_pixels': 6, 'editor_max_body_bytes': 300})
        self.assertEqual(self.edits.limits(), (6, 300))
        self.edits.configure({'editor_max_pixels': 12})
        self.assertEqual(self.edits.limits(), (12, 24_000_000))
        self.edits.configure({})
        self.assertEqual(self.edits.limits(), (16_000_000, 24_000_000))

    def test_invalid_config_is_rejected_without_partial_update(self):
        for key in ('editor_max_pixels', 'editor_max_body_bytes'):
            for value in (0, -1, 1.5, True, '100', None):
                with self.subTest(key=key, value=value):
                    with self.assertRaisesRegex(ValueError, key):
                        self.edits.configure({key: value})
                    self.assertEqual(self.edits.limits(), (16_000_000, 24_000_000))
        with self.assertRaises(ValueError):
            self.edits.configure({'editor_max_pixels': 10, 'editor_max_body_bytes': 0})
        self.assertEqual(self.edits.limits(), (16_000_000, 24_000_000))

    def test_pixel_boundary_rejects_before_decode_and_can_be_raised(self):
        payload = png(np.zeros((2, 3), dtype=np.uint8))
        self.edits.configure({'editor_max_pixels': 5})
        with patch.object(Image.Image, 'convert', side_effect=AssertionError('decoded too early')):
            with self.assertRaisesRegex(ValueError, 'pixel limit'):
                self.edits._image(payload)
        self.edits.configure({'editor_max_pixels': 6})
        self.assertEqual(self.edits._image(payload).shape, (2, 3))

    def test_manifest_init_applies_limits_before_other_setup(self):
        from mapping_rbnx import atlas_bridge
        with patch.object(atlas_bridge, '_check_binding_complete', side_effect=RuntimeError('fixture stop')):
            atlas_bridge.init({'editor_max_pixels': 10, 'editor_max_body_bytes': 500})
        self.assertEqual(self.edits.limits(), (10, 500))
        with patch.object(atlas_bridge, '_check_binding_complete') as check:
            result = atlas_bridge.init({'editor_max_pixels': 0})
            self.assertIsInstance(result, atlas_bridge.Err)
            check.assert_not_called()


class EditsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / 'lab'
        self.path.mkdir()
        self.base = np.array([[0, 127, 205], [254, 255, 230]], dtype=np.uint8)
        Image.fromarray(self.base).save(self.path / 'occupancy.pgm')
        self.meta = dict(image='occupancy.pgm', resolution=.05, origin=[1., -2., .3], negate=0, free_thresh=.25, occupied_thresh=.65)
        (self.path / 'occupancy.yaml').write_text(yaml.safe_dump(self.meta))
        (self.path / 'posegraph.posegraph').write_bytes(b'original graph')
        self.store = Store(self.path)

    def save(self, pixels, store=None):
        store = store or self.store
        return store.save(png(pixels), store.revision(), store.fingerprint)

    def test_sparse_delta_restart_restore_and_original_preserved(self):
        original = {p.name:p.read_bytes() for p in self.path.iterdir()}
        pixels = self.base.copy(); pixels[0, 1] = 254
        saved = self.save(pixels)
        delta = json.loads((self.store.root / saved['version'] / 'delta.json').read_text())
        self.assertEqual(delta['cells'], [[1, 254]])
        restarted = Store(self.path)
        np.testing.assert_array_equal(restarted.pixels(), pixels)
        self.assertEqual(restarted.metadata()['changed_cells'], 1)
        restarted.save(b'', saved['version'], restarted.fingerprint, restore=True)
        np.testing.assert_array_equal(Store(self.path).pixels(), self.base)
        self.assertTrue((self.store.root / saved['version']).is_dir())
        self.assertEqual(original, {name:(self.path/name).read_bytes() for name in original})

    def test_noop_preserves_noncanonical_gray(self):
        self.assertEqual(self.save(self.base)['changed_cells'], 0)
        np.testing.assert_array_equal(self.store.pixels(), self.base)

    def test_revision_conflict_keeps_current_edit(self):
        first = self.save(self.base)
        with self.assertRaises(Conflict):
            self.store.save(png(self.base), 'original', self.store.fingerprint)
        self.assertEqual(self.store.revision(), first['version'])

    def test_changed_base_rejected(self):
        self.save(self.base)
        (self.path/'occupancy.yaml').write_text(yaml.safe_dump(dict(self.meta, resolution=.1)))
        with self.assertRaises(Conflict):
            Store(self.path).pixels()

    def test_dimensions_alpha_and_path_rejected(self):
        with self.assertRaises(ValueError): self.save(self.base[:1])
        with self.assertRaises(ValueError): self.store.save(png(np.zeros((2,3,4),dtype=np.uint8)), 'original', self.store.fingerprint)
        for name in ('..', '../lab', '/lab', 'lab/x', ''):
            with self.assertRaises(ValueError): map_directory(self.root, name)
        (self.root/'link').symlink_to(self.root.parent)
        with self.assertRaises(ValueError): map_directory(self.root, 'link')

    def test_negate_and_geometry_exports(self):
        for negate in (0, 1):
            (self.path/'occupancy.yaml').write_text(yaml.safe_dump(dict(self.meta, negate=negate)))
            store = Store(self.path)
            palette = store.metadata()['palette']
            pixels = np.array([[palette['wall'],palette['free'],palette['unknown']]]*2, dtype=np.uint8)
            self.save(pixels, store)
            self.assertEqual(store.occupancy(), [100,0,-1]*2)
            exported = yaml.safe_load(store.export('yaml'))
            self.assertEqual(exported['origin'], self.meta['origin'])
            self.assertEqual(exported['negate'], negate)
            # Each subcase gets a fresh edit history against its base metadata.
            import shutil
            shutil.rmtree(store.root)

    def test_effective_cache_and_corruption(self):
        self.assertEqual(effective_directory(self.path), str(self.path))
        saved = self.save(self.base)
        effective = Path(effective_directory(self.path))
        self.assertEqual(effective.name, saved['version'])
        Image.fromarray(np.zeros_like(self.base)).save(effective/'occupancy.pgm')
        with self.assertRaises(ValueError): effective_directory(self.path)

    def test_selection_refresh_restore_mapping_and_other_map(self):
        selection = Selection(self.root)
        state = dict(map_id='lab', mode='localization', generation=1)
        initial = selection.get(state)[1]
        pixels = self.base.copy(); pixels[0,0] = 254
        self.save(pixels)
        self.assertNotEqual(initial, selection.get(state)[1])
        self.store.save(b'', self.store.revision(), self.store.fingerprint, restore=True)
        self.assertEqual(initial, selection.get(state)[1])
        self.assertIsNone(selection.get(dict(state,mode='mapping')))
        with self.assertRaises(OSError): selection.get(dict(state,map_id='other'))

    def test_localization_pins_edited_grid_but_keeps_graph_directory(self):
        from mapping_rbnx import map_ops
        self.save(self.base)
        ops = Mock(); ops.stop_engine.return_value=(True,'ok')
        backend = map_ops._LocalizationBackend(Mock(), ops, str(self.path), 'lab')
        run = Mock()
        backend.prepare(run)
        self.assertEqual(backend.grid_dir, effective_directory(self.path))
        self.assertEqual(backend.map_dir, str(self.path))
        ops.switch_mode.return_value=(False,'stop before ROS')
        backend.handover(run,(1,2,0))
        self.assertEqual(ops.switch_mode.call_args.args[2], str(self.path))

    def test_engine_confirmation_uses_original_grid(self):
        from mapping_rbnx import map_ops
        pixels=self.base.copy(); pixels[0,0]=255
        self.save(pixels)
        ops=Mock();ops.stop_engine.return_value=(True,'ok')
        ops.switch_mode.return_value=(True,'ok');ops.wait_ready.return_value=(True,'ok')
        backend=map_ops._LocalizationBackend(Mock(),ops,str(self.path),'lab')
        run=Mock();run.cancelled.wait.return_value=False
        backend.prepare(run)
        self.assertNotEqual(backend.grid.cells,backend.tracking_grid.cells)
        with patch.object(map_ops,'_confirm_engine_pose',return_value=(True,'ok')) as confirm:
            backend.handover(run,(1,2,0))
            self.assertIs(confirm.call_args.args[1],backend.tracking_grid)

    def test_failed_activation_does_not_change_active_revision(self):
        before = self.save(self.base)['version']
        with patch('mapping_rbnx.grid_edits.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError): self.save(self.base)
        self.assertEqual(self.store.revision(), before)
        np.testing.assert_array_equal(Store(self.path).pixels(), self.base)

    def test_two_editors_cannot_silently_overwrite(self):
        from concurrent.futures import ThreadPoolExecutor
        def commit(value):
            pixels = self.base.copy();pixels[0,0]=value
            try:
                Store(self.path).save(png(pixels), 'original', self.store.fingerprint)
                return True
            except Conflict:
                return False
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(commit, [254,205]))
        self.assertEqual(sorted(results),[False,True])

    def test_free_cells_for_negated_initial_localization(self):
        from mapping_rbnx import localizers
        (self.path/'occupancy.yaml').write_text(yaml.safe_dump(dict(self.meta, negate=1,origin=[0,0,0])))
        store = Store(self.path)
        pixels = np.array([[0,255,127],[0,255,127]],dtype=np.uint8)
        self.save(pixels,store)
        grid = localizers.load_grid(effective_directory(self.path))
        self.assertEqual(list(grid.known),[1,1,0]*2)
        self.assertEqual(list(grid.cells),[0,1,0]*2)

    def test_backend_declares_effective_occupancy_topic(self):
        from mapping_rbnx import atlas_bridge
        from mapping_rbnx.grid_relay import TOPIC
        with patch.object(atlas_bridge.mapping,'declare_ros2_topic') as declare:
            atlas_bridge._declare_outputs(Mock(),'slam_toolbox',{})
        args=[c.args for c in declare.call_args_list if c.args[0]=='robonix/service/map/occupancy_grid']
        self.assertEqual(args,[('robonix/service/map/occupancy_grid',TOPIC)])


class HttpTests(unittest.TestCase):
    setUp = EditsTests.setUp
    save = EditsTests.save

    def test_api_save_conflict_export_and_bad_request(self):
        from mapping_rbnx import webui
        with patch.object(webui, 'MAPS_DIR', str(self.root)):
            server = ThreadingHTTPServer(('127.0.0.1',0),webui._Handler)
            thread = threading.Thread(target=server.serve_forever,daemon=True); thread.start()
            try:
                base = 'http://127.0.0.1:%s'%server.server_address[1]
                meta = json.load(urlopen(base+'/api/editor/meta?map_id=lab'))
                body = dict(png=base64.b64encode(png(self.base)).decode(),version=meta['version'],base_sha256=meta['base_sha256'])
                request = Request(base+'/api/editor/save?map_id=lab',data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
                from mapping_rbnx import grid_edits
                self.addCleanup(grid_edits.configure, {})
                grid_edits.configure({'editor_max_body_bytes': len(request.data) - 1})
                with self.assertRaises(HTTPError) as exc: urlopen(request)
                self.assertEqual(exc.exception.code,413)
                self.assertEqual(self.store.revision(), 'original')
                grid_edits.configure({'editor_max_body_bytes': len(request.data)})
                response = json.load(urlopen(request)); self.assertTrue(response['ok'])
                with self.assertRaises(HTTPError) as exc: urlopen(request)
                self.assertEqual(exc.exception.code,409)
                exported = urlopen(base+'/api/editor/export.pgm?map_id=lab').read()
                self.assertTrue(exported.startswith(b'P5'))
                with self.assertRaises(HTTPError) as exc: urlopen(base+'/api/editor/meta?map_id=..')
                self.assertEqual(exc.exception.code,400)
                self.assertIn(b'editor.js',urlopen(base+'/editor?map_id=lab').read())
            finally:
                server.shutdown();server.server_close();thread.join()

if __name__ == '__main__': unittest.main()
