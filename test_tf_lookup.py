# SPDX-License-Identifier: MulanPSL-2.0
"""TF query lifecycle regressions, runnable without ROS."""
import os
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from mapping_rbnx import localizers, tf_lookup


class TransformUnavailable(Exception):
    pass


class TransformLookupTests(unittest.TestCase):
    def setUp(self):
        self.node = mock.Mock()
        self.executor = mock.Mock()
        self.buffer = mock.Mock()
        self.listener = mock.Mock()
        self.node_type = mock.Mock(return_value=self.node)
        self.executor_type = mock.Mock(return_value=self.executor)
        self.listener_type = mock.Mock(return_value=self.listener)
        modules = {
            'rclpy': SimpleNamespace(),
            'rclpy.node': SimpleNamespace(Node=self.node_type),
            'rclpy.executors': SimpleNamespace(SingleThreadedExecutor=self.executor_type),
            'rclpy.time': SimpleNamespace(Time=lambda: 0),
            'tf2_ros': SimpleNamespace(Buffer=lambda: self.buffer,
                                      TransformListener=self.listener_type,
                                      TransformException=TransformUnavailable),
        }
        patcher = mock.patch.dict(sys.modules, modules)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.source = mock.Mock()

    def test_receives_on_own_node_and_cleans_up_after_success(self):
        result = object()
        self.buffer.lookup_transform.side_effect = [TransformUnavailable(), result]
        self.assertIs(tf_lookup.lookup_transform(self.source, 'base', 'laser', 1), result)
        self.assertIs(self.node_type.call_args.kwargs['context'], self.source.context)
        self.listener_type.assert_called_once_with(self.buffer, self.node, spin_thread=False)
        self.executor.add_node.assert_called_once_with(self.node)
        self.source.assert_not_called()
        self.assertEqual(self.source.method_calls, [])
        self.executor.spin_once.assert_called_once()
        self.executor.shutdown.assert_called_once()
        self.listener.unregister.assert_called_once()
        self.node.destroy_node.assert_called_once()

    def test_failure_paths_release_resources(self):
        for failure in ('timeout', 'dispatch', 'listener'):
            with self.subTest(failure=failure):
                for obj in (self.executor, self.listener, self.node):
                    obj.reset_mock()
                self.buffer.lookup_transform.side_effect = TransformUnavailable('missing frame')
                self.executor.spin_once.side_effect = RuntimeError('dispatch') if failure == 'dispatch' else None
                self.listener_type.side_effect = RuntimeError('listener') if failure == 'listener' else None
                with self.assertRaises((TransformUnavailable, RuntimeError)):
                    tf_lookup.lookup_transform(self.source, 'base', 'laser', 0 if failure == 'timeout' else 1)
                self.executor.shutdown.assert_called_once()
                self.node.destroy_node.assert_called_once()
                self.assertEqual(self.listener.unregister.call_count, int(failure != 'listener'))

    def test_identical_frames_need_no_listener(self):
        with mock.patch.object(localizers, '_sensor_offset', None), \
                mock.patch.dict(os.environ, {localizers.SENSOR_OFFSET_ENV: ''}):
            self.assertEqual(localizers.sensor_offset_xy(self.source, 'base_link'), (0., 0.))
        self.listener_type.assert_not_called()

    def test_explicit_mount_override_is_preserved(self):
        with mock.patch.object(localizers, '_sensor_offset', None), \
                mock.patch.dict(os.environ, {localizers.SENSOR_OFFSET_ENV: '.2 -.1'}):
            self.assertEqual(localizers.sensor_offset_xy(self.source, 'base_link'), (.2, -.1))
        self.listener_type.assert_not_called()

    def test_failed_query_is_not_cached(self):
        transform = SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=.2, y=-.1)))
        with mock.patch.object(localizers, '_sensor_offset', None), \
                mock.patch.dict(os.environ, {localizers.SENSOR_OFFSET_ENV: ''}), \
                mock.patch.object(tf_lookup, 'lookup_transform', side_effect=[RuntimeError('missing'), transform]):
            self.assertEqual(localizers.sensor_offset_xy(self.source, 'laser'), (0., 0.))
            self.assertIsNone(localizers._sensor_offset)
            self.assertEqual(localizers.sensor_offset_xy(self.source, 'laser'), (.2, -.1))


if __name__ == '__main__':
    unittest.main()
