# SPDX-License-Identifier: MulanPSL-2.0
"""Bounded TF queries without borrowing the caller's executor or subscriptions."""
import time


def lookup_transform(source_node, target_frame, source_frame, timeout_s):
    """Receive TF on a temporary node, then release every owned ROS resource."""
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.time import Time
    import tf2_ros

    node = Node(f"mapping_tf_query_{time.monotonic_ns()}",
                context=source_node.context)
    executor = SingleThreadedExecutor(context=source_node.context)
    listener = None
    try:
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)
        executor.add_node(node)
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                return buffer.lookup_transform(target_frame, source_frame, Time())
            except tf2_ros.TransformException:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise
                executor.spin_once(timeout_sec=min(.1, remaining))
    finally:
        # spin_once runs on this thread: no callback is active during teardown.
        executor.shutdown()
        if listener is not None:
            listener.unregister()
        node.destroy_node()
