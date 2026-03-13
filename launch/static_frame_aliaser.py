#!/usr/bin/env python3
"""
Static Frame Aliaser.

Waits for a composed static transform between source_parent → source_child
to become available in /tf_static, then re-publishes it under a new pair of
frame names (target_parent → target_child) on /tf_static.

This is used to create a "3D alias" subtree for elevation_mapping without
hardcoding calibration values — the actual transform is read at runtime from
the existing static TF (from the rosbag or from the live realsense driver).

Usage in the elevation_mapping 3D pipeline:
  source_parent: inertial_link
  source_child:  camera_depth_optical_frame   (composed through the chain)
  target_parent: inertial_link_3d
  target_child:  camera_depth_optical_frame_3d

The node keeps running after publishing because StaticTransformBroadcaster
publishes on a TRANSIENT_LOCAL latched topic — the publisher object must
remain alive so late-joining subscribers (RViz, new nodes) receive the transform.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import StaticTransformBroadcaster


class StaticFrameAliaser(Node):
    def __init__(self):
        super().__init__("static_frame_aliaser")

        self.declare_parameter("source_parent", "inertial_link")
        self.declare_parameter("source_child", "camera_depth_optical_frame")
        self.declare_parameter("target_parent", "inertial_link_3d")
        self.declare_parameter("target_child", "camera_depth_optical_frame_3d")

        self._source_parent = self.get_parameter("source_parent").value
        self._source_child = self.get_parameter("source_child").value
        self._target_parent = self.get_parameter("target_parent").value
        self._target_child = self.get_parameter("target_child").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._published = False

        # Poll every 0.5 s until the source static transform is available.
        self._timer = self.create_timer(0.5, self._try_publish)

        self.get_logger().info(
            f"Waiting for static TF: {self._source_parent} -> {self._source_child} "
            f"to alias as {self._target_parent} -> {self._target_child}"
        )

    def _try_publish(self):
        if self._published:
            return

        try:
            t: TransformStamped = self._tf_buffer.lookup_transform(
                self._source_parent,
                self._source_child,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as ex:
            self.get_logger().info(
                f"Still waiting for {self._source_parent} -> {self._source_child}: {ex}",
                throttle_duration_sec=5.0,
            )
            return

        # Re-publish under alias frame names.
        alias = TransformStamped()
        alias.header.stamp = self.get_clock().now().to_msg()
        alias.header.frame_id = self._target_parent
        alias.child_frame_id = self._target_child
        alias.transform = t.transform  # Same calibration, new frame names

        self._static_broadcaster.sendTransform(alias)
        self._published = True

        self.get_logger().info(
            f"Published static alias: {self._target_parent} -> {self._target_child} "
            f"(copied from {self._source_parent} -> {self._source_child})"
        )
        # Cancel the polling timer — we are done publishing.
        # The node keeps alive to hold the StaticTransformBroadcaster publisher open.
        self._timer.cancel()


def main():
    rclpy.init()
    node = StaticFrameAliaser()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
