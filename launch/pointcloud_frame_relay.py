#!/usr/bin/env python3
"""
Point Cloud Frame Relay.

Subscribes to a PointCloud2 topic, replaces the header.frame_id, and
republishes on a new topic. This is used to feed elevation_mapping a copy
of the depth point cloud that is associated with the 3D alias frame
(camera_depth_optical_frame_3d) so that elevation_mapping's TF lookup
traverses the 3D subtree (base_link_3d → inertial_link_3d → ...) instead
of the 2D one (base_link → inertial_link → ...).

The point cloud DATA is identical — only the frame_id header field changes.

Parameters:
  input_topic    Source PointCloud2 topic  (default: /camera/depth/color/points)
  output_topic   Output PointCloud2 topic  (default: /camera/depth/color/points_3d)
  output_frame_id  New frame_id to stamp   (default: camera_depth_optical_frame_3d)
  use_sim_time   Use simulation clock      (default: false)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2


class PointcloudFrameRelay(Node):
    def __init__(self):
        super().__init__("pointcloud_frame_relay")

        self.declare_parameter("input_topic", "/camera/depth/color/points")
        self.declare_parameter("output_topic", "/camera/depth/color/points_3d")
        self.declare_parameter("output_frame_id", "camera_depth_optical_frame_3d")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._output_frame_id = self.get_parameter("output_frame_id").value

        # Match the typical camera driver QoS: best-effort, keep last 1.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self._cb, sensor_qos
        )

        self.get_logger().info(
            f"Relaying {input_topic} -> {output_topic} "
            f"with frame_id={self._output_frame_id}"
        )

    def _cb(self, msg: PointCloud2):
        # Shallow-copy the message and override only the frame_id.
        # PointCloud2 is a flat struct — fields, data, row_step etc. are
        # shared by reference which is safe since we publish immediately.
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self._output_frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self._pub.publish(out)


def main():
    rclpy.init()
    node = PointcloudFrameRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
