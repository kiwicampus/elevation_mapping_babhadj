#!/usr/bin/env python3
"""
Hybrid Odometry Publisher for Elevation Mapping.

Runs on the live robot. It merges the PERFECT 2D (X, Y, Yaw) pose from the
robot's live Navigation TF tree with the accurate 3D (Z, Roll, Pitch)
pose from the dedicated 3D elevation EKF.

This gives elevation_mapping the real camera tilt and height without creating
parallel TF trees or causing X/Y map smearing against Nav2's costmaps.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
import numpy as np


class HybridOdomPublisher(Node):
    def __init__(self):
        super().__init__("hybrid_odom_publisher")
        
        # Topic where the 3D EKF publishes its non-TF estimate
        self.declare_parameter("odom_3d_topic", "/odometry/elevation")

        # Topic to output the merged hybrid pose
        self.declare_parameter("output_topic", "/odometry/elevation_hybrid")

        # TF frames for the live 2D pose lookup
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # Variance for 2D pose (x, y, yaw) in hybrid output — 2D comes from TF so use small values
        self.declare_parameter("covariance_2d_xy", 0.0001)
        self.declare_parameter("covariance_2d_yaw", 0.0001)

        # 3D alias TF: publish odom -> base_link_3d with correct Z on /tf so that
        # elevation_mapping (and RViz/Foxglove) can use the full 3D subtree.
        # Roll/pitch are intentionally NOT included here — they come from
        # inertial_link_broadcaster publishing base_link_3d -> inertial_link_3d.
        self.declare_parameter("publish_tf_3d", True)
        self.declare_parameter("base_frame_3d", "base_link_3d")

        odom_3d_topic = self.get_parameter("odom_3d_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._var_2d_xy = self.get_parameter("covariance_2d_xy").value
        self._var_2d_yaw = self.get_parameter("covariance_2d_yaw").value
        self._publish_tf_3d = self.get_parameter("publish_tf_3d").value
        self._base_frame_3d = self.get_parameter("base_frame_3d").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Cached Z from EKF — updated at EKF rate, consumed at timer rate
        self._z_3d = 0.0
        self._ekf_covariance = [0.0] * 36

        # Using Odometry as elevation_mapping can consume it directly
        self._pub = self.create_publisher(Odometry, output_topic, 100)
        self._sub = self.create_subscription(Odometry, odom_3d_topic, self._ekf_cb, 100)

        # High-rate timer drives the TF broadcast so base_link_3d tracks the 2D TF
        # at wheel-odometry rate rather than at the slower EKF rate.
        self.declare_parameter("tf_publish_hz", 50.0)
        tf_hz = self.get_parameter("tf_publish_hz").value
        self.create_timer(1.0 / tf_hz, self._tf_timer_cb)

        self.get_logger().info(
            f"Starting Hybrid Odom Publisher: Merging {self._map_frame}->{self._base_frame} (X,Y,Yaw) "
            f"with {odom_3d_topic} (Z) -> {output_topic}, TF at {tf_hz:.0f} Hz"
        )

    def _ekf_cb(self, msg_3d: Odometry):
        """Cache Z from the 3D EKF and publish the hybrid odometry topic."""
        self._z_3d = msg_3d.pose.pose.position.z
        self._ekf_covariance = list(msg_3d.pose.covariance)

        # Also publish the hybrid odometry topic (for diagnostics / consumers other than TF)
        try:
            t_2d: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return

        x_2d = t_2d.transform.translation.x
        y_2d = t_2d.transform.translation.y
        q_2d = t_2d.transform.rotation
        r_2d = Rotation.from_quat([q_2d.x, q_2d.y, q_2d.z, q_2d.w])
        _, _, yaw_2d = r_2d.as_euler("xyz", degrees=False)

        q_3d = msg_3d.pose.pose.orientation
        r_3d = Rotation.from_quat([q_3d.x, q_3d.y, q_3d.z, q_3d.w])
        roll_3d, pitch_3d, _ = r_3d.as_euler("xyz", degrees=False)

        hybrid_quat = Rotation.from_euler("xyz", [roll_3d, pitch_3d, yaw_2d], degrees=False).as_quat()

        out = Odometry()
        out.header.frame_id = self._map_frame
        out.header.stamp = msg_3d.header.stamp
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = x_2d
        out.pose.pose.position.y = y_2d
        out.pose.pose.position.z = self._z_3d
        out.pose.pose.orientation.x = hybrid_quat[0]
        out.pose.pose.orientation.y = hybrid_quat[1]
        out.pose.pose.orientation.z = hybrid_quat[2]
        out.pose.pose.orientation.w = hybrid_quat[3]
        cov = [0.0] * 36
        cov[0]  = self._var_2d_xy             # x-x
        cov[7]  = self._var_2d_xy             # y-y
        cov[14] = self._ekf_covariance[14]    # z-z  (from EKF)
        cov[21] = self._ekf_covariance[21]    # roll-roll  (from EKF)
        cov[28] = self._ekf_covariance[28]    # pitch-pitch  (from EKF)
        cov[35] = self._var_2d_yaw            # yaw-yaw
        out.pose.covariance = cov
        self._pub.publish(out)

    def _tf_timer_cb(self):
        """Broadcast odom -> base_link_3d at high rate using cached EKF Z."""
        if not self._publish_tf_3d:
            return
        try:
            t_2d: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except Exception as ex:
            self.get_logger().warn(
                f"Waiting for 2D TF ({self._map_frame} -> {self._base_frame}): {ex}",
                throttle_duration_sec=2.0,
            )
            return

        x_2d = t_2d.transform.translation.x
        y_2d = t_2d.transform.translation.y
        q_2d = t_2d.transform.rotation
        _, _, yaw_2d = Rotation.from_quat([q_2d.x, q_2d.y, q_2d.z, q_2d.w]).as_euler("xyz", degrees=False)

        yaw_only = Rotation.from_euler("z", yaw_2d).as_quat()
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self._map_frame
        ts.child_frame_id = self._base_frame_3d
        ts.transform.translation.x = x_2d
        ts.transform.translation.y = y_2d
        ts.transform.translation.z = self._z_3d
        ts.transform.rotation.x = yaw_only[0]
        ts.transform.rotation.y = yaw_only[1]
        ts.transform.rotation.z = yaw_only[2]
        ts.transform.rotation.w = yaw_only[3]
        self._tf_broadcaster.sendTransform(ts)


def main():
    rclpy.init()
    node = HybridOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
