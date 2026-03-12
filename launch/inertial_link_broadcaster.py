#!/usr/bin/env python3
"""
Broadcasts base_link -> inertial_link using roll/pitch from imu_filter_madgwick.

average_imu's GetTransformedMessage applies tf2::doTransform on a world-referenced
orientation quaternion as if it were a sensor-frame rotation, which is wrong:
    doTransform does:  q_out = q_camera_to_imu * q_camera_to_world   (WRONG)
    we need:           q_base_to_world = q_camera_to_world * q_base_to_camera

This node does that correctly:
  1. Receive Madgwick orientation q_camera_to_world (frame_id = camera gyro frame)
  2. Look up static TF: tf_source_frame -> camera frame  (q_source_to_camera)
     tf_source_frame defaults to inertial_link because the bag's static TF has
     inertial_link -> camera_link (robot_base_frame=inertial_link on recording).
  3. Compute q_source_to_world = q_camera_to_world * q_source_to_camera
  4. Extract roll, pitch — zero yaw (inertial_link is gravity-aligned, no yaw)
  5. Broadcast base_link -> inertial_link at the IMU message rate
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
import numpy as np


class InertialLinkBroadcaster(Node):
    def __init__(self):
        super().__init__("inertial_link_broadcaster")
        self.declare_parameter("imu_topic", "/camera/imu_filtered")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("inertial_frame", "inertial_link")
        # The frame the camera is registered to in the robot's static TF tree.
        # On Kiwibot bags the realsense driver publishes:
        #   inertial_link -> camera_link -> ... -> camera_color_optical_frame
        # (robot_base_frame=inertial_link in bot.vision.launch.py).
        # So we look up camera_frame -> tf_source_frame (NOT base_link, which is
        # disconnected from the static TF tree). This is correct because
        # q_inertial_to_world = q_camera_to_world * q_inertial_to_camera, and
        # the roll/pitch of inertial_link in world equals that of base_link.
        self.declare_parameter("tf_source_frame", "inertial_link")
        # Fallback when the IMU message's frame_id is absent from /tf_static
        # (e.g. camera_gyro_optical_frame missing in some bags). The color and
        # gyro optical frames share the same orientation on the D435i.
        self.declare_parameter("tf_fallback_frame", "camera_color_optical_frame")
        # When True, broadcast base_link -> inertial_link as identity (roll=0, pitch=0).
        # Mirrors average_imu's orientation_2d:true behaviour: inertial_link becomes
        # a copy of base_link so the camera mount transform is fully static.
        # Useful for debugging: if Z estimation improves with this flag, the issue
        # is in the dynamic TF lookup/extrapolation, not in the orientation math.
        self.declare_parameter("orientation_2d", False)

        imu_topic = self.get_parameter("imu_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._inertial_frame = self.get_parameter("inertial_frame").value
        self._tf_source_frame = self.get_parameter("tf_source_frame").value
        self._tf_fallback_frame = self.get_parameter("tf_fallback_frame").value
        self._orientation_2d = self.get_parameter("orientation_2d").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        self._q_base_to_camera = None
        self._camera_frame = None

        self.create_subscription(Imu, imu_topic, self._cb, 100)
        self.create_timer(5.0, self._log_frames)
        self.get_logger().info(
            f"Listening to {imu_topic} -> broadcasting {self._base_frame} -> {self._inertial_frame}"
        )

    def _log_frames(self):
        frames = self._tf_buffer.all_frames_as_string()
        self.get_logger().info(f"Known TF frames:\n{frames}")

    def _cb(self, msg: Imu):
        camera_frame = msg.header.frame_id

        # Cache the static base->camera rotation (only look it up once).
        # Try the message's own frame_id first; if it's not in the TF tree
        # (e.g. camera_gyro_optical_frame missing from /tf_static in some bags),
        # fall back to tf_fallback_frame which shares the same orientation.
        if self._q_base_to_camera is None or camera_frame != self._camera_frame:
            lookup_frame = camera_frame
            t = None
            for frame in [camera_frame, self._tf_fallback_frame]:
                try:
                    t = self._tf_buffer.lookup_transform(
                        frame,
                        self._tf_source_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.1),
                    )
                    lookup_frame = frame
                    break
                except Exception:
                    pass

            if t is None:
                self.get_logger().warn(
                    f"Waiting for TF {self._tf_source_frame}->{camera_frame} "
                    f"(also tried {self._tf_fallback_frame})",
                    throttle_duration_sec=2.0,
                )
                return

            if lookup_frame != camera_frame:
                self.get_logger().info(
                    f"TF for {camera_frame} not available; using {lookup_frame} as proxy "
                    f"(same optical orientation, valid for roll/pitch)",
                    throttle_duration_sec=10.0,
                )

            r = t.transform.rotation
            self._q_base_to_camera = np.array([r.x, r.y, r.z, r.w])
            self._camera_frame = camera_frame

        # q_camera_to_world from Madgwick (xyzw)
        o = msg.orientation
        q_camera_to_world = np.array([o.x, o.y, o.z, o.w])

        # q_source_to_world = q_camera_to_world * q_source_to_camera
        # where source = tf_source_frame (inertial_link on Kiwibot bags).
        # scipy composes left-to-right: (A * B).apply(v) rotates v by B first, then A.
        # We want: vector in source frame -> camera frame -> world frame.
        r_source_to_world = Rotation.from_quat(q_camera_to_world) * Rotation.from_quat(
            self._q_base_to_camera
        )

        roll, pitch, _ = r_source_to_world.as_euler("xyz", degrees=False)

        if self._orientation_2d:
            roll, pitch = 0.0, 0.0

        self.get_logger().info(
            f"RPY deg: roll={np.degrees(roll):.2f}  pitch={np.degrees(pitch):.2f}"
            f"{'  [2D: zeroed]' if self._orientation_2d else ''}",
            throttle_duration_sec=1.0,
        )

        # Broadcast base_link -> inertial_link: gravity-aligned, no yaw
        q_no_yaw = Rotation.from_euler("xyz", [roll, pitch, 0.0]).as_quat()

        ts = TransformStamped()
        ts.header.stamp = msg.header.stamp
        ts.header.frame_id = self._base_frame
        ts.child_frame_id = self._inertial_frame
        ts.transform.rotation.x = q_no_yaw[0]
        ts.transform.rotation.y = q_no_yaw[1]
        ts.transform.rotation.z = q_no_yaw[2]
        ts.transform.rotation.w = q_no_yaw[3]
        self._broadcaster.sendTransform(ts)


def main():
    rclpy.init()
    node = InertialLinkBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
