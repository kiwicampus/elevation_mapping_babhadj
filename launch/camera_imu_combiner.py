#!/usr/bin/env python3
"""
Combines the D435i's separate gyroscope and accelerometer topics into a
single sensor_msgs/Imu message for imu_filter_madgwick.

The D435i publishes:
  /camera/gyro/sample   -> angular_velocity only  (~200 Hz, BEST_EFFORT)
  /camera/accel/sample  -> linear_acceleration only (~60 Hz, BEST_EFFORT)

Both are in the D435i hardware IMU convention where Y points DOWN.
The combiner time-synchronises them and merges into one Imu message using
the gyro frame and gyro timestamp. The accel vector is rotated into the gyro
frame via the static TF (camera_accel_optical_frame -> camera_gyro_optical_frame)
if the frames differ; on the D435i they are very close but not identical.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class CameraImuCombiner(Node):
    def __init__(self):
        super().__init__("camera_imu_combiner")
        self.declare_parameter("gyro_topic", "/camera/gyro/sample")
        self.declare_parameter("accel_topic", "/camera/accel/sample")
        self.declare_parameter("output_topic", "/camera/imu_combined")
        self.declare_parameter("queue_size", 50)
        self.declare_parameter("slop", 0.01)  # 10 ms — accel at ~60 Hz, gyro at ~200 Hz

        gyro_topic = self.get_parameter("gyro_topic").value
        accel_topic = self.get_parameter("accel_topic").value
        output_topic = self.get_parameter("output_topic").value
        queue_size = self.get_parameter("queue_size").value
        slop = self.get_parameter("slop").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        best_effort_qos = QoSProfile(
            depth=queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pub = self.create_publisher(Imu, output_topic, 100)
        gyro_sub = Subscriber(self, Imu, gyro_topic, qos_profile=best_effort_qos)
        accel_sub = Subscriber(self, Imu, accel_topic, qos_profile=best_effort_qos)

        self._sync = ApproximateTimeSynchronizer(
            [gyro_sub, accel_sub], queue_size=queue_size, slop=slop
        )
        self._sync.registerCallback(self._cb)
        self.get_logger().info(
            f"Combining {gyro_topic} + {accel_topic} -> {output_topic}"
        )

    def _cb(self, gyro: Imu, accel: Imu):
        linear_acceleration = accel.linear_acceleration
        accel_cov = accel.linear_acceleration_covariance

        if accel.header.frame_id != gyro.header.frame_id:
            try:
                v = Vector3Stamped()
                v.header = accel.header
                v.vector = accel.linear_acceleration
                v_rot = self._tf_buffer.transform(
                    v, gyro.header.frame_id, timeout=Duration(seconds=0.05)
                )
                linear_acceleration = v_rot.vector
            except Exception as ex:
                self.get_logger().warn(
                    f"accel->gyro TF failed, using raw: {ex}", throttle_duration_sec=5.0
                )

        out = Imu()
        out.header = gyro.header
        out.angular_velocity = gyro.angular_velocity
        out.angular_velocity_covariance = gyro.angular_velocity_covariance
        out.linear_acceleration = linear_acceleration
        out.linear_acceleration_covariance = accel_cov
        out.orientation_covariance[0] = -1.0  # no orientation yet; Madgwick fills it
        self._pub.publish(out)


def main():
    rclpy.init()
    node = CameraImuCombiner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
