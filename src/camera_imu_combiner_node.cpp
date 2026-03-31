/*
 * camera_imu_combiner_node.cpp
 *
 * C++ port of launch/camera_imu_combiner.py.
 *
 * Time-synchronises the D435i's separate gyroscope (~200 Hz) and accelerometer
 * (~60 Hz) topics and merges them into a single sensor_msgs/Imu message for
 * imu_filter_madgwick.
 *
 * Improvements over the Python version:
 *  - No Python interpreter / GIL overhead on the 200 Hz sync callback
 *  - Static accel->gyro rotation is cached after the first TF lookup; the TF
 *    listener is then destroyed so its background thread is freed
 *  - Eigen rotation instead of tf2_geometry_msgs::doTransform per message
 *  - message_filters C++ synchronizer has much lower overhead than Python's
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <Eigen/Geometry>

using ImuMsg    = sensor_msgs::msg::Imu;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, ImuMsg>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;


class CameraImuCombiner : public rclcpp::Node
{
public:
  CameraImuCombiner()
  : Node("camera_imu_combiner")
  {
    declare_parameter("gyro_topic",    "/camera/gyro/sample");
    declare_parameter("accel_topic",   "/camera/accel/sample");
    declare_parameter("output_topic",  "/camera/imu_combined");
    declare_parameter("queue_size",    50);
    declare_parameter("slop",          0.01);  // seconds — accel ~60 Hz, gyro ~200 Hz

    const auto gyro_topic   = get_parameter("gyro_topic").as_string();
    const auto accel_topic  = get_parameter("accel_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    const int  queue_size   = get_parameter("queue_size").as_int();
    const double slop       = get_parameter("slop").as_double();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    pub_ = create_publisher<ImuMsg>(output_topic, 100);

    // BEST_EFFORT QoS to match the D435i driver publisher.
    const auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size))
      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    gyro_sub_.subscribe(this, gyro_topic, best_effort_qos.get_rmw_qos_profile());
    accel_sub_.subscribe(this, accel_topic, best_effort_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(queue_size), gyro_sub_, accel_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));
    sync_->registerCallback(
      std::bind(&CameraImuCombiner::syncCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Combining %s + %s -> %s",
      gyro_topic.c_str(), accel_topic.c_str(), output_topic.c_str());
  }

private:
  void syncCallback(
    const ImuMsg::ConstSharedPtr & gyro,
    const ImuMsg::ConstSharedPtr & accel)
  {
    // ── One-time setup: detect if frames are the same or cache the rotation ──
    if (!ready_) {
      if (gyro->header.frame_id == accel->header.frame_id) {
        frames_same_ = true;
        ready_       = true;
        // TF listener not needed — free its background thread.
        tf_listener_.reset();
        tf_buffer_.reset();
        RCLCPP_INFO(get_logger(),
          "Gyro and accel share frame '%s'; no rotation needed.",
          gyro->header.frame_id.c_str());
      } else {
        // Look up static accel->gyro rotation and cache it.
        try {
          const auto t = tf_buffer_->lookupTransform(
            gyro->header.frame_id, accel->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));
          const auto & r = t.transform.rotation;
          q_accel_to_gyro_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z);
          ready_ = true;
          // Rotation is now cached — TF listener no longer needed.
          tf_listener_.reset();
          tf_buffer_.reset();
          RCLCPP_INFO(get_logger(),
            "Cached static rotation %s -> %s; TF listener freed.",
            accel->header.frame_id.c_str(), gyro->header.frame_id.c_str());
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Waiting for TF %s -> %s: %s",
            accel->header.frame_id.c_str(), gyro->header.frame_id.c_str(), ex.what());
          return;
        }
      }
    }

    // ── Build the combined Imu message ─────────────────────────────────────
    ImuMsg out;
    out.header                      = gyro->header;
    out.angular_velocity            = gyro->angular_velocity;
    out.angular_velocity_covariance = gyro->angular_velocity_covariance;
    out.linear_acceleration_covariance = accel->linear_acceleration_covariance;
    out.orientation_covariance[0]   = -1.0;  // no orientation; Madgwick fills it

    if (frames_same_) {
      out.linear_acceleration = accel->linear_acceleration;
    } else {
      // Rotate accel vector into gyro frame using cached Eigen quaternion.
      const Eigen::Vector3d a(
        accel->linear_acceleration.x,
        accel->linear_acceleration.y,
        accel->linear_acceleration.z);
      const Eigen::Vector3d a_rot = q_accel_to_gyro_ * a;
      out.linear_acceleration.x = a_rot.x();
      out.linear_acceleration.y = a_rot.y();
      out.linear_acceleration.z = a_rot.z();
    }

    pub_->publish(out);
  }

  // TF (freed after first successful sync)
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;

  // Sync infrastructure
  message_filters::Subscriber<ImuMsg> gyro_sub_;
  message_filters::Subscriber<ImuMsg> accel_sub_;
  std::shared_ptr<Synchronizer>       sync_;

  rclcpp::Publisher<ImuMsg>::SharedPtr pub_;

  // Cached accel->gyro rotation (set once, then TF listener is freed)
  Eigen::Quaterniond q_accel_to_gyro_{Eigen::Quaterniond::Identity()};
  bool frames_same_{false};
  bool ready_{false};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraImuCombiner>());
  rclcpp::shutdown();
  return 0;
}
