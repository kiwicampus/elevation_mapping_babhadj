/*
 * inertial_link_broadcaster_node.cpp
 *
 * C++ port of launch/inertial_link_broadcaster.py.
 *
 * Broadcasts base_link -> inertial_link using roll/pitch from imu_filter_madgwick.
 * Optionally also broadcasts base_link_3d -> inertial_link_3d with the same
 * roll/pitch so that elevation_mapping can traverse the 3D alias subtree.
 *
 * Improvements over the Python version:
 *  - Eigen quaternion math instead of scipy (no Python interpreter overhead)
 *  - publish_rate parameter (default 50 Hz) throttles the IMU callback so we
 *    don't run heavy math at the full ~200 Hz Madgwick rate
 */

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <Eigen/Geometry>


class InertialLinkBroadcaster : public rclcpp::Node
{
public:
  InertialLinkBroadcaster()
  : Node("inertial_link_broadcaster")
  {
    declare_parameter("imu_topic", "/camera/imu_filtered");
    declare_parameter("base_frame", "base_link");
    declare_parameter("inertial_frame", "inertial_link");
    // Frame the camera is registered to in the robot's static TF tree.
    // On Kiwibot bags the chain is inertial_link -> camera_link -> ... -> camera_*_optical_frame.
    declare_parameter("tf_source_frame", "inertial_link");
    // Fallback when the IMU msg frame_id is absent from /tf_static.
    declare_parameter("tf_fallback_frame", "camera_color_optical_frame");
    // When true, broadcast base_link -> inertial_link as identity (roll=0, pitch=0).
    declare_parameter("orientation_2d", false);
    // When true, also broadcast base_frame_3d -> inertial_frame_3d with the same rotation.
    declare_parameter("also_publish_3d_alias", false);
    declare_parameter("base_frame_3d", "base_link_3d");
    declare_parameter("inertial_frame_3d", "inertial_link_3d");
    // Throttle: max TF broadcast rate regardless of IMU rate.
    declare_parameter("publish_rate", 50.0);

    imu_topic_          = get_parameter("imu_topic").as_string();
    base_frame_         = get_parameter("base_frame").as_string();
    inertial_frame_     = get_parameter("inertial_frame").as_string();
    tf_source_frame_    = get_parameter("tf_source_frame").as_string();
    tf_fallback_frame_  = get_parameter("tf_fallback_frame").as_string();
    orientation_2d_     = get_parameter("orientation_2d").as_bool();
    also_publish_3d_alias_ = get_parameter("also_publish_3d_alias").as_bool();
    base_frame_3d_      = get_parameter("base_frame_3d").as_string();
    inertial_frame_3d_  = get_parameter("inertial_frame_3d").as_string();

    double publish_rate  = get_parameter("publish_rate").as_double();
    publish_period_ns_   = static_cast<int64_t>(1e9 / publish_rate);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&InertialLinkBroadcaster::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Listening to %s -> broadcasting %s -> %s (throttled to %.0f Hz)",
      imu_topic_.c_str(), base_frame_.c_str(), inertial_frame_.c_str(), publish_rate);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Throttle to publish_rate — skip messages that arrive too soon.
    const int64_t now_ns = now().nanoseconds();
    if (now_ns - last_publish_ns_ < publish_period_ns_) {
      return;
    }

    const std::string & camera_frame = msg->header.frame_id;

    // Cache the static source->camera rotation (look up once, or when the IMU
    // frame_id changes — which never happens in practice).
    if (!q_source_to_camera_valid_ || camera_frame != camera_frame_) {
      geometry_msgs::msg::TransformStamped t;
      bool found = false;

      std::vector<std::string> frames = {camera_frame, tf_fallback_frame_};
      for (const auto & frame : frames) {
        try {
          t = tf_buffer_->lookupTransform(
            frame, tf_source_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));
          found = true;
          if (frame != camera_frame) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
              "TF for %s not available; using %s as proxy (same optical orientation, valid for roll/pitch)",
              camera_frame.c_str(), frame.c_str());
          }
          break;
        } catch (const tf2::TransformException &) {}
      }

      if (!found) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for TF %s->%s (also tried %s)",
          tf_source_frame_.c_str(), camera_frame.c_str(), tf_fallback_frame_.c_str());
        return;
      }

      const auto & r    = t.transform.rotation;
      q_source_to_camera_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z);
      q_source_to_camera_valid_ = true;
      camera_frame_ = camera_frame;
    }

    // q_camera_to_world from Madgwick (world-referenced orientation).
    const auto & o = msg->orientation;
    Eigen::Quaterniond q_camera_to_world(o.w, o.x, o.y, o.z);
    q_camera_to_world.normalize();

    // q_source_to_world = q_camera_to_world * q_source_to_camera
    // Eigen: (A * B) rotates by B first, then A  ->  source -> camera -> world. Correct.
    Eigen::Quaterniond q_source_to_world = (q_camera_to_world * q_source_to_camera_).normalized();

    // Decompose using extrinsic XYZ convention (same as scipy as_euler("xyz")):
    //   R = Rz(yaw) * Ry(pitch) * Rx(roll)
    //   roll  = atan2(R[2,1], R[2,2])
    //   pitch = asin(-R[2,0])
    const Eigen::Matrix3d R = q_source_to_world.toRotationMatrix();
    double roll  = std::atan2(R(2, 1), R(2, 2));
    double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));

    if (orientation_2d_) {
      roll  = 0.0;
      pitch = 0.0;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "RPY deg: roll=%.2f  pitch=%.2f%s",
      roll * 180.0 / M_PI, pitch * 180.0 / M_PI,
      orientation_2d_ ? "  [2D: zeroed]" : "");

    // Reconstruct quaternion from roll/pitch only (zero yaw) — extrinsic XYZ:
    //   q = Rz(0) * Ry(pitch) * Rx(roll) = Ry(pitch) * Rx(roll)
    // Eigen multiplication: right operand applied first.
    Eigen::Quaterniond q_no_yaw =
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
    q_no_yaw.normalize();
    // Normalize sign so w >= 0 — prevents the q / -q ambiguity that causes
    // a 360° visual flip in RViz when the quaternion convention flips.
    if (q_no_yaw.w() < 0.0) {
      q_no_yaw.coeffs() = -q_no_yaw.coeffs();
    }

    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp    = msg->header.stamp;
    ts.header.frame_id = base_frame_;
    ts.child_frame_id  = inertial_frame_;
    ts.transform.rotation.x = q_no_yaw.x();
    ts.transform.rotation.y = q_no_yaw.y();
    ts.transform.rotation.z = q_no_yaw.z();
    ts.transform.rotation.w = q_no_yaw.w();

    if (also_publish_3d_alias_) {
      geometry_msgs::msg::TransformStamped ts2;
      ts2.header.stamp    = msg->header.stamp;
      ts2.header.frame_id = base_frame_3d_;
      ts2.child_frame_id  = inertial_frame_3d_;
      ts2.transform.rotation = ts.transform.rotation;
      broadcaster_->sendTransform({ts, ts2});
    } else {
      broadcaster_->sendTransform(ts);
    }

    last_publish_ns_ = now_ns;
  }

  // Parameters
  std::string imu_topic_, base_frame_, inertial_frame_;
  std::string tf_source_frame_, tf_fallback_frame_;
  bool orientation_2d_{false};
  bool also_publish_3d_alias_{false};
  std::string base_frame_3d_, inertial_frame_3d_;
  int64_t publish_period_ns_{0};

  // TF infrastructure
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // Cached static source->camera rotation (set once after first successful lookup).
  Eigen::Quaterniond q_source_to_camera_{Eigen::Quaterniond::Identity()};
  bool               q_source_to_camera_valid_{false};
  std::string        camera_frame_;

  // Throttle state
  int64_t last_publish_ns_{0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InertialLinkBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
