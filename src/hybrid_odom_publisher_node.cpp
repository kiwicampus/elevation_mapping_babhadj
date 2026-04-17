/*
 * hybrid_odom_publisher_node.cpp
 *
 * C++ port of launch/hybrid_odom_publisher.py.
 *
 * Merges the perfect 2D (X, Y, Yaw) pose from the robot's live Navigation TF
 * tree with the accurate 3D Z height from the dedicated elevation EKF.
 * Broadcasts odom -> base_link_3d on /tf at a configurable rate.
 * Roll/pitch are NOT included here — they come from inertial_link_broadcaster
 * publishing base_link_3d -> inertial_link_3d.
 *
 * Improvements over the Python version:
 *  - Eigen quaternion math instead of scipy
 *  - std::mutex protecting shared EKF state between EKF callback and timer
 *  - No Python interpreter / GIL overhead on the 50 Hz timer
 */

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <Eigen/Geometry>


class HybridOdomPublisher : public rclcpp::Node
{
public:
  HybridOdomPublisher()
  : Node("hybrid_odom_publisher")
  {
    declare_parameter("odom_3d_topic", "/odometry/elevation");
    declare_parameter("output_topic", "/odometry/elevation_hybrid");
    declare_parameter("map_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("covariance_2d_xy", 0.0001);
    declare_parameter("covariance_2d_yaw", 0.0001);
    declare_parameter("publish_tf_3d", true);
    declare_parameter("base_frame_3d", "base_link_3d");
    // ekf_filter_node_odom (the 2D TF source) runs at 30 Hz — no benefit running faster.
    declare_parameter("tf_publish_hz", 30.0);

    odom_3d_topic_  = get_parameter("odom_3d_topic").as_string();
    output_topic_   = get_parameter("output_topic").as_string();
    map_frame_      = get_parameter("map_frame").as_string();
    base_frame_     = get_parameter("base_frame").as_string();
    var_2d_xy_      = get_parameter("covariance_2d_xy").as_double();
    var_2d_yaw_     = get_parameter("covariance_2d_yaw").as_double();
    publish_tf_3d_  = get_parameter("publish_tf_3d").as_bool();
    base_frame_3d_  = get_parameter("base_frame_3d").as_string();
    const double tf_hz = get_parameter("tf_publish_hz").as_double();

    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 100);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_3d_topic_, 100,
      std::bind(&HybridOdomPublisher::ekfCallback, this, std::placeholders::_1));

    // Wall timer drives TF broadcast so base_link_3d tracks the 2D TF at wheel-
    // odometry rate rather than at the slower EKF rate.
    const auto period = std::chrono::duration<double>(1.0 / tf_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HybridOdomPublisher::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "Hybrid Odom Publisher: merging %s->%s (X,Y,Yaw) with %s (Z) -> %s, TF at %.0f Hz",
      map_frame_.c_str(), base_frame_.c_str(),
      odom_3d_topic_.c_str(), output_topic_.c_str(), tf_hz);
  }

private:
  // ── EKF callback ──────────────────────────────────────────────────────────
  // Runs at EKF rate. Caches Z + covariance, then publishes the merged
  // odometry topic (for diagnostics / consumers other than TF).
  void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(ekf_mutex_);
      z_3d_           = msg->pose.pose.position.z;
      ekf_covariance_ = msg->pose.covariance;
    }

    // Also publish the hybrid odometry topic.
    geometry_msgs::msg::TransformStamped t_2d;
    try {
      t_2d = tf_buffer_->lookupTransform(
        map_frame_, base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "EKF cb: cannot lookup %s->%s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    // Extract 2D pose: X, Y, yaw.
    const auto & tr = t_2d.transform.translation;
    const auto & q2 = t_2d.transform.rotation;
    const Eigen::Quaterniond q_2d(q2.w, q2.x, q2.y, q2.z);
    const Eigen::Matrix3d R_2d = q_2d.toRotationMatrix();
    const double yaw_2d = std::atan2(R_2d(1, 0), R_2d(0, 0));

    // Extract 3D roll/pitch from EKF orientation (extrinsic XYZ decomposition).
    const auto & q3 = msg->pose.pose.orientation;
    const Eigen::Quaterniond q_3d(q3.w, q3.x, q3.y, q3.z);
    const Eigen::Matrix3d R_3d = q_3d.toRotationMatrix();
    const double roll_3d  = std::atan2(R_3d(2, 1), R_3d(2, 2));
    const double pitch_3d = std::asin(std::max(-1.0, std::min(1.0, -R_3d(2, 0))));

    // Merge: extrinsic XYZ  ->  q = Rz(yaw) * Ry(pitch) * Rx(roll).
    Eigen::Quaterniond q_hybrid =
      Eigen::AngleAxisd(yaw_2d,   Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch_3d, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll_3d,  Eigen::Vector3d::UnitX());
    q_hybrid.normalize();

    nav_msgs::msg::Odometry out;
    out.header.frame_id = map_frame_;
    out.header.stamp    = msg->header.stamp;
    out.child_frame_id  = base_frame_;
    out.pose.pose.position.x    = tr.x;
    out.pose.pose.position.y    = tr.y;
    out.pose.pose.position.z    = z_3d_;
    out.pose.pose.orientation.x = q_hybrid.x();
    out.pose.pose.orientation.y = q_hybrid.y();
    out.pose.pose.orientation.z = q_hybrid.z();
    out.pose.pose.orientation.w = q_hybrid.w();
    out.pose.covariance[0]  = var_2d_xy_;           // x-x
    out.pose.covariance[7]  = var_2d_xy_;           // y-y
    out.pose.covariance[14] = ekf_covariance_[14];  // z-z      (from EKF)
    out.pose.covariance[21] = ekf_covariance_[21];  // roll-roll (from EKF)
    out.pose.covariance[28] = ekf_covariance_[28];  // pitch-pitch (from EKF)
    out.pose.covariance[35] = var_2d_yaw_;          // yaw-yaw
    pub_->publish(out);
  }

  // ── Timer callback ────────────────────────────────────────────────────────
  // Broadcasts odom -> base_link_3d at high rate using the cached EKF Z.
  void timerCallback()
  {
    if (!publish_tf_3d_) {
      return;
    }

    geometry_msgs::msg::TransformStamped t_2d;
    try {
      t_2d = tf_buffer_->lookupTransform(
        map_frame_, base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for 2D TF (%s->%s): %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    const auto & tr = t_2d.transform.translation;
    const auto & q2 = t_2d.transform.rotation;

    // Extract yaw only — fast direct formula, avoids building a rotation matrix.
    const double yaw_2d = std::atan2(
      2.0 * (q2.w * q2.z + q2.x * q2.y),
      1.0 - 2.0 * (q2.y * q2.y + q2.z * q2.z));
    const Eigen::Quaterniond q_yaw_only(Eigen::AngleAxisd(yaw_2d, Eigen::Vector3d::UnitZ()));

    double z_cached;
    {
      std::lock_guard<std::mutex> lock(ekf_mutex_);
      z_cached = z_3d_;
    }

    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp             = now();
    ts.header.frame_id          = map_frame_;
    ts.child_frame_id           = base_frame_3d_;
    ts.transform.translation.x  = tr.x;
    ts.transform.translation.y  = tr.y;
    ts.transform.translation.z  = z_cached;
    ts.transform.rotation.x     = q_yaw_only.x();
    ts.transform.rotation.y     = q_yaw_only.y();
    ts.transform.rotation.z     = q_yaw_only.z();
    ts.transform.rotation.w     = q_yaw_only.w();
    tf_broadcaster_->sendTransform(ts);
  }

  // Parameters
  std::string odom_3d_topic_, output_topic_, map_frame_, base_frame_, base_frame_3d_;
  double var_2d_xy_{0.0001}, var_2d_yaw_{0.0001};
  bool publish_tf_3d_{true};

  // TF infrastructure
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // EKF state shared between ekfCallback (EKF rate) and timerCallback (50 Hz).
  std::mutex                   ekf_mutex_;
  double                       z_3d_{0.0};
  std::array<double, 36>       ekf_covariance_{};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
