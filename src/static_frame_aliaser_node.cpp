/*
 * static_frame_aliaser_node.cpp
 *
 * C++ port of launch/static_frame_aliaser.py.
 *
 * Waits for a composed static transform (source_parent -> source_child) to
 * appear on /tf_static, then re-publishes it under new frame names
 * (target_parent -> target_child).
 *
 * Used in the elevation_mapping 3D pipeline to create the alias:
 *   inertial_link -> camera_depth_optical_frame
 *     republished as
 *   inertial_link_3d -> camera_depth_optical_frame_3d
 *
 * Improvements over the Python version:
 *  - After publishing, the TF buffer and listener are destroyed, freeing the
 *    background thread that was consuming ~37% of one CPU core.  The
 *    StaticTransformBroadcaster remains alive so late-joining subscribers
 *    (RViz, new nodes) receive the TRANSIENT_LOCAL latched message.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/time.h>


class StaticFrameAliaser : public rclcpp::Node
{
public:
  StaticFrameAliaser()
  : Node("static_frame_aliaser")
  {
    declare_parameter("source_parent", "inertial_link");
    declare_parameter("source_child", "camera_depth_optical_frame");
    declare_parameter("target_parent", "inertial_link_3d");
    declare_parameter("target_child", "camera_depth_optical_frame_3d");

    source_parent_ = get_parameter("source_parent").as_string();
    source_child_  = get_parameter("source_child").as_string();
    target_parent_ = get_parameter("target_parent").as_string();
    target_child_  = get_parameter("target_child").as_string();

    tf_buffer_          = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_        = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Poll every 0.5 s until the source static transform is available.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&StaticFrameAliaser::tryPublish, this));

    RCLCPP_INFO(get_logger(),
      "Waiting for static TF: %s -> %s to alias as %s -> %s",
      source_parent_.c_str(), source_child_.c_str(),
      target_parent_.c_str(), target_child_.c_str());
  }

private:
  void tryPublish()
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(
        source_parent_, source_child_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Still waiting for %s -> %s: %s",
        source_parent_.c_str(), source_child_.c_str(), ex.what());
      return;
    }

    // Re-publish the same calibration under the alias frame names.
    geometry_msgs::msg::TransformStamped alias;
    alias.header.stamp    = now();
    alias.header.frame_id = target_parent_;
    alias.child_frame_id  = target_child_;
    alias.transform       = t.transform;
    static_broadcaster_->sendTransform(alias);

    RCLCPP_INFO(get_logger(),
      "Published static alias: %s -> %s (copied from %s -> %s)",
      target_parent_.c_str(), target_child_.c_str(),
      source_parent_.c_str(), source_child_.c_str());

    // Cancel the polling timer — no more lookups needed.
    timer_->cancel();

    // Destroy TF listener and buffer to free the background subscriber thread.
    // static_broadcaster_ stays alive so TRANSIENT_LOCAL late-joiners receive the transform.
    tf_listener_.reset();
    tf_buffer_.reset();
  }

  std::string source_parent_, source_child_, target_parent_, target_child_;

  std::shared_ptr<tf2_ros::Buffer>                    tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr                         timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFrameAliaser>());
  rclcpp::shutdown();
  return 0;
}
