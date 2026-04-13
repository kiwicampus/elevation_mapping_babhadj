/*
 * pointcloud_frame_relay_node.cpp
 *
 * Republishes a PointCloud2 message with an overridden frame_id. This is used
 * to move the depth cloud onto the *_3d TF subtree without touching the point
 * data itself.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointcloudFrameRelay : public rclcpp::Node
{
public:
  PointcloudFrameRelay()
  : Node("pointcloud_frame_relay")
  {
    declare_parameter("input_topic", "/camera/depth/color/points");
    declare_parameter("output_topic", "/camera/depth/color/points_3d");
    declare_parameter("output_frame_id", "camera_depth_optical_frame_3d");

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, sensor_qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, sensor_qos,
      std::bind(&PointcloudFrameRelay::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Relaying %s -> %s with frame_id=%s",
      input_topic.c_str(), output_topic.c_str(), output_frame_id_.c_str());
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto out = *msg;
    out.header.frame_id = output_frame_id_;
    pub_->publish(out);
  }

  std::string output_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudFrameRelay>());
  rclcpp::shutdown();
  return 0;
}
