/*
 * pointcloud_frame_relay_node.cpp
 *
 * Republishes one or more sensor messages with an overridden frame_id. Used to
 * move sensor streams onto the *_3d TF subtree without touching the data
 * payload. Currently supports PointCloud2 + Image (camera color frame).
 *
 * Each input pair is gated by its `*_input_topic` param: empty string disables
 * that relay. Pointcloud relay defaults are populated for backward compat;
 * image relay defaults to disabled.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


// Tiny helper that owns one (sub, pub, override frame) triplet for any sensor msg type.
template <typename MsgT>
class FrameRelay
{
public:
    FrameRelay(rclcpp::Node* node, const std::string& in_topic, const std::string& out_topic,
               const std::string& out_frame, const rclcpp::QoS& qos)
        : out_frame_(out_frame)
    {
        if (in_topic.empty() || out_topic.empty()) return;
        pub_ = node->create_publisher<MsgT>(out_topic, qos);
        sub_ = node->create_subscription<MsgT>(
            in_topic, qos, [this](typename MsgT::SharedPtr msg) {
                msg->header.frame_id = out_frame_;
                pub_->publish(*msg);
            });
        RCLCPP_INFO(node->get_logger(), "Relaying %s -> %s with frame_id=%s",
                    in_topic.c_str(), out_topic.c_str(), out_frame_.c_str());
    }

    bool active() const { return static_cast<bool>(sub_); }

private:
    std::string out_frame_;
    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
};


class PointcloudFrameRelay : public rclcpp::Node
{
public:
    PointcloudFrameRelay() : Node("pointcloud_frame_relay")
    {
        // Pointcloud relay (original feature, defaults preserved).
        declare_parameter("input_topic", "/camera/depth/color/points");
        declare_parameter("output_topic", "/camera/depth/color/points_3d");
        declare_parameter("output_frame_id", "camera_depth_optical_frame_3d");

        // Image relay (color stream onto the _3d subtree). Empty = disabled.
        declare_parameter("image_input_topic", std::string(""));
        declare_parameter("image_output_topic", std::string(""));
        declare_parameter("image_output_frame_id", "camera_color_optical_frame_3d");

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        pc_relay_ = std::make_unique<FrameRelay<sensor_msgs::msg::PointCloud2>>(
            this,
            get_parameter("input_topic").as_string(),
            get_parameter("output_topic").as_string(),
            get_parameter("output_frame_id").as_string(),
            qos);

        img_relay_ = std::make_unique<FrameRelay<sensor_msgs::msg::Image>>(
            this,
            get_parameter("image_input_topic").as_string(),
            get_parameter("image_output_topic").as_string(),
            get_parameter("image_output_frame_id").as_string(),
            qos);

        if (!pc_relay_->active() && !img_relay_->active())
        {
            RCLCPP_WARN(get_logger(),
                        "No relays active: both input_topic and image_input_topic are empty.");
        }
    }

private:
    std::unique_ptr<FrameRelay<sensor_msgs::msg::PointCloud2>> pc_relay_;
    std::unique_ptr<FrameRelay<sensor_msgs::msg::Image>> img_relay_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudFrameRelay>());
    rclcpp::shutdown();
    return 0;
}
