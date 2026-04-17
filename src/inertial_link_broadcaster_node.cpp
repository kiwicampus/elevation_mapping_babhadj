/*
 * inertial_link_broadcaster_node.cpp
 *
 * C++ port of launch/inertial_link_broadcaster.py.
 *
 * Broadcasts base_link -> inertial_link using roll/pitch from imu_filter_madgwick.
 * Optionally also broadcasts base_link_3d -> inertial_link_3d with the same
 * roll/pitch so that elevation_mapping can traverse the 3D alias subtree.
 *
 * The 2D and 3D broadcasts are controlled independently. This matters because
 * robot_localization transforms IMU orientation into base_link before fusing
 * it; if base_link -> inertial_link already contains the same dynamic tilt
 * from that IMU, the transform can partially cancel the very pitch/roll that
 * the elevation EKF needs for Z estimation. The 3D elevation-mapping pipeline
 * therefore keeps the original base_link subtree connected with a flat
 * base_link -> inertial_link transform, while publishing the real dynamic
 * roll/pitch only on the *_3d alias subtree.
 *
 * Improvements over the Python version:
 *  - Eigen quaternion math instead of scipy (no Python interpreter overhead)
 *  - IMU callback only caches the latest message (O(1), no math); a separate
 *    wall timer drives TF computation and broadcast at publish_rate Hz.
 *    This avoids waking up the heavy computation path at the full ~200 Hz
 *    Madgwick output rate.
 */

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class InertialLinkBroadcaster : public rclcpp::Node
{
   public:
    InertialLinkBroadcaster() : Node("inertial_link_broadcaster")
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
        // When false, do not publish base_frame -> inertial_frame.
        declare_parameter("publish_2d_transform", true);
        // When true, publish the 2D tree as flat/identity even if the 3D alias
        // subtree is receiving the real dynamic roll/pitch.
        declare_parameter("flat_2d_transform", false);
        // When true, also broadcast base_frame_3d -> inertial_frame_3d with the same rotation.
        declare_parameter("also_publish_3d_alias", false);
        declare_parameter("base_frame_3d", "base_link_3d");
        declare_parameter("inertial_frame_3d", "inertial_link_3d");
        // TF broadcast rate [Hz]. ekf_filter_elevation runs at 20 Hz and elevation
        // mapping updates at 5-8 Hz, so 25 Hz is well above what's needed.
        declare_parameter("publish_rate", 25.0);
        // Publish the computed [roll, pitch, yaw] vector for debugging. Off by default.
        declare_parameter("publish_rpy_vector", false);
        declare_parameter("rpy_vector_topic", "/debug/inertial_link_rpy");
        // Optional second IMU stream for comparison, e.g. /imu/data on the robot body.
        declare_parameter("compare_imu_enabled", false);
        declare_parameter("compare_imu_topic", "/imu/data");
        declare_parameter("compare_tf_source_frame", "base_link");
        declare_parameter("compare_tf_fallback_frame", "");
        declare_parameter("compare_rpy_vector_topic", "/debug/base_imu_rpy");

        imu_topic_ = get_parameter("imu_topic").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        inertial_frame_ = get_parameter("inertial_frame").as_string();
        tf_source_frame_ = get_parameter("tf_source_frame").as_string();
        tf_fallback_frame_ = get_parameter("tf_fallback_frame").as_string();
        orientation_2d_ = get_parameter("orientation_2d").as_bool();
        publish_2d_transform_ = get_parameter("publish_2d_transform").as_bool();
        flat_2d_transform_ = get_parameter("flat_2d_transform").as_bool();
        also_publish_3d_alias_ = get_parameter("also_publish_3d_alias").as_bool();
        base_frame_3d_ = get_parameter("base_frame_3d").as_string();
        inertial_frame_3d_ = get_parameter("inertial_frame_3d").as_string();
        publish_rpy_vector_ = get_parameter("publish_rpy_vector").as_bool();
        rpy_vector_topic_ = get_parameter("rpy_vector_topic").as_string();
        compare_imu_enabled_ = get_parameter("compare_imu_enabled").as_bool();
        compare_imu_topic_ = get_parameter("compare_imu_topic").as_string();
        compare_tf_source_frame_ = get_parameter("compare_tf_source_frame").as_string();
        compare_tf_fallback_frame_ = get_parameter("compare_tf_fallback_frame").as_string();
        compare_rpy_vector_topic_ = get_parameter("compare_rpy_vector_topic").as_string();

        const double publish_rate = get_parameter("publish_rate").as_double();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        if (publish_rpy_vector_)
        {
            rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(rpy_vector_topic_, 50);
        }
        if (compare_imu_enabled_)
        {
            compare_rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(compare_rpy_vector_topic_, 50);
        }

        // IMU callback: only cache the latest message — no math, no TF lookup.
        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10, std::bind(&InertialLinkBroadcaster::imuCallback, this, std::placeholders::_1));

        if (compare_imu_enabled_)
        {
            compare_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                compare_imu_topic_, 10,
                std::bind(&InertialLinkBroadcaster::compareImuCallback, this, std::placeholders::_1));
        }

        // Timer drives TF computation and broadcast at publish_rate Hz,
        // decoupled from the ~200 Hz IMU subscription.
        const auto period = std::chrono::duration<double>(1.0 / publish_rate);
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&InertialLinkBroadcaster::publishCallback, this));

        RCLCPP_INFO(get_logger(), "Listening to %s -> broadcasting %s -> %s at %.0f Hz",
                    imu_topic_.c_str(), base_frame_.c_str(), inertial_frame_.c_str(), publish_rate);
    }

   private:
    bool computeSourceRpy(const sensor_msgs::msg::Imu& msg, const std::string& tf_source_frame,
                          const std::string& tf_fallback_frame, Eigen::Quaterniond& q_source_to_imu,
                          bool& q_source_to_imu_valid, std::string& imu_frame_cache, double& roll, double& pitch,
                          double& yaw)
    {
        const std::string& imu_frame = msg.header.frame_id;

        if (!q_source_to_imu_valid || imu_frame != imu_frame_cache)
        {
            geometry_msgs::msg::TransformStamped t;
            bool found = false;

            std::vector<std::string> frames = {imu_frame};
            if (!tf_fallback_frame.empty() && tf_fallback_frame != imu_frame)
            {
                frames.push_back(tf_fallback_frame);
            }

            for (const auto& frame : frames)
            {
                try
                {
                    t = tf_buffer_->lookupTransform(frame, tf_source_frame, tf2::TimePointZero,
                                                    tf2::durationFromSec(0.1));
                    found = true;
                    if (frame != imu_frame)
                    {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                             "TF for %s not available; using %s as proxy (same optical orientation, "
                                             "valid for roll/pitch)",
                                             imu_frame.c_str(), frame.c_str());
                    }
                    break;
                } catch (const tf2::TransformException&)
                {
                }
            }

            if (!found)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for TF %s->%s (also tried %s)",
                                     tf_source_frame.c_str(), imu_frame.c_str(),
                                     tf_fallback_frame.empty() ? "<none>" : tf_fallback_frame.c_str());
                return false;
            }

            const auto& r = t.transform.rotation;
            q_source_to_imu = Eigen::Quaterniond(r.w, r.x, r.y, r.z);
            q_source_to_imu_valid = true;
            imu_frame_cache = imu_frame;
        }

        const auto& o = msg.orientation;
        Eigen::Quaterniond q_imu_to_world(o.w, o.x, o.y, o.z);
        q_imu_to_world.normalize();

        Eigen::Quaterniond q_source_to_world = (q_imu_to_world * q_source_to_imu).normalized();

        const Eigen::Matrix3d R = q_source_to_world.toRotationMatrix();
        roll = std::atan2(R(2, 1), R(2, 2));
        pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
        yaw = std::atan2(R(1, 0), R(0, 0));
        return true;
    }

    void publishRpyVector(const builtin_interfaces::msg::Time& stamp, const std::string& frame_id,
                          const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr& pub, double roll,
                          double pitch, double yaw)
    {
        if (!pub)
        {
            return;
        }

        geometry_msgs::msg::Vector3Stamped rpy_msg;
        rpy_msg.header.stamp = stamp;
        rpy_msg.header.frame_id = frame_id;
        rpy_msg.vector.x = roll;
        rpy_msg.vector.y = pitch;
        rpy_msg.vector.z = yaw;
        pub->publish(rpy_msg);
    }

    // IMU callback: only cache the latest message. O(1), no math, no TF lookup.
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        cached_imu_ = msg;
    }

    // Timer callback: runs at publish_rate Hz. Reads the cached IMU message,
    // computes roll/pitch, and broadcasts the TF transform(s).
    void publishCallback()
    {
        sensor_msgs::msg::Imu::SharedPtr imu_copy;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_copy = cached_imu_;
        }
        if (!imu_copy)
        {
            return;
        }

        double roll, pitch, yaw;
        if (!computeSourceRpy(*imu_copy, tf_source_frame_, tf_fallback_frame_, q_source_to_camera_,
                              q_source_to_camera_valid_, camera_frame_, roll, pitch, yaw))
        {
            return;
        }

        if (orientation_2d_)
        {
            roll = 0.0;
            pitch = 0.0;
        }

        if (publish_rpy_vector_)
        {
            publishRpyVector(imu_copy->header.stamp, tf_source_frame_, rpy_pub_, roll, pitch, yaw);
        }

        // Reconstruct quaternion from roll/pitch only (zero yaw) — extrinsic XYZ:
        //   q = Rz(0) * Ry(pitch) * Rx(roll) = Ry(pitch) * Rx(roll)
        // Eigen multiplication: right operand applied first.
        Eigen::Quaterniond q_no_yaw =
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        q_no_yaw.normalize();
        // Normalize sign so w >= 0 — prevents the q / -q ambiguity that causes
        // a 360° visual flip in RViz when the quaternion convention flips.
        if (q_no_yaw.w() < 0.0)
        {
            q_no_yaw.coeffs() = -q_no_yaw.coeffs();
        }

        Eigen::Quaterniond q_2d_tree = flat_2d_transform_ ? Eigen::Quaterniond::Identity() : q_no_yaw;

        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        if (publish_2d_transform_)
        {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp = imu_copy->header.stamp;
            ts.header.frame_id = base_frame_;
            ts.child_frame_id = inertial_frame_;
            ts.transform.rotation.x = q_2d_tree.x();
            ts.transform.rotation.y = q_2d_tree.y();
            ts.transform.rotation.z = q_2d_tree.z();
            ts.transform.rotation.w = q_2d_tree.w();
            transforms.push_back(ts);
        }

        if (also_publish_3d_alias_)
        {
            geometry_msgs::msg::TransformStamped ts2;
            ts2.header.stamp = imu_copy->header.stamp;
            ts2.header.frame_id = base_frame_3d_;
            ts2.child_frame_id = inertial_frame_3d_;
            ts2.transform.rotation.x = q_no_yaw.x();
            ts2.transform.rotation.y = q_no_yaw.y();
            ts2.transform.rotation.z = q_no_yaw.z();
            ts2.transform.rotation.w = q_no_yaw.w();
            transforms.push_back(ts2);
        }

        if (!transforms.empty())
        {
            broadcaster_->sendTransform(transforms);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Both publish_2d_transform and also_publish_3d_alias are false; no TF is being broadcast.");
        }
    }

    void compareImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double roll, pitch, yaw;
        if (!computeSourceRpy(*msg, compare_tf_source_frame_, compare_tf_fallback_frame_, q_compare_source_to_imu_,
                              q_compare_source_to_imu_valid_, compare_imu_frame_, roll, pitch, yaw))
        {
            return;
        }

        publishRpyVector(msg->header.stamp, compare_tf_source_frame_, compare_rpy_pub_, roll, pitch, yaw);
    }

    // Parameters
    std::string imu_topic_, base_frame_, inertial_frame_;
    std::string tf_source_frame_, tf_fallback_frame_;
    bool orientation_2d_{false};
    bool publish_2d_transform_{true};
    bool flat_2d_transform_{false};
    bool also_publish_3d_alias_{false};
    bool publish_rpy_vector_{false};
    std::string base_frame_3d_, inertial_frame_3d_;
    std::string rpy_vector_topic_;
    bool compare_imu_enabled_{false};
    std::string compare_imu_topic_, compare_tf_source_frame_, compare_tf_fallback_frame_, compare_rpy_vector_topic_;

    // TF infrastructure
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr compare_rpy_pub_;

    // Cached static source->camera rotation (set once after first successful TF lookup).
    Eigen::Quaterniond q_source_to_camera_{Eigen::Quaterniond::Identity()};
    bool q_source_to_camera_valid_{false};
    std::string camera_frame_;
    Eigen::Quaterniond q_compare_source_to_imu_{Eigen::Quaterniond::Identity()};
    bool q_compare_source_to_imu_valid_{false};
    std::string compare_imu_frame_;

    // Latest IMU message — written by imuCallback (200 Hz), read by publishCallback (25 Hz).
    std::mutex imu_mutex_;
    sensor_msgs::msg::Imu::SharedPtr cached_imu_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr compare_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InertialLinkBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
