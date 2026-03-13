"""
Test launch for Elevation Mapping with selectable TF mode (2D or 3D).

Usage:
  # 3D mode, bag with combined /camera/imu (default):
  ros2 launch elevation_mapping test_elevation_3d.launch.py \
      bag_path:=/workspace/localization_offline

  # 3D mode, bag with separate gyro/accel topics (localization_off_imu_filter_mor_separated):
  ros2 launch elevation_mapping test_elevation_3d.launch.py \
      bag_path:=/workspace/localization_off_imu_filter_mor_separated \
      separate_camera_imu:=true

  # 2D mode:
  ros2 launch elevation_mapping test_elevation_3d.launch.py \
      bag_path:=/workspace/localization_offline \
      mode:=2d

  # Slow playback:
  ros2 launch elevation_mapping test_elevation_3d.launch.py \
      bag_path:=/workspace/localization_offline mode:=3d rate:=0.5
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    odom_remap = LaunchConfiguration("odom_remap")
    play_rate = LaunchConfiguration("rate")
    mode = LaunchConfiguration("mode")
    separate_camera_imu = LaunchConfiguration("separate_camera_imu")

    is_3d = PythonExpression(["'", mode, "' == '3d'"])
    use_combiner = PythonExpression(["'", separate_camera_imu, "' == 'true'"])

    loc_share = get_package_share_directory("location")
    elev_share = get_package_share_directory("elevation_mapping")
    trav_share = get_package_share_directory("traversability_estimation")

    loc_params = os.path.join(loc_share, "config", "localization_params.yaml")
    elev_configs = [
        os.path.join(elev_share, "config", f)
        for f in [
            "robots/kiwi.yaml",
            "elevation_maps/kiwi_map.yaml",
            "postprocessing/postprocessor_pipeline.yaml",
        ]
    ]
    trav_configs = [
        os.path.join(trav_share, "config", f)
        for f in [
            "robot.yaml",
            "robot_footprint_parameter.yaml",
            "robot_filter_parameter.yaml",
        ]
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="/workspace/localization_offline",
                description="Path to rosbag directory",
            ),
            DeclareLaunchArgument(
                "odom_remap",
                default_value="/wheel_odometry/global_odometry",
                description="Source topic to remap as wheel odometry input (only used in 3d mode)",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="1.0",
                description="Bag playback rate",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="3d",
                description="TF mode: '3d' (dedicated 3D EKF on /tf_elev) or '2d' (bag's original /tf)",
            ),
            DeclareLaunchArgument(
                "separate_camera_imu",
                default_value="false",
                description="Set to 'true' if the bag has separate /camera/gyro/sample and "
                "/camera/accel/sample topics instead of a combined /camera/imu",
            ),
            # ── Rosbag playback (Offline Testing) ──
            # Plays back the bag exactly as the real robot recorded it (with the live 2D odom->base_link).
            # We don't need tf_odom_filter anymore because the 3D EKF is silent and uses hybrid_odom_publisher.
            GroupAction(
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "bag",
                            "play",
                            bag_path,
                            "--clock",
                            "--rate",
                            play_rate,
                            "--qos-profile-overrides-path",
                            "/workspace/rover/ros2/src/navigation/config/tf_static_override.yaml",
                            "--remap",
                            "/tf:=/tf_bag",
                            "/tf_static:=/tf_static_bag",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="elevation_mapping",
                        executable="tf_odom_filter.py",
                        name="tf_odom_filter",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                    ),
                ],
            ),
            # ── Hybrid Odometry Publisher (3D mode) ──
            # Merges 2D TF (X, Y, yaw) with 3D EKF (Z) and publishes:
            #   - /odometry/elevation_hybrid  (Odometry topic, for diagnostics)
            #   - odom -> base_link_3d on /tf (yaw-only rotation + correct Z)
            # Roll/pitch are NOT in odom->base_link_3d; they come from
            # inertial_link_broadcaster via base_link_3d -> inertial_link_3d.
            Node(
                package="elevation_mapping",
                executable="hybrid_odom_publisher.py",
                name="hybrid_odom_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"publish_tf_3d": True},
                    {"base_frame_3d": "base_link_3d"},
                ],
                condition=IfCondition(is_3d),
            ),
            # ── (Rosbag playback disabled for Live execution) ──
            # ── D435i gyro+accel combiner (only when bag has separate topics) ──
            # Merges /camera/gyro/sample + /camera/accel/sample -> /camera/imu_combined.
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(os.path.dirname(__file__), "camera_imu_combiner.py"),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=false",
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            mode,
                            "' == '3d' and '",
                            separate_camera_imu,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            # ── imu_filter_madgwick for D435i — separate topics path ──
            # Input: /camera/imu_combined (from combiner above, gyro frame, Y-axis down).
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick_camera",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_frame": "enu",
                        "use_mag": False,
                        "publish_tf": False,
                        "gain": 0.1,
                        "zeta": 0.0,
                    }
                ],
                remappings=[
                    ("imu/data_raw", "/camera/imu_combined"),
                    ("imu/data", "/camera/imu_filtered"),
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            mode,
                            "' == '3d' and '",
                            separate_camera_imu,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            # ── imu_filter_madgwick for D435i — combined topic path ──
            # Input: /camera/imu (already merged by the camera driver, Y-axis down).
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick_camera",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_frame": "enu",
                        "use_mag": False,
                        "publish_tf": False,
                        "gain": 0.1,
                        "zeta": 0.0,
                    }
                ],
                remappings=[
                    ("imu/data_raw", "/camera/imu"),
                    ("imu/data", "/camera/imu_filtered"),
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            mode,
                            "' == '3d' and '",
                            separate_camera_imu,
                            "' != 'true'",
                        ]
                    )
                ),
            ),
            # ── inertial_link_broadcaster (3D mode) ──
            # Broadcasts base_link -> inertial_link with real roll/pitch from Madgwick.
            # Also broadcasts base_link_3d -> inertial_link_3d with the same rotation,
            # so that elevation_mapping's TF lookup through base_link_3d gets correct
            # camera orientation at Madgwick IMU rate (not limited by EKF rate).
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(
                        os.path.dirname(__file__), "inertial_link_broadcaster.py"
                    ),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                    "-p",
                    "orientation_2d:=false",
                    "-p",
                    "also_publish_3d_alias:=true",
                    "-p",
                    "base_frame_3d:=base_link_3d",
                    "-p",
                    "inertial_frame_3d:=inertial_link_3d",
                ],
                output="screen",
                condition=IfCondition(is_3d),
            ),
            # ── 3D EKF (only in 3d mode) ──
            # Publishes odom->base_link on /tf with full 3D Z estimation.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_elevation",
                output="screen",
                parameters=[
                    loc_params,
                    {"use_sim_time": True},
                ],
                remappings=[
                    ("odometry/filtered", "odometry/elevation"),
                ],
                arguments=["--ros-args", "--log-level", "INFO"],
                condition=IfCondition(is_3d),
            ),
            # ── Point cloud frame relay (3D mode) ──
            # Re-stamps /camera/depth/color/points with frame_id=camera_depth_optical_frame_3d
            # so that elevation_mapping's TF lookup traverses the 3D alias subtree
            # (base_link_3d -> inertial_link_3d -> camera_depth_optical_frame_3d).
            Node(
                package="elevation_mapping",
                executable="pointcloud_frame_relay.py",
                name="pointcloud_frame_relay",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"input_topic": "/camera/depth/color/points"},
                    {"output_topic": "/camera/depth/color/points_3d"},
                    {"output_frame_id": "camera_depth_optical_frame_3d"},
                ],
                condition=IfCondition(is_3d),
            ),
            # ── Elevation mapping ──
            # Delayed 2 s to let the EKF and TF tree settle first.
            # In 3D mode elevation_mapping uses base_link_3d (correct Z + Madgwick
            # roll/pitch via inertial_link_3d) and the relayed point cloud topic.
            # The static_frame_aliaser is delayed 3 s to ensure /tf_static is
            # populated before attempting the composed lookup.
            TimerAction(
                period=2.0,
                actions=[
                    GroupAction(
                        condition=IfCondition(is_3d),
                        actions=[
                            Node(
                                package="elevation_mapping",
                                executable="elevation_mapping",
                                name="elevation_mapping",
                                output="screen",
                                parameters=elev_configs
                                + [
                                    {"use_sim_time": True},
                                    {"map_frame_id": "odom"},
                                    {"robot_base_frame_id": "base_link_3d"},
                                    {"track_point_frame_id": "base_link_3d"},
                                    {"front_cam.topic": "/camera/depth/color/points_3d"},
                                    {"robot_motion_map_update.covariance_scale": 0.0},
                                ],
                            ),
                        ],
                    ),
                    GroupAction(
                        condition=UnlessCondition(is_3d),
                        actions=[
                            Node(
                                package="elevation_mapping",
                                executable="elevation_mapping",
                                name="elevation_mapping",
                                output="screen",
                                parameters=elev_configs
                                + [
                                    {"use_sim_time": True},
                                    {"map_frame_id": "odom"},
                                    {
                                        "robot_pose_with_covariance_topic": "/odometry/local"
                                    },
                                    {"robot_motion_map_update.covariance_scale": 0.0},
                                ],
                            ),
                        ],
                    ),
                ],
            ),
            # ── Static frame aliaser (3D mode) ──
            # Delayed 3 s to ensure /tf_static is fully populated from the bag.
            # Reads the composed inertial_link -> camera_depth_optical_frame transform
            # and republishes it as inertial_link_3d -> camera_depth_optical_frame_3d.
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="elevation_mapping",
                        executable="static_frame_aliaser.py",
                        name="static_frame_aliaser",
                        output="screen",
                        parameters=[
                            {"use_sim_time": True},
                            {"source_parent": "inertial_link"},
                            {"source_child": "camera_depth_optical_frame"},
                            {"target_parent": "inertial_link_3d"},
                            {"target_child": "camera_depth_optical_frame_3d"},
                        ],
                        condition=IfCondition(is_3d),
                    ),
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    GroupAction(
                        actions=[
                            Node(
                                package="traversability_estimation",
                                executable="traversability_estimation_node",
                                name="traversability_estimation",
                                output="screen",
                                parameters=trav_configs + [
                                    {"use_sim_time": True},
                                ],
                            ),
                        ],
                    ),
                ],
            ),
        ]
    )
