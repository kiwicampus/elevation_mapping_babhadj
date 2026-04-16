"""
Elevation Mapping launch for the REAL ROBOT (live data).

Replaces: elevationMapping_launch.py

Assumes the robot's main stack (Nav2, localization, realsense driver) is already
running and publishing /tf, /tf_static, /camera/depth/color/points, /camera/imu.
This launch adds only the 3D elevation estimation stack on top.

Usage:
  ros2 launch elevation_mapping elevation_mapping_robot.launch.py

  # If camera driver publishes separate gyro/accel topics:
  ros2 launch elevation_mapping elevation_mapping_robot.launch.py separate_camera_imu:=true

NOTE: Kill stale processes before relaunching:
  pkill -f "hybrid_odom_publisher|ekf_filter_elevation|inertial_link_broadcaster|pointcloud_frame_relay|static_frame_aliaser"
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    separate_camera_imu = LaunchConfiguration("separate_camera_imu")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_traversability = LaunchConfiguration("launch_traversability")
    use_combiner = PythonExpression(["'", separate_camera_imu, "' == 'true'"])

    loc_share = get_package_share_directory("location")
    navigation_share = get_package_share_directory("navigation")
    trav_share = get_package_share_directory("traversability_estimation")

    loc_params = os.path.join(loc_share, "config", "localization_params.yaml")
    cupy_core_params = os.path.join(
        navigation_share, "config", "elevation_mapping_kiwi_parameters.yaml"
    )
    cupy_sensor_params = os.path.join(
        navigation_share, "config", "elevation_mapping_kiwi_sensor_parameter.yaml"
    )
    cupy_plugin_params = os.path.join(
        navigation_share, "config", "elevation_mapping_kiwi_plugin_config.yaml"
    )
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
                "separate_camera_imu",
                default_value="false",
                description="Set to 'true' if the camera driver publishes separate "
                "/camera/gyro/sample and /camera/accel/sample topics",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use ROS simulated time from /clock for rosbag-based testing",
            ),
            DeclareLaunchArgument(
                "launch_traversability",
                default_value="true",
                description="Set to 'false' to skip traversability_estimation (useful for isolated testing)",
            ),

            # ── D435i IMU combiner (only when driver has separate topics) ──
            Node(
                package="elevation_mapping",
                executable="camera_imu_combiner",
                name="camera_imu_combiner",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_combiner),
            ),

            # ── Madgwick filter ──
            # Input topic switches based on separate_camera_imu:
            #   true  → /camera/imu_combined (after combiner node)
            #   false → /camera/imu          (driver publishes single topic)
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick_camera",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "world_frame": "enu",
                    "use_mag": False,
                    "publish_tf": False,
                    "gain": 0.03,
                    "zeta": 0.0,
                }],
                remappings=[
                    ("imu/data_raw", PythonExpression([
                        "'/camera/imu_combined' if '", separate_camera_imu, "' == 'true' else '/camera/imu'"
                    ])),
                    ("imu/data", "/camera/imu_filtered"),
                ],
            ),

            # ── inertial_link_broadcaster ──
            # Keeps base_link->inertial_link flat for the elevation EKF's IMU
            # transform path, while broadcasting the real dynamic roll/pitch only
            # on base_link_3d->inertial_link_3d for elevation mapping.
            Node(
                package="elevation_mapping",
                executable="inertial_link_broadcaster",
                name="inertial_link_broadcaster",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "orientation_2d": False,
                    "publish_2d_transform": False,
                    "flat_2d_transform": True,
                    "also_publish_3d_alias": True,
                    "compare_imu_enabled": False,
                    "compare_imu_topic": "/imu/data",
                    "compare_tf_source_frame": "base_link",
                    "compare_rpy_vector_topic": "/debug/base_imu_rpy",
                    "base_frame_3d": "base_link_3d",
                    "inertial_frame_3d": "inertial_link_3d",
                }],
            ),

            # ── 3D EKF ──
            # Estimates Z height from wheel odometry vz + camera IMU roll/pitch.
            # publish_tf=false — pose is consumed via hybrid_odom_publisher.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_elevation",
                output="screen",
                parameters=[loc_params, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "odometry/elevation")],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),

            # ── Hybrid odometry publisher ──
            # Merges 2D TF (X, Y, yaw) with EKF Z.
            # Broadcasts odom->base_link_3d on /tf at 50 Hz (timer-driven, not EKF-rate).
            # Roll/pitch come from inertial_link_broadcaster, not from this transform.
            Node(
                package="elevation_mapping",
                executable="hybrid_odom_publisher",
                name="hybrid_odom_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"publish_tf_3d": True},
                    {"base_frame_3d": "base_link_3d"},
                ],
            ),

            # ── Point cloud frame relay ──
            # Copies /camera/depth/color/points, changes header.frame_id to
            # camera_depth_optical_frame_3d, publishes as /camera/depth/color/points_3d.
            # This makes elevation_mapping's TF lookup traverse the 3D alias subtree.
            Node(
                package="elevation_mapping",
                executable="pointcloud_frame_relay",
                name="pointcloud_frame_relay",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": "/camera/depth/color/points"},
                    {"output_topic": "/camera/depth/color/points_3d"},
                    {"output_frame_id": "camera_depth_optical_frame_3d"},
                ],
            ),

            # ── Static frame aliaser ──
            # Reads the composed inertial_link->camera_depth_optical_frame from /tf_static
            # (published by the realsense driver at startup) and republishes it as
            # inertial_link_3d->camera_depth_optical_frame_3d. Polls every 0.5 s internally
            # until the realsense driver has posted the static frames.
            Node(
                package="elevation_mapping",
                executable="static_frame_aliaser",
                name="static_frame_aliaser",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"source_parent": "inertial_link"},
                    {"source_child": "camera_depth_optical_frame"},
                    {"target_parent": "inertial_link_3d"},
                    {"target_child": "camera_depth_optical_frame_3d"},
                ],
            ),

            # ── Cupy elevation mapping ──
            # Delayed 2 s to let TF tree settle (EKF, hybrid publisher, static aliaser).
            # Uses base_link_3d as robot frame (correct Z + Madgwick roll/pitch via
            # inertial_link_3d). A lightweight elevation-only publisher is exposed
            # for traversability init and /get_raw_submap remains at the expected name.
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="elevation_mapping_cupy",
                        executable="elevation_mapping_node",
                        name="elevation_mapping_node",
                        output="screen",
                        parameters=[
                            cupy_core_params,
                            cupy_sensor_params,
                            {"plugin_config_file": cupy_plugin_params},
                            {"use_sim_time": use_sim_time},
                        ],
                        remappings=[
                            ("get_raw_submap", "/get_raw_submap"),
                            (
                                "elevation_mapping_node/elevation_map_raw",
                                "/elevation_map_raw",
                            ),
                        ],
                    ),
                ],
            ),

            # ── Traversability estimation ──
            TimerAction(
                period=20.0,
                actions=[
                    Node(
                        package="traversability_estimation",
                        executable="traversability_estimation_node",
                        name="traversability_estimation",
                        output="screen",
                        parameters=trav_configs + [{"use_sim_time": use_sim_time}],
                        condition=IfCondition(launch_traversability),
                    ),
                ],
            ),
        ]
    )
