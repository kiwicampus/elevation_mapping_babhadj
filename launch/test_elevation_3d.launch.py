"""
Test launch for Elevation Mapping with selectable TF mode (2D or 3D).

Usage:
  # 3D mode (default): dedicated 3D EKF, publishes on /tf_elev
  ros2 launch elevation_mapping test_elevation_3d.launch.py \
      bag_path:=/workspace/localization_offline \
      odom_remap:=/wheel_odometry/global_odometry

  # 2D mode: uses the bag's original /tf (odom -> base_link, 2D)
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

    is_3d = PythonExpression(["'", mode, "' == '3d'"])

    loc_share = get_package_share_directory("location")
    elev_share = get_package_share_directory("elevation_mapping")

    loc_params = os.path.join(loc_share, "config", "localization_params.yaml")
    elev_configs = [
        os.path.join(elev_share, "config", f)
        for f in [
            "robots/kiwi.yaml",
            "elevation_maps/kiwi_map.yaml",
            "postprocessing/postprocessor_pipeline.yaml",
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

            # ── Static transform: base_link -> inertial_link (both modes need it) ──
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_inertial",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "inertial_link"],
                parameters=[{"use_sim_time": True}],
            ),

            # ── Rosbag playback (3D mode: remap odom topic) ──
            GroupAction(
                condition=IfCondition(is_3d),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "bag", "play", bag_path,
                            "--clock", "--rate", play_rate,
                            "--remap",
                            [odom_remap, ":=/wheel_odometry/global_odometry"],
                        ],
                        output="screen",
                    ),
                ],
            ),

            # ── Rosbag playback (2D mode: no remap needed) ──
            GroupAction(
                condition=UnlessCondition(is_3d),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "bag", "play", bag_path,
                            "--clock", "--rate", play_rate,
                        ],
                        output="screen",
                    ),
                ],
            ),

            # ── 3D EKF (only in 3d mode) ──
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
                    ("/tf", "/tf_elev"),
                    ("odometry/filtered", "odometry/elevation"),
                    ("set_pose", "/ekf_elevation/set_pose"),
                ],
                arguments=["--ros-args", "--log-level", "INFO"],
                condition=IfCondition(is_3d),
            ),

            # ── Elevation mapping (mode-dependent config) ──
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
                                parameters=elev_configs + [
                                    {"use_sim_time": True},
                                    {"map_frame_id": "odom_elev"},
                                    {"robot_pose_with_covariance_topic": "/odometry/elevation"},
                                    # Must be 0.0: the 3D EKF has no absolute Z reference,
                                    # so its pose covariance grows unbounded. Any nonzero
                                    # scale injects that into cells, exceeding max_variance
                                    # and invalidating the map.
                                    {"robot_motion_map_update/covariance_scale": 0.0},
                                ],
                                remappings=[("/tf", "/tf_elev")],
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
                                parameters=elev_configs + [
                                    {"use_sim_time": True},
                                    {"map_frame_id": "odom"},
                                    {"robot_pose_with_covariance_topic": "/odometry/local"},
                                    # Original behavior: code defaulted to 1.0 because the
                                    # granular _translation_x/_rotation_x params in the yaml
                                    # were never read. Restore that for a fair comparison.
                                    {"robot_motion_map_update/covariance_scale": 0.0},
                                ],
                            ),
                        ],
                    ),
                ],
            ),
        ]
    )
