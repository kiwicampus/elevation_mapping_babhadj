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
            # ── Rosbag playback (3D mode) ──
            # /tf is remapped to /tf_bag to prevent the bag's 2D odom->base_link from
            # conflicting with the 3D EKF. The tf_odom_filter node (below) republishes
            # all other transforms from /tf_bag back onto /tf.
            # --qos-profile-overrides-path is required so /tf_static is replayed with
            # transient_local durability; without it, late-joining nodes (e.g. RViz)
            # never receive the static sensor extrinsics.
            GroupAction(
                condition=IfCondition(is_3d),
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
                            PythonExpression(
                                [
                                    "'",
                                    odom_remap,
                                    "' + ':=/wheel_odometry/global_odometry'",
                                ]
                            ),
                            "/imu_average:=/imu_average_bag",
                            "/tf:=/tf_bag",
                        ],
                        output="screen",
                    ),
                ],
            ),
            # ── TF odom filter (3D mode only) ──
            # Forwards /tf_bag -> /tf, dropping frames whose parent is 'odom'
            # (i.e. the bag's 2D odom->base_link). Everything else passes through,
            # including dynamic transforms like base_link->inertial_link.
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(os.path.dirname(__file__), "tf_odom_filter.py"),
                ],
                output="screen",
                condition=IfCondition(is_3d),
            ),
            # ── Rosbag playback (2D mode: bag's /tf used as-is) ──
            GroupAction(
                condition=UnlessCondition(is_3d),
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
                            "/imu_average:=/imu_average_bag",
                        ],
                        output="screen",
                    ),
                ],
            ),
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
            # Broadcasts base_link -> inertial_link as identity (orientation_2d:=true).
            #
            # Why identity and NOT real roll/pitch:
            #   orientation_2d:false causes circular cancellation —
            #   the broadcaster derives base_link->inertial_link FROM q_camera_to_ENU,
            #   then robot_localization uses that dynamic TF to transform the same
            #   q_camera_to_ENU back to base_link, perfectly cancelling pitch → Z=0.
            #
            # With identity, robot_localization traverses the static chain:
            #   base_link -> inertial_link(=base_link) -> camera_link -> camera_optical
            # getting real pitch with no cancellation → EKF estimates Z correctly.
            #
            # Note: we cannot publish base_link->camera_link directly to break the loop
            # because the bag's /tf_static already has inertial_link->camera_link,
            # creating a TF parent conflict (camera_link would have two parents).
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
            # ── Elevation mapping ──
            # Delayed 2 s to let the EKF and TF tree settle first.
            # No /tf remap here — in 3D mode /tf already carries the correct
            # odom->base_link from the EKF (tf_odom_filter forwards the rest).
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
                                    {
                                        "robot_pose_with_covariance_topic": "/odometry/elevation"
                                    },
                                    {"robot_motion_map_update/covariance_scale": 1.0},
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
                                    {"robot_motion_map_update/covariance_scale": 1.0},
                                ],
                            ),
                        ],
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
