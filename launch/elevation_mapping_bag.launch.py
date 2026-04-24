"""
Elevation Mapping launch for ROSBAG playback (offline testing) — v2 architecture.

Runs the simplified v2 stack against a rosbag:
  - rosbag playback (with /tf, /tf_static remapped to *_bag)
  - tf_odom_filter: forwards bag TFs selectively back to /tf, dropping inertial_link
    (flatten_relay re-publishes it)
  - imu_filter_madgwick: /camera/imu -> /camera/imu_filtered
  - ekf_filter_3d_odom: 3D EKF, publish_tf=false, base_link_frame=base_link_3d
  - flatten_relay (in location pkg): publishes odom->base_link_3d + base_link_3d->inertial_link_3d
    + static base_link->inertial_link identity + static inertial_link_3d->camera_depth_optical_frame_3d alias
  - pointcloud_frame_relay: rewrites pointcloud frame_id for _3d subtree projection
  - elevation_mapping (this pkg, CPU): consumes /odometry/for_elevation + /camera/depth/color/points_3d
  - traversability_estimation

Usage:
  ros2 launch elevation_mapping elevation_mapping_bag.launch.py \
      bag_path:=/workspace/localization_offline.mcap rate:=1.0

Kill stale processes before relaunching:
  pkill -f "ekf_filter_3d_odom|flatten_relay|pointcloud_frame_relay|tf_odom_filter|ros2 bag play|imu_filter_madgwick|elevation_mapping|traversability_estimation"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    play_rate = LaunchConfiguration("rate")

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

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            default_value="/workspace/localization_offline.mcap",
            description="Path to rosbag directory or .mcap file",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="1.0",
            description="Bag playback rate",
        ),

        # Rosbag playback. Remap /tf and /tf_static so tf_odom_filter can selectively
        # forward frames back to /tf, dropping inertial_link (re-published by flatten_relay).
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", bag_path,
                "--clock",
                "--rate", play_rate,
                "--qos-profile-overrides-path",
                "/workspace/rover/ros2/src/navigation/config/tf_static_override.yaml",
                "--remap", "/tf:=/tf_bag", "/tf_static:=/tf_static_bag",
            ],
            output="screen",
        ),

        # TF filter — drops child=inertial_link from the bag (flatten_relay publishes it
        # as static identity).
        Node(
            package="elevation_mapping",
            executable="tf_odom_filter.py",
            name="tf_odom_filter",
            output="screen",
            parameters=[{"use_sim_time": True,
                         "filtered_child_frames": ["inertial_link"]}],
        ),

        # Madgwick — produces /camera/imu_filtered from raw /camera/imu.
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick_camera",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "world_frame": "enu",
                "use_mag": False,
                "publish_tf": False,
                "gain": 0.03,
                "zeta": 0.0,
            }],
            remappings=[
                ("imu/data_raw", "/camera/imu"),
                ("imu/data", "/camera/imu_filtered"),
            ],
        ),

        # 3D EKF (primary odom estimator). publish_tf=false; flatten_relay owns the _3d TFs.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_3d_odom",
            output="screen",
            parameters=[loc_params, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "/odometry/filtered_3d")],
            arguments=["--ros-args", "--log-level", "INFO"],
        ),

        # flatten_relay — publishes the _3d parallel subtree + static aliases.
        Node(
            package="location",
            executable="flatten_relay",
            name="flatten_relay",
            output="screen",
            parameters=[loc_params, {"use_sim_time": True}],
        ),

        # pointcloud_frame_relay — rewrites pointcloud.frame_id so the mapper's TF
        # lookup traverses the _3d (tilted) subtree and sees correct R,P.
        Node(
            package="elevation_mapping",
            executable="pointcloud_frame_relay",
            name="pointcloud_frame_relay",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"input_topic": "/camera/depth/color/points"},
                {"output_topic": "/camera/depth/color/points_3d"},
                {"output_frame_id": "camera_depth_optical_frame_3d"},
            ],
        ),

        # Elevation mapping — delayed 3 s to let TF tree settle.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="elevation_mapping",
                    executable="elevation_mapping",
                    name="elevation_mapping",
                    output="screen",
                    parameters=elev_configs + [
                        {"use_sim_time": True},
                        {"robot_pose_with_covariance_topic": "/odometry/for_elevation"},
                    ],
                    remappings=[
                        ("get_raw_submap", "/get_raw_submap"),
                        ("elevation_map_raw_post", "/elevation_map_raw"),
                    ],
                ),
            ],
        ),

        # Traversability estimation — delayed 5 s so the elevation map is publishing.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="traversability_estimation",
                    executable="traversability_estimation_node",
                    name="traversability_estimation",
                    output="screen",
                    parameters=trav_configs + [{"use_sim_time": True}],
                ),
            ],
        ),
    ])
