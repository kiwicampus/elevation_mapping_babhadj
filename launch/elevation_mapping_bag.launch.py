"""
Elevation Mapping launch for ROSBAG playback (offline testing).

Replaces: test_elevation_3d.launch.py

Usage:
  ros2 launch elevation_mapping elevation_mapping_bag.launch.py \
      bag_path:=/workspace/localization_offline

  # With separate gyro/accel topics:
  ros2 launch elevation_mapping elevation_mapping_bag.launch.py \
      bag_path:=/workspace/localization_off_imu_filter_mor_separated \
      separate_camera_imu:=true

  # Slow playback:
  ros2 launch elevation_mapping elevation_mapping_bag.launch.py \
      bag_path:=/workspace/localization_offline rate:=0.5

NOTE: Kill stale processes before relaunching:
  pkill -f "hybrid_odom_publisher|ekf_filter_elevation|inertial_link_broadcaster|pointcloud_frame_relay|static_frame_aliaser"
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    play_rate = LaunchConfiguration("rate")
    separate_camera_imu = LaunchConfiguration("separate_camera_imu")

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
    launch_dir = os.path.dirname(__file__)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="/workspace/localization_offline",
                description="Path to rosbag directory",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="1.0",
                description="Bag playback rate",
            ),
            DeclareLaunchArgument(
                "separate_camera_imu",
                default_value="false",
                description="Set to 'true' if the bag has separate /camera/gyro/sample "
                "and /camera/accel/sample topics instead of a combined /camera/imu",
            ),

            # ── Rosbag playback ──
            # Remaps /tf and /tf_static to /tf_bag and /tf_static_bag so that
            # tf_odom_filter can selectively forward frames to the live /tf topic,
            # dropping frames that our live nodes will re-publish (e.g. inertial_link).
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "play", bag_path,
                    "--clock",
                    "--rate", play_rate,
                    "--qos-profile-overrides-path",
                    "/workspace/rover/ros2/src/navigation/config/tf_static_override.yaml",
                    "--remap", "/tf:=/tf_bag",
                    "--remap", "/tf_static:=/tf_static_bag",
                ],
                output="screen",
            ),

            # ── TF filter ──
            # Forwards /tf_bag -> /tf, dropping transforms for frames that our
            # live nodes own (inertial_link is republished by inertial_link_broadcaster).
            Node(
                package="elevation_mapping",
                executable="tf_odom_filter.py",
                name="tf_odom_filter",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),

            # ── D435i IMU combiner (only when bag has separate gyro/accel topics) ──
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(launch_dir, "camera_imu_combiner.py"),
                    "--ros-args", "-p", "use_sim_time:=true",
                ],
                output="screen",
                condition=IfCondition(use_combiner),
            ),

            # ── Madgwick filter — separate topics path ──
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
                    "gain": 0.1,
                    "zeta": 0.0,
                }],
                remappings=[
                    ("imu/data_raw", "/camera/imu_combined"),
                    ("imu/data", "/camera/imu_filtered"),
                ],
                condition=IfCondition(use_combiner),
            ),

            # ── Madgwick filter — combined topic path ──
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
                    "gain": 0.1,
                    "zeta": 0.0,
                }],
                remappings=[
                    ("imu/data_raw", "/camera/imu"),
                    ("imu/data", "/camera/imu_filtered"),
                ],
                condition=IfCondition(
                    PythonExpression(["'", separate_camera_imu, "' != 'true'"])
                ),
            ),

            # ── inertial_link_broadcaster ──
            # Broadcasts base_link->inertial_link (roll/pitch from Madgwick, yaw=0).
            # Also broadcasts base_link_3d->inertial_link_3d with identical rotation,
            # so elevation_mapping gets IMU-rate roll/pitch corrections via base_link_3d.
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(launch_dir, "inertial_link_broadcaster.py"),
                    "--ros-args",
                    "-p", "use_sim_time:=true",
                    "-p", "orientation_2d:=false",
                    "-p", "also_publish_3d_alias:=true",
                    "-p", "base_frame_3d:=base_link_3d",
                    "-p", "inertial_frame_3d:=inertial_link_3d",
                ],
                output="screen",
            ),

            # ── 3D EKF ──
            # Estimates Z height from wheel odometry vz + camera IMU roll/pitch.
            # publish_tf=false — pose is consumed via hybrid_odom_publisher.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_elevation",
                output="screen",
                parameters=[loc_params, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/elevation")],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),

            # ── Hybrid odometry publisher ──
            # Merges 2D TF (X, Y, yaw) with EKF Z.
            # Broadcasts odom->base_link_3d on /tf at 50 Hz (timer-driven, not EKF-rate).
            # Roll/pitch come from inertial_link_broadcaster, not from this transform.
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
            ),

            # ── Point cloud frame relay ──
            # Copies /camera/depth/color/points, changes header.frame_id to
            # camera_depth_optical_frame_3d, publishes as /camera/depth/color/points_3d.
            # This makes elevation_mapping's TF lookup traverse the 3D alias subtree.
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
            ),

            # ── Static frame aliaser + elevation mapping ──
            # Delayed 2 s to let sim-time advance so /tf_static is populated and
            # the EKF has settled. The static_frame_aliaser reads the composed
            # inertial_link->camera_depth_optical_frame transform from /tf_static
            # and republishes it as inertial_link_3d->camera_depth_optical_frame_3d.
            # It polls every 0.5 s internally until the source frame is available.
            TimerAction(
                period=2.0,
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
                    ),
                    Node(
                        package="elevation_mapping",
                        executable="elevation_mapping",
                        name="elevation_mapping",
                        output="screen",
                        parameters=elev_configs + [
                            {"use_sim_time": True},
                            {"map_frame_id": "odom"},
                            {"robot_base_frame_id": "base_link_3d"},
                            {"track_point_frame_id": "base_link_3d"},
                            {"robot_motion_map_update.covariance_scale": 0.0},
                        ],
                        # Redirect the internal subscription from kiwi.yaml topic to
                        # the relayed cloud (frame_id = camera_depth_optical_frame_3d).
                        remappings=[
                            ("/camera/depth/color/points", "/camera/depth/color/points_3d"),
                        ],
                    ),
                ],
            ),

            # ── Traversability estimation ──
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
        ]
    )
