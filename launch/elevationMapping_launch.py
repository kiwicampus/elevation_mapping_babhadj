"""
Live-robot launch for Elevation Mapping with full 3D pose estimation.

This launch brings up the complete 3D elevation stack on the real robot:
  - Madgwick IMU filter for the D435i camera
  - inertial_link_broadcaster: publishes base_link->inertial_link AND
      base_link_3d->inertial_link_3d (roll/pitch at IMU rate)
  - 3D EKF (ekf_filter_elevation): estimates Z height, publish_tf=false
  - hybrid_odom_publisher: merges 2D TF (X,Y,yaw) + EKF Z,
      broadcasts odom->base_link_3d on /tf
  - static_frame_aliaser: copies inertial_link->camera_depth_optical_frame
      to inertial_link_3d->camera_depth_optical_frame_3d on /tf_static
  - pointcloud_frame_relay: re-stamps depth cloud with _3d frame_id
  - elevation_mapping: uses base_link_3d and the relayed point cloud

The existing Nav2 stack is unaffected — it continues to see the 2D
odom->base_link transform from the wheel-odometry EKF.

Usage:
  ros2 launch elevation_mapping elevationMapping_launch.py
  ros2 launch elevation_mapping elevationMapping_launch.py separate_camera_imu:=true
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
    separate_camera_imu = LaunchConfiguration("separate_camera_imu")

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

    launch_dir = os.path.dirname(__file__)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "separate_camera_imu",
                default_value="false",
                description="Set to 'true' if the camera driver publishes separate "
                "/camera/gyro/sample and /camera/accel/sample topics",
            ),
            # ── D435i gyro+accel combiner (only when driver has separate topics) ──
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(launch_dir, "camera_imu_combiner.py"),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=false",
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", separate_camera_imu, "' == 'true'"])
                ),
            ),
            # ── Madgwick filter for D435i — separate topics path ──
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick_camera",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
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
                    PythonExpression(["'", separate_camera_imu, "' == 'true'"])
                ),
            ),
            # ── Madgwick filter for D435i — combined topic path ──
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick_camera",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
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
                    PythonExpression(["'", separate_camera_imu, "' != 'true'"])
                ),
            ),
            # ── inertial_link_broadcaster ──
            # Publishes base_link->inertial_link with real roll/pitch from Madgwick.
            # Also publishes base_link_3d->inertial_link_3d with the same rotation
            # so elevation_mapping gets camera orientation at IMU rate via base_link_3d.
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(launch_dir, "inertial_link_broadcaster.py"),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=false",
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
            ),
            # ── 3D EKF for elevation mapping ──
            # Estimates Z height from wheel odometry body-frame vz + camera IMU
            # roll/pitch. publish_tf=false — pose is consumed by hybrid_odom_publisher.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_elevation",
                output="screen",
                parameters=[
                    loc_params,
                    {"use_sim_time": False},
                ],
                remappings=[
                    ("odometry/filtered", "odometry/elevation"),
                ],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
            # ── Hybrid Odometry Publisher ──
            # Merges 2D TF (X, Y, yaw) with 3D EKF (Z) and broadcasts
            # odom->base_link_3d on /tf with yaw-only rotation + correct Z.
            Node(
                package="elevation_mapping",
                executable="hybrid_odom_publisher.py",
                name="hybrid_odom_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
                    {"publish_tf_3d": True},
                    {"base_frame_3d": "base_link_3d"},
                ],
            ),
            # ── Point cloud frame relay ──
            # Re-stamps the depth cloud with camera_depth_optical_frame_3d so that
            # elevation_mapping's TF lookup goes through the 3D alias subtree.
            Node(
                package="elevation_mapping",
                executable="pointcloud_frame_relay.py",
                name="pointcloud_frame_relay",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
                    {"input_topic": "/camera/depth/color/points"},
                    {"output_topic": "/camera/depth/color/points_3d"},
                    {"output_frame_id": "camera_depth_optical_frame_3d"},
                ],
            ),
            # ── Elevation mapping + static frame aliaser ──
            # Delayed 3 s to let the realsense driver populate /tf_static and
            # the EKF + hybrid publisher settle before mapping starts.
            TimerAction(
                period=3.0,
                actions=[
                    GroupAction(
                        actions=[
                            # Alias inertial_link->camera_depth_optical_frame
                            # to inertial_link_3d->camera_depth_optical_frame_3d.
                            # The realsense driver publishes calibrated static frames
                            # at startup; this node reads them and creates the alias.
                            Node(
                                package="elevation_mapping",
                                executable="static_frame_aliaser.py",
                                name="static_frame_aliaser",
                                output="screen",
                                parameters=[
                                    {"use_sim_time": False},
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
                                parameters=elev_configs
                                + [
                                    {"use_sim_time": False},
                                    {"map_frame_id": "odom"},
                                    {"robot_base_frame_id": "base_link_3d"},
                                    {"track_point_frame_id": "base_link_3d"},
                                    {"front_cam.topic": "/camera/depth/color/points_3d"},
                                    {"robot_motion_map_update.covariance_scale": 0.0},
                                ],
                            ),
                        ],
                    ),
                ],
            ),
        ]
    )
