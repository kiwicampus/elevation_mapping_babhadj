"""
Elevation-only launch — starts just the mapper + traversability.

Assumes localization is already running (e.g. robot_localization.launch.py) and
is publishing:
  - /odometry/for_elevation           (3D pose for the mapper)
  - TF: odom → base_link_3d → inertial_link_3d
  - /tf_static: inertial_link_3d → camera_depth_optical_frame_3d

In v1 mode robot_localization.launch.py emits identity statics that alias the
URDF frames to the v2 *_3d names, so the mapper always reads the same frames
regardless of which localization stack is running.

The mapper rewrites incoming pointcloud frame_id in place via its
override_frame_id param (see robots/kiwi.yaml), so no relay is required in the
normal path. Set LAUNCH_POINTCLOUD_FRAME_RELAY=1 to run pointcloud_frame_relay
alongside for side-by-side debugging.

Usage:
  ros2 launch elevation_mapping elevation_robot.launch.py
  LAUNCH_POINTCLOUD_FRAME_RELAY=1 ros2 launch elevation_mapping elevation_robot.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_traversability = LaunchConfiguration("launch_traversability")
    launch_pointcloud_frame_relay = bool(
        int(os.getenv("LAUNCH_POINTCLOUD_FRAME_RELAY", "0"))
    )
    # Same respawn knobs the rest of the stack reads (see robot_localization.launch.py).
    respawn_nodes = bool(int(os.getenv("RESPAWN_NODES", "1")))
    respawn_delay = float(os.getenv("RESPAWN_DELAY", "5"))

    elev_share = get_package_share_directory("elevation_mapping")
    trav_share = get_package_share_directory("traversability_estimation")

    elev_configs = [
        os.path.join(elev_share, "config", f)
        for f in [
            "robots/kiwi.yaml",
            "elevation_maps/kiwi_map.yaml",
            "postprocessing/postprocessor_pipeline.yaml",
        ]
    ]
    # robot_footprint_parameter.yaml is intentionally omitted: the active filter
    # chain is per-cell (NormalVectorsFilter → SlopeFilter), and the published
    # /traversability_map is consumed by elevation_grid_layer which lets the Nav2
    # costmap handle footprint inflation. The footprint params only feed the
    # service-based isTraversable/checkInclination queries we don't use, plus the
    # cosmetic /footprint_polygon and /untraversable_polygon viz topics.
    trav_configs = [
        os.path.join(trav_share, "config", f)
        for f in [
            "robot.yaml",
            # "robot_footprint_parameter.yaml",
            "robot_filter_parameter.yaml",
        ]
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulated time from /clock (for bag-based testing).",
        ),
        DeclareLaunchArgument(
            "launch_traversability",
            default_value="true",
            description="Set 'false' to skip traversability_estimation (useful for "
            "isolated mapper testing).",
        ),

        # Optional pointcloud_frame_relay (env-var gated, default off). The mapper owns
        # frame_id rewriting via its override_frame_id param; run this node only when
        # you want a side-by-side cloud on /camera/depth/color/points_3d for debug.
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
                {"image_input_topic": "/camera/color/image_raw"},
                {"image_output_topic": "/camera/color/image_raw_3d"},
                {"image_output_frame_id": "camera_color_optical_frame_3d"},
            ],
            condition=IfCondition(str(launch_pointcloud_frame_relay)),
            respawn=respawn_nodes,
            respawn_delay=respawn_delay,
        ),

        # Elevation mapping — short delay to let TF settle.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="elevation_mapping",
                    executable="elevation_mapping",
                    name="elevation_mapping",
                    output="screen",
                    parameters=elev_configs + [{"use_sim_time": use_sim_time}],
                    remappings=[
                        ("get_raw_submap", "/get_raw_submap"),
                        ("elevation_map_raw_post", "/elevation_map_raw"),
                    ],
                    respawn=respawn_nodes,
                    respawn_delay=respawn_delay,
                ),
            ],
        ),

        # Traversability — wait for the map to be publishing.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="traversability_estimation",
                    executable="traversability_estimation_node",
                    name="traversability_estimation",
                    output="screen",
                    parameters=trav_configs + [{"use_sim_time": use_sim_time}],
                    condition=IfCondition(launch_traversability),
                    respawn=respawn_nodes,
                    respawn_delay=respawn_delay,
                ),
            ],
        ),
    ])
