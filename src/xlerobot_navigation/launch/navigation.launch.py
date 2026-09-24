"""Wheel-odometry-only Nav2: controller/planner/behavior/bt_navigator/velocity_smoother,
no map_server or amcl (ADR-0003 - no sensor yet). map->odom is published here as a static
identity transform since there's no localization source; real SLAM replaces this once the
VLP-16 is up (ARCHITECTURE §5).

Run xlerobot_bringup's bringup.launch.py first - this assumes diff_drive_controller/odom is
already publishing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    nav_share = get_package_share_directory("xlerobot_navigation")
    params_file = os.path.join(nav_share, "config", "nav2_params.yaml")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        ),
        LifecycleNode(
            package="nav2_controller", executable="controller_server",
            name="controller_server", namespace="",
            parameters=[params_file],
        ),
        LifecycleNode(
            package="nav2_planner", executable="planner_server",
            name="planner_server", namespace="",
            parameters=[params_file],
        ),
        LifecycleNode(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", namespace="",
            parameters=[params_file],
        ),
        LifecycleNode(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", namespace="",
            parameters=[params_file],
        ),
        LifecycleNode(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", namespace="",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{"autostart": True, "node_names": lifecycle_nodes}],
        ),
    ])
