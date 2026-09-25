"""Wheel-odometry-only Nav2: controller/planner/behavior/bt_navigator/velocity_smoother,
no map_server or amcl (ADR-0003 - no sensor yet). map->odom is published here as a static
identity transform since there's no localization source; real SLAM replaces this once the
VLP-16 is up (ARCHITECTURE §5).

Run xlerobot_bringup's bringup.launch.py first - this assumes diff_drive_controller/odom is
already publishing. rviz:=true opens RViz with Nav2's default view plus the robot model,
"2D Nav Goal" sends a navigate_to_pose goal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    nav_share = get_package_share_directory("xlerobot_navigation")
    params_file = os.path.join(nav_share, "config", "nav2_params.yaml")
    rviz_config = os.path.join(nav_share, "rviz", "xlerobot_nav.rviz")

    rviz = LaunchConfiguration("rviz")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
            # Ogre's GLX renderer segfaults under native Wayland. Forcing xcb inherits this
            # session's QT_AUTO_SCREEN_SCALE_FACTOR=1 (set for native Wayland apps), which
            # against XWayland's own DPI reporting doubles up into huge UI text and a
            # mouse/viewport coordinate mismatch (the 3D view jitters when orbiting).
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
                "QT_AUTO_SCREEN_SCALE_FACTOR": "0",
                "QT_SCALE_FACTOR": "1",
                "QT_ENABLE_HIGHDPI_SCALING": "0",
            },
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        ),
        LifecycleNode(
            package="nav2_controller", executable="controller_server",
            name="controller_server", namespace="",
            parameters=[params_file],
            remappings=[("cmd_vel", "cmd_vel_nav")],
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
            remappings=[("cmd_vel", "cmd_vel_nav")],
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
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "/diff_drive_controller/cmd_vel"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{"autostart": True, "node_names": lifecycle_nodes}],
        ),
    ])
