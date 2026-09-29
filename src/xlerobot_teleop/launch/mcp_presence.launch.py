"""Presence guard for agent-issued (MCP) motion commands. Standalone from full teleop.launch.py
on purpose - requires /joy to already be published (either teleop.launch.py's joy_node, or a
bare `ros2 run joy joy_node`), doesn't start its own joy_node to avoid a duplicate-node clash if
teleop.launch.py is already running alongside it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("xlerobot_teleop")
    presence_yaml = os.path.join(pkg_share, "config", "mcp_presence.yaml")

    return LaunchDescription([
        Node(
            package="xlerobot_teleop",
            executable="mcp_presence_guard",
            parameters=[presence_yaml],
            output="screen",
        ),
    ])
