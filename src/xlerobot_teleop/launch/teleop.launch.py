"""Joystick teleop with twist_mux in front of the base controller, plus the
safety supervisor. Nav2's cmd_vel_nav plugs into the same mux once phase 2
exists, so the joystick keeps override priority over autonomy from day one
instead of being bolted on after the fact.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("xlerobot_teleop")
    joystick_yaml = os.path.join(pkg_share, "config", "joystick.yaml")
    twist_mux_yaml = os.path.join(pkg_share, "config", "twist_mux.yaml")

    return LaunchDescription([
        Node(package="joy", executable="joy_node"),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[joystick_yaml],
            remappings=[("cmd_vel", "cmd_vel_joy")],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_yaml],
            remappings=[("cmd_vel_out", "diff_drive_controller/cmd_vel")],
        ),
        Node(
            package="xlerobot_teleop",
            executable="safety_supervisor",
            output="screen",
        ),
    ])
