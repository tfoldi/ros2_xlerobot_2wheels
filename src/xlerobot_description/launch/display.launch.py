"""View the model, no hardware, no controllers. Just robot_state_publisher,
joint_state_publisher_gui and rviz. Good first check after touching the xacro."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("xlerobot_description")
    default_xacro = os.path.join(pkg_share, "urdf", "xlerobot.urdf.xacro")
    default_rviz = os.path.join(pkg_share, "rviz", "xlerobot.rviz")

    xacro_file = LaunchConfiguration("xacro_file")
    with_arms = LaunchConfiguration("with_arms")
    sensor = LaunchConfiguration("sensor")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_ros2_control:=true use_mock_hardware:=true",
            " with_arms:=", with_arms,
            " sensor:=", sensor,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("xacro_file", default_value=default_xacro),
        DeclareLaunchArgument("with_arms", default_value="false"),
        DeclareLaunchArgument("sensor", default_value="none"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", default_rviz],
        ),
    ])
