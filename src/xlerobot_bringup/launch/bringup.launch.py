"""Base bringup: robot_state_publisher + controller_manager + spawners.
No arms, no sensors yet, that's the phase 1 scope. Point bus1_port/bus2_port
at your udev symlinks, not the raw ttyACM device, those numbers swap on reboot.

use_sim:=true switches to the Isaac Sim twin's hardware interface instead
of the real bus (or sts_hardware_interface's mock mode), see docs/sim-twin.md.
bus1_port/bus2_port are ignored in that case. The Isaac Sim scene needs to
be playing (Play pressed in the GUI, or app_utils.play() in a script) for
its side of the ROS 2 bridge to actually publish/subscribe anything.

teleop:=true also brings up xlerobot_teleop's joystick/twist_mux/safety
supervisor stack (xlerobot_teleop/launch/teleop.launch.py), off by default
so a plain bringup doesn't grab a joystick device or start the safety
watchdog when nothing needs it. The watchdog engages e_stop until this
bringup's own joint_state_broadcaster is actually publishing, so there's
no real ordering issue bringing both up together this way, unlike running
teleop.launch.py standalone before bringup is up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("xlerobot_description")
    bringup_share = get_package_share_directory("xlerobot_bringup")
    teleop_share = get_package_share_directory("xlerobot_teleop")

    xacro_file = os.path.join(description_share, "urdf", "xlerobot.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")
    teleop_launch_file = os.path.join(teleop_share, "launch", "teleop.launch.py")

    bus1_port = LaunchConfiguration("bus1_port")
    bus2_port = LaunchConfiguration("bus2_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim = LaunchConfiguration("use_sim")
    teleop = LaunchConfiguration("teleop")
    with_head = LaunchConfiguration("with_head")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_ros2_control:=true",
            " use_mock_hardware:=", use_mock_hardware,
            " use_sim:=", use_sim,
            " bus1_port:=", bus1_port,
            " bus2_port:=", bus2_port,
            " with_arms:=false",
            " with_head:=", with_head,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("bus1_port", default_value="/dev/xle-bus1"),
        DeclareLaunchArgument("bus2_port", default_value="/dev/xle-bus2"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("teleop", default_value="false"),
        # off by default: bus1 (head pan/tilt) isn't wired on a base-only
        # robot, see xlerobot.urdf.xacro. Flip on once the head is plugged in.
        DeclareLaunchArgument("with_head", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controllers_yaml],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_file),
            condition=IfCondition(teleop),
        ),
    ])
