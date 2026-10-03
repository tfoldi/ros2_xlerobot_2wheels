"""Both arms' MoveIt Servo instances + the joystick-to-Servo mapping node, together.

Needs bringup (with_arms:=true) and move_group already running - same prerequisites as
src/xlerobot_moveit_config/launch/servo.launch.py, which this includes twice (arm:=left,
arm:=right).

  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true torque_off:=false
  ros2 launch xlerobot_moveit_config move_group.launch.py
  ros2 launch xlerobot_teleop arm_servo_teleop.launch.py

/joy must already exist (bringup's own teleop:=true default covers this) - this launch file
does not start joy_node itself, same reasoning as mcp_presence.launch.py: avoid a duplicate
joy_node if teleop.launch.py is already running.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    teleop_share = get_package_share_directory("xlerobot_teleop")
    servo_launch_file = os.path.join(
        get_package_share_directory("xlerobot_moveit_config"), "launch", "servo.launch.py")
    config = os.path.join(teleop_share, "config", "arm_servo_teleop.yaml")

    return LaunchDescription([
        # auto_activate:=false - arm_servo_teleop itself owns calling switch_command_type (it
        # has to, to track which mode the user has toggled to), a second auto-activation here
        # would race it, see servo.launch.py's own docstring.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servo_launch_file),
            launch_arguments={"arm": "left", "auto_activate": "false"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servo_launch_file),
            launch_arguments={"arm": "right", "auto_activate": "false"}.items(),
        ),
        Node(
            package="xlerobot_teleop",
            executable="arm_servo_teleop",
            parameters=[config],
            output="screen",
        ),
    ])
