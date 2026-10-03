"""MoveIt Servo for one arm - real-time Cartesian/joint jogging, as opposed to the one-shot
plan+execute moves tools/move_to_pose.py and tools/goto_position.py send through move_group.

Run next to bringup and move_group (both already provide what this needs - robot_description,
controllers, joint_states):

  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true torque_off:=false
  ros2 launch xlerobot_moveit_config move_group.launch.py
  ros2 launch xlerobot_moveit_config servo.launch.py arm:=right

One instance per arm - launch it twice (arm:=left, arm:=right) for both at once, each gets its
own node name (servo_left / servo_right) so their relative topics don't collide:
  /servo_right/delta_twist_cmds   (geometry_msgs/TwistStamped, Cartesian jog)
  /servo_right/delta_joint_cmds   (control_msgs/JointJog, joint-space jog)
  /servo_right/status

Starts paused (no motion until something sends a command on either topic above) - unlike
move_to_pose.py/goto_position.py, Servo has no presence/safety gate of its own, that's the
joystick-mapping node's job once it exists (see the servo-teleop plan in project memory). Don't
drive this directly from a script without something upstream enforcing a deadman, the way every
other motion path in this project does.

Beyond "paused", Servo 2.x also refuses every command with "Command type has not been set" until
something calls its ~/switch_command_type service (moveit_msgs/srv/ServoCommandType) - not
documented in the YAML config, found by testing: sending real JointJog/TwistStamped commands did
nothing and the node logged that warning on every message until the service was called once.
This launch file calls it a few seconds after the node starts, defaulting to TWIST (Cartesian
jogging) - the joystick-mapping node can call the same service to switch to JOINT_JOG when it
wants joint-space control instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    description_share = get_package_share_directory("xlerobot_description")
    arm = LaunchConfiguration("arm").perform(context)
    if arm not in ("left", "right"):
        raise RuntimeError(f"arm must be 'left' or 'right', got {arm!r}")

    moveit_config = (
        MoveItConfigsBuilder("xlerobot", package_name="xlerobot_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "xlerobot.urdf.xacro"),
            mappings={"with_arms": "true", "with_head": "true", "use_ros2_control": "true"},
        )
        .robot_description_semantic(file_path="config/xlerobot.srdf.xacro", mappings={"disabled_collisions": "true"})
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("xlerobot_moveit_config").yaml("config/servo.yaml").to_dict()
        | {
            "move_group_name": f"{arm}_arm",
            "command_out_topic": f"/{arm}_arm_controller/joint_trajectory",
        }
    }

    return [
        Node(
            package="moveit_servo",
            executable="servo_node",
            name=f"servo_{arm}",
            parameters=[
                servo_params,
                # Needed directly (not nested under "moveit_servo") by the acceleration-limiting
                # smoothing plugin (use_smoothing: true in servo.yaml) and the planning scene
                # monitor's collision checking respectively - matches moveit_servo's own
                # demo_twist.launch.py, which sets these the same separate, non-nested way.
                {"update_period": 0.02},
                {"planning_group_name": f"{arm}_arm"},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output="screen",
        ),
        # See the module docstring - Servo silently refuses every command until this is called
        # once. 5s is comfortably past the ~3s this node took to log "Servo initialized
        # successfully" when tested on hardware.
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "service", "call", f"/servo_{arm}/switch_command_type",
                         "moveit_msgs/srv/ServoCommandType", "{command_type: 1}"],
                    output="screen",
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", description="left or right"),
        OpaqueFunction(function=launch_setup),
    ])
