"""MoveIt 2 for the two arms: move_group and, with rviz:=true, RViz with the MotionPlanning panel.

Run it next to the bringup, it uses the controllers and joint states of that one:

  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true torque_off:=false      # the robot
  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_sim:=true          # the Isaac twin
  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_mock_hardware:=true torque_off:=false
  ros2 launch xlerobot_moveit_config move_group.launch.py [rviz:=true] [use_sim_time:=true]

The robot description here is only used for kinematics and collision, it does not have to match
torque_off or the hardware, but it must have the arms.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    description_share = get_package_share_directory("xlerobot_description")
    config_share = get_package_share_directory("xlerobot_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("xlerobot", package_name="xlerobot_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "xlerobot.urdf.xacro"),
            mappings={"with_arms": "true", "with_head": "true", "use_ros2_control": "true"},
        )
        .robot_description_semantic(
            file_path="config/xlerobot.srdf.xacro",
            mappings={"disabled_collisions": LaunchConfiguration("disabled_collisions").perform(context)},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", os.path.join(config_share, "config", "moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        # false starts without the self-collision matrix, only for generating it
        DeclareLaunchArgument("disabled_collisions", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
