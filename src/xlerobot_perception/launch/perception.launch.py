"""Sensor bringup, dispatched by the sensor arg: none, vlp16, gemini,
realsense. See ADR-0003, none of these paths are tested yet.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sensor = LaunchConfiguration("sensor")

    # TODO: each branch launches the matching driver + pointcloud_to_laserscan
    # once we know which sensor we're actually running. Empty groups for now,
    # kept structurally so the dispatch pattern is settled.
    vlp16_group = GroupAction(
        condition=LaunchConfigurationEquals("sensor", "vlp16"),
        actions=[],
    )
    gemini_group = GroupAction(
        condition=LaunchConfigurationEquals("sensor", "gemini"),
        actions=[],
    )
    realsense_group = GroupAction(
        condition=LaunchConfigurationEquals("sensor", "realsense"),
        actions=[],
    )

    return LaunchDescription([
        DeclareLaunchArgument("sensor", default_value="none"),
        vlp16_group,
        gemini_group,
        realsense_group,
    ])
