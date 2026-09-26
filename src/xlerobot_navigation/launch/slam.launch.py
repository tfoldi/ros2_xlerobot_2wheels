"""slam_toolbox in online async mapping mode on the LD19 scan. Publishes /map and
map -> odom, so navigation.launch.py has to run with slam:=true (no static map ->
odom then). Run xlerobot_bringup's bringup.launch.py with lidar:=true first.

Save the map with
    ros2 run nav2_map_server map_saver_cli -f <name>
and the pose graph, to continue mapping later, with
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '<name>'}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("xlerobot_navigation"), "config", "slam_toolbox.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("slam_params_file", default_value=default_params),
        LifecycleNode(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", namespace="",
            parameters=[LaunchConfiguration("slam_params_file")],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_slam",
            # slam_toolbox has no bond to the manager
            parameters=[{"autostart": True, "node_names": ["slam_toolbox"], "bond_timeout": 0.0}],
        ),
    ])
