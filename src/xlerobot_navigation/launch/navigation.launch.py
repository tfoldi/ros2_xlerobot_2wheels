"""Nav2: controller/planner/behavior/bt_navigator/velocity_smoother. By default it runs on
wheel odometry only, map->odom is a static identity transform published here and the global
costmap is a rolling window. slam:=true also starts slam_toolbox (slam.launch.py), which
publishes /map and map->odom, and the global costmap becomes that map (nav2_slam.yaml).

Run xlerobot_bringup's bringup.launch.py first - this assumes diff_drive_controller/odom is
already publishing. rviz:=true opens RViz with Nav2's default view plus the robot model,
"2D Nav Goal" sends a navigate_to_pose goal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def launch_setup(context):
    nav_share = get_package_share_directory("xlerobot_navigation")
    slam = LaunchConfiguration("slam").perform(context) == "true"
    # later files override earlier ones
    params_file = [os.path.join(nav_share, "config", "nav2_params.yaml")]
    if slam:
        params_file.append(os.path.join(nav_share, "config", "nav2_slam.yaml"))
    rviz_config = os.path.join(nav_share, "rviz", "xlerobot_nav.rviz")

    rviz = LaunchConfiguration("rviz")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    return [
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "slam.launch.py")),
            condition=IfCondition(LaunchConfiguration("slam")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
            condition=UnlessCondition(LaunchConfiguration("slam")),
        ),
        LifecycleNode(
            package="nav2_controller", executable="controller_server",
            name="controller_server", namespace="",
            parameters=params_file,
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        LifecycleNode(
            package="nav2_planner", executable="planner_server",
            name="planner_server", namespace="",
            parameters=params_file,
        ),
        LifecycleNode(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", namespace="",
            parameters=params_file,
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        LifecycleNode(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", namespace="",
            parameters=params_file,
        ),
        LifecycleNode(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", namespace="",
            parameters=params_file,
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("slam", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
