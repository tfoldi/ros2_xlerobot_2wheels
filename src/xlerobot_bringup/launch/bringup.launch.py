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

record:=true records every topic to bags/<timestamp> (gitignored) for
offline debugging, camera video as H.264 packets only (raw and the other
image transports are left out, they are far too big and would make the
cameras encode them). foxglove:=true starts foxglove_bridge on the default
port 8765 for live inspection from Foxglove Studio. Both off by default.

cameras:=true starts the USB cameras from config/cameras.yaml, see
cameras.launch.py. Off by default, a camera that is not connected is skipped.
"""

import datetime
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_port_holders(port_path):
    """PIDs (with cmdline) that currently have port_path open, or [] if free.

    sts_hardware_interface's SCS::syncReadBegin isn't safe against a second
    process opening the same serial port mid-configure - confirmed on this
    robot: it doesn't error, it segfaults controller_manager (heap
    corruption inside the vendor SDK's read buffer setup). That's exactly
    what happens if a previous bringup hasn't fully released the port yet
    (e.g. Ctrl-C then an immediate relaunch) - the old process is still
    dying when the new one opens the same device. Catch that here, before
    ros2_control_node ever starts, instead of after it crashes.
    """
    if not os.path.exists(port_path):
        return []
    real_path = os.path.realpath(port_path)
    holders = []
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_dir.split("/")[2]
        try:
            for fd in os.listdir(fd_dir):
                if os.path.realpath(os.path.join(fd_dir, fd)) == real_path:
                    try:
                        with open(f"/proc/{pid}/cmdline", "rb") as f:
                            cmdline = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                    except OSError:
                        cmdline = "?"
                    holders.append((pid, cmdline))
                    break
        except OSError:
            continue  # process exited while we were scanning it, not a real holder
    return holders


def _check_serial_ports_free(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim").perform(context) == "true"
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context) == "true"
    if use_sim or use_mock_hardware:
        return []  # no real serial port gets opened in either mode

    with_head = LaunchConfiguration("with_head").perform(context) == "true"
    with_arms = LaunchConfiguration("with_arms").perform(context) == "true"
    ports = [LaunchConfiguration("bus2_port").perform(context)]
    if with_head or with_arms:
        ports.append(LaunchConfiguration("bus1_port").perform(context))

    problems = []
    for port in ports:
        for pid, cmdline in _find_port_holders(port):
            problems.append(f"  {port} is already open by pid {pid}: {cmdline}")

    if problems:
        raise RuntimeError(
            "Refusing to start bringup: a real servo bus port is still held open by "
            "another process (a previous bringup that hasn't fully exited yet is the "
            "usual cause - wait a couple seconds after Ctrl-C before relaunching):\n"
            + "\n".join(problems)
        )
    return []


def generate_launch_description():
    description_share = get_package_share_directory("xlerobot_description")
    bringup_share = get_package_share_directory("xlerobot_bringup")
    teleop_share = get_package_share_directory("xlerobot_teleop")

    xacro_file = os.path.join(description_share, "urdf", "xlerobot.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")
    teleop_launch_file = os.path.join(teleop_share, "launch", "teleop.launch.py")
    cameras_launch_file = os.path.join(bringup_share, "launch", "cameras.launch.py")

    bus1_port = LaunchConfiguration("bus1_port")
    bus2_port = LaunchConfiguration("bus2_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim = LaunchConfiguration("use_sim")
    teleop = LaunchConfiguration("teleop")
    with_head = LaunchConfiguration("with_head")
    with_arms = LaunchConfiguration("with_arms")
    record = LaunchConfiguration("record")
    foxglove = LaunchConfiguration("foxglove")
    cameras = LaunchConfiguration("cameras")

    # Relative to wherever `ros2 launch` is run from - this repo's own convention (see
    # tools/*.py) is always the repo root, where bags/ is gitignored.
    bag_path = os.path.join("bags", datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_ros2_control:=true",
            " use_mock_hardware:=", use_mock_hardware,
            " use_sim:=", use_sim,
            " bus1_port:=", bus1_port,
            " bus2_port:=", bus2_port,
            " with_arms:=", with_arms,
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
        # off by default for the same reason: both SO-101 arms are on bus1/bus2, and
        # with_arms:=true adds their joints to ros2_control (real buses, or use
        # use_mock_hardware:=true to see the model). Controllers come with phase 3.
        DeclareLaunchArgument("with_arms", default_value="false"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("cameras", default_value="false"),

        # Must run before the Node actions below: OpaqueFunction is resolved
        # synchronously at this point in the list, so a raised error here
        # aborts the launch before ros2_control_node ever spawns and touches
        # the port. See _check_serial_ports_free's docstring.
        OpaqueFunction(function=_check_serial_ports_free),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cameras_launch_file),
            condition=IfCondition(cameras),
        ),

        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-a", "-o", bag_path,
                 "--exclude-regex", r".*/image_raw(/compressed|/compressedDepth|/theora|/zstd)?"],
            condition=IfCondition(record),
            output="screen",
        ),
        Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            condition=IfCondition(foxglove),
        ),
    ])
