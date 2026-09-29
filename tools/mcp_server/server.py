#!/usr/bin/env python3
"""MCP server exposing XleRobot navigation, both arms, the gripper and cameras.

Run via .mcp.json (see README.md), or standalone for a quick check:
  python3 -m mcp_server.server

Needs ROS sourced (rclpy, cv_bridge, moveit_msgs, nav2_msgs, ...) plus the `mcp` pip package -
see README.md for the venv setup. Needs bringup running (with_arms:=true cameras:=true),
move_group, navigation.launch.py localization:=true, and mcp_presence.launch.py (see
xlerobot_teleop) for any motion tool to work.

Tool surface: read-only tools (list_positions, list_waypoints, get_joint_states, get_robot_pose,
get_status, capture_image) work with no presence button held. Every motion tool
(goto_position, move_to_pose, move_gripper, play_sequence, goto_waypoint, save_position,
save_waypoint) requires /mcp_presence_ok and is cancelled + the arm's bus e-stopped (or the base
stopped) if presence lapses mid-motion. estop/release_estop are never gated - always callable,
including to recover from a PresenceLost error.
"""
import functools
import time

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError

import bridge
from bridge import MoveError, PresenceLost

mcp = MCPServer("xlerobot")


def catch_bridge_errors(fn):
    """MCPServer treats a bare exception as a crash (UnexpectedToolError, message withheld from
    the model, logged at ERROR) - these are anticipated operational failures (bad presence
    state, a rejected plan, an unknown saved name, a bad argument), not bugs, so re-raise them
    as ToolError: the model sees the real message, no traceback, logged at INFO. LookupError
    covers both position_store.NotFoundError/waypoint_store.NotFoundError and a plain KeyError
    from an invalid group/arm name."""
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except (MoveError, PresenceLost, ValueError, LookupError) as e:
            raise ToolError(str(e)) from e
    return wrapper


@mcp.tool()
@catch_bridge_errors
def list_positions() -> dict:
    """List saved named arm/gripper/head joint positions (src/xlerobot_bringup/config/saved_positions.yaml)."""
    return bridge.get_bridge().list_positions()


@mcp.tool()
@catch_bridge_errors
def list_waypoints() -> dict:
    """List saved named base waypoints, map-frame x/y/yaw (src/xlerobot_bringup/config/saved_waypoints.yaml)."""
    return bridge.get_bridge().list_waypoints()


@mcp.tool()
@catch_bridge_errors
def get_joint_states(group: str = "") -> dict:
    """Current joint positions (radians). group is a MoveIt group name (left_arm, right_arm,
    left_gripper, right_gripper, head, both_arms) or omitted for every cached joint."""
    return bridge.get_bridge().get_joint_states(group or None)


@mcp.tool()
@catch_bridge_errors
def get_robot_pose() -> dict | None:
    """Current localized base pose (map frame), from /amcl_pose. None if not localized yet."""
    return bridge.get_bridge().get_robot_pose()


@mcp.tool()
@catch_bridge_errors
def get_status() -> dict:
    """Liveness summary: presence heartbeat, joint_states freshness, localization, which cameras
    have a frame. NOT a torque-state check - see tools/calibrate_arm.py status for that."""
    return bridge.get_bridge().get_status()


@mcp.tool()
@catch_bridge_errors
def capture_image(camera: str) -> dict:
    """Grab the latest frame from a camera (head, left, right - the arm wrist cams) and save it
    as a PNG under captures/. Returns the file path to read directly. No presence needed - this
    doesn't move anything."""
    output = bridge.CAPTURE_DIR / f"{camera}_{int(time.time())}.png"
    return bridge.get_bridge().capture_image(camera, output)


@mcp.tool()
@catch_bridge_errors
def goto_position(name: str, scaling: float = 0.0) -> str:
    """Plan and execute a move to a saved named joint position (collision-aware, via MoveIt).
    scaling 0 uses joint_limits.yaml's default. Requires the presence heartbeat."""
    return bridge.get_bridge().goto_position(name, scaling)


@mcp.tool()
@catch_bridge_errors
def move_to_pose(arm: str, x: float, y: float, z: float, frame: str = "base_footprint", scaling: float = 0.0) -> str:
    """Move an arm's gripper to a Cartesian point (position-only - the SO-101 arms' IK has no
    orientation control, gripper orientation follows whatever the IK solution lands on). arm is
    'left' or 'right'. Requires the presence heartbeat."""
    return bridge.get_bridge().move_to_pose(arm, x, y, z, frame, scaling)


@mcp.tool()
@catch_bridge_errors
def move_gripper(arm: str, action: str) -> str:
    """Open or close an arm's gripper. arm is 'left' or 'right', action is 'open' or 'close'.
    Requires the presence heartbeat."""
    return bridge.get_bridge().move_gripper(arm, action)


@mcp.tool()
@catch_bridge_errors
def play_sequence(names: list[str], loop: str = "1", scaling: float = 0.0) -> str:
    """Move through several saved named positions in order. loop is a repeat count or 'forever'
    (use with care - only cancellable via presence loss or estop). Requires the presence heartbeat."""
    return bridge.get_bridge().play_sequence(names, loop, scaling)


@mcp.tool()
@catch_bridge_errors
def goto_waypoint(name: str, timeout: float = 120.0) -> str:
    """Navigate to a saved named base waypoint via Nav2. Requires navigation.launch.py running
    with localization:=true, and the presence heartbeat."""
    return bridge.get_bridge().goto_waypoint(name, timeout)


@mcp.tool()
@catch_bridge_errors
def save_position(name: str, group: str) -> str:
    """Save the arm/gripper/head's current joint positions under a name, for later goto_position/
    play_sequence calls. Requires the presence heartbeat (it's not motion, but teaching a position
    you'll later replay deserves the same "someone's watching" guarantee)."""
    return bridge.get_bridge().save_position(name, group)


@mcp.tool()
@catch_bridge_errors
def save_waypoint(name: str) -> str:
    """Save the robot's current localized pose as a named waypoint, for later goto_waypoint
    calls. Requires /amcl_pose and the presence heartbeat."""
    return bridge.get_bridge().save_waypoint(name)


@mcp.tool()
@catch_bridge_errors
def estop() -> dict:
    """Emergency-stop every servo bus (arms, head, wheels lose torque/command). Never gated -
    always callable. Follow up with tools/calibrate_arm.py status (stack stopped) to verify."""
    return bridge.get_bridge().estop(active=True)


@mcp.tool()
@catch_bridge_errors
def release_estop() -> dict:
    """Release e-stop on every servo bus - joints hold where they are, then move back toward
    their last commanded trajectory at the speed cap. Never gated - always callable."""
    return bridge.get_bridge().estop(active=False)


if __name__ == "__main__":
    mcp.run()
