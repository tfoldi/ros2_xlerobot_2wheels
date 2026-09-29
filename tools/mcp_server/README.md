# XleRobot MCP server

Exposes navigation, both arms, the gripper and the USB cameras as MCP tools, so an agent can
drive the robot directly instead of a human running `tools/*.py` scripts by hand. See
`server.py`'s docstring for the full tool list and what each one needs running.

Scope: orchestration, control and perception. Spatial memory (remembering where objects were
seen) is a separate, later slice.

## Setup

Needs ROS sourced (for `rclpy`, `cv_bridge`, `moveit_msgs`, `nav2_msgs`) plus the `mcp` pip
package. A `--system-site-packages` venv gets both without touching system Python:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash   # from the workspace root
sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-opencv     # if not already present
python3 -m venv --system-site-packages ~/venvs/xlerobot-mcp
source ~/venvs/xlerobot-mcp/bin/activate
pip install -r tools/mcp_server/requirements.txt
```

Register in `.mcp.json` at the repo root (gitignored - it only ever holds a machine-specific
absolute path):

```json
{
  "mcpServers": {
    "xlerobot": {
      "command": "/absolute/path/to/ros2_xlerobot_2wheels/tools/mcp_server/run.sh"
    }
  }
}
```

`run.sh` sources ROS and the venv itself before running `server.py` - confirmed by testing, MCP
stdio clients only pass an allow-listed subset of environment variables to the server process
(`PATH`, `HOME`, ... not `PYTHONPATH`/`AMENT_PREFIX_PATH`/`LD_LIBRARY_PATH`), so `rclpy` is not
visible otherwise, regardless of what's sourced in the shell that started Claude Code. Edit the
two `source` lines in `run.sh` to match your ROS install and venv location, and make sure
`ROS_DOMAIN_ID` (if you use one) matches the rest of the bringup - add an `export` for it in
`run.sh` alongside the other `source` lines if so.

## Before any motion tool works

1. Bringup with arms and cameras: `ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true with_head:=true cameras:=true`
2. `ros2 launch xlerobot_moveit_config move_group.launch.py`
3. `ros2 launch xlerobot_navigation navigation.launch.py localization:=true` (needed for
   `goto_waypoint`/`save_waypoint`, not for arm-only tools)
4. `ros2 launch xlerobot_teleop teleop.launch.py` (or a bare `ros2 run joy joy_node`) so `/joy`
   exists, then `ros2 launch xlerobot_teleop mcp_presence.launch.py`

Read-only tools (`list_positions`, `list_waypoints`, `get_joint_states`, `get_robot_pose`,
`get_status`, `capture_image`) work without step 4 - they don't move anything.

## Presence heartbeat

Every motion tool checks `/mcp_presence_ok` before accepting a goal and keeps checking it while
one is in flight - see `xlerobot_teleop/mcp_presence_guard.py` and
`xlerobot_teleop/config/mcp_presence.yaml`. Default: hold or tap the right bumper (button 5 on a
generic Xbox layout - check against upstream's `calibrate_gamepad.py` mapping) at least once
every 5 seconds. A lapse mid-motion cancels the goal and, for an arm move, e-stops that arm's
bus; for navigation, stops the base and cancels the Nav2 goal.

`estop`/`release_estop` are never gated - always callable, including to recover from a
`PresenceLost` error surfaced by another tool.

## Verification

- Read-only tools first, no presence button held - `get_status`, `list_positions`,
  `capture_image` should all just work.
- A motion tool (e.g. `goto_position` to `rest`) should be refused with a `PresenceLost` error
  with no button held, and succeed with it held.
- Staged presence-loss test (arm supported by hand first): start a `goto_position` move, release
  the presence button mid-motion, confirm the goal cancels and the bus e-stops.
- After any estop, verify with `python3 tools/calibrate_arm.py status --arm <left|right|head>`
  (stack stopped) - `get_status` is a liveness summary, not a torque-state check.
