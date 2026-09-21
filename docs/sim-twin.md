# Isaac Sim digital twin

Real hardware isn't reachable at all times. This is a way to test most of
the stack without it. Started 2026-09-20, robot is elsewhere, so nothing
below is checked against the real STS3215 bus, only reasoned from source
and docs, same discipline as the rest of ARCHITECTURE.md.

## What this validates, and what it doesn't

Validates: Nav2, `diff_drive_controller` kinematics and odometry integration,
the safety supervisor, arm trajectories, our actual launch files and configs,
against real (simulated) physical response instead of a compile-time check.

Does not validate: R1, the STS3215 encoder wrap bug (ARCHITECTURE §4.1).
That's a specific real-servo register behavior, physics sim has no reason to
reproduce it. Isaac Sim wraps continuous joint position too, but on its own
different, ±2π convention, see below. Real hardware bench test is still the
only way to close out R1.

## Setup, done so far

- Isaac Sim 6.1.0, pip install, own venv (`~/venvs/isaacsim`, Python 3.12
  matches Isaac Sim 6.x's own requirement and the system default, no need
  for a separate Python version):
  ```bash
  python3 -m venv ~/venvs/isaacsim
  source ~/venvs/isaacsim/bin/activate
  pip install "isaacsim[all,extscache]" --extra-index-url https://pypi.nvidia.com
  ```
  There's also an older 4.5.0-rc.36 Omniverse-launcher-style bundle at
  `~/isaacsim` from Jan 2025, unrelated, too old for the MCP tool below
  (needs 5.1+), not used here.

- Omniverse MCP (colleague's tool, github.com/serionist/omniverse-mcp),
  cloned to `~/Developer/omniverse-mcp`, own venv, installed with `pip install .`.
  Registered in `~/Developer/XLeRobot/.mcp.json` (the session's actual root).
  **Needs a Claude Code session reload to actually become callable here** -
  registering the server doesn't hot-load it into a running session.

- Its AI guide is copied into this repo's own `CLAUDE.md`, per the tool's
  own setup instructions.

- Still to do, one-time, needs the Isaac Sim GUI or its CLI flags (not done
  yet as of this writing): add `~/Developer/omniverse-mcp/extension` as an
  extension search path, enable + autoload "Omniverse MCP Bridge" under
  Window > Extensions > THIRD PARTY > User, restart the app once.

## The integration point: no first-party ros2_control plugin for Isaac Sim

Checked, NVIDIA doesn't ship one. The real pattern, confirmed from the
ros-controls org itself, is
[`topic_based_hardware_interfaces`](https://github.com/ros-controls/topic_based_hardware_interfaces)
(Apache-2.0, Jazzy supported), specifically
`joint_state_topic_hardware_interface/JointStateTopicSystem`, not the
`cm_topic_hardware_component` package (that one's for replaying
`controller_manager`'s own introspection topic from a bag file, wrong tool,
checked and discarded before using it).

`JointStateTopicSystem` is a real `hardware_interface::SystemInterface`
that reads/writes plain `sensor_msgs/JointState` on two topics. Isaac Sim's
ROS2 bridge (OmniGraph nodes) can produce/consume exactly that. So our
*actual* `controller_manager`, `diff_drive_controller`, Nav2 stack runs
unchanged on the host, only the hardware plugin swaps.

Added to `xlerobot.repos`. Xacro side: `xlerobot.ros2_control_sim.xacro`,
selected by `use_sim:=true` (separate from `use_mock_hardware`, different
purpose and different topology, one combined hardware component for all 16
joints instead of the real two-bus split, since sim has no physical bus to
split by).

Its own README documents `sum_wrapped_joint_states`, on here, specifically
because "Isaac Sim only reports joint states from 2*pi to -2*pi" - i.e. it
wraps too, just on its own convention, not the STS3215's 0-4095 step
convention. This package's own unwrap implementation is worth reading before
finishing the `sts_hardware_interface` R1 patch, real prior art for the
same class of problem.

Topic names: `/isaac_joint_commands`, `/isaac_joint_states`. Isaac Sim's
side needs to match these exactly.

## Both sides, same ROS_DOMAIN_ID

Isaac Sim on Ubuntu 24.04 auto-uses its own bundled internal ROS 2 Jazzy
libraries when nothing else is sourced - confirmed from NVIDIA's own docs.
That matches our host distro exactly, so no library-path juggling, no
rebuilding our workspace against Isaac Sim's Python. DDS handles the wire
transport regardless of which Jazzy build each side links against, standard
messages (`JointState`, `Twist`, `Odometry`) are just IDL-compatible either
way. Don't source the host's `/opt/ros/jazzy/setup.bash` in the same shell
that launches Isaac Sim - let it use its own internal libraries.

## Not yet confirmed, needs Isaac Sim actually running to check

- Exact OmniGraph node names for the ROS2 Subscribe/Publish JointState
  nodes and how to wire an Articulation Controller to them in Isaac Sim
  6.1 specifically - didn't find a reliable current source, not guessing
  at exact GUI steps here. There's also a reported crash adding ROS2 Joint
  State Action Graph nodes on 6.0.0 + Newton physics on the NVIDIA forums;
  we're on 6.1.0 which may have it fixed, but worth watching for.
- Whether our real xacro (`xlerobot.urdf.xacro`, rendered with
  `use_ros2_control:=false` for the sim import, since Isaac Sim's URDF
  importer has no use for our `<ros2_control>` block at all) imports
  cleanly - real wheel joints, but still-placeholder box geometry for
  most links, should still be enough for physics testing even before the
  real meshes are wired in.
- Whether `topic_based_hardware_interfaces` actually builds clean against
  our workspace, same rigor as the `sts_hardware_interface` build/test
  pass, not done yet.

Resolve these once Isaac Sim is confirmed running with the bridge
extension loaded, either through the MCP tool (after a session reload) or
a plain standalone Python script using Isaac Sim's own API as a fallback -
that path doesn't depend on the MCP tool being available at all, and is
the more reproducible one to check into the repo regardless.
