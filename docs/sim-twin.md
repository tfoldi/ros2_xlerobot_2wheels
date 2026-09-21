# Isaac Sim simulation support

An Isaac Sim scene of the base + head, wired to the real `ros2_control`
stack over ROS 2 topics, for development and testing without hardware
attached.

## Scope

Supports: Nav2, `diff_drive_controller` kinematics and odometry
integration, the safety supervisor, and launch/configuration testing
against simulated physical response.

Does not support: reproducing R1, the STS3215 encoder wrap issue
(ARCHITECTURE §4.1). That is real-servo register behavior; a physics
simulation has no reason to reproduce it. Isaac Sim wraps continuous joint
position on its own convention (see `sum_wrapped_joint_states` below),
which is a different problem from R1 and does not substitute for hardware
validation of it.

Arms are not included in the current scene (still placeholder geometry,
phase 3 not started). `xlerobot.ros2_control_sim.xacro` supports
`with_arms:=true`; extending `sim/build_isaac_scene.py`'s joint list is
the only change needed to include them.

## Requirements

- Isaac Sim 6.1 or later (pip package, `isaacsim[all,extscache]`), own
  Python 3.12 virtual environment.
- ROS 2 Jazzy on the host.
- [`joint_state_topic_hardware_interface`](https://github.com/ros-controls/topic_based_hardware_interfaces)
  (see `xlerobot.repos`).

## Installation

```bash
python3 -m venv ~/venvs/isaacsim
source ~/venvs/isaacsim/bin/activate
pip install "isaacsim[all,extscache]" --extra-index-url https://pypi.nvidia.com
```

Isaac Sim's ROS 2 bridge extension requires `ROS_DISTRO`,
`RMW_IMPLEMENTATION`, and `LD_LIBRARY_PATH` to be set in the environment
before the process starts; setting them after launch does not take effect.

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
```

`OMNI_KIT_ACCEPT_EULA=YES` is required for non-interactive use and is set
in the venv's `activate` script.

Isaac Sim on Ubuntu 24.04 uses its own bundled ROS 2 Jazzy libraries by
default when nothing else is sourced. This matches the host distro, so no
library-path changes or workspace rebuilds against Isaac Sim's Python are
needed. Do not source `/opt/ros/jazzy/setup.bash` in the same shell that
launches Isaac Sim; let it use its own internal libraries. DDS handles
transport regardless of which Jazzy build either side links against.

## Integration approach

There is no first-party `ros2_control` hardware interface for Isaac Sim.
The supported pattern is
[`topic_based_hardware_interfaces`](https://github.com/ros-controls/topic_based_hardware_interfaces)
(Apache-2.0, official `ros-controls` package, Jazzy supported), specifically
`joint_state_topic_hardware_interface/JointStateTopicSystem` — a
`hardware_interface::SystemInterface` that reads and writes plain
`sensor_msgs/JointState` on two topics, matching Isaac Sim's ROS 2 bridge
OmniGraph nodes.

`cm_topic_hardware_component` (same upstream repo) is a different tool for
replaying `controller_manager`'s own introspection topic from a bag file
and is not used here.

With this hardware interface, the same `controller_manager`,
`diff_drive_controller`, and Nav2 configuration run unchanged; only the
hardware plugin differs from the physical-robot configuration.

Selected by `use_sim:=true` in `xlerobot.urdf.xacro`, which loads
`xlerobot.ros2_control_sim.xacro` — a single combined hardware component
covering all joints, since simulation has no physical bus to split by
(the real robot's two-bus split, described in ARCHITECTURE §1.1, does not
apply here).

Topic names: `/isaac_joint_commands`, `/isaac_joint_states`. These must
match on both the `ros2_control` xacro and the Isaac Sim Action Graph.

`sum_wrapped_joint_states` is enabled, since Isaac Sim reports continuous
joint position wrapped to a ±2π range. This is unrelated to the STS3215's
0–4095 step wrap (R1); the two use different unwrapping conventions and
are handled independently.

## Scene construction

`sim/build_isaac_scene.py` imports the robot description and builds a ROS 2
bridge Action Graph:

- `ROS2SubscribeJointState` (`/isaac_joint_commands`) → `IsaacArticulationController`
- `IsaacArticulationController` → drives the articulation
- `ROS2PublishJointState` (`/isaac_joint_states`) ← articulation state
- `ROS2PublishClock` (`/clock`)

The articulation controller and joint-state publisher target
`/World/xlerobot/Geometry/base_footprint/base_link`, where the URDF
importer applies `PhysicsArticulationRootAPI` — not the top-level
reference prim, which does not carry the articulation schema. The
importer's output also gates its physics representation behind a
`Physics` USD variant set (`physics`/`physx`/`mujoco`/`none`); the
`physics` variant is selected explicitly, since it is not selected by
default when the asset is referenced. The script asserts
`prim.HasAPI(UsdPhysics.ArticulationRootAPI)` after setup and fails
loudly if the articulation root is not present.

Ground-truth odometry (published directly from the chassis pose, as
opposed to being derived from joint states) is intentionally not wired
into the scene: it would duplicate what `diff_drive_controller` is meant
to produce, and mixing the two would make it unclear which source is
authoritative.

## Layout

| Path | Purpose |
|---|---|
| `sim/xlerobot_import.urdf` | Generated. Plain URDF for the Isaac Sim importer, rendered from the xacro with `use_ros2_control:=false` and `with_arms:=false`. Gitignored. |
| `sim/build_isaac_scene.py` | Imports the URDF, builds the ROS 2 bridge graph, saves the stage. |
| `sim/output/` | Generated. `xlerobot_twin.usd` is the scene to open in Isaac Sim. Gitignored. |
| `sim/launch_gui.py` | Opens a given USD stage in the interactive Isaac Sim GUI (not headless). Defaults to `sim/output/xlerobot_twin.usd`. |

Regenerate the import URDF after xacro changes:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
xacro src/xlerobot_description/urdf/xlerobot.urdf.xacro \
  use_ros2_control:=false with_arms:=false sensor:=none \
  > sim/xlerobot_import.urdf
```

Then rebuild the scene:

```bash
source ~/venvs/isaacsim/bin/activate
python sim/build_isaac_scene.py
```

Topic names are duplicated between `xlerobot.ros2_control_sim.xacro` and
`build_isaac_scene.py` rather than derived from one source; keep them in
sync manually when either changes.

## Viewing a scene interactively

```bash
source ~/venvs/isaacsim/bin/activate
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
python sim/launch_gui.py [path/to/stage.usd]
```

Requires a display. Opens the given stage, or `sim/output/xlerobot_twin.usd`
by default, and stays open until the window is closed.

The current scene uses placeholder box/cylinder visual geometry, not the
robot's real meshes — `base_2wheels.xacro` and `head_pantilt.xacro` do not
yet reference the mesh files bundled with upstream XLeRobot
(`simulation/Isaac_sim/xlerobot.zip` in the XLeRobot repo). For a visual
reference of the physical robot's actual appearance, that bundle's own
`xlerobot/xlerobot/xlerobot.usd` can be opened with the same script; note
that it is upstream's 0.3.0 three-wheel model (virtual planar base joint,
no real wheel links) and is not wired to the ROS 2 bridge graph described
above.

## Known limitations

- No joint stiffness/damping is specified in the xacro (`base_2wheels.xacro`,
  `head_pantilt.xacro`), so the URDF importer creates PhysX actuators with
  no PD gain parameters. This may produce unrealistic joint response under
  velocity or position commands. Fix by adding `<dynamics>` to the
  affected joints, or by setting `override_joint_stiffness` /
  `override_joint_damping` in `URDFImporterConfig`.
- `ROS2PublishJointState.inputs:targetPrim` is a deprecated input on
  current Isaac Sim releases in favor of a separate `IsaacReadJointState`
  node. It remains functional; migrating is not required but is preferred
  for new work on this graph.
- The scene has not been validated with the simulation running
  continuously against live ROS 2 topics (only a bounded number of
  simulation steps, to confirm the Action Graph evaluates without error).
  Confirming topic content — not just that the graph builds — needs a
  running session with `ros2 topic echo` against `/isaac_joint_states`
  and a command published to `/isaac_joint_commands`.
- `joint_state_topic_hardware_interface` builds against this workspace
  independently of the Isaac Sim side; the two have not been run together.

## Omniverse MCP (optional)

[`serionist/omniverse-mcp`](https://github.com/serionist/omniverse-mcp)
provides interactive scene inspection and scripting against a running
Isaac Sim instance, for iterating on a scene once it exists (not for
constructing it from scratch — `sim/build_isaac_scene.py` is the
reproducible path for that).

Setup:

1. Clone and install in its own virtual environment, then register it as
   an MCP server (see the project's own README for the exact
   `.mcp.json` entry).
2. Add its `extension/` directory as an Isaac Sim extension search path,
   and enable + autoload "Omniverse MCP Bridge" under Window > Extensions
   in the Isaac Sim UI. This is a one-time step; the extension autoloads
   on subsequent launches.
3. A running MCP client needs to reload its server configuration to pick
   up a newly registered server.

## TODO

- Validate topic content with the simulation running continuously.
- Address the joint stiffness/damping gap.
- Reference the real meshes from upstream's Isaac Sim bundle in
  `base_2wheels.xacro` and `head_pantilt.xacro` in place of the current
  placeholder geometry.
- Extend the scene to include the arms once phase 3 starts.
- Migrate off the deprecated `ROS2PublishJointState.inputs:targetPrim`.
