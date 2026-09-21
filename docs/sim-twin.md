# Isaac Sim digital twin

Real hardware isn't reachable at all times. This is a way to test most of
the stack without it. Started 2026-09-20, robot is elsewhere. Base + head
scene is built, confirmed working end to end by actually running it, not
just reasoning about it.

**Current status: scene builds, imports, and wires up clean.
`sim/build_isaac_scene.py` produces a saved, working
`sim/output/xlerobot_twin.usd`. Not yet done: continuous Play + watching
real ROS topics from outside the sim (the script only steps 5 frames to
prove the graph evaluates without crashing, that's not the same as
confirming topic content is correct).**

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

- Still to do, one-time, needs the Isaac Sim GUI (checked: the extension
  manager's persistent-autoload state lives in a compiled `omni.ext`
  binding, not readable/scriptable Python source, not worth chasing
  further tonight): add `~/Developer/omniverse-mcp/extension` as an
  extension search path, enable + autoload "Omniverse MCP Bridge" under
  Window > Extensions > THIRD PARTY > User, restart the app once. After
  that it's automatic on every future launch.

- **Launching the pip-installed Isaac Sim needs `ROS_DISTRO`,
  `RMW_IMPLEMENTATION`, and `LD_LIBRARY_PATH` set in the shell *before*
  `python` starts**, found this out by actually hitting the failure: the
  bridge extension logs "ROS2 Bridge startup failed" and silently stays
  off if these aren't set first, and setting them after the process is
  already running does not fix it (same "fix the parent shell, then
  relaunch" rule Isaac Sim's own shipped skill documents for its
  launcher scripts, applies just as much when launching the pip package
  directly with plain `python` rather than `isaac-sim.sh`):
  ```bash
  source ~/venvs/isaacsim/bin/activate
  export ROS_DISTRO=jazzy
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
  python sim/build_isaac_scene.py
  ```
  (`OMNI_KIT_ACCEPT_EULA=YES` also needs setting once for a non-interactive
  install/run; already baked into the venv's own `activate` script here.)

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

## Confirmed by actually running it (2026-09-20 overnight)

Went with a standalone Python script (`sim/build_isaac_scene.py`) over the
MCP tool for the first build: MCP needs Isaac Sim already running with its
bridge extension loaded and the extension isn't registered yet (one-time
GUI step, above), and a checked-in script is the more reproducible artifact
anyway - MCP is for interactive iteration once the base scene exists, not
how the base scene should get built in the first place.

Found Isaac Sim 6's own shipped first-party reference for this, not from
search results or older doc snapshots: `isaacsim/skills/isaac-sim-ros2-bridge/`
inside the pip package itself, an actual Claude/agent skill NVIDIA ships
with Isaac Sim 6 (`isaacsim/skills/isaac-sim-ros2-bridge/scripts/multi_robot_namespacing.py`
is a complete, working Nav2-fleet reference using `og.Controller.edit` with
the exact current node names). Used its node-creation pattern; did NOT
reuse its odometry/TF publishing, that publishes ground truth from the
chassis pose directly, which is not what we want as our primary odom
source - `diff_drive_controller` computing odometry FROM joint states is
the actual thing under test here.

Three real bugs found and fixed by actually running it, each would have
been a silent "looks wired, does nothing" failure otherwise:

1. **ROS2 bridge extension silently fails to start** without `ROS_DISTRO`
   / `RMW_IMPLEMENTATION` / `LD_LIBRARY_PATH` set before the process
   launches, see the setup section above.
2. **A bonus ground-truth-odom node's attribute name was wrong**
   (`ROS2PublishOdometry.inputs:chassisPrim` doesn't exist on this Isaac
   Sim 6.1 build) - `OmniGraphError`, dropped the node rather than guess
   again at ~100s/relaunch, it was never essential.
3. **The real `PhysicsArticulationRootAPI` doesn't land where you'd
   expect.** Referencing the imported USD into a wrapper Xform
   (`/World/xlerobot`) does not promote the articulation root schema to
   that wrapper - PhysX's tensor plugin correctly reported no articulation
   there. The importer's own USD also gates its physics representation
   behind a `Physics` **variant set** (`physics`/`physx`/`mujoco`/`none`),
   which a plain reference doesn't select either - first fix (selecting
   the `physics` variant) still wasn't enough on its own. Root cause,
   confirmed by reading the importer's own generated
   `payloads/Physics/physics.usda`: `PhysicsArticulationRootAPI` is
   applied at `/World/xlerobot/Geometry/base_footprint/base_link` (matches
   our own `base_footprint`/`base_link` naming), not at the wrapper.
   Pointing `ArticulationController` and `PublishJointState`'s
   `targetPrim` there fixed it - verified with an explicit
   `prim.HasAPI(UsdPhysics.ArticulationRootAPI)` check in the script
   itself, not just "no error this time," after two wrong guesses already
   looked plausible enough to almost trust.

Also confirmed, not fixed yet:

- **No joint stiffness/damping specified anywhere** in our xacro (no
  `<dynamics damping=... friction=.../>` on the wheel or head joints), so
  the importer creates PhysX actuators with no PD gain parameters at all
  (`"Stiffness and damping not available ... actuator will be created
  without gain parameters"`, once per joint). Untuned joints in physics
  sim can be mushy or unrealistic under velocity/position commands.
  Fix is either add `<dynamics>` to the joints in
  `base_2wheels.xacro`/`head_pantilt.xacro`, or set
  `override_joint_stiffness` / `override_joint_damping` in
  `URDFImporterConfig` - not done, first thing to check if commanded
  motion looks wrong once this actually gets driven.
- `ROS2PublishJointState.inputs:targetPrim` is flagged deprecated on this
  build ("Connect an Isaac Read Joint State node and use its outputs
  instead"). Still functional, not broken, just not the currently
  recommended pattern. Left as is for now, worth fixing when touching this
  graph again.
- Never watched real ROS topics from outside the sim (`ros2 topic echo
  /isaac_joint_states`, publishing to `/isaac_joint_commands` and watching
  the sim respond). The script only proves the graph builds and steps
  without crashing, not that the topic content is right. Do this before
  trusting the twin for anything beyond "does it explode."
- `topic_based_hardware_interfaces` (the host-side half of this) builds
  clean against our workspace - confirmed, see the commit that added it -
  but the two haven't been run against each other yet, only each
  independently.

## Layout

- `sim/xlerobot_import.urdf` - plain URDF for Isaac Sim's importer,
  rendered from our xacro with `use_ros2_control:=false` (Isaac Sim has no
  use for that block) and `with_arms:=false` (still stub geometry, no
  reason to simulate them yet). Regenerate after any xacro change:
  ```bash
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  xacro src/xlerobot_description/urdf/xlerobot.urdf.xacro \
    use_ros2_control:=false with_arms:=false sensor:=none \
    > sim/xlerobot_import.urdf
  ```
- `sim/build_isaac_scene.py` - imports it, wires the ROS2 bridge graph,
  saves the stage. Re-run after touching either the URDF above or the
  topic names in `xlerobot.ros2_control_sim.xacro` (keep them in sync by
  hand, they're duplicated between the two files on purpose rather than
  having the Python script parse xacro).
- `sim/output/` - generated, gitignored. `xlerobot_twin.usd` is the
  scene to open in Isaac Sim.

## Next session

1. Open `sim/output/xlerobot_twin.usd` in Isaac Sim (GUI this time), press
   Play, confirm the robot doesn't fall through the ground plane and
   doesn't explode on activation (a common sign of bad joint drive gains,
   see the stiffness/damping gap above).
2. Load the Omniverse MCP bridge extension (one-time GUI step, above),
   then a Claude Code session reload should get `omniverse`/`context7`
   connected for interactive work.
3. With the sim playing, from a normal ROS 2 Jazzy shell (not the Isaac
   Sim venv): `ros2 topic echo /isaac_joint_states`, confirm real data
   flows, then bring up `xlerobot_bringup` with `use_sim:=true` and watch
   `/cmd_vel` actually move the simulated wheels.
4. Only then does this become a real test bed for Nav2 / the safety
   supervisor, not before.
