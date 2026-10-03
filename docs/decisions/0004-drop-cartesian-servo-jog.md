# ADR-0004: Drop MoveIt Servo's Cartesian (TWIST) jogging, keep JOINT_JOG only

- Status: Accepted
- Date: 2026-10-03

## Context

`arm_servo_teleop` drove each SO-101 arm (5 non-gripper DOF: shoulder_pan, shoulder_lift,
elbow_flex, wrist_flex, wrist_roll) through `moveit_servo`'s `servo_node` in two modes:
`JOINT_JOG` (each stick axis maps straight to one joint's velocity) and `TWIST` (each stick
axis maps to Cartesian linear velocity, IK solved by `pick_ik`, position-only).

On real hardware, `TWIST` reliably refused to move the arm, reporting
`DECELERATE_FOR_SINGULARITY` / `HALT_FOR_SINGULARITY` on `servo_*/status` even for ordinary,
non-extreme stick input. `JOINT_JOG` worked correctly throughout.

Root cause, confirmed by reading `moveit_servo` 2.12.4 source
(`utils/common.cpp`, `utils/command.cpp`) and by testing on hardware:

- Servo's singularity gate takes the SVD of the raw 6x5 Jacobian (linear rows in metres,
  angular rows in radians, unweighted) and compares the condition number against
  `lower_singularity_threshold` / `hard_stop_singularity_threshold`. This runs regardless of
  whether the IK-solved path or the Jacobian pseudo-inverse path produced the motion.
- First hypothesis: the default thresholds (10.0 / 30.0), inherited from the Panda/UR example
  configs, are tuned for 6-7 DOF arms with ~0.5-1m links. This arm's much smaller links
  (~0.11-0.13m) could plausibly push the condition number into the gate's range even in normal
  poses, as a unit-scaling artifact rather than a real singularity. Two open, unanswered
  upstream issues report the same symptom on other small/low-DOF arms (moveit2 #3411,
  moveit #3350).
- Tested by raising both thresholds to 10000.0 / 100000.0 (confirmed active on the running node
  via `ros2 param get`) and re-recording a rosbag of the same test. `DECELERATE_FOR_SINGULARITY`
  still fired. Checked the arm's actual pose during the test: `elbow_flex` was at -3.14 to
  -3.16 rad (essentially -π) for the entire window - the arm was genuinely fully extended, a
  real kinematic singularity where the Jacobian loses rank, not a scaling artifact. No finite
  threshold fixes a true rank loss.
- That pose is this project's own saved `rest` position for both arms
  (`src/xlerobot_bringup/config/saved_positions.yaml`: `elbow_flex: -3.0`,
  `shoulder_lift: 2.6`) - the arm's normal parked/resting configuration sits right at this
  singularity, so testing Cartesian jogging starting from rest (the obvious thing to do) hits
  it immediately.

Considered NVIDIA Isaac ROS cuMotion (cuRobo-based) as a replacement:

- cuMotion is a trajectory/motion planner (a `move_group` planner plugin plus IK/motion-gen
  actions), not a reactive jogging controller - the packaged stack has no continuous
  teleop-style servo mode to replace Servo's role here.
- cuRobo's IK is optimization-based like `pick_ik` already is, so it doesn't invert a Jacobian
  and wouldn't blow up the same way - but at a true rank-loss pose it can instead jump to a
  discontinuous configuration, which is worse behavior for live teleop than Servo's clean stop.
- Orin support needs JetPack 7.2 (this project is on an older JetPack - see ADR-0002), plus a
  custom collision-sphere XRDF for this arm. Multi-day integration risk for a problem it
  wouldn't actually solve.

## Decision

Drop `TWIST`/Cartesian jogging entirely. `arm_servo_teleop` now only ever drives `JOINT_JOG`
(`mode_button` and the trigger-driven Z axis are removed, along with the `TwistStamped`
publishers). `servo.yaml` and `servo.launch.py` keep the Cartesian-related fields and topic
(Servo expects them at startup regardless of command type) but they're unused.

A real fix would mean either steering clear of near-singular poses (e.g. a dedicated
non-rest "ready" pose, or refusing TWIST near a singularity and falling back to JOINT_JOG) or
weighting/damping the Jacobian ourselves - both add real complexity for a mode whose only
advantage over JOINT_JOG is a more intuitive stick mapping, on a robot where JOINT_JOG already
works reliably.

## Consequences

- `arm_servo_teleop` is simpler: no mode state, no mode toggle, no per-arm TWIST publisher.
- Operators get one jogging mode, not two - direct joint control only, no Cartesian stick
  mapping.
- Future Cartesian-style input (e.g. a SpaceMouse or a small dedicated teleop rig, sending
  pose deltas through `move_group`'s one-shot plan+execute path instead of Servo's continuous
  jogging) is not ruled out, just not pursued now - not the priority.
