# ADR-0001: ros2_control talks to the servo bus directly, lerobot is not in the control path

- Status: Accepted
- Date: 2026-09-20

## Context

Upstream XLeRobot controls the robot from Python via `lerobot`. Shipped topology is a ZMQ
host on the robot (`xlerobot_2wheels_host.py`) talking to a client, commands on port 5555,
observations on 5556.

Two ways to put ROS 2 on top:

**A. Bridge.** Keep the lerobot host, write a ROS 2 node that wraps its ZMQ sockets,
translate `/cmd_vel` into `x.vel`/`theta.vel` and observations into `/joint_states` and
`/odom`. This is what `robotmindio/cleanroom` does for the related LeKiwi platform.

**B. Native.** A `ros2_control` SystemInterface talks Feetech directly. lerobot is not in
the loop.

## Decision

B. `adityakamath/sts_hardware_interface`, one per serial bus, under a normal
`controller_manager`.

## Why

1. **Rate.** The lerobot host runs at `max_loop_freq_hz = 30`, and the source comment warns
   that going higher causes jitter. Nav2's controller server wants 20 Hz minimum, prefers 50.
   30 Hz shared with observation serialisation and camera capture is thin.

2. **Odometry.** The ZMQ observation only has `x.vel` and `theta.vel`, body velocities already
   derived from `Present_Velocity`. Integrating a velocity report to get pose is worse than
   differencing wheel position, and `Present_Velocity` on STS servos is noisiest at low
   speed, exactly where docking and goal approach happen. Native keeps access to raw
   per-joint position.

3. **Ecosystem.** `diff_drive_controller`, `joint_trajectory_controller`, MoveIt 2, cuMotion
   all assume `ros2_control`. Option A means reimplementing odometry integration, TF, command
   timeouts and trajectory interpolation by hand.

4. **Dependency direction.** With B, lerobot becomes an optional client of ROS topics instead
   of a required server underneath everything. Matches huggingface/lerobot#4368, a generic
   config driven plugin over standard `JointState` / `JointTrajectory` / `Twist` / `Odometry`
   instead of per-robot bridges.

5. **Hardware churn.** Upstream says 0.4.0 is the last Feetech/SO101 chassis version,
   brushless is next. Behind `ros2_control`, that swap replaces one plugin, not the stack.

## Consequences

Costs accepted:

- Upstream teleop scripts, VR interface and dataset recording examples don't work against
  this stack unchanged. Bringing them back is phase 5, not a side effect.
- Hard dependency on a third party hardware interface, with two things not yet verified:
  multi-bus instantiation, multi-turn wheel position. Both are week 1 tests with fallbacks
  identified, see ARCHITECTURE §7 R1 and R2.
- No lerobot policy inference until phase 5.

Gained: standard odometry, standard controllers, MoveIt and Isaac compatibility, and
per-joint temperature/current telemetry for a real safety supervisor.

## Revisit if

`sts_hardware_interface` fails both the multi-bus test and the `feetech_ros2_driver`
fallback, or a working lerobot policy is needed before phase 2 is done. Then option A is a
legitimate stopgap for the base only.
