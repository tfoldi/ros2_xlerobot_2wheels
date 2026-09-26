# Arms bring-up plan

Phase 3 of [ARCHITECTURE](../ARCHITECTURE.md#8-phases): both SO-101 arms and the head
pan/tilt on `ros2_control`, planned with MoveIt 2. Each step is checked before the next
one starts.

## What exists

- The model: `so101_arm.xacro` and `head_pantilt.xacro`, from the shared upstream URDF, with
  the mounts measured on the robot (`config/robot_dimensions.yaml`). Both arms point forward.
- The hardware side: `xlerobot.ros2_control.xacro` declares six position joints per arm
  (`<prefix>_<joint>_joint`, IDs 1-6, the upstream URDF's limits) and the head (IDs 7, 8).
  The left arm and the head are on bus 1, the right arm and the wheels on bus 2.
- `bringup.launch.py with_arms:=true` puts the arms in the model and their joints in
  `ros2_control`. It is off by default, so the arms are not in the model or in Rerun otherwise.
- Not there yet: arm controllers, MoveIt config (MoveIt is not installed), joint zero
  offsets, a gripper controller, head controllers.

## Steps

1. **Read only.** Bring up the arm and head joints with state interfaces only. The hardware
   interface disables torque on a joint that has no command interface, so the arms can be
   moved by hand while joint states are read. Needs an xacro option for read-only arm joints.
   Check that all twelve arm servos and both head servos answer on their buses, that the wheels
   do not lose cycles while the arms are read, and run the two-instance test that
   `sts_hardware_interface` per-instance service names (upstream PR #22) still owes.
   Done when: 14 joints report at the controller rate on both buses, wheel odometry is
   unchanged.
2. **Calibration.** Find each joint's zero and direction so the model's zero pose (arm
   stretched straight) matches the real arm. Hand-guided: put each arm in the zero pose, read
   the raw positions, store the offsets in `robot_dimensions.yaml` next to the limits (and
   check the limits against the real end stops). The interface has a `one_key_calibration`
   service that changes the servo's stored midpoint, it is a maintenance tool, not part of this.
   Done when: moving a joint by hand moves the same joint the same way in Rerun, and both
   arms read zero within about 2 degrees in the zero pose.
3. **Model in Rerun.** With real joint states the arms show in the Rerun view as the robot
   sits, folded at rest.
4. **Controllers.** `joint_trajectory_controller` for each arm, a gripper controller,
   torque on. Start with small moves, low velocity and a joint at a time, a per-joint
   velocity limit in `robot_dimensions.yaml`, and the safety supervisor's e-stop covering the
   arm buses.
   Done when: a small trajectory on each arm tracks within a few degrees.
5. **MoveIt 2.** Install it, generate the config from the URDF, self-collision matrix
   including the mast, the head and the cart, planning groups per arm and for both.
   Done when: MoveIt plans and executes a collision-free move on each arm from RViz.
6. **Head pan/tilt.** Controller on bus 1, limits checked against the mechanical stops, look-at
   command for the head camera.

## Risks

- **Bus 2 carries the right arm and the wheels.** Arm traffic competes with wheel traffic,
  see ARCHITECTURE §4.3. Watch wheel odometry jitter while the right arm moves, and if the
  rate has to drop, drop the arms' first.
- **Wrong zero or direction moves an arm into the cart or the head.** Hence torque off
  until step 2 is done, then small moves.
- **Servo power.** Two arms and the head on a battery robot: check the supply sag with
  both arms moving.
- **Rest pose.** The arms rest folded, the model's zero pose is stretched, define the folded
  pose as a named MoveIt state so the arms can go back to it safely.
