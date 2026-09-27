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
- `torque_off:=true` (the default) declares the arm and head joints with state interfaces
  only, so torque stays off and nothing is written to them. `torque_off:=false` gives them
  position command interfaces, for step 4.
- Not there yet: MoveIt config (MoveIt is not installed), head controllers.

## Steps

1. **Read only.** Bring up the arm and head joints with state interfaces only. The hardware
   interface disables torque on a joint that has no command interface, so the arms can be
   moved by hand while joint states are read. Needs an xacro option for read-only arm joints.
   Check that all twelve arm servos and both head servos answer on their buses, that the wheels
   do not lose cycles while the arms are read, and run the two-instance test that
   `sts_hardware_interface` per-instance service names (upstream PR #22) still owes.
   Done when: 14 joints report at the controller rate on both buses, wheel odometry is
   unchanged.
2. **Calibration.** Two parts, hand-guided, torque off.
   - *Servo calibration, the lerobot way.* `tools/calibrate_arm.py calibrate --arm left|right`
     runs lerobot's SO-101 procedure (arm in the middle of its range, then every joint swept
     through its range). It writes the homing offset and the position limits into servos 1-6 and
     saves `so101_follower/<id>.json` in the calibration directory lerobot uses
     (`$HF_LEROBOT_HOME/calibration`, id `xlerobot_left_arm` / `xlerobot_right_arm`), so
     `lerobot-calibrate` and the teleoperation tools see the same arms. It needs the bus to
     itself, so the ROS stack is stopped for it, and it leaves the head and wheels alone.
     `tools/calibrate_arm.py status` is the read-only check of what the servos hold.
   - *Head.* `calibrate_arm.py calibrate --arm head` does the same for pan and tilt (servos 7, 8
     on bus 1) with the head looking straight ahead and level as the middle, file
     `xlerobot_head.json` (lerobot has no robot type for the head, so it sits in
     `robots/xlerobot_head/`). The model's head zero is that same pose, so no extra offset.
   - *Model zero and direction.* The calibrated middle (step 2047) is not the model's zero
     (arm stretched straight), and a servo may turn the other way from the model's joint.
     `arms.joints` in `robot_dimensions.yaml` holds both per joint: `zero_step`, the step
     where the model reads 0, and `direction`. `xlerobot.ros2_control.xacro` passes the
     zero as `position_center_steps`, and where the direction differs from the driver's,
     `so101_arm.xacro` reverses the joint axis and mirrors the limits. The values are
     read off the arms held stretched straight (`status`, per arm). The gripper's zero comes from
     the closed end stop of the calibration instead: the model's jaws are closed at its lower limit. The directions are checked by moving each joint by hand in
     Rerun. The head is the same with
     `head.pan_zero_step` and `head.pan_direction` (and tilt).
     The interface has a `one_key_calibration` service that changes the servo's stored
     midpoint, it is a maintenance tool and not used here.
   - *Backup.* `docs/calibration-examples/` has the files of this robot as examples. The
     calibration itself is not part of the repo, every robot has its own.
   Done when: moving a joint by hand moves the same joint the same way in Rerun, and both
   arms read zero within about 2 degrees in the zero pose.
3. **Model in Rerun.** With real joint states the arms show in the Rerun view as the robot
   sits, folded at rest.
4. **Controllers.** `joint_trajectory_controller` for each arm (five joints) and for each gripper,
   torque on. They are spawned with `with_arms:=true torque_off:=false` (always in the sim), the
   read-only bringup has none.
   - The driver brings a commanded position joint up holding where it is: it reads the position,
     sets the command and the servo's goal to it, and only then enables torque, so nothing jumps.
     It refuses to start if a joint is more than 0.15 rad outside its limits (wrong calibration).
     Releasing an e-stop does the same, an arm that sagged is held where it is now.
   - Speed: `max_velocity` per joint in `robot_dimensions.yaml` (0.5 rad/s, the planning limit),
     and the driver's `position_rate_limit` (1.25 x that) caps how fast the commanded position
     may move whatever a trajectory asks. A trajectory faster than the cap lags and the
     controller aborts it on the path tolerance (0.25 rad). The servos' `proportional_vel_max`
     also caps large jumps. The driver clamps commands to the joint limits.
   - E-stop: `tools/estop.py` calls the e-stop service of every bus (arms, head, wheels), the
     servos lose torque and the arms go limp. `--release` holds them where they are.
   - Testing without the robot: `use_mock_hardware:=true`, the Isaac Sim twin, or
     `tools/fake_sts_bus.py`, which runs the real driver against simulated servos and logs every
     torque and goal write.
   Live test order: right arm first, a joint at a time, small moves from the folded rest pose,
   the arm held by hand.
   Done when: a small trajectory on each arm tracks within a few degrees.
5. **MoveIt 2.** Install it, generate the config from the URDF, self-collision matrix
   including the mast, the head and the cart, planning groups per arm and for both.
   Done when: MoveIt plans and executes a collision-free move on each arm from RViz.
6. **Head pan/tilt.** Controller on bus 1 (calibrated in step 2), look-at command for the head
   camera.

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
