# Architecture

ROS 2 Jazzy stack for XLeRobot 0.4.0, the servo two-wheel version.

This repo does not fork [XLeRobot](https://github.com/Vector-Wangel/XLeRobot). It reuses it:
meshes and dimensions from the CAD, motor IDs and kinematics from the `lerobot` plugin.
Upstream stays the hardware and imitation learning project. This is the ROS 2 / Isaac
autonomy layer next to it.

---

## 1. Hardware

All of this was read out of the upstream sources, not assumed. Some of the numbers look
wrong and I say so where they do.

### 1.1 Motors and buses

16 joints, all Feetech STS3215, on two half-duplex TTL buses. The split is per cable run,
not per subsystem, and that is the main constraint on the whole design.

| Bus | Port (upstream default) | ID | Joint | Mode |
|-----|-------------------------|----|-------|------|
| 1 | `/dev/ttyACM0` | 1 | `left_arm_shoulder_pan` | position |
| 1 | | 2 | `left_arm_shoulder_lift` | position |
| 1 | | 3 | `left_arm_elbow_flex` | position |
| 1 | | 4 | `left_arm_wrist_flex` | position |
| 1 | | 5 | `left_arm_wrist_roll` | position |
| 1 | | 6 | `left_arm_gripper` | position |
| 1 | | 7 | `head_pan` (`head_motor_1`) | position |
| 1 | | 8 | `head_tilt` (`head_motor_2`) | position |
| 2 | `/dev/ttyACM1` | 1 | `right_arm_shoulder_pan` | position |
| 2 | | 2 | `right_arm_shoulder_lift` | position |
| 2 | | 3 | `right_arm_elbow_flex` | position |
| 2 | | 4 | `right_arm_wrist_flex` | position |
| 2 | | 5 | `right_arm_wrist_roll` | position |
| 2 | | 6 | `right_arm_gripper` | position |
| 2 | | **9** | `left_wheel` | **velocity** |
| 2 | | **10** | `right_wheel` | **velocity** |

From `XLeRobot/software/src/robots/xlerobot_2wheels/xlerobot_2wheels.py`.

Two things follow from this:

1. Bus 2 has to run mixed modes at the same time, six position servos and two velocity
   servos on one UART. Any driver that assumes one mode per bus is useless here.
2. The wheels share a bus with an arm. Same half-duplex line, so arm traffic and wheel
   traffic compete. Base latency is tied to arm latency. Better to budget the bus up front
   (§4.3) than to find it during a Nav2 run.

### 1.2 Wheel signs

Upstream's model:

```
v_left  = (v - ω·L/2) / r          left raw command is negated
v_right = (v + ω·L/2) / r
```

The negation is physical, not a frame choice. The left motor is mounted mirrored. Upstream's
comment says without it `x.vel` turns the robot and `theta.vel` drives it forward.

Here the sign goes in the `ros2_control` joint config, once. The URDF axis stays honest and
`diff_drive_controller` sees two wheels that both spin positive forward. Don't apply it twice.

### 1.3 Raw units

```
steps_per_deg = 4096 / 360
raw = round(deg_per_sec * steps_per_deg)      clamped to int16
```

Upstream clamps at `max_raw = 3000` ticks/s, so 264 °/s, 4.6 rad/s. With the wheel radius
that gives roughly **0.23 to 0.29 m/s** depending on which radius is right (§1.4). Nav2 and
the velocity smoother must stay under this.

### 1.4 Numbers that are probably wrong

`config_xlerobot_2wheels.py` ships:

```python
wheel_radius: float = 0.05   # m
wheelbase:    float = 0.25   # m
```

The first one looks like a placeholder and the 0.4.0 build notes contradict it. The dual
wheel assembly page says **5 inch universal walker wheels**, so 127 mm diameter, **r ≈ 0.0635**.
If 0.05 is used, every reported distance is off by about **27%**. SLAM, Nav2 goal tolerances
and anything learned from `x.vel` will be consistently wrong and it won't be obvious.

`wheelbase` is nominal too. Note the name: the code uses it as **track width**, the lateral
distance between the driven wheels, which is what `diff_drive_controller` calls
`wheel_separation`. Not wheelbase in the car sense.

So both are calibration results, not constants. They live in
`xlerobot_description/config/robot_dimensions.yaml` with a procedure in §6.

### 1.5 Chassis

IKEA RÅSKOG cart, two driven wheels and a caster. The caster has no encoder. In the URDF it
is a fixed link with a low friction sphere, not a steerable joint, because a joint would only
publish invented numbers.

The thing is tall and top heavy, two 6-DOF arms and a head above a narrow track. CoM height
is a real limit on acceleration.

---

## 2. Why this shape

### 2.1 Stack

```
                      ┌─────────────────────────────────────────────┐
                      │  Nav2 (bt_navigator, planner, controller)    │
   autonomy           │  slam_toolbox  |  isaac_ros_visual_slam      │
                      │  nvblox (optional, costmap layer)            │
                      └───────────────┬─────────────────────────────┘
                                      │ /cmd_vel (TwistStamped)
                      ┌───────────────▼─────────────────────────────┐
                      │  ros2_control controller_manager             │
   control            │   diff_drive_controller  → /odom + odom→base │
                      │   joint_trajectory_controller ×2 (arms)      │
                      │   head position controller                   │
                      │   joint_state_broadcaster                    │
                      └───────┬──────────────────────┬──────────────┘
                              │                      │
                  ┌───────────▼──────────┐ ┌─────────▼────────────┐
   hardware       │ sts_hardware_interface│ │ sts_hardware_interface│
                  │   port=/dev/xle-bus1  │ │   port=/dev/xle-bus2  │
                  │   8 joints, position  │ │   6 pos + 2 velocity  │
                  └───────────┬──────────┘ └─────────┬────────────┘
                              │ Feetech TTL          │ Feetech TTL
                         ┌────▼─────┐           ┌────▼─────┐
                         │ IDs 1-8  │           │ IDs 1-6, │
                         │          │           │   9, 10  │
                         └──────────┘           └──────────┘
```

### 2.2 The main decision

`ros2_control` talks to the servo bus directly. `lerobot` is not in the control path.

The other option was wrapping upstream's `xlerobot_2wheels_host` ZMQ server in a ROS node,
like the LeKiwi `cleanroom` stack does. I went the other way, for these reasons:

1. **Rate.** The upstream host runs at `max_loop_freq_hz = 30` and the source warns that
   raising it causes jitter. Nav2's controller server wants 20 Hz minimum and is happier at
   50. 30 Hz shared with observation serialisation is not much to build on.
2. **Odometry.** The ZMQ path only gives `x.vel` and `theta.vel`, already integrated from
   `Present_Velocity`. Integrating a velocity report is worse than differencing wheel
   positions, and `Present_Velocity` is noisiest at low speed, which is where docking and
   goal approach happen. See §4.1.
3. **Ecosystem.** `diff_drive_controller`, `joint_trajectory_controller`, MoveIt 2 and
   cuMotion all assume `ros2_control`. Avoiding it means writing odometry, TF, command
   timeouts and trajectory interpolation by hand.
4. **Direction.** This way `lerobot` becomes an optional client of ROS topics instead of a
   server under everything. Policies and recording still work, they just talk to controllers.
   That also matches huggingface/lerobot#4368, which proposes a generic config driven plugin
   over standard `JointState` / `JointTrajectory` / `Twist` / `Odometry` instead of one
   bridge per robot.

[ADR-0001](docs/decisions/0001-ros2-control-over-lerobot-bridge.md).

The cost is real: upstream's teleop scripts, VR interface and recording examples don't work
against this without changes. Getting them back is phase 5.

### 2.3 Jazzy, and the JetPack gap on the actual board

Isaac ROS 4.0 dropped Humble and moved to Jazzy, current release is 4.6.0 (Aug 2026). Since
the point of the Jetson is cuVSLAM / nvblox / cuMotion, NVIDIA picks the distro, not us.
Jazzy on Ubuntu 24.04, both on the dev box and everything non-GPU on the Jetson.

The actual Orin NX is on JetPack 6.2 (L4T 36.4.3, CUDA 12.6). Current Isaac ROS needs
JetPack 7.2, which only just added Orin NX support in June 2026, Thor only before that. So
on this board today: the GPU-accelerated Isaac packages that work on 6.2 are Isaac ROS 3.2,
which is Humble.

That doesn't block phase 1 or 2. Jetson containers bind-mount the host's CUDA/TensorRT
libraries at runtime, so JetPack version only matters for what actually touches the GPU.
Plain Jazzy, `ros2_control`, `sts_hardware_interface`, Nav2, `slam_toolbox`, none of that
needs CUDA, all of it runs on JetPack 6.2 as is. The gap only lands at phase 4, and by then
either reflash to JetPack 7.2 or run Isaac ROS 3.2 on Humble in a separate container bridged
over topics. Full reasoning and the two paths in
[ADR-0002](docs/decisions/0002-jazzy-and-isaac-ros.md).

---

## 3. What comes from the community

As of September 2026, licenses included.

| Need | Package | License | Verdict |
|------|---------|---------|---------|
| Feetech STS bus → `ros2_control` | [`adityakamath/sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) | Apache-2.0 | **Use it.** The critical dependency. CI tested on Jazzy and Kilted. Per-joint `operating_mode` (0 position, 1 velocity, 2 PWM) set in the URDF, so bus 2's mixed mode case works natively. Exports position, velocity, effort, voltage, temperature, current and moving status per joint, so we get a real safety supervisor almost for free. Position and velocity modes are tested on hardware. |
| Diff drive + odometry | `diff_drive_controller` | Apache-2.0 | Use as is. |
| Arm trajectories | `joint_trajectory_controller` | Apache-2.0 | Use as is. |
| SO-101 description and MoveIt | [`adityakamath/so_arm_ros2`](https://github.com/adityakamath/so_arm_ros2) | Apache-2.0 | **Reference only.** Same author as the hardware interface so conventions match. Jazzy/Kilted, SO100 and SO101 with gripper. But it assumes one arm owns one bus, and here two arms share buses with a head and wheels. Take the joint limits, mesh conventions and MoveIt setup, rewrite the xacro. Author marks it work in progress with AI generated content, so read before copying. |
| Alt SO-ARM description | [`ros-physical-ai/ros2_so_arm`](https://github.com/ros-physical-ai/ros2_so_arm) | BSD-3 | Cross check for joint limits and MoveIt config. Uses `feetech_ros2_driver`. |
| Alt Feetech driver | [`JafarAbdi/feetech_ros2_driver`](https://github.com/JafarAbdi/feetech_ros2_driver) | check | Fallback if `sts_hardware_interface` fails the multi-bus test (§7, R2). |
| 2D SLAM | `slam_toolbox` | LGPL-2.1 | Use. Primary mapping until VSLAM is proven. |
| Navigation | `nav2` | Apache-2.0 | Use. |
| Visual SLAM | [`isaac_ros_visual_slam`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) | NVIDIA | Use if we get a suitable camera (§5). Stereo or RGB-D only, no monocular. Optional IMU fusion. |
| 3D recon → costmap | `isaac_ros_nvblox` | NVIDIA | Phase 4. Feeds Nav2 through a costmap plugin. |
| Lidar → scan | `pointcloud_to_laserscan` | BSD | If the VLP-16 is used. |
| VLP-16 driver | `velodyne` (ros2 branch) | BSD | If used. |
| Orbbec camera | [`OrbbecSDK_ROS2`](https://github.com/orbbec/OrbbecSDK_ROS2) | Apache-2.0 | Conditional, see §5. |
| Similar robot | [`robotmindio/cleanroom`](https://github.com/robotmindio/cleanroom) | Apache-2.0 | **Read it, don't import it.** Closest thing out there: Jazzy, Feetech driven LeKiwi, Nav2, RTAB-Map. Uses the ZMQ bridge we rejected so the driver layer doesn't transfer, but the Nav2 tuning, safety supervisor and RTAB-Map config are useful. Their finding that lidar ICP beat visual loop closure in feature poor corridors is worth knowing. |
| USB device naming | [`Grigorij-Dudnik/RoboCrew`](https://github.com/Grigorij-Dudnik/RoboCrew) | MIT | **Ported.** `tools/xlerobot_setup_usb_modules.py` is RoboCrew's `robocrew_setup_usb_modules` device-detection and udev rule generation, adapted for the two Feetech buses (`xle-bus1`/`xle-bus2`) instead of RoboCrew's own camera/mic/arm layout. |

### 3.1 What we have to write

Nothing out there covers these, and they are the actual work:

1. **A URDF with wheels.** Neither upstream URDF has any. Both
   `simulation/Maniskill/assets/xlerobot/xlerobot.urdf` and the Isaac Sim one hang
   `base_link` on a virtual planar joint (`root_x_axis_joint` prismatic →
   `root_y_axis_joint` prismatic → `root_z_rotation_joint` continuous). That is a free
   floating base sim trick. No wheel links, no caster, and it still has the 0.3.0 three
   omni wheel geometry. `diff_drive_controller` can't work with it and neither can a
   physics sim of the real drivetrain. Prerequisite for everything else.
   Also the inherited `base_link` inertial says `mass = 70.1 kg`, which is ManiSkill's
   loaded cart, not the robot. Don't carry that over.
2. **Two-bus mixed-mode `ros2_control` config**, plus whatever shim
   `sts_hardware_interface` turns out to need.
3. **Multi-turn wheel position accumulation**, see §4.1. Sharpest risk here.
4. **Calibration tooling**, given §1.4.
5. **Safety supervisor** on the per-joint temperature and current interfaces.
6. Bringup, Nav2 tuning and sensor integration for this machine.

---

## 4. Details that matter

### 4.1 Odometry and the wrap problem

Biggest known risk in the stack. Based on the driver source
(`adityakamath/sts_hardware_interface`), not an assumption.

`diff_drive_controller` wants wheel position in radians and differences it. STS3215 reports
`Present_Position` as 12 bit, 0 to 4095, and it wraps. In velocity mode the wheel turns
continuously, so it wraps about every 0.4 m at r = 0.0635.

The driver does not unwrap it. `read()` pulls the raw encoder register every cycle and maps
it straight to radians with a pure linear formula, `raw_position_to_radians` in
`sts_conversions.hpp`, no revolution counter anywhere, and the same code path runs regardless
of `operating_mode`. So a continuously spinning wheel's exported position state jumps by
about 2π radians every physical revolution. `diff_drive_controller` differences that every
control cycle, so odometry breaks the moment the wheel completes one turn, not gradually.

The library's own doc comment explains they used to wrap with `fmod` and deliberately
removed it, clamping to `[0, 4095]` instead, because the old wrapping mapped negative angles
to the wrong end of the encoder. That is the right fix for a bounded position-mode joint,
but it means continuous rotation was never a design target here.

Options, in order of preference:

1. Patch upstream: add a per-joint continuous-position accumulator, opt-in so it does not
   change behaviour for position-mode joints. Contained change, same class of fix the project
   already made once for the fmod case, and the license allows it.
2. Wrap locally in a fork, same logic, kept out of tree if upstream is slow.
3. Fall back to integrating the velocity state interface instead, which is not wrapped
   (`raw_velocity_to_rad_s` is a straight sign-flip, no windowing), and accept worse dead
   reckoning than a real position accumulator would give.

Whichever fix, it has to survive read failures: a dropped packet that looks like a 2000 tick
jump must be rejected by a max-plausible-delta gate derived from commanded velocity, not
integrated silently.

**Still to do, on real hardware, not just from reading source:** confirm the STS3215's own
`Present_Position` register actually behaves this way in wheel mode and not some multi-turn
variant the datasheet doesn't make obvious. Jack the robot up, command a slow constant wheel
velocity, plot reported position over several turns. Twenty minutes, and it's the difference
between "apply the known fix" and "there's something else going on."

Either way wheel odometry here is a prior, not truth:

- Two driven wheels and an unsensed caster on a tall cart. Caster scrub during in place
  rotation is unmodelled error, and that is exactly what Nav2 does to align to goals.
- No suspension. Grout lines and carpet edges cause slip.
- No IMU in the base as shipped.

So yaw will drift. The plan is fusion from the start: `ekf_node` (robot_localization) with
wheel odometry plus an IMU, and SLAM correcting `map→odom`. An IMU is close to mandatory for
a usable `odom` frame, either the one in a D435i/D455 or a separate part. Budget for it.

### 4.2 Frames

Plain REP-105:

```
map → odom → base_footprint → base_link → {left_wheel, right_wheel, caster,
                                            torso → {left_arm_*, right_arm_*,
                                                     head_pan → head_tilt → camera_*}}
```

`base_footprint` is the ground projection and what Nav2 plans for.
`odom→base_footprint` from `diff_drive_controller`, `map→odom` from slam_toolbox or cuVSLAM.
Exactly one of those two at a time, never both.

Camera optical frames follow REP-103. Upstream's URDF already gets the head camera frames
right, so keep those names.

### 4.3 Bus budget

Half duplex TTL at 1 Mbps. A sync read of N motors is one packet out, N back.
`sts_hardware_interface`'s `SYNC_READ_FEEDBACK_LEN` is 15 bytes (registers 56 through 70),
so for an 8-motor bus (bus 1 and bus 2 both carry 8):

```
SyncRead request + 8 responses   ~184 bytes  ~1.84 ms
SyncWrite                        ~56 bytes   ~0.56 ms
round trip                       ~2.4 ms     -> ~400 Hz on the wire alone
```

So the servo protocol itself supports well above the 200 Hz `controller_manager` is set to.
The real ceiling is more likely the USB-serial adapter, not the wire: FTDI/CH340/CP2102
chips commonly default their Linux `latency_timer` to 16 ms per USB transaction, which caps
effective round-trip rate around 60 Hz regardless of what the servo protocol allows. Check
`/sys/bus/usb-serial/devices/*/latency_timer` on the real hardware before trusting either
number.

With arms idle bus 2 only carries the two wheels. With an arm streaming a trajectory at the
same rate, arm and wheel traffic interleave on the sync read/write and the wheel update rate
drops, proportionally to how many joints share the bus, not by much else.

If measurement shows this matters: run the two buses in **separate** `controller_manager`
instances at different rates, base fast and arms slower, instead of one manager stuck at the
lowest common rate. That is why the hardware interfaces are two independent `<ros2_control>`
blocks from the start, it keeps the option open.

Still needs a real bench measurement. The wire-protocol math and the latency_timer caveat are
both reasoning from source and from how these USB-serial chips generally behave, not from a
scope on the actual bus. The `sts_hardware_interface` diagnostics node plus `controller_manager`
timing stats should answer it for real once there's hardware to run it on.

### 4.4 Safety

STS3215 is a hobby servo with no external thermal cutout, driving a top heavy cart with two
arms on it. The per-joint temperature and current interfaces are there, so a supervisor node
should:

- stop the base and hold arm position over a temperature threshold,
- detect stall (high current, zero velocity, non-zero command) and release torque,
- enforce a command watchdog, upstream uses 500 ms, `cmd_vel_timeout` should be tighter,
  around 250 ms,
- clamp commanded acceleration to what the CoM tolerates.

**Don't build any of this around `sts_hardware_interface`'s own `/emergency_stop` service.**
Each bus instance opens its own internal node hardcoded to the name
`sts_hardware_interface_node` (not configurable), and each advertises `/emergency_stop` on
that same absolute name. Two servers under one service name means a call only reaches one of
them, not both, so it looks like a global e-stop and isn't one. With two buses that's a
stop-half-the-robot bug, not a cosmetic one.

Use `controller_manager`'s own `switch_controller` to deactivate everything instead, or just
rely on `cmd_vel_timeout` plus zero-command on watchdog trip, both of which act uniformly
across every loaded hardware component. Worth an upstream issue on the node-naming collision
too, small fix (name the internal node from the hardware component's own `info_.name`, e.g.
"bus1"/"bus2").

---

## 5. Perception, still open

`xlerobot_perception` picks the sensor by launch argument, because the sensor question is
not settled.

**On hand:**

- **Orbbec Gemini**, USB `2bc5:0670`, USB 2.0, serial AY6G65301J0. This is the original
  structured light Gemini, not a Gemini 2 and not a 330 series. It is **not** in the
  OrbbecSDK **v2** device list. The `OrbbecSDK_ROS2` `main` branch (SDK v1 path) does ship
  `gemini.launch.py` and claims Foxy / Humble / Jazzy, so it may well stream. Open questions:
  whether depth and colour are hardware synced with usable timestamps, whether registered
  depth to colour is available, whether USB 2.0 holds a useful resolution and rate, and
  whether cuVSLAM's RGB-D mode takes it. The original Gemini also has no IMU, which §4.1
  wants. **Worth an evening to test, might be free.**
- **Velodyne VLP-16.** Definitely works. `velodyne` → `pointcloud_to_laserscan` →
  `slam_toolbox` → Nav2 is the most predictable path and needs no NVIDIA stack. Costs
  ~830 g and ~8 W on a battery powered servo cart, plus bulk, plus no cuVSLAM.
- **RealSense D435i / D455**, not owned. Easiest path to the Isaac stack: tested with
  cuVSLAM, has the IMU, and upstream XLeRobot already ships head gimbal mounts for exactly
  these two (`Gimbal_Pitch_Holder_d435.stl`, `Gimbal_Pitch_Holder_d455.stl`), with a
  commented out `RealSenseCameraConfig` head camera already in the lerobot config.

**Suggested order:** bring up Nav2 on the VLP-16 first. It separates "is my odometry sane"
from "does my camera work", and a known good scan matcher is the measuring tool for §1.4 and
§4.1. Test the Gemini in parallel. Move to cuVSLAM and nvblox once there is a camera for it.
[ADR-0003](docs/decisions/0003-perception-sensor.md).

---

## 6. Calibration

Three numbers decide whether anything downstream means anything. All of them are data.

| What | How | Where |
|------|-----|-------|
| `wheel_radius` | Drive a measured straight line, 5 m or more, compare reported to tape measured, scale. Repeat on the real floor surface. | `robot_dimensions.yaml` |
| `wheel_separation` | Rotate in place N full turns, compare reported yaw to actual, scale. Or UMBmark. | `robot_dimensions.yaml` |
| per-wheel scale | UMBmark square path, corrects unequal effective radii | `robot_dimensions.yaml` |

`tools/calibrate_wheel_odometry.py` runs the drive and compare loop and writes the YAML. Do
it **before** tuning Nav2, otherwise the Nav2 gains are just compensating a scale error.

---

## 7. Risks

| # | Risk | Impact | What to do |
|---|------|--------|------------|
| R1 | `sts_hardware_interface` doesn't unwrap multi-turn wheel position (§4.1) | Odometry unusable | Patch upstream or wrap locally, see §4.1. |
| R2 | Both hardware instances share a hardcoded internal node name and collide on `/emergency_stop` | One instance's e-stop doesn't reach the other bus | Don't build the safety supervisor around this service; use `controller_manager`'s `switch_controller` instead, see §4.4. |
| R3 | Bus 2 contention, arms starve wheel updates | Odometry jitter while manipulating | Instrument from day one. Split controller managers by rate. |
| R4 | `wheel_radius` really is 0.0635 not 0.05 (§1.4) | 27% distance error | Calibrate first. Never hardcode. |
| R5 | Gemini unusable for cuVSLAM (§5) | No Isaac VSLAM without buying | VLP-16 + slam_toolbox is enough for navigation. Isaac is an upgrade, not a dependency. |
| R6 | Servo thermal limits under long navigation | Shutdown mid mission, maybe damage | Safety supervisor in phase 1, not later (§4.4). |
| R7 | Top heavy cart tips under hard acceleration | Broken hardware | Conservative accel limits in `diff_drive_controller` and the velocity smoother from the start. |
| R8 | Upstream says 0.4.0 is the last Feetech / SO101 chassis version | Platform moves to brushless | Keep the hardware behind `ros2_control`, then a brushless base swaps one plugin instead of the stack. The ODrive variant already exists upstream. |
| R9 | Orin NX is on JetPack 6.2, current Isaac ROS needs 7.2 (§2.3) | No Jazzy cuVSLAM/nvblox without a reflash | Doesn't block phase 1/2, plain Jazzy runs on 6.2 fine. Decide reflash vs Humble side-container at phase 4, see ADR-0002. |

---

## 8. Phases

**Phase 1, description and a wheel that turns.** Real URDF with wheel joints, measured
dimensions, RViz. Two-bus `ros2_control` config. Settle R1 and R2. `diff_drive_controller`
publishing `/odom`. Joystick teleop. Safety supervisor.
Done when: it drives from a gamepad and `/odom` is within 5% over 10 m.

**Phase 2, mapping and navigation.** VLP-16 (or Gemini) bringup, EKF, `slam_toolbox`, Nav2
tuned for a 0.25 m track diff drive robot at ~0.25 m/s.
Done when: autonomous point to point in a mapped room.

**Phase 3, arms.** Both arms on `joint_trajectory_controller`, MoveIt 2, head pan/tilt,
self collision and arm vs base geometry.
Done when: MoveIt plans and executes on both arms.

**Phase 4, Isaac.** cuVSLAM replacing or fusing with slam_toolbox, nvblox costmap layer,
cuMotion for the arms. First decision here is JetPack: reflash the Orin NX to 7.2 for Jazzy,
or run Isaac ROS 3.2 on Humble in a separate container, see ADR-0002.
Done when: phase 2 and 3 parity on the Jetson.

**Phase 5, lerobot comes back.** A ROS 2 node exposing this stack as a `lerobot` robot so
policies and dataset recording work over ROS topics. Follow huggingface/lerobot#4368
conventions instead of a custom bridge.

Phases 1 and 2 are the commitment. The rest depends on what phase 1 finds.

---

## 9. Layout

```
ros2_xlerobot_2wheels/          # the repo is a colcon workspace
├── src/
│   ├── xlerobot_description/   # xacro, meshes, dimensions, rviz
│   ├── xlerobot_bringup/       # controller configs, top level launch
│   ├── xlerobot_teleop/        # joystick, twist mux, safety supervisor
│   ├── xlerobot_navigation/    # slam_toolbox, Nav2, EKF params
│   └── xlerobot_perception/    # sensor agnostic camera and lidar bringup
├── tools/                      # calibration and bus diagnostics
├── docs/decisions/             # ADRs
└── xlerobot.repos              # community deps via vcs
```
