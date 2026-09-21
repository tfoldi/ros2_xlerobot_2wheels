# ros2_xlerobot_2wheels

ROS 2 Jazzy stack for [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) **0.4.0**, the
servo two-wheel differential drive version. Targets the NVIDIA Isaac stack on Jetson,
developed on an x86 box with an NVIDIA GPU.

> **Status: scaffold.** Architecture is worked out, layout is real, code is not written yet.
> See [ARCHITECTURE.md](ARCHITECTURE.md) and [§ Phases](ARCHITECTURE.md#8-phases) for what
> lands when. Nothing here has run on hardware.

This is not a fork of XLeRobot. It reuses upstream's CAD, motor conventions and kinematics,
and replaces the Python control path with `ros2_control` so Nav2, MoveIt 2 and Isaac ROS work
natively. Upstream stays the hardware project and the imitation learning path. `lerobot`
comes back in phase 5 as a client of this stack, not the thing driving it.

---

## What this is for

A standard ROS 2 autonomy stack on the XLeRobot two-wheel base:

- `ros2_control` hardware interface for both Feetech buses, 16 joints
- `diff_drive_controller` for odometry and TF
- SLAM and Nav2
- MoveIt 2 for both SO-101 arms (phase 3)
- Isaac ROS cuVSLAM / nvblox / cuMotion where they earn their place (phase 4)

## Target hardware

| | |
|---|---|
| Robot | XLeRobot 0.4.0, two-wheel servo base, IKEA RÅSKOG chassis |
| Actuators | 16x Feetech STS3215 on 2 half-duplex TTL buses |
| Compute, deploy | Jetson Orin NX, currently JetPack 6.2 |
| Compute, dev | x86-64 + NVIDIA GPU, Ubuntu 24.04 |
| ROS | Jazzy Jalisco |
| Sensors | undecided, have a Velodyne VLP-16 and an Orbbec Gemini, a RealSense D435i/D455 is the likely answer for cuVSLAM, see [ARCHITECTURE §5](ARCHITECTURE.md#5-perception-still-open) |

Jazzy because Isaac ROS 4.0 dropped Humble. If you want cuVSLAM and nvblox, the distro is
picked for you.

---

## Layout

This repo is a colcon workspace, clone and build in place.

```
src/xlerobot_description/   xacro, meshes, measured dimensions, rviz config
src/xlerobot_bringup/       controller configs and top level launch
src/xlerobot_teleop/        joystick, twist mux, safety supervisor
src/xlerobot_navigation/    slam_toolbox, Nav2, robot_localization params
src/xlerobot_perception/    sensor agnostic camera and lidar bringup
tools/                      calibration and bus diagnostics
docs/decisions/             architecture decision records
xlerobot.repos              community deps, pulled with vcs
```

## Getting started

Nothing builds usefully yet. Once it does:

```bash
git clone <this-repo> ~/Developer/ros2_xlerobot_2wheels
cd ~/Developer/ros2_xlerobot_2wheels

# community deps (sts_hardware_interface, etc.)
vcs import src < xlerobot.repos

rosdep install --from-paths src --ignore-src -r -y

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

View the model with no hardware attached:

```bash
ros2 launch xlerobot_description display.launch.py
```

---

## Before you plug anything in

Three things will bite. Writing them down instead of finding them the hard way:

**1. The wheel radius in upstream's config is probably wrong.**
`config_xlerobot_2wheels.py` has `wheel_radius = 0.05`, but the 0.4.0 build notes say 5 inch
walker wheels, so r ≈ 0.0635. That's a ~27% error on every distance the robot reports.
Measure yours and put it in `src/xlerobot_description/config/robot_dimensions.yaml` before
tuning anything.
See [ARCHITECTURE §1.4](ARCHITECTURE.md#14-numbers-that-are-probably-wrong).

**2. Neither upstream URDF has wheels.**
Both the ManiSkill and Isaac Sim URDFs put the base on a virtual planar joint
(`root_x_axis_joint` / `root_y_axis_joint` / `root_z_rotation_joint`), a sim trick for a free
floating base, no wheel links, still the 0.3.0 three omni wheel geometry. Writing a real
URDF is the first job here, not a copy job.

**3. The left wheel motor is mounted mirrored.**
Its command has to be negated or `x.vel` turns the robot and `theta.vel` drives it forward.
This stack applies that sign exactly once, in the `ros2_control` joint config. Don't apply
it again anywhere else.

---

## Bus layout

Two buses, and the split crosses subsystems, that's the main constraint here.

| Bus | IDs | Joints | Mode |
|-----|-----|--------|------|
| 1 | 1-6 | left arm | position |
| 1 | 7-8 | head pan, head tilt | position |
| 2 | 1-6 | right arm | position |
| 2 | 9-10 | left wheel, right wheel | **velocity** |

So bus 2 runs position and velocity mode servos at the same time on one UART, and wheel
traffic competes with arm traffic. `sts_hardware_interface` supports per-joint operating
modes, which is why it was picked.

`/dev/ttyACM0` and `ttyACM1` swap on reboot. Pin them with:

```bash
sudo python3 tools/xlerobot_setup_usb_modules.py
```

Prompts to disconnect both buses, then connect each in turn, and writes
`/etc/udev/rules.d/99-xlerobot.rules` for `/dev/xle-bus1` and `/dev/xle-bus2`
itself, matched by USB physical port and serial rather than by hand. Also
scans for cameras and microphones, so the same tool covers a depth camera's
device name later (type `a` at the prompt to add one). Device detection and
udev rule generation ported from
[RoboCrew](https://github.com/Grigorij-Dudnik/RoboCrew)'s
`robocrew_setup_usb_modules` (MIT licensed).

`tools/check_feetech_bus.py` scans a bus and reports which IDs answer, useful for figuring
out which cable is which.

---

## Relationship to upstream

| Taken from XLeRobot | How |
|---|---|
| Meshes and CAD | From the Isaac Sim URDF bundle and STEP files |
| Motor IDs, bus split, mirrored wheel sign | Read from `xlerobot_2wheels.py`, restated in [ARCHITECTURE §1](ARCHITECTURE.md#1-hardware) |
| Nominal dimensions | Starting point only, treated as calibration targets |
| Head camera frame names | Kept as is |

Not taken: the ZMQ host/client control path, the `lerobot` runtime dependency, the virtual
planar joint URDF.

---

## Community dependencies

Full table with licenses and reasoning in
[ARCHITECTURE §3](ARCHITECTURE.md#3-what-comes-from-the-community). The one that matters most:

- [`adityakamath/sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface)
  (Apache-2.0), `ros2_control` interface for Feetech STS servos, CI tested on Jazzy, per-joint
  operating modes, and per-joint temperature/current/voltage that the safety supervisor needs.

Worth reading before starting: [`robotmindio/cleanroom`](https://github.com/robotmindio/cleanroom),
a Jazzy stack for the closely related LeKiwi platform. Uses the ZMQ bridge approach this repo
rejected, but the Nav2 tuning and safety patterns are directly useful.

## License

Apache-2.0, same as upstream XLeRobot.
