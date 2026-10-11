# ADR-0003: Perception sensor - Orbbec Gemini 2

- Status: Accepted
- Date: 2026-09-20, resolved 2026-10-05

## Context

SLAM and Nav2 need a sensor. Three candidates, two already on hand.

### Orbbec Gemini 2 (have it)

USB `2bc5:0670`, `iProduct` "Orbbec(R) Gemini(TM)", bcdUSB 2.00, serial AY6G65301J0.

**Correction (2026-10-05): this is a Gemini 2, not the original Gemini.** It was
misidentified here on first read, from the `iProduct` string alone not mentioning "2" -
that was the wrong signal to go on. Confirmed from the device's own firmware response via
the official vendor driver, not inferred: `ros2 launch orbbec_camera gemini2.launch.py`
logs `Device Orbbec Gemini 2 connected`, serial `AY6G65301J0` (same unit), firmware
`1.4.92`. Independently corroborated by the vendor driver's own udev rules and device
config (`ros-jazzy-orbbec-camera`'s `99-obsensor-libusb.rules` maps idProduct `0670` to
`SYMLINK+="Gemini_2"`; `config/depthfilter/Gemini2_v1.7.json` is keyed to
`vid: 0x2bc5, pid: 0x0670`) - this is the vendor's own authoritative device table, not a
web search summary (one early search-tool summary during this investigation incorrectly
claimed the original Gemini had a dedicated `gemini.launch.py` in the current
`OrbbecSDK_ROS2` repo; reading the actual repo directly showed no such file exists in
either branch - a different, now-irrelevant project from the one that matters here).

Also explains why it has an IMU (confirmed live: accel + gyro streams) - the original
Gemini doesn't, the Gemini 2 does, matching real spec rather than the original wrong
identification.

- **Is** in `ros-jazzy-orbbec-camera`'s (apt, both arm64/x86_64, already installed on both
  machines) supported-device set - `gemini2.launch.py`, no source build, no patching.
- Live-tested on real hardware: color (1280x720 @ ~24Hz, MJPG), depth (1280x720 @ ~20Hz,
  real 16-bit, hardware-registered to color by default), point cloud (~8Hz), IR, IMU
  (accel+gyro) - all streaming real data. Needs a one-time per-machine udev rule install
  (`sudo cp .../99-obsensor-libusb.rules /etc/udev/rules.d/`, see
  `xlerobot_bringup/launch/cameras.launch.py`'s docstring) - without it the device doesn't
  get correct permissions.
- Depth/colour sync, registration, and resolution/rate questions from the original "open
  questions" below are now answered: yes to all, confirmed live.

### Velodyne VLP-16 (have it)

Works, no question. `velodyne` → `pointcloud_to_laserscan` → `slam_toolbox` → Nav2 is the
most predictable path, no NVIDIA dependency at all. Cost: ~830 g and ~8 W on a battery
powered servo cart, real bulk on a top heavy chassis, no cuVSLAM (needs stereo or RGB-D,
lidar doesn't qualify).

### RealSense D435i / D455 (don't have it)

Easiest path to the Isaac stack: explicitly tested with cuVSLAM, has the IMU that odometry
fusion wants, and upstream XLeRobot already ships head gimbal mounts for exactly these two
(`Gimbal_Pitch_Holder_d435.stl`, `Gimbal_Pitch_Holder_d455.stl`), with a commented out
`RealSenseCameraConfig` head camera stanza already in the lerobot config.

## Decision

Orbbec Gemini 2, confirmed working as a supported head camera option. `sensor:=usbcam`
(plain webcam) stays the stack's own default - most builds won't have a depth camera;
`sensor:=gemini` is the opt-in swap for one, threaded through `xlerobot.urdf.xacro`,
`cameras.yaml` and `cameras.launch.py`. See `xlerobot_bringup/launch/cameras.launch.py` and
`xlerobot_description/urdf/sensors.xacro` for the integration (the driver publishes its own
calibrated extrinsic TF tree rooted at a link named `<camera_name>_link`, i.e. `head_link`
here since `camera_name:="head"` - not a fixed `camera_link` as first assumed from a
standalone test that left `camera_name` at its own default - the URDF only places that one
link at the physical mount point, not per-stream optical frames by hand. Also needs
`enable_publish_extrinsic:=true`, passed explicitly in `cameras.launch.py` - the vendor
launch file's own default is `false`, which silently advertises `/tf`/`/tf_static` without
ever publishing anything on them).

The 2D-lidar-primary path mentioned below (originally VLP-16, now the lighter LD19 under the
second basket - see `xlerobot.urdf.xacro`'s `lidar` arg) already shipped independently of
this decision and is unaffected by it.

## Consequences

- Nav2 still runs off the 2D lidar unchanged - this decision is about the head camera/depth
  sensor specifically, not navigation's primary obstacle source.
- cuVSLAM, nvblox, and depth-aware planning are now actually unblocked (previously gated on
  "once there is a camera for it") - still separate adoption decisions, not started by this
  ADR, just no longer blocked on a sensor question.
- Camera mount position/orientation in the URDF is a first-cut placeholder (identity
  rotation, un-measured box collision geometry) pending hands-on tuning now that it's
  physically mounted and visible - not yet precise.

## Resolve by

Resolved 2026-10-05.
