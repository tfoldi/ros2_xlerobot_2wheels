# ADR-0003: Perception sensor deferred, stack stays sensor agnostic

- Status: Open, decision deferred to end of phase 1
- Date: 2026-09-20

## Context

SLAM and Nav2 need a sensor. Three candidates, two already on hand.

### Orbbec Gemini (have it)

USB `2bc5:0670`, `iProduct` "Orbbec(R) Gemini(TM)", bcdUSB 2.00, serial AY6G65301J0. This is
the original structured light Gemini, not a Gemini 2, not a 330 series.

- Not in the OrbbecSDK v2 supported device table. v2 supports Gemini 2/2L, Gemini
  210/215/305/330/335/336/338/345/435Le, Astra 2, Femto Bolt/Mega, Astra Mini Pro.
- `OrbbecSDK_ROS2` `main` branch (SDK v1 path) does ship `gemini.launch.py` and claims Foxy /
  Humble / Jazzy. So it might enumerate and stream.
- Open questions that decide it: hardware depth/colour sync and trustworthy timestamps,
  registered depth to colour availability, whether USB 2.0 holds a useful resolution and
  rate, whether cuVSLAM's RGB-D mode accepts it. No IMU either, which §4.1 wants for a usable
  odom frame.

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

## Decision (provisional)

Not deciding yet. `xlerobot_perception` picks the sensor at launch and carries a config per
candidate. Phase 1 needs none of them.

Order:

1. Phase 2 primary: VLP-16. Separates "is my odometry sane" from "does my camera work", a
   known good scan matcher is the measuring tool for the wheel radius and multi-turn
   position questions (ARCHITECTURE §1.4, §4.1). Debugging both at once is avoidable.
2. In parallel: test the Gemini. An evening. If depth streams on Jazzy with usable sync, it
   might do for RGB-D cuVSLAM and nothing needs buying.
3. If the Gemini fails: buy a D435i, the mount already exists.

## Consequences

- Phase 2 targets a 2D `LaserScan` interface, which both the lidar and a depth camera can
  produce, so the Nav2 config doesn't need rewriting when the sensor changes.
- cuVSLAM and nvblox are phase 4, explicitly optional. Navigation doesn't depend on them.
- The head gimbal design constrains camera choice more than the software does.

## Resolve by

End of phase 1, once the Gemini test result is in.
