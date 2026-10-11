# ADR-0002: ROS 2 Jazzy for the stack, JetPack path for the actual Orin NX split into phases

- Status: Accepted, JetPack gap resolved
- Date: 2026-09-20, updated 2026-09-20, JetPack gap resolved 2026-10-11

## Context

Deploy target is an Orin NX, currently on JetPack 6.2 (L4T 36.4.3, Ubuntu 22.04 base,
CUDA 12.6, TensorRT 10.3). Intent is to use cuVSLAM, nvblox, later cuMotion.

Isaac ROS 4.0 dropped Humble for Jazzy. Current release is 4.6.0 (Aug 2026), and it needs
JetPack 7.2. JetPack 7.2 (June 2026) is the first 7.x release that supports Orin NX at all,
before that 7.x was Thor only. So today, on the Orin NX as it sits:

- The GPU-accelerated Isaac packages (cuVSLAM, nvblox, cuMotion) that work on JetPack 6.2
  are Isaac ROS 3.2, and 3.2 is Humble, not Jazzy.
- Getting Jazzy + current Isaac ROS on this board means a full reflash to JetPack 7.2, not
  an update. Support for Orin NX on 7.2 is about 3 months old as of this writing.

But that coupling is narrower than it looks. Jetson containers bind-mount the host's
CUDA/TensorRT/VPI libraries at runtime, there's no separate driver install, the L4T BSP is
the whole GPU stack. So JetPack version only matters for packages that actually touch the
GPU. A plain Jazzy container with nothing CUDA in it doesn't care what L4T is underneath,
it just needs a kernel new enough to run it, and JetPack 6.2's 5.15 is fine.

## Decision

Jazzy for the stack, unconditionally. `ros2_control`, `sts_hardware_interface`,
`diff_drive_controller`, `slam_toolbox`, Nav2, `robot_localization`, all of phase 1 and 2,
run in a Jazzy container on the Orin NX right now, no reflash needed.

Isaac ROS specifically (phase 4, cuVSLAM/nvblox/cuMotion) is deferred, decision made when
phase 4 actually starts, not now. Two paths were kept open:

1. Reflash to JetPack 7.2. Keeps everything on Jazzy, matches the dev box, matches where
   Isaac ROS is going. Cost is a full reflash and some early-adopter risk on a 3 month old
   Orin NX support window.
2. Stay on 6.2, run Isaac ROS 3.2 for cuVSLAM/nvblox on Humble in a separate container,
   bridge to the Jazzy graph over topics. DDS/RTPS wire compatibility between Humble and
   Jazzy usually works fine for plain message types (sensor_msgs, geometry_msgs, nav_msgs),
   so this is two ROS graphs to maintain, not two incompatible stacks.

**Path 1 taken, 2026-10-11.** The Orin NX is reflashed to JetPack 7.2 (L4T 39.2.0, Ubuntu
24.04 base, CUDA 13.2, TensorRT 10.16) - confirmed on the board. Dev box and Jetson are now
on the same distro, CUDA and TensorRT generation; Isaac ROS 4.x is no longer blocked by the
OS underneath it. Phase 4 adoption itself (actually bringing in cuVSLAM/nvblox/cuMotion) is
still not started - this resolved the platform question, not the adoption decision.

## Why

- Isaac ROS picked Jazzy, using anything else on the parts that need Isaac gives up the
  reason for the Jetson.
- Jazzy is the current LTS, supported to 2029.
- `sts_hardware_interface` is CI tested on Jazzy and Kilted.
- Phase 1 and 2 don't touch the GPU at all, so they aren't blocked by any of this. No reason
  to hold the whole stack back for a decision only phase 4 needs.

## Consequences

- Dev box (Ubuntu 24.04, Jazzy) and the Orin NX (now also Ubuntu 24.04, Jazzy, JetPack 7.2)
  are fully aligned, not just for phase 1/2 - Isaac ROS 4.x itself can now run on the Jetson
  too, whenever phase 4 actually starts.
- **New gap opened by the reflash, inverted from the one this ADR resolved:** upstream
  XLeRobot's own `jetson-containers` package targets JetPack 6 / L4T r36.4+, which no longer
  matches this board. If phase 5 (lerobot) needs that container lineage, check whether it
  runs on JetPack 7.2 before assuming it does - this wasn't a problem before the reflash.
- Path 2 (Isaac ROS 3.2 on Humble, bridged to the Jazzy graph) was not taken - kept here for
  the record, not a live option on this board anymore.

## Revisit if

Phase 4 starts - confirm Isaac ROS 4.x (cuVSLAM/nvblox/cuMotion) actually installs and runs
on this board as reflashed, first real test of it. Also check the `jetson-containers`/JetPack
7.2 gap above before phase 5 needs it.
