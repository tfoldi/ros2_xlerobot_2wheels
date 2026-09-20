# ADR-0002: ROS 2 Jazzy, tracking Isaac ROS 4.x

- Status: Accepted
- Date: 2026-09-20

## Context

Deploy target is a Jetson, intent is to use the NVIDIA stack, cuVSLAM, nvblox, later
cuMotion. Distro choice is constrained by what Isaac ROS supports, not by preference.

Isaac ROS 4.0 dropped Humble and moved to Jazzy. Humble tutorials and images were removed.
Current release is 4.6.0 (Aug 2026), recent releases target JetPack 7.x on Jetson Thor and
Orin, and Ubuntu 24.04 on x86-64 with an NVIDIA GPU.

## Decision

ROS 2 Jazzy Jalisco on Ubuntu 24.04, both Jetson and dev box. Track Isaac ROS 4.x.

## Why

- Isaac ROS picked it, using anything else gives up the reason for the Jetson.
- Jazzy is the current LTS, supported to 2029.
- Ubuntu 24.04 matches both the Jetson image and the dev box, so the two environments only
  differ by architecture.
- `sts_hardware_interface` is CI tested on Jazzy and Kilted.

## Consequences

- Isaac ROS packages mostly ship as containers. Dev box can run them native on 24.04 or
  containerised, Jetson uses the published images.
- Some ROS 2 packages still lag on Jazzy, check before depending on one.
- Upstream XLeRobot's jetson-containers package targets JetPack 6 / L4T r36.4+, current
  Isaac ROS targets JetPack 7.x. The two container lineages don't automatically line up. If
  lerobot and Isaac ROS both need to run on the same Jetson, that's a phase 4/5 problem to
  solve on purpose, not something to assume works.

## Revisit if

The Jetson can't run a JetPack version current Isaac ROS supports, or the Isaac dependency
is dropped in favour of the CPU path (slam_toolbox + Nav2), which has no distro constraint
beyond Jazzy itself.
