#!/usr/bin/env python3
"""Find and kill every process belonging to this robot's ROS2 stack - the "ps aux | grep | kill"
cleanup this session kept doing by hand, several times, on both the Jetson and this laptop.

  python3 tools/kill_ros.py              # list what it would kill, ask to confirm
  python3 tools/kill_ros.py --yes        # kill without asking
  python3 tools/kill_ros.py --dry-run    # just list, never kill, never asks

Run it on whichever machine has the stuck processes - it only ever looks at local processes, no
SSH/multi-host orchestration. Matches every process this project's launch files start: the
bringup stack (robot_state_publisher, ros2_control_node, controller spawners), cameras, lidar,
MoveIt's move_group, the Nav2/SLAM stack, and teleop/safety (joy_node, twist_mux,
safety_supervisor, mcp_presence_guard). Deliberately does NOT match interactive tools you want to
keep running across a cleanup - RViz2, the Rerun viewer, your editor/terminal, ssh sessions, this
script's own `ros2 launch` parent shells are matched (so the whole tree goes), but a bare `ros2`
CLI command (topic echo, node list, ...) is not, since those patterns are intentionally specific
executable names, not a broad "ros2" substring match.

SIGTERM first, give it a few seconds, SIGKILL anything still alive after that - same two-step
this session used by hand every time, because a few of these (ros2_control_node especially) don't
always die cleanly on the first signal.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

# Executable basenames / cmdline substrings, not regexes - matched against each process's full
# command line. One entry per node this project's launch files can start; add to this list
# instead of using a broader pattern if a new launch file starts something new.
PATTERNS = [
    "ros2 launch",  # the launch parent itself, so the whole tree it spawned goes with it
    "robot_state_publisher",
    "ros2_control_node",
    "controller_manager/spawner",
    "usb_cam_node_exe",
    "move_group",
    "ldlidar_container",
    "ldlidar_lifecycle_manager",
    "nav2_map_server",
    "nav2_amcl",
    "nav2_controller",
    "nav2_planner",
    "nav2_behaviors",
    "nav2_bt_navigator",
    "nav2_velocity_smoother",
    "nav2_lifecycle_manager",
    "async_slam_toolbox_node",
    "joy/joy_node",
    "teleop_twist_joy",
    "twist_mux/twist_mux",
    "xlerobot_teleop/safety_supervisor",
    "xlerobot_teleop/mcp_presence_guard",
]


def find_matches():
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True, check=True).stdout
    my_pid = os.getpid()
    matches = []
    for line in out.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        pid_str, _, cmd = line.partition(" ")
        try:
            pid = int(pid_str)
        except ValueError:
            continue
        if pid == my_pid or "kill_ros.py" in cmd:
            continue  # never target this script's own process
        if cmd.startswith("ssh ") or "nohup bash -c" in cmd:
            continue  # a wrapper that just MENTIONS a pattern in a remote/quoted command string,
            # not the local process itself - confirmed false-positive testing this script:
            # matched this script's own `ssh ... "ros2 launch ..."` call on the machine running
            # the ssh command, not on the machine actually running ros2 launch.
        if any(p in cmd for p in PATTERNS):
            matches.append((pid, cmd))
    return matches


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yes", action="store_true", help="kill without asking for confirmation")
    parser.add_argument("--dry-run", action="store_true", help="list matches, never kill, never asks")
    args = parser.parse_args()

    matches = find_matches()
    if not matches:
        print("nothing matching this project's ROS2 stack found running.")
        return

    print(f"found {len(matches)} matching process(es):")
    for pid, cmd in matches:
        print(f"  {pid:>7}  {cmd[:110]}")

    if args.dry_run:
        return

    if not args.yes:
        answer = input(f"\nkill all {len(matches)}? [y/N] ").strip().lower()
        if answer != "y":
            print("not killing anything.")
            return

    pids = [pid for pid, _ in matches]
    print(f"\nsending SIGTERM to {len(pids)} process(es)...")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    time.sleep(3)

    still_alive = []
    for pid in pids:
        try:
            os.kill(pid, 0)  # signal 0: check it exists, don't actually signal it
            still_alive.append(pid)
        except ProcessLookupError:
            pass

    if still_alive:
        print(f"{len(still_alive)} still alive after SIGTERM, sending SIGKILL: {still_alive}")
        for pid in still_alive:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(1)

    remaining = find_matches()
    if remaining:
        print(f"WARNING: {len(remaining)} process(es) still matched after cleanup:")
        for pid, cmd in remaining:
            print(f"  {pid:>7}  {cmd[:110]}")
        sys.exit(1)
    print("done, nothing matching left running.")


if __name__ == "__main__":
    main()
