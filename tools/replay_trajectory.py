#!/usr/bin/env python3
"""Replay a trajectory recorded with tools/record_trajectory.py.

  python3 tools/replay_trajectory.py <name> [--speed 1.0]

Sends the raw recording straight back as one FollowJointTrajectory goal - no collision checking,
since it's just resending exactly what was hand-guided when recorded. Same care as any other real
move: arm supported, tools/estop.py ready.
"""
import argparse
import sys

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

import position_store


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("--speed", type=float, default=1.0, help="1.0 = as recorded, 2.0 = 2x faster")
    args = parser.parse_args()

    path = position_store.trajectory_path(args.name)
    if not path.exists():
        sys.exit(f"no recorded trajectory '{args.name}' at {path}")
    with open(path) as f:
        data = yaml.safe_load(f)

    controller = position_store.GROUP_CONTROLLER[data["group"]]
    joints = data["joints"]
    points = data["points"]
    print(f"replaying '{args.name}': {len(points)} points, {points[-1]['time_from_start']:.1f}s "
          f"recorded, group={data['group']} controller={controller}, speed={args.speed}x")

    rclpy.init()
    node = Node("replay_trajectory")
    client = ActionClient(node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=15.0):
        sys.exit(f"/{controller}/follow_joint_trajectory not available - is bringup running?")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joints
    for sample in points:
        point = JointTrajectoryPoint(positions=sample["positions"])
        point.time_from_start = Duration(seconds=sample["time_from_start"] / args.speed).to_msg()
        goal.trajectory.points.append(point)

    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        sys.exit("goal not accepted")
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future,
                                      timeout_sec=points[-1]["time_from_start"] / args.speed + 15.0)
    result = result_future.result()
    if result is None:
        sys.exit("no result (timed out)")
    print(f"error_code: {result.result.error_code}"
          + (" (SUCCESSFUL)" if result.result.error_code == 0 else " (FAILED)"))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
