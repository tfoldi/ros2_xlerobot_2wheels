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


def controllers_for(group, joints):
    """A recorded group is either one of GROUP_CONTROLLER's own single-controller groups, or a
    composite of them (currently just "both_arms" - position_store.GROUP_JOINTS["both_arms"] =
    left_arm + right_arm joints across both buses, recorded as one hand-guided session, but
    there's no single controller that owns all 10 joints: left_arm_controller and
    right_arm_controller are separate ros2_control controllers, each with its own action server).
    Splits by matching which of GROUP_CONTROLLER's own joint sets are fully present in this
    trajectory's joints, so a composite recording replays as one goal per real controller."""
    if group in position_store.GROUP_CONTROLLER:
        return [(position_store.GROUP_CONTROLLER[group], joints)]
    joint_set = set(joints)
    parts = [(controller, position_store.GROUP_JOINTS[subgroup])
             for subgroup, controller in position_store.GROUP_CONTROLLER.items()
             if set(position_store.GROUP_JOINTS[subgroup]) <= joint_set]
    if not parts:
        sys.exit(f"group '{group}' doesn't match any known controller's joints")
    return parts


def build_goal(joint_names, all_joints, points, speed):
    indices = [all_joints.index(j) for j in joint_names]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joint_names
    for sample in points:
        point = JointTrajectoryPoint(positions=[sample["positions"][i] for i in indices])
        point.time_from_start = Duration(seconds=sample["time_from_start"] / speed).to_msg()
        goal.trajectory.points.append(point)
    return goal


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

    joints = data["joints"]
    points = data["points"]
    parts = controllers_for(data["group"], joints)
    controllers = [c for c, _ in parts]
    print(f"replaying '{args.name}': {len(points)} points, {points[-1]['time_from_start']:.1f}s "
          f"recorded, group={data['group']} controller(s)={', '.join(controllers)}, speed={args.speed}x")

    rclpy.init()
    node = Node("replay_trajectory")
    clients = {c: ActionClient(node, FollowJointTrajectory, f"/{c}/follow_joint_trajectory") for c in controllers}
    for controller, client in clients.items():
        if not client.wait_for_server(timeout_sec=15.0):
            sys.exit(f"/{controller}/follow_joint_trajectory not available - is bringup running?")

    # Sent together, not one after another - a composite group (both arms clapping) needs its
    # controllers' goals to start at the same time to stay in sync, same as when it was recorded.
    sends = {}
    for controller, joint_names in parts:
        goal = build_goal(joint_names, joints, points, args.speed)
        sends[controller] = clients[controller].send_goal_async(goal)
    for future in sends.values():
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handles = {}
    for controller, future in sends.items():
        handle = future.result()
        if handle is None or not handle.accepted:
            sys.exit(f"goal not accepted by {controller}")
        handles[controller] = handle

    result_futures = {c: h.get_result_async() for c, h in handles.items()}
    timeout = points[-1]["time_from_start"] / args.speed + 15.0
    for future in result_futures.values():
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    for controller, future in result_futures.items():
        result = future.result()
        if result is None:
            print(f"{controller}: no result (timed out)")
            continue
        code = result.result.error_code
        print(f"{controller}: error_code={code}" + (" (SUCCESSFUL)" if code == 0 else " (FAILED)"))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
