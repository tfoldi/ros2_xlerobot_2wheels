#!/usr/bin/env python3
"""Characterize one joint's real tracking behavior: send a single-point move at a known nominal
average speed, log /joint_states at full rate through the move and a hold period after it, and
report whether it's (a) still converging when time runs out (a speed/timing problem) or (b) has
settled at a persistent offset (a steady-state/gravity/compliance problem).

Holds every other joint of the controller at its current position (allow_partial_joints_goal is
false). Arm held/supported as usual, tools/estop.py ready.

  python3 tools/characterize_joint.py <controller> <joint> <delta> <duration> [--hold 3.0]

Example: python3 tools/characterize_joint.py right_arm_controller right_arm_shoulder_pan_joint 0.3 2.0
"""
import argparse
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

CONTROLLER_JOINTS = {
    "left_arm_controller": ["left_arm_shoulder_pan_joint", "left_arm_shoulder_lift_joint",
                             "left_arm_elbow_flex_joint", "left_arm_wrist_flex_joint",
                             "left_arm_wrist_roll_joint"],
    "right_arm_controller": ["right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint",
                              "right_arm_elbow_flex_joint", "right_arm_wrist_flex_joint",
                              "right_arm_wrist_roll_joint"],
    "left_gripper_controller": ["left_arm_gripper_joint"],
    "right_gripper_controller": ["right_arm_gripper_joint"],
    "head_controller": ["head_pan_joint", "head_tilt_joint"],
}


class Characterize(Node):
    def __init__(self, controller):
        super().__init__("characterize_joint")
        self.positions = {}
        self.log = []  # (t, position) for the target joint
        self.t0 = None
        self.target_joint = None
        self.create_subscription(JointState, "/joint_states", self.on_states, 50)
        self.client = ActionClient(self, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))
        if self.target_joint and self.target_joint in self.positions:
            now = time.time()
            if self.t0 is not None:
                self.log.append((now - self.t0, self.positions[self.target_joint]))

    def wait_ready(self, joints):
        if not self.client.wait_for_server(timeout_sec=10.0):
            sys.exit("action server not available")
        deadline = time.time() + 10
        while not all(j in self.positions for j in joints) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        missing = [j for j in joints if j not in self.positions]
        if missing:
            sys.exit(f"no /joint_states for: {', '.join(missing)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("controller", choices=sorted(CONTROLLER_JOINTS))
    parser.add_argument("joint")
    parser.add_argument("delta", type=float)
    parser.add_argument("duration", type=float)
    parser.add_argument("--hold", type=float, default=3.0)
    args = parser.parse_args()

    joints = CONTROLLER_JOINTS[args.controller]
    if args.joint not in joints:
        sys.exit(f"{args.joint} is not in {args.controller}: {joints}")

    rclpy.init()
    node = Characterize(args.controller)
    node.wait_ready(joints)
    node.target_joint = args.joint

    current = {j: node.positions[j] for j in joints}
    start = current[args.joint]
    target = dict(current)
    target[args.joint] = start + args.delta
    nominal_avg_vel = args.delta / args.duration
    print(f"{args.joint}: start={start:.4f} target={target[args.joint]:.4f} delta={args.delta:+.4f} "
          f"duration={args.duration:.2f}s (nominal avg vel {nominal_avg_vel:+.4f} rad/s)")

    point = JointTrajectoryPoint(positions=[target[j] for j in joints])
    point.time_from_start = Duration(seconds=args.duration).to_msg()
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joints
    goal.trajectory.points = [point]

    node.t0 = time.time()
    node.log.append((0.0, start))
    send = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        sys.exit("goal not accepted")

    total = args.duration + args.hold
    while time.time() - node.t0 < total:
        rclpy.spin_once(node, timeout_sec=0.05)

    # sample the log at ~0.2s intervals for a readable table
    print(f"\n{'t':>6} {'position':>10} {'error':>10}")
    next_mark = 0.0
    at_duration = None
    at_end = None
    for t, pos in node.log:
        err = pos - target[args.joint]
        if at_duration is None and t >= args.duration:
            at_duration = (t, pos, err)
        if t >= next_mark:
            print(f"{t:6.2f} {pos:10.4f} {err:10.4f}")
            next_mark += 0.2
    at_end = (node.log[-1][0], node.log[-1][1], node.log[-1][1] - target[args.joint])

    achieved_avg_vel = (start - node.log[0][1] + (at_duration[1] - start)) / args.duration if at_duration else None
    print(f"\nnominal avg velocity: {nominal_avg_vel:+.4f} rad/s")
    if at_duration:
        achieved_avg_vel = (at_duration[1] - start) / at_duration[0] if at_duration[0] > 0 else 0.0
        print(f"achieved avg velocity to t={at_duration[0]:.2f}s: {achieved_avg_vel:+.4f} rad/s "
              f"(error at nominal end: {at_duration[2]:+.4f} rad)")
    print(f"error at end of {args.hold:.1f}s hold: {at_end[2]:+.4f} rad")
    if at_duration:
        shrink = abs(at_duration[2]) - abs(at_end[2])
        verdict = ("still converging (error shrank %.4f rad during hold) - looks like a "
                   "speed/timing problem" % shrink) if shrink > 0.01 else \
                  ("flat / not converging further during hold - looks like a steady-state offset")
        print(f"verdict: {verdict}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
