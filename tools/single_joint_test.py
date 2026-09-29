#!/usr/bin/env python3
"""Move exactly one joint of an arm/gripper controller, holding every other joint of that
controller at its current measured position (allow_partial_joints_goal is false, a goal must
name all of a controller's joints).

For the step-4 live test: one joint at a time, arm held by hand, tools/estop.py ready.

  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true torque_off:=false
  python3 tools/single_joint_test.py right_arm_controller right_arm_elbow_flex_joint 0.2

Positional args: controller name, joint name, delta in rad (added to the joint's current
position). --duration (default 2.0 s) is the time_from_start for the single trajectory point.
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
}


class SingleJointTest(Node):
    def __init__(self, controller):
        super().__init__("single_joint_test")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.client = ActionClient(self, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_ready(self, joints):
        if not self.client.wait_for_server(timeout_sec=10.0):
            sys.exit("action server not available, is the controller running?")
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
    parser.add_argument("delta", type=float, help="rad, added to the joint's current position")
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    joints = CONTROLLER_JOINTS[args.controller]
    if args.joint not in joints:
        sys.exit(f"{args.joint} is not in {args.controller}: {joints}")

    rclpy.init()
    node = SingleJointTest(args.controller)
    node.wait_ready(joints)

    current = {j: node.positions[j] for j in joints}
    target = dict(current)
    target[args.joint] = current[args.joint] + args.delta
    print("current:", ", ".join(f"{j}={v:.3f}" for j, v in current.items()))
    print("target: ", ", ".join(f"{j}={v:.3f}" for j, v in target.items()))

    point = JointTrajectoryPoint(positions=[target[j] for j in joints])
    point.time_from_start = Duration(seconds=args.duration).to_msg()
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joints
    goal.trajectory.points = [point]

    send = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        sys.exit("goal not accepted")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.duration + 10.0)
    result = result_future.result()
    if result is None:
        sys.exit("no result (timed out)")

    error_code = result.result.error_code
    print(f"error_code: {error_code}" + (" (SUCCESSFUL)" if error_code == 0 else " (FAILED)"))
    rclpy.spin_once(node, timeout_sec=0.3)
    landed = node.positions[args.joint]
    print(f"landed: {args.joint}={landed:.4f}, target={target[args.joint]:.4f}, "
          f"error={landed - target[args.joint]:+.4f} rad")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if error_code == 0 else 1)


if __name__ == "__main__":
    main()
