#!/usr/bin/env python3
"""Plan a single-joint move through MoveIt (plan_only, no execution), then send that exact
trajectory directly to the controller's FollowJointTrajectory action (bypassing move_group's own
executor), logging the controller's own desired/actual/error feedback at full rate.

Does not touch controllers.yaml - uses whatever tolerance is currently configured, so it can
still abort early; the printed feedback log is informative either way (successful or partial).

  python3 moveit_tracking_diag.py <joint_name> <delta_rad> [--scaling 0.8]
"""
import argparse
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ["right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_flex_joint",
              "right_arm_wrist_flex_joint", "right_arm_wrist_roll_joint"]


class Diag(Node):
    def __init__(self):
        super().__init__("moveit_tracking_diag")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.controller = ActionClient(self, FollowJointTrajectory, "/right_arm_controller/follow_joint_trajectory")
        self.feedback_log = []

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_ready(self):
        self.move_group.wait_for_server(timeout_sec=30.0)
        self.controller.wait_for_server(timeout_sec=30.0)
        deadline = time.time() + 15
        while not all(j in self.positions for j in ARM_JOINTS) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

    def plan(self, joint, delta, scaling):
        target = {j: self.positions[j] for j in ARM_JOINTS}
        target[joint] += delta
        request = MotionPlanRequest()
        request.group_name = "right_arm"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = scaling
        request.max_acceleration_scaling_factor = scaling
        constraints = Constraints()
        for j, v in target.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=j, position=float(v), tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
        request.goal_constraints.append(constraints)
        options = PlanningOptions()
        options.plan_only = True
        goal_msg = MoveGroup.Goal(request=request, planning_options=options)
        send = self.move_group.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            sys.exit(f"plan failed, code {result.result.error_code.val}")
        return result.result.planned_trajectory.joint_trajectory

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        t = fb.header.stamp.sec + fb.header.stamp.nanosec * 1e-9
        row = {"t": t}
        for i, name in enumerate(fb.joint_names):
            row[name] = {
                "desired": fb.desired.positions[i] if fb.desired.positions else None,
                "actual": fb.actual.positions[i] if fb.actual.positions else None,
                "error": fb.error.positions[i] if fb.error.positions else None,
            }
        self.feedback_log.append(row)

    def execute(self, trajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        # zero stamp = start on receipt; forwarding the original planning-time stamp would make
        # the controller think the trajectory should already be under way, aborting instantly
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        send = self.controller.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            sys.exit("goal not accepted")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        result = result_future.result()
        return result.result.error_code if result else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("joint", choices=ARM_JOINTS)
    parser.add_argument("delta", type=float)
    parser.add_argument("--scaling", type=float, default=0.8)
    args = parser.parse_args()

    rclpy.init()
    node = Diag()
    node.wait_ready()
    traj = node.plan(args.joint, args.delta, args.scaling)
    print(f"planned {len(traj.points)} points, duration "
          f"{traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec*1e-9:.3f}s")
    error_code = node.execute(traj)
    print(f"execution result error_code: {error_code}")

    print(f"\n{len(node.feedback_log)} feedback samples for {args.joint}:")
    print(f"{'t_rel':>8} {'desired':>10} {'actual':>10} {'error':>10}")
    t0 = node.feedback_log[0]["t"] if node.feedback_log else 0
    worst = 0.0
    for row in node.feedback_log:
        j = row[args.joint]
        worst = max(worst, abs(j["error"]))
        print(f"{row['t']-t0:8.3f} {j['desired']:10.4f} {j['actual']:10.4f} {j['error']:10.4f}")
    print(f"\nworst abs error: {worst:.4f} rad")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
