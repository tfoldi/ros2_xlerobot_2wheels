#!/usr/bin/env python3
"""Quick MoveIt2 check for the "head" group: plan+execute to a small joint target via move_group,
matching moveit_smoke_test.py's pattern."""
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HeadTest(Node):
    def __init__(self):
        super().__init__("head_moveit_test")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.move_group = ActionClient(self, MoveGroup, "/move_action")

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_ready(self):
        self.move_group.wait_for_server(timeout_sec=30.0)
        deadline = time.time() + 15
        while ("head_pan_joint" not in self.positions or "head_tilt_joint" not in self.positions) \
                and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

    def move(self, goal):
        request = MotionPlanRequest()
        request.group_name = "head"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.0
        request.max_acceleration_scaling_factor = 0.0
        constraints = Constraints()
        for joint, value in goal.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=joint, position=float(value), tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
        request.goal_constraints.append(constraints)
        options = PlanningOptions()
        options.plan_only = False
        goal_msg = MoveGroup.Goal(request=request, planning_options=options)
        send = self.move_group.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print("FAIL: goal not accepted")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        result = result_future.result()
        code = result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            print(f"FAIL: MoveIt error code {code}")
            return
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.3)
        worst = max(abs(self.positions[j] - v) for j, v in goal.items())
        print(f"{'ok  ' if worst < 0.08 else 'FAIL'} worst joint error {worst:.4f} rad "
              f"(current: pan={self.positions['head_pan_joint']:.4f} tilt={self.positions['head_tilt_joint']:.4f})")


def main():
    rclpy.init()
    node = HeadTest()
    node.wait_ready()
    print("current:", {j: round(node.positions[j], 4) for j in ("head_pan_joint", "head_tilt_joint")})
    print("move to a small target (pan=0.2, tilt=0.1):")
    node.move({"head_pan_joint": 0.2, "head_tilt_joint": 0.1})
    print("move back to forward (0, 0):")
    node.move({"head_pan_joint": 0.0, "head_tilt_joint": 0.0})
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
