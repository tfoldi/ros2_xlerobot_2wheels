#!/usr/bin/env python3
"""End-to-end check of the MoveIt setup against whatever is running (mock, the Isaac twin, the robot).

Needs the bringup with the arm controllers and move_group:

  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_mock_hardware:=true torque_off:=false
  ros2 launch xlerobot_moveit_config move_group.launch.py
  python3 tools/moveit_smoke_test.py [--arm left|right|both]

For each arm it plans and executes a move to the rest pose and to a nearby pose through the
FollowJointTrajectory controllers, moves the gripper, checks position IK against FK and that a goal
in collision is refused. It
compares where the joints end up with the goal. Moves are small when the arm already is near
the target. On the real robot run it with the arm held, like any other first move.
"""

import argparse
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
import random
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
REST = {"shoulder_pan": 0.0, "shoulder_lift": 2.6, "elbow_flex": -3.0, "wrist_flex": 0.0,
        "wrist_roll": 0.0, "gripper": 0.35}
TOLERANCE = 0.05  # rad, what the trajectory controllers are held to (goal tolerance 0.06)


class SmokeTest(Node):
    def __init__(self):
        super().__init__("moveit_smoke_test")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.failures = []

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_ready(self):
        for what, ready in (("/move_action", lambda: self.move_group.wait_for_server(timeout_sec=30.0)),
                            ("/compute_fk", lambda: self.fk.wait_for_service(timeout_sec=30.0)),
                            ("/compute_ik", lambda: self.ik.wait_for_service(timeout_sec=30.0)),
                            ("/check_state_validity", lambda: self.validity.wait_for_service(timeout_sec=30.0))):
            if not ready():
                sys.exit(f"{what} not available, is move_group running?")
        deadline = time.time() + 30
        while not self.positions and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.positions:
            sys.exit("no /joint_states")

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def check(self, name, ok, detail=""):
        print(f"  {'ok  ' if ok else 'FAIL'} {name} {detail}")
        if not ok:
            self.failures.append(name)

    def move(self, group, goal, label, expect_failure=False):
        """Plan and execute a joint goal, then compare the joints with it. expect_failure: MoveIt must refuse."""
        request = MotionPlanRequest()
        request.group_name = group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.0  # 0 = the default in joint_limits.yaml
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
            self.check(label, False, "goal not accepted")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        result = result_future.result()
        if result is None:
            self.check(label, False, "no result")
            return
        code = result.result.error_code.val
        if expect_failure:
            self.check(label, code != MoveItErrorCodes.SUCCESS, f"(MoveIt error code {code})")
            return
        if code != MoveItErrorCodes.SUCCESS:
            self.check(label, False, f"MoveIt error code {code}")
            return
        self.spin_for(1.0)
        worst = max(abs(self.positions[j] - v) for j, v in goal.items())
        self.check(label, worst < TOLERANCE, f"(worst joint error {worst:.3f} rad)")

    def find_colliding_pose(self, side, tries=2000):
        """A pose of one arm the planning scene calls invalid (a collision, joint limits are respected)."""
        rng = random.Random(3)
        ranges = {"shoulder_pan": (-2.0, 2.0), "shoulder_lift": (-0.3, 3.4), "elbow_flex": (-3.2, 0.3),
                  "wrist_flex": (-1.6, 1.6), "wrist_roll": (-2.0, 2.0)}
        names = [f"{side}_arm_{j}_joint" for j in ARM_JOINTS]
        for _ in range(tries):
            pose = {f"{side}_arm_{j}_joint": rng.uniform(*ranges[j]) for j in ARM_JOINTS}
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = names
            request.robot_state.joint_state.position = [pose[n] for n in names]
            future = self.validity.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            result = future.result()
            if result is not None and not result.valid:
                return pose
        return None

    def refuse_collision(self, side, label):
        """MoveIt has to refuse a goal that is in collision, and the arm must not move."""
        pose = self.find_colliding_pose(side)
        if pose is None:
            self.check(label, False, "found no colliding pose to try")
            return
        before = {j: self.positions[j] for j in pose}
        before_failures = len(self.failures)
        self.move(f"{side}_arm", pose, label + " (expected to be refused)", expect_failure=True)
        moved = max(abs(self.positions[j] - before[j]) for j in pose)
        self.check(label + ", arm stayed", moved < 0.02, f"(moved {moved:.3f} rad)")
        del before_failures

    def ik_against_fk(self, group, tip, joints, label):
        """FK of a joint set, then position-only IK for that point: the solution must reach it."""
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "base_footprint"
        fk_request.fk_link_names = [tip]
        fk_request.robot_state.joint_state.name = list(joints)
        fk_request.robot_state.joint_state.position = [float(v) for v in joints.values()]
        future = self.fk.call_async(fk_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        pose = future.result().pose_stamped[0]
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = group
        ik_request.ik_request.avoid_collisions = False
        ik_request.ik_request.pose_stamped = pose
        ik_request.ik_request.timeout.sec = 1
        ik_request.ik_request.robot_state.joint_state.name = list(joints)
        ik_request.ik_request.robot_state.joint_state.position = [float(v) + 0.2 for v in joints.values()]
        future = self.ik.call_async(ik_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        solved = future.result()
        if solved.error_code.val != MoveItErrorCodes.SUCCESS:
            self.check(label, False, f"IK error code {solved.error_code.val}")
            return
        solution = dict(zip(solved.solution.joint_state.name, solved.solution.joint_state.position))
        back = GetPositionFK.Request()
        back.header.frame_id = "base_footprint"
        back.fk_link_names = [tip]
        back.robot_state.joint_state.name = list(joints)
        back.robot_state.joint_state.position = [float(solution[j]) for j in joints]
        future = self.fk.call_async(back)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        got = future.result().pose_stamped[0].pose.position
        want = pose.pose.position
        error = ((got.x - want.x) ** 2 + (got.y - want.y) ** 2 + (got.z - want.z) ** 2) ** 0.5
        self.check(label, error < 0.005, f"(position error {error * 1000:.1f} mm)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both")
    args = parser.parse_args()
    sides = ["left", "right"] if args.arm == "both" else [args.arm]

    rclpy.init()
    node = SmokeTest()
    node.wait_ready()
    for side in sides:
        print(f"{side} arm")
        joints = {f"{side}_arm_{j}_joint": REST[j] for j in ARM_JOINTS}
        node.move(f"{side}_arm", joints, "move to the rest pose")
        near = dict(joints)
        near[f"{side}_arm_shoulder_pan_joint"] = 0.3
        near[f"{side}_arm_wrist_flex_joint"] = 0.3
        node.move(f"{side}_arm", near, "move to a nearby pose")
        gripper = f"{side}_arm_gripper_joint"
        node.move(f"{side}_gripper", {gripper: -0.7}, "open the gripper")
        node.move(f"{side}_gripper", {gripper: 0.35}, "close the gripper")
        reach = {f"{side}_arm_{j}_joint": v for j, v in
                 zip(ARM_JOINTS, (0.2, 1.4, -1.6, 0.4, 0.0))}
        node.ik_against_fk(f"{side}_arm", f"{side}_arm_gripper_link", reach, "position IK reaches the FK point")
        node.refuse_collision(side, "goal in collision")
    node.destroy_node()
    rclpy.shutdown()
    if node.failures:
        print(f"{len(node.failures)} failed: {', '.join(node.failures)}")
        sys.exit(1)
    print("all passed")


if __name__ == "__main__":
    main()
