#!/usr/bin/env python3
"""Plan and execute a move to a saved named position via MoveIt (collision-aware).

  python3 tools/goto_position.py <name> [--scaling 0.3]

Needs move_group running. See tools/save_position.py to create positions, tools/list_positions.py
to see what's saved, tools/playback_sequence.py to move through several in a row.
"""
import argparse
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

import position_store


class MoveError(RuntimeError):
    """Raised by GotoPosition on any planning/execution failure - callers embedding this class
    in a long-lived process (tools/mcp_server/) need something catchable, unlike sys.exit."""


def joint_goal_request(group, goal, scaling):
    """Build a MotionPlanRequest for a joint-space goal. Shared with tools/mcp_server/bridge.py,
    which sends it through its own long-lived action client instead of this module's Node."""
    request = MotionPlanRequest()
    request.group_name = group
    request.num_planning_attempts = 5
    request.allowed_planning_time = 10.0
    request.max_velocity_scaling_factor = scaling
    request.max_acceleration_scaling_factor = scaling
    constraints = Constraints()
    for joint, value in goal.items():
        constraints.joint_constraints.append(JointConstraint(
            joint_name=joint, position=float(value), tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
    request.goal_constraints.append(constraints)
    return request


class GotoPosition(Node):
    def __init__(self):
        super().__init__("goto_position")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.move_group = ActionClient(self, MoveGroup, "/move_action")

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_ready(self, joints):
        if not self.move_group.wait_for_server(timeout_sec=15.0):
            raise MoveError("/move_action not available - is move_group running?")
        deadline = time.time() + 10
        while not all(j in self.positions for j in joints) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        missing = [j for j in joints if j not in self.positions]
        if missing:
            raise MoveError(f"no /joint_states for: {', '.join(missing)} - is bringup running?")

    def move(self, group, goal, scaling):
        request = joint_goal_request(group, goal, scaling)
        options = PlanningOptions()
        options.plan_only = False
        goal_msg = MoveGroup.Goal(request=request, planning_options=options)
        send = self.move_group.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise MoveError("goal not accepted")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        result = result_future.result()
        if result is None:
            raise MoveError("no result (timed out)")
        code = result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise MoveError(f"MoveIt error code {code}")
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.3)
        worst = max(abs(self.positions[j] - v) for j, v in goal.items())
        print(f"landed, worst joint error {worst:.4f} rad")
        return worst


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("--scaling", type=float, default=0.0, help="0 = joint_limits.yaml default")
    args = parser.parse_args()
    try:
        entry = position_store.get(args.name)
    except position_store.NotFoundError as e:
        sys.exit(str(e))

    rclpy.init()
    node = GotoPosition()
    try:
        node.wait_ready(list(entry["joints"]))
        print(f"moving to '{args.name}' ({entry['group']})")
        node.move(entry["group"], entry["joints"], args.scaling)
    except MoveError as e:
        sys.exit(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
