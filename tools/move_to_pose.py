#!/usr/bin/env python3
"""Plan and execute a move of an arm's gripper to a Cartesian point via MoveIt (collision-aware).

  python3 tools/move_to_pose.py <left|right> <x> <y> <z> [--frame base_footprint] [--scaling 0.3]

Position-only, matching kinematics.yaml's pick_ik config (rotation_scale: 0.0) - the gripper's
orientation is whatever the IK solution lands on, there's no way to also constrain it. For a
joint-space move to a named saved position, use tools/goto_position.py instead; this is for
"put the gripper at this point in space" (pick approach/descend/lift), where the target comes
from vision rather than a fixed pose.

Needs move_group running.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, MotionPlanRequest, MoveItErrorCodes,
    PlanningOptions, PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

import position_store

from goto_position import MoveError

# How close the constraint region requires the tip link to land - a point, not a volume, but
# MoveIt's PositionConstraint always needs some region, so this is deliberately tight rather
# than 0 (a zero-radius sphere is degenerate for some planners).
TOLERANCE_M = 0.01


def position_goal_request(group, tip_link, frame, point, scaling):
    """Build a MotionPlanRequest for a Cartesian position goal. Shared with
    tools/mcp_server/bridge.py, which sends it through its own long-lived action client instead
    of this module's Node."""
    request = MotionPlanRequest()
    request.group_name = group
    request.num_planning_attempts = 5
    request.allowed_planning_time = 10.0
    request.max_velocity_scaling_factor = scaling
    request.max_acceleration_scaling_factor = scaling

    constraint = PositionConstraint()
    constraint.header.frame_id = frame
    constraint.link_name = tip_link
    constraint.target_point_offset = Point(x=0.0, y=0.0, z=0.0)
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[TOLERANCE_M])
    constraint.constraint_region = BoundingVolume(
        primitives=[sphere],
        primitive_poses=[Pose(position=Point(x=point[0], y=point[1], z=point[2]))],
    )
    constraint.weight = 1.0
    request.goal_constraints.append(Constraints(position_constraints=[constraint]))
    return request


class MoveToPose(Node):
    def __init__(self):
        super().__init__("move_to_pose")
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

    def move(self, group, tip_link, frame, point, scaling):
        request = position_goal_request(group, tip_link, frame, point, scaling)
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
        print(f"landed near ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) in {frame}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("arm", choices=["left", "right"])
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("--frame", default="base_footprint")
    parser.add_argument("--scaling", type=float, default=0.0, help="0 = joint_limits.yaml default")
    args = parser.parse_args()

    group = f"{args.arm}_arm"
    tip_link = position_store.GROUP_TIP_LINK[group]
    joints = position_store.GROUP_JOINTS[group]

    rclpy.init()
    node = MoveToPose()
    try:
        node.wait_ready(joints)
        print(f"moving {group} to ({args.x}, {args.y}, {args.z}) in {args.frame}")
        node.move(group, tip_link, args.frame, (args.x, args.y, args.z), args.scaling)
    except MoveError as e:
        sys.exit(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
