#!/usr/bin/env python3
"""Send Nav2 to a saved named waypoint and watch it through to completion.

  python3 tools/goto_waypoint.py <name> [--timeout 120]

Same pattern as tools/nav_forward.py: publishes on /goal_pose (what RViz's "2D Goal Pose" does,
so RViz and rerun_bridge.py show it too) and watches /navigate_to_pose/_action/status rather
than holding an action goal handle. Needs navigation.launch.py running with localization:=true -
see tools/save_waypoint.py to create waypoints, tools/list_waypoints.py to see what's saved.
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

import waypoint_store

TERMINAL = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted"}


class GotoWaypoint(Node):
    def __init__(self):
        super().__init__("goto_waypoint")
        self.status = None
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self.on_status, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cancel_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")

    def on_status(self, msg):
        if msg.status_list:
            self.status = msg.status_list[-1].status

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def cancel(self):
        if self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.cancel_client.call_async(CancelGoal.Request())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("--timeout", type=float, default=120.0, help="cancel after this many seconds")
    args = parser.parse_args()
    try:
        w = waypoint_store.get(args.name)
    except waypoint_store.NotFoundError as e:
        sys.exit(str(e))

    rclpy.init()
    node = GotoWaypoint()
    try:
        if not ActionClient(node, NavigateToPose, "navigate_to_pose").wait_for_server(timeout_sec=10.0):
            sys.exit("Nav2 is not up (no navigate_to_pose action server), start navigation.launch.py")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = w["x"], w["y"]
        goal.pose.orientation.z = math.sin(w["yaw"] / 2)
        goal.pose.orientation.w = math.cos(w["yaw"] / 2)
        node.status = None
        node.goal_pub.publish(goal)
        print(f"going to '{args.name}': x={w['x']:.3f} y={w['y']:.3f} yaw={math.degrees(w['yaw']):.1f} deg")

        t0 = time.monotonic()
        reason = None
        active = False
        while reason is None:
            node.spin(0.05)
            active = active or node.status in (
                GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            if active and node.status in TERMINAL:
                reason = TERMINAL[node.status]
            elif not active and time.monotonic() - t0 > 5.0:
                reason = "CANCELLED: Nav2 did not accept the goal within 5 s"
            elif time.monotonic() - t0 > args.timeout:
                reason = f"CANCELLED: {args.timeout:.0f} s timeout"
        if reason.startswith("CANCELLED"):
            node.cancel()
        print(f"{reason} after {time.monotonic() - t0:.1f} s")
        if reason != "succeeded":
            sys.exit(1)
    except KeyboardInterrupt:
        print("interrupted, cancelling")
        node.cancel()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
