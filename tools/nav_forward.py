#!/usr/bin/env python3
"""Sends Nav2 a goal straight ahead of the robot and watches it, with limits.

The goal is published on /goal_pose, the same way RViz's "2D Goal Pose" does,
so RViz and rerun_bridge.py show it. The goal is DISTANCE metres ahead of the
current pose, same heading. The robot is cancelled if it travels further than
DISTANCE + OVERSHOOT, if TIMEOUT seconds pass, or on Ctrl-C.

Usage (bringup and navigation already running):
    python3 tools/nav_forward.py                # 0.7 m ahead
    python3 tools/nav_forward.py --distance 0.5
    python3 tools/nav_forward.py --dry-run      # print the goal, send nothing

With the wheels lifted the robot does not move but odometry still counts
the wheel turns, so this is also a safe check of the whole chain.
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

MAX_DISTANCE = 1.0  # m, refused above this
TERMINAL = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted"}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavForward(Node):
    def __init__(self):
        super().__init__("nav_forward")
        self.odom = None
        self.status = None
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.on_odom, 20)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self.on_status, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.cancel_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

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
        for _ in range(10):  # and command zero, in case cancelling is slow
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            self.spin(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--distance", type=float, default=0.7, help="metres ahead (default 0.7)")
    parser.add_argument("--overshoot", type=float, default=0.25, help="cancel this far past the goal")
    parser.add_argument("--timeout", type=float, default=25.0, help="cancel after this many seconds")
    parser.add_argument("--dry-run", action="store_true", help="print the goal and send nothing")
    args = parser.parse_args()
    if not 0.0 < args.distance <= MAX_DISTANCE:
        sys.exit(f"--distance must be above 0 and at most {MAX_DISTANCE} m")

    rclpy.init()
    node = NavForward()
    try:
        start = time.monotonic()
        while node.odom is None and time.monotonic() - start < 10.0:
            node.spin(0.1)
        if node.odom is None:
            sys.exit("no odometry on /diff_drive_controller/odom, is bringup running?")
        x0, y0, yaw0 = node.odom
        gx, gy = x0 + args.distance * math.cos(yaw0), y0 + args.distance * math.sin(yaw0)
        print(f"start ({x0:.2f}, {y0:.2f}) heading {math.degrees(yaw0):.0f} deg, "
              f"goal ({gx:.2f}, {gy:.2f}) {args.distance:.2f} m ahead")
        if args.dry_run:
            return

        if not ActionClient(node, NavigateToPose, "navigate_to_pose").wait_for_server(timeout_sec=10.0):
            sys.exit("Nav2 is not up (no navigate_to_pose action server), start navigation.launch.py")

        goal = PoseStamped()
        goal.header.frame_id = "map"  # identical to odom until there is localisation
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw0 / 2), math.cos(yaw0 / 2)
        node.status = None
        node.goal_pub.publish(goal)

        t0 = time.monotonic()
        reason = None
        active = False  # this goal was seen running, so a terminal status is its own
        while reason is None:
            node.spin(0.05)
            x, y, _ = node.odom
            travelled = math.hypot(x - x0, y - y0)
            active = active or node.status in (
                GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            if active and node.status in TERMINAL:
                reason = TERMINAL[node.status]
            elif not active and time.monotonic() - t0 > 5.0:
                reason = "CANCELLED: Nav2 did not accept the goal within 5 s"
            elif travelled > args.distance + args.overshoot:
                reason = f"CANCELLED: travelled {travelled:.2f} m, past the {args.overshoot:.2f} m overshoot limit"
            elif time.monotonic() - t0 > args.timeout:
                reason = f"CANCELLED: {args.timeout:.0f} s timeout"
        if reason.startswith("CANCELLED"):
            node.cancel()
        node.spin(1.0)
        x, y, yaw = node.odom
        print(f"{reason} after {time.monotonic() - t0:.1f} s: end ({x:.2f}, {y:.2f}), "
              f"travelled {math.hypot(x - x0, y - y0):.2f} m, "
              f"heading change {math.degrees(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))):.1f} deg, "
              f"goal error {math.hypot(gx - x, gy - y):.2f} m")
    except KeyboardInterrupt:
        print("interrupted, cancelling")
        node.cancel()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
