#!/usr/bin/env python3
"""Save the robot's current localized pose as a named waypoint.

  python3 tools/save_waypoint.py <name>

Reads /amcl_pose, so navigation.launch.py must be running with localization:=true (AMCL against
a saved map) - not just SLAM or the odom-only static identity, either of those would save a
wrong or drifting pose. Overwrites an existing entry of the same name. See waypoint_store.py for
the storage format and list_waypoints.py / goto_waypoint.py to inspect and use saved waypoints.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

import waypoint_store


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseReader(Node):
    def __init__(self):
        super().__init__("save_waypoint")
        self.pose = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, 10)

    def on_pose(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    args = parser.parse_args()

    rclpy.init()
    node = PoseReader()
    deadline = time.time() + 10
    while node.pose is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.pose is None:
        sys.exit("no /amcl_pose - is navigation.launch.py running with localization:=true?")

    x, y, yaw = node.pose
    waypoints = waypoint_store.load()
    waypoints[args.name] = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}
    waypoint_store.save(waypoints)
    print(f"saved '{args.name}': x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} deg")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
