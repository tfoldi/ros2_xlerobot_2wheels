#!/usr/bin/env python3
"""Wheel radius and wheel separation calibration, see ARCHITECTURE §6.

Two tests:
  straight-line: drive a known distance, compare to /odom, scale wheel_radius
  rotate-in-place: turn N full turns, compare reported yaw, scale wheel_separation

Do this before touching any Nav2 tuning parameter. A wrong wheel_radius (and
upstream's default is likely wrong by ~27%, see ARCHITECTURE §1.4) means
every subsequent tuning session is compensating for a units error instead of
an actual controller problem.

Not implemented yet: needs a working diff_drive_controller odometry topic to
read from, which is phase 1 work. This is the shape the tool should take.
"""

import argparse

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomCalibrator(Node):
    def __init__(self):
        super().__init__("odom_calibrator")
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.on_odom, 10)
        self.start_pose = None
        self.last_pose = None

    def on_odom(self, msg: Odometry):
        self.last_pose = msg.pose.pose
        if self.start_pose is None:
            self.start_pose = msg.pose.pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "test", choices=["straight-line", "rotate-in-place"],
        help="which calibration test to run",
    )
    parser.add_argument(
        "--measured", type=float, required=True,
        help="tape-measured distance in metres (straight-line) or turns (rotate-in-place)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = OdomCalibrator()
    node.get_logger().info(
        f"listening on /diff_drive_controller/odom for the {args.test} test, "
        f"drive it now, then Ctrl-C to compute the scale factor"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # TODO: on Ctrl-C, compute distance/yaw travelled from start_pose to
        # last_pose, divide by args.measured, print the scale factor to apply
        # to wheel_radius or wheel_separation in robot_dimensions.yaml.
        node.get_logger().info("not implemented yet, see ARCHITECTURE §6")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
