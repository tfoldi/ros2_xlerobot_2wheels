#!/usr/bin/env python3
"""Drive a calibration motion and suggest a parameter correction, see ARCHITECTURE §6.

  straight-line   --distance METRES   drives forward, suggests wheel_radius
  rotate-in-place --turns TURNS       rotates in place, suggests wheel_separation_multiplier

Stops automatically once /diff_drive_controller/odom reports the commanded
target, then prompts for the actual tape/protractor measurement.

Usage:
  python3 tools/calibrate_wheel_odometry.py straight-line --distance 1.0
  python3 tools/calibrate_wheel_odometry.py rotate-in-place --turns 0.5
"""

import argparse
import math
import pathlib

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

CONTROLLERS_YAML = (
    pathlib.Path(__file__).resolve().parent.parent
    / "src/xlerobot_bringup/config/controllers.yaml"
)
CMD_VEL_TOPIC = "/diff_drive_controller/cmd_vel"
ODOM_TOPIC = "/diff_drive_controller/odom"


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def current_param(name, default):
    params = yaml.safe_load(CONTROLLERS_YAML.read_text())
    return params.get("diff_drive_controller", {}).get("ros__parameters", {}).get(name, default)


class CalibrationDrive(Node):
    def __init__(self, mode, target, speed):
        super().__init__("calibrate_wheel_odometry")
        self.mode = mode
        self.target = target
        self.speed = speed
        self.pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.cmd_timer = self.create_timer(1.0 / 20.0, self.on_cmd_tick)
        self.baseline_xy = None
        self.baseline_yaw = None
        self.accumulated_yaw = 0.0
        self.result = None

    def on_odom(self, msg: Odometry):
        if self.result is not None:
            return
        pos = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.mode == "straight-line":
            if self.baseline_xy is None:
                self.baseline_xy = (pos.x, pos.y)
                return
            progress = math.hypot(pos.x - self.baseline_xy[0], pos.y - self.baseline_xy[1])
        else:
            if self.baseline_yaw is None:
                self.baseline_yaw = yaw
                return
            delta = yaw - self.baseline_yaw
            if delta > math.pi:
                delta -= 2.0 * math.pi
            elif delta < -math.pi:
                delta += 2.0 * math.pi
            self.accumulated_yaw += delta
            self.baseline_yaw = yaw
            progress = abs(self.accumulated_yaw)

        if progress >= self.target:
            self.result = progress
            self.publish_cmd(0.0)

    def on_cmd_tick(self):
        if self.result is not None or (self.baseline_xy is None and self.baseline_yaw is None):
            return
        self.publish_cmd(self.speed)

    def publish_cmd(self, value):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.mode == "straight-line":
            msg.twist.linear.x = value
        else:
            msg.twist.angular.z = value
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("straight-line")
    p.add_argument("--distance", type=float, required=True, help="metres to drive")
    p.add_argument("--speed", type=float, default=0.08, help="m/s")

    p = sub.add_parser("rotate-in-place")
    p.add_argument("--turns", type=float, required=True, help="full rotations")
    p.add_argument("--speed", type=float, default=0.3, help="rad/s")

    args = parser.parse_args()
    target = args.distance if args.mode == "straight-line" else args.turns * 2.0 * math.pi

    rclpy.init()
    node = CalibrationDrive(args.mode, target, args.speed)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.mode == "straight-line":
        print(f"odom distance: {node.result:.4f} m")
        measured = float(input("actual measured distance (m): "))
        r_cfg = current_param("wheel_radius", 0.0625)
        r_new = r_cfg * (measured / node.result)
        print(f"suggested wheel_radius: {r_new:.5f} (current {r_cfg})")
    else:
        odom_deg = math.degrees(node.result)
        print(f"odom rotation: {odom_deg:.2f} deg")
        measured = float(input("actual measured rotation (deg): "))
        m_cfg = current_param("wheel_separation_multiplier", 1.0)
        m_new = m_cfg * (odom_deg / measured)
        print(f"suggested wheel_separation_multiplier: {m_new:.4f} (current {m_cfg})")


if __name__ == "__main__":
    main()
