#!/usr/bin/env python3
"""Streams cmd_vel/odom/wheel data into Rerun for timeline debugging, opens the
Rerun Viewer. Complements RViz/Foxglove rather than replacing them: this is
for scrubbing scalar time series (commanded vs actual velocity), not the 3D
scene.

Usage (bringup already running):
    python3 tools/rerun_bridge.py
"""

import math

import rclpy
import rerun as rr
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RerunBridge(Node):
    def __init__(self):
        super().__init__("rerun_bridge")
        self.path_x = []
        self.path_y = []
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.on_odom, 20)
        self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel", self.on_cmd_vel_final, 20)
        self.create_subscription(TwistStamped, "/cmd_vel_nav", self.on_cmd_vel_nav, 20)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 20)

    def on_odom(self, msg: Odometry):
        rr.set_time("ros_time", timestamp=stamp_seconds(msg.header))
        p = msg.pose.pose.position
        self.path_x.append(p.x)
        self.path_y.append(p.y)
        rr.log("odom/path", rr.LineStrips2D([list(zip(self.path_x, self.path_y))]))
        rr.log("odom/pose", rr.Points2D([[p.x, p.y]]))
        rr.log("odom/yaw", rr.Scalars(yaw_from_quat(msg.pose.pose.orientation)))
        rr.log("odom/linear_x", rr.Scalars(msg.twist.twist.linear.x))
        rr.log("odom/angular_z", rr.Scalars(msg.twist.twist.angular.z))

    def on_cmd_vel_final(self, msg: TwistStamped):
        rr.set_time("ros_time", timestamp=stamp_seconds(msg.header))
        rr.log("cmd_vel/linear_x", rr.Scalars(msg.twist.linear.x))
        rr.log("cmd_vel/angular_z", rr.Scalars(msg.twist.angular.z))

    def on_cmd_vel_nav(self, msg: TwistStamped):
        rr.set_time("ros_time", timestamp=stamp_seconds(msg.header))
        rr.log("cmd_vel_nav/linear_x", rr.Scalars(msg.twist.linear.x))
        rr.log("cmd_vel_nav/angular_z", rr.Scalars(msg.twist.angular.z))

    def on_joint_states(self, msg: JointState):
        rr.set_time("ros_time", timestamp=stamp_seconds(msg.header))
        for name, velocity in zip(msg.name, msg.velocity):
            if name in ("left_wheel_joint", "right_wheel_joint"):
                rr.log(f"wheels/{name}/velocity", rr.Scalars(velocity))


def main():
    rr.init("xlerobot", spawn=True)
    rclpy.init()
    node = RerunBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
