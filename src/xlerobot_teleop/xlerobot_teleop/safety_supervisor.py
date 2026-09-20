#!/usr/bin/env python3
"""Watches per-joint temperature and current from sts_hardware_interface and
stops the robot before something cooks or stalls. See ARCHITECTURE §4.4.

Not wired up yet, hardware_interface exports state through /joint_states, and
whether that message carries effort/temperature/current the way
sts_hardware_interface exposes them, or whether we need its own diagnostics
topic instead, is one of the week 1 things to check (ARCHITECTURE §7 R1/R2).
This is the shape the node should end up with, thresholds are placeholders.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# STS3215 datasheet gives no hard shutdown temperature for continuous duty.
# 65C is conservative for a servo with no active cooling, doing sustained
# navigation work. Tune down if joints run hotter than expected in practice.
TEMPERATURE_LIMIT_C = 65.0

# Placeholder, current units and scale depend on what sts_hardware_interface
# actually reports, check against the real hardware before trusting this.
STALL_CURRENT_THRESHOLD = 0.8
STALL_VELOCITY_THRESHOLD = 0.01

WATCHDOG_TIMEOUT_S = 0.25  # tighter than upstream's 500ms host watchdog


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("xlerobot_safety_supervisor")
        self.estop_pub = self.create_publisher(Bool, "e_stop", 10)
        self.create_subscription(JointState, "joint_states", self.on_joint_states, 10)
        self.last_msg_time = None
        self.create_timer(0.1, self.check_watchdog)
        self.get_logger().info("safety supervisor up, watchdog %.2fs" % WATCHDOG_TIMEOUT_S)

    def on_joint_states(self, msg: JointState):
        self.last_msg_time = self.get_clock().now()
        # TODO: JointState has no standard temperature/current field.
        # Either sts_hardware_interface publishes those on a separate topic
        # (diagnostics_msgs?), or effort is being reused as a stand-in. Check
        # the actual driver before writing the real threshold logic here.
        pass

    def check_watchdog(self):
        if self.last_msg_time is None:
            return
        age = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if age > WATCHDOG_TIMEOUT_S:
            self.trigger_estop("no joint_states for %.2fs" % age)

    def trigger_estop(self, reason: str):
        self.get_logger().error("e-stop: %s" % reason)
        self.estop_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
