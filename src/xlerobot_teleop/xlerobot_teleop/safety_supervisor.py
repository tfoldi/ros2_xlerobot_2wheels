#!/usr/bin/env python3
"""Watches per-joint temperature and current from sts_hardware_interface and
stops the robot before something cooks or stalls. See ARCHITECTURE §4.4.

temperature/current/voltage are exported as custom-named state interfaces,
not the standard position/velocity/effort triple. joint_state_broadcaster
can't put those in sensor_msgs/JointState, there's no field for them, they
go out on /dynamic_joint_states instead (control_msgs/DynamicJointState).
This subscribes to that topic; the threshold logic below is still a
placeholder.

Rust (rclrs) is the target for nodes written for this project; this one is
still Python/rclpy since rclrs on Jazzy has open install/build issues and
this stub isn't load-bearing yet.

Don't wire trigger_estop to sts_hardware_interface's own /emergency_stop
service: with two bus instances that service name collides, a call only
reaches one bus. Use controller_manager's switch_controller to deactivate
everything instead, see ARCHITECTURE §4.4.
"""

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
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
        self.create_subscription(
            DynamicJointState, "dynamic_joint_states", self.on_dynamic_joint_states, 10)
        self.last_msg_time = None
        self.estop_latched = False
        self.create_timer(0.1, self.check_watchdog)
        self.get_logger().info("safety supervisor up, watchdog %.2fs" % WATCHDOG_TIMEOUT_S)

    def on_dynamic_joint_states(self, msg: DynamicJointState):
        self.last_msg_time = self.get_clock().now()
        # TODO: msg.joint_names lines up with msg.interface_values, each entry
        # has its own interface_names/values list ("temperature", "current",
        # ...). Real threshold check against TEMPERATURE_LIMIT_C /
        # STALL_CURRENT_THRESHOLD goes here, still not written.
        pass

    def check_watchdog(self):
        # twist_mux's estop lock (timeout: 0.0) never expires on its own -
        # it only changes on a new message. Publish current health on every
        # tick, not just on the trip, so a recovered joint_states stream
        # actually clears the lock instead of leaving it stuck forever.
        if self.last_msg_time is None:
            return
        age = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        stale = age > WATCHDOG_TIMEOUT_S
        if stale and not self.estop_latched:
            self.get_logger().error("e-stop: no joint_states for %.2fs" % age)
        elif not stale and self.estop_latched:
            self.get_logger().info("e-stop cleared, joint_states resumed")
        self.estop_latched = stale
        self.estop_pub.publish(Bool(data=stale))


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
