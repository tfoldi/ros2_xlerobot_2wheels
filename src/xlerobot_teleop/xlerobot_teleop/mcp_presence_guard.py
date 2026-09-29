#!/usr/bin/env python3
"""Presence/heartbeat watchdog for agent-issued (MCP) motion commands.

teleop's own deadman (joystick.yaml's enable_button, "left bumper") only guards commands sent
while a human is actively driving with the stick. Once an agent is the one issuing motion
commands, nothing requires a human to still be watching - this closes that gap: a human holds or
repeatedly taps a *different* button (config'd in mcp_presence.yaml, default button 7, RB on the
gamepad this was tested against - confirmed via `ros2 topic echo /joy`, not a generic-Xbox-layout
guess; deliberately not the drive-enable button, the two concerns don't overlap) and
/mcp_presence_ok stays true only while that's recent. MCP motion tools must check this before
accepting a goal and keep checking it while one is in flight, same as any other watchdog here.

Same stale-after-timeout latch shape as safety_supervisor.py's check_watchdog - not merged into
that node since this watches operator attention, not servo health, and is only relevant when
something is actually driving the robot via MCP.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class McpPresenceGuard(Node):
    def __init__(self):
        super().__init__("mcp_presence_guard")
        self.declare_parameter("presence_button", 5)
        self.declare_parameter("timeout_sec", 5.0)
        self.button = self.get_parameter("presence_button").value
        self.timeout = self.get_parameter("timeout_sec").value

        self.presence_pub = self.create_publisher(Bool, "/mcp_presence_ok", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.last_press_time = None
        self.ok_latched = False
        self.create_timer(0.1, self.check_watchdog)
        self.get_logger().info(
            f"mcp presence guard up, button {self.button}, timeout {self.timeout:.1f}s")

    def on_joy(self, msg: Joy):
        if self.button < len(msg.buttons) and msg.buttons[self.button]:
            self.last_press_time = self.get_clock().now()

    def check_watchdog(self):
        if self.last_press_time is None:
            fresh = False
        else:
            age = (self.get_clock().now() - self.last_press_time).nanoseconds / 1e9
            fresh = age <= self.timeout
        if fresh and not self.ok_latched:
            self.get_logger().info("presence confirmed")
        elif not fresh and self.ok_latched:
            self.get_logger().warning("presence lost - no button press within timeout")
        self.ok_latched = fresh
        self.presence_pub.publish(Bool(data=fresh))


def main():
    rclpy.init()
    node = McpPresenceGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
