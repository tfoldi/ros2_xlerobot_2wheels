#!/usr/bin/env python3
"""Joystick -> MoveIt Servo teleop for one arm at a time, picking up where joystick.yaml's base
drive (left stick + button 4) leaves off. Needs both src/xlerobot_moveit_config/launch/
servo.launch.py arm:=left and arm:=right already running - this node only ever selects which
one it's currently sending commands to, it doesn't start them.

Deadman: enable_button must be held for anything to move - released, this publishes one zero
command and pauses the active arm's Servo instance (~/pause_servo), same shape as
teleop_twist_joy's own deadman for the base. Unlike the base drive, nothing else in this
project gates arm-jog motion (no presence/MCP check) - this button IS the only safety gate,
treat it with the same care as the base's deadman.

Two jog modes, toggled with mode_button (tap while held):
  JOINT_JOG (default, starts here) - moves specific joints directly, immune to the Cartesian
    singularities this arm hits in several poses (see project memory, 2026-10-02/03) - the
    reliable fallback.
  TWIST - Cartesian X/Y (right stick) + Z (triggers), more intuitive when it's not near a
    singularity, refuses to move (Servo's own safety stop) when it is.

arm_select_button (tap while held) toggles which arm is being driven, starts on right.
gripper_toggle_button (tap while held) opens/closes that arm's gripper directly (not via Servo -
grippers aren't part of either arm's planning group).

Every button/axis index in config/arm_servo_teleop.yaml is an unverified guess - check with
`ros2 topic echo /joy` before trusting any of it, same as joystick.yaml and mcp_presence.yaml
both needed.
"""
import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_JOG, TWIST = ServoCommandType.Request.JOINT_JOG, ServoCommandType.Request.TWIST
GRIPPER_OPEN, GRIPPER_CLOSED = -0.7, 0.35  # rad, matches the SRDF's open/closed group states


class ArmServoTeleop(Node):
    def __init__(self):
        super().__init__("arm_servo_teleop")
        self.declare_parameters("", [
            ("enable_button", 5),
            ("arm_select_button", 6),
            ("mode_button", 2),
            ("axis_x", 3),
            ("axis_y", 4),
            ("axis_z_up", 5),
            ("axis_z_down", 2),
            ("gripper_toggle_button", 0),
            ("joint_jog_axis_x_joint", "wrist_flex_joint"),
            ("joint_jog_axis_y_joint", "wrist_roll_joint"),
            ("joint_jog_dpad_x_axis", 6),
            ("joint_jog_dpad_x_joint", "shoulder_pan_joint"),
            ("joint_jog_dpad_y_axis", 7),
            ("joint_jog_dpad_y_joint", "elbow_flex_joint"),
            ("deadzone", 0.1),
            ("publish_rate_hz", 50.0),
        ])
        p = self.get_parameter
        self.enable_button = p("enable_button").value
        self.arm_select_button = p("arm_select_button").value
        self.mode_button = p("mode_button").value
        self.axis_x = p("axis_x").value
        self.axis_y = p("axis_y").value
        self.axis_z_up = p("axis_z_up").value
        self.axis_z_down = p("axis_z_down").value
        self.gripper_toggle_button = p("gripper_toggle_button").value
        self.jj_axis_x_joint = p("joint_jog_axis_x_joint").value
        self.jj_axis_y_joint = p("joint_jog_axis_y_joint").value
        self.jj_dpad_x_axis = p("joint_jog_dpad_x_axis").value
        self.jj_dpad_x_joint = p("joint_jog_dpad_x_joint").value
        self.jj_dpad_y_axis = p("joint_jog_dpad_y_axis").value
        self.jj_dpad_y_joint = p("joint_jog_dpad_y_joint").value
        self.deadzone = p("deadzone").value

        self.arm = "right"
        self.mode = JOINT_JOG
        self.synced = False  # whether the active arm's actual Servo mode matches self.mode yet -
        # servo.launch.py's own startup timer defaults every Servo instance to TWIST regardless
        # of what this node claims as its default, so without this sync step the two disagree
        # until the user manually toggles mode_button once.
        self.enabled_prev = False
        self.arm_select_prev = False
        self.mode_button_prev = False
        self.gripper_prev = False
        self.gripper_state = {"left": GRIPPER_CLOSED, "right": GRIPPER_CLOSED}
        self.joy = None

        self.twist_pubs = {
            arm: self.create_publisher(TwistStamped, f"/servo_{arm}/delta_twist_cmds", 10)
            for arm in ("left", "right")
        }
        self.jog_pubs = {
            arm: self.create_publisher(JointJog, f"/servo_{arm}/delta_joint_cmds", 10)
            for arm in ("left", "right")
        }
        self.pause_clients = {
            arm: self.create_client(SetBool, f"/servo_{arm}/pause_servo") for arm in ("left", "right")
        }
        self.command_type_clients = {
            arm: self.create_client(ServoCommandType, f"/servo_{arm}/switch_command_type")
            for arm in ("left", "right")
        }
        self.gripper_clients = {
            arm: ActionClient(self, FollowJointTrajectory, f"/{arm}_gripper_controller/follow_joint_trajectory")
            for arm in ("left", "right")
        }

        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_timer(1.0 / p("publish_rate_hz").value, self.tick)

        for arm in ("left", "right"):
            self.set_pause(arm, True)
        self.get_logger().info(
            f"arm_servo_teleop up - hold button {self.enable_button} to jog, "
            f"starts on {self.arm} arm, {self.mode_name()} mode")

    def mode_name(self):
        return "JOINT_JOG" if self.mode == JOINT_JOG else "TWIST"

    def set_pause(self, arm, paused):
        if self.pause_clients[arm].service_is_ready():
            self.pause_clients[arm].call_async(SetBool.Request(data=paused))

    def set_command_type(self, arm, command_type):
        if self.command_type_clients[arm].service_is_ready():
            future = self.command_type_clients[arm].call_async(ServoCommandType.Request(command_type=command_type))
            future.add_done_callback(
                lambda f, arm=arm, ct=command_type: self.get_logger().info(
                    f"set_command_type({arm}, {ct}) -> {f.result()}"))

    def send_gripper(self, arm, position):
        client = self.gripper_clients[arm]
        if not client.server_is_ready():
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f"{arm}_arm_gripper_joint"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        client.send_goal_async(goal)

    def on_joy(self, msg):
        self.joy = msg

    def axis(self, index):
        if self.joy is None or index >= len(self.joy.axes):
            return 0.0
        v = self.joy.axes[index]
        return 0.0 if abs(v) < self.deadzone else v

    def button(self, index):
        if self.joy is None or index >= len(self.joy.buttons):
            return False
        return bool(self.joy.buttons[index])

    def tick(self):
        if not self.synced and self.command_type_clients[self.arm].service_is_ready():
            self.set_command_type(self.arm, self.mode)
            self.synced = True

        if self.joy is None:
            return
        enabled = self.button(self.enable_button)

        # Edge-triggered toggles, only while the deadman is held - a stray press while not
        # jogging shouldn't silently change which arm/mode activates next time it is.
        if enabled:
            if self.button(self.arm_select_button) and not self.arm_select_prev:
                self.stop_and_pause(self.arm)
                self.arm = "left" if self.arm == "right" else "right"
                self.set_command_type(self.arm, self.mode)
                self.get_logger().info(f"active arm: {self.arm}")
            if self.button(self.mode_button) and not self.mode_button_prev:
                self.mode = TWIST if self.mode == JOINT_JOG else JOINT_JOG
                self.set_command_type(self.arm, self.mode)
                self.get_logger().info(f"mode: {self.mode_name()}")
            if self.button(self.gripper_toggle_button) and not self.gripper_prev:
                current = self.gripper_state[self.arm]
                target = GRIPPER_OPEN if current == GRIPPER_CLOSED else GRIPPER_CLOSED
                self.gripper_state[self.arm] = target
                self.send_gripper(self.arm, target)
        self.arm_select_prev = self.button(self.arm_select_button)
        self.mode_button_prev = self.button(self.mode_button)
        self.gripper_prev = self.button(self.gripper_toggle_button)

        if enabled and not self.enabled_prev:
            self.set_pause(self.arm, False)
        elif not enabled and self.enabled_prev:
            self.stop_and_pause(self.arm)
        self.enabled_prev = enabled

        if not enabled:
            return

        if self.mode == TWIST:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_footprint"
            msg.twist.linear.x = self.axis(self.axis_x)
            msg.twist.linear.y = self.axis(self.axis_y)
            # Two independent half-axes (e.g. separate L2/R2 trigger axes), each contributing
            # only its positive engagement - simpler and less layout-dependent than assuming a
            # shared axis's rest/pressed polarity, which varies by controller/driver.
            msg.twist.linear.z = max(0.0, self.axis(self.axis_z_up)) - max(0.0, self.axis(self.axis_z_down))
            self.twist_pubs[self.arm].publish(msg)
        else:
            names, vels = [], []
            for axis_idx, joint in (
                (self.axis_x, self.jj_axis_x_joint),
                (self.axis_y, self.jj_axis_y_joint),
                (self.jj_dpad_x_axis, self.jj_dpad_x_joint),
                (self.jj_dpad_y_axis, self.jj_dpad_y_joint),
            ):
                v = self.axis(axis_idx)
                if v != 0.0:
                    names.append(f"{self.arm}_arm_{joint}")
                    vels.append(v)
            if names:
                msg = JointJog()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.joint_names = names
                msg.velocities = vels
                msg.duration = 0.1
                self.jog_pubs[self.arm].publish(msg)

    def stop_and_pause(self, arm):
        zero_twist = TwistStamped()
        zero_twist.header.stamp = self.get_clock().now().to_msg()
        zero_twist.header.frame_id = "base_footprint"
        self.twist_pubs[arm].publish(zero_twist)
        zero_jog = JointJog()
        zero_jog.header.stamp = self.get_clock().now().to_msg()
        self.jog_pubs[arm].publish(zero_jog)
        self.set_pause(arm, True)


def main():
    rclpy.init()
    node = ArmServoTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for arm in ("left", "right"):
            node.stop_and_pause(arm)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
