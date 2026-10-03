#!/usr/bin/env python3
"""Joystick -> MoveIt Servo teleop for both arms at once, each driven by its own stick - left
stick jogs the left arm, right stick jogs the right arm, same side-to-side mapping as the base
drive's own left-stick-drives-the-whole-robot convention. Needs both
src/xlerobot_moveit_config/launch/servo.launch.py arm:=left and arm:=right already running -
this node only ever sends them commands, it doesn't start them.

Deadman: enable_button must be held for anything to move - released, this publishes a zero
command and pauses both arms' Servo instances (~/pause_servo), same shape as teleop_twist_joy's
own deadman for the base. Unlike the base drive, nothing else in this project gates arm-jog
motion (no presence/MCP check) - this button IS the only safety gate for it, treat it with the
same care as the base's deadman. Both arms share one deadman - there's no way to jog just one
arm while leaving the other paused, by design, to keep this simple.

JOINT_JOG only: each stick's X/Y drives two named joints of its own arm directly. Servo's
Cartesian (TWIST) mode was tried and dropped - see docs/decisions/0004-drop-cartesian-servo-jog.md
for why.

gripper_toggle_button (tap while held) opens/closes BOTH grippers together (not via Servo -
grippers aren't part of either arm's planning group) - there's no independent left/right gripper
control in this node, use tools/single_joint_test.py or tools/goto_position.py for that.

Confirmed on the real controller (2026-10-03): A=0 B=1 X=3 Y=4 LB=6 RB=7, left stick [0,1],
right stick [2,3], D-pad [6,7], LT=8 RT=9 (digital buttons, not axes). Y is joystick.yaml's
enable_button (base drive) and RB is mcp_presence_guard's presence_button as of 2026-10-03 (the
two were swapped so drive could move to RB) - neither read here, kept clear on purpose.
"""
import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

ARMS = ("left", "right")
JOINT_JOG = ServoCommandType.Request.JOINT_JOG
GRIPPER_OPEN, GRIPPER_CLOSED = -0.7, 0.35  # rad, matches the SRDF's open/closed group states


class ArmServoTeleop(Node):
    def __init__(self):
        super().__init__("arm_servo_teleop")
        self.declare_parameters("", [
            ("enable_button", 6),       # LB
            ("gripper_toggle_button", 0),  # A
            ("left_axis_x", 0),
            ("left_axis_y", 1),
            ("right_axis_x", 2),
            ("right_axis_y", 3),
            ("joint_jog_axis_x_joint", "wrist_flex_joint"),
            ("joint_jog_axis_y_joint", "wrist_roll_joint"),
            ("deadzone", 0.1),
            ("publish_rate_hz", 50.0),
        ])
        p = self.get_parameter
        self.enable_button = p("enable_button").value
        self.gripper_toggle_button = p("gripper_toggle_button").value
        self.axes = {
            "left": (p("left_axis_x").value, p("left_axis_y").value),
            "right": (p("right_axis_x").value, p("right_axis_y").value),
        }
        self.jj_axis_x_joint = p("joint_jog_axis_x_joint").value
        self.jj_axis_y_joint = p("joint_jog_axis_y_joint").value
        self.deadzone = p("deadzone").value

        self.synced = {arm: False for arm in ARMS}  # per-arm: has this node told the real Servo
        # instance to use JOINT_JOG yet - servo.launch.py's own startup timer defaults every
        # Servo instance to TWIST regardless of what this node wants, so without this each arm
        # stays in TWIST until told otherwise.
        self.enabled_prev = False
        self.gripper_prev = False
        self.gripper_state = {"left": GRIPPER_CLOSED, "right": GRIPPER_CLOSED}
        self.joy = None

        self.jog_pubs = {
            arm: self.create_publisher(JointJog, f"/servo_{arm}/delta_joint_cmds", 10) for arm in ARMS
        }
        self.pause_clients = {
            arm: self.create_client(SetBool, f"/servo_{arm}/pause_servo") for arm in ARMS
        }
        self.command_type_clients = {
            arm: self.create_client(ServoCommandType, f"/servo_{arm}/switch_command_type") for arm in ARMS
        }
        self.gripper_clients = {
            arm: ActionClient(self, FollowJointTrajectory, f"/{arm}_gripper_controller/follow_joint_trajectory")
            for arm in ARMS
        }

        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_timer(1.0 / p("publish_rate_hz").value, self.tick)

        for arm in ARMS:
            self.set_pause(arm, True)
        self.get_logger().info(
            f"arm_servo_teleop up - hold button {self.enable_button} to jog both arms "
            f"(left stick -> left arm, right stick -> right arm), JOINT_JOG mode")

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
        for arm in ARMS:
            if not self.synced[arm] and self.command_type_clients[arm].service_is_ready():
                self.set_command_type(arm, JOINT_JOG)
                self.synced[arm] = True

        if self.joy is None:
            return
        enabled = self.button(self.enable_button)

        # Edge-triggered, only while the deadman is held - a stray press while not jogging
        # shouldn't silently toggle the gripper next time it is.
        if enabled and self.button(self.gripper_toggle_button) and not self.gripper_prev:
            for arm in ARMS:
                target = GRIPPER_OPEN if self.gripper_state[arm] == GRIPPER_CLOSED else GRIPPER_CLOSED
                self.gripper_state[arm] = target
                self.send_gripper(arm, target)
        self.gripper_prev = self.button(self.gripper_toggle_button)

        if enabled and not self.enabled_prev:
            for arm in ARMS:
                self.set_pause(arm, False)
        elif not enabled and self.enabled_prev:
            for arm in ARMS:
                self.stop_and_pause(arm)
        self.enabled_prev = enabled

        if not enabled:
            return

        for arm in ARMS:
            axis_x, axis_y = self.axes[arm]
            names, vels = [], []
            for axis_idx, joint in ((axis_x, self.jj_axis_x_joint), (axis_y, self.jj_axis_y_joint)):
                v = self.axis(axis_idx)
                if v != 0.0:
                    names.append(f"{arm}_arm_{joint}")
                    vels.append(v)
            if names:
                msg = JointJog()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.joint_names = names
                msg.velocities = vels
                msg.duration = 0.1
                self.jog_pubs[arm].publish(msg)

    def stop_and_pause(self, arm):
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
        for arm in ARMS:
            node.stop_and_pause(arm)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
