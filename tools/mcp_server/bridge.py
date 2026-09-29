"""XleRobotBridge: one long-lived rclpy node backing every MCP tool in server.py.

Unlike tools/*.py, which each do rclpy.init()/shutdown() around a single action, this node is
created once and spun continuously in a background thread for the life of the MCP server
process. Tool calls run on FastMCP's own thread/event loop, not the executor's spin thread, so
they can't use rclpy.spin_until_future_complete (that would fight the background spin for the
same node) - instead every blocking call here uses a Future.add_done_callback() + threading.Event
pair: the callback (which runs on the executor thread) sets the event, the calling thread just
waits on it. See _wait_for() below.

Motion calls are serialized through motion_lock - the robot only does one thing at a time by
design, not a limitation worth working around.
"""
import math
import pathlib
import sys
import threading
import time

TOOLS_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS_DIR))

# Matches tools/capture_image.py's own convention (repo_root/captures/, gitignored - see
# .gitignore, ephemeral like /trajectories/, not curated).
CAPTURE_DIR = TOOLS_DIR.parent / "captures"

import cv2
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, PlanningOptions
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

import position_store
import waypoint_store
from goto_position import MoveError, joint_goal_request
from move_to_pose import position_goal_request

CAMERAS = ["head", "left", "right"]
ESTOP_BUSES = ["xlerobot_bus1", "xlerobot_bus2"]
NAV_TERMINAL = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled",
                GoalStatus.STATUS_ABORTED: "aborted"}


class PresenceLost(RuntimeError):
    """Raised when a motion-class tool is called, or a motion in flight is cancelled, because
    /mcp_presence_ok is false - see xlerobot_teleop/mcp_presence_guard.py."""


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class XleRobotBridge(Node):
    def __init__(self):
        super().__init__("mcp_bridge")
        self.cv_bridge = CvBridge()
        self.motion_lock = threading.Lock()

        # cached state, updated by subscription callbacks on the executor thread
        self.joint_positions = {}
        self.last_joint_state_time = None
        self.amcl_pose = None  # (x, y, yaw)
        self.presence_ok = False
        self.nav_status = None
        self.camera_frames = {c: None for c in CAMERAS}

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)
        self.create_subscription(Bool, "/mcp_presence_ok", self._on_presence, 10)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_nav_status, 10)
        for camera in CAMERAS:
            self.create_subscription(
                Image, f"cameras/{camera}/image_raw",
                (lambda msg, c=camera: self._on_image(c, msg)), 5)

        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.nav_action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.nav_cancel_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.cmd_pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.estop_clients = {b: self.create_client(SetBool, f"/emergency_stop_{b}") for b in ESTOP_BUSES}

    # -- subscription callbacks (executor thread) --------------------------------------------

    def _on_joint_states(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))
        self.last_joint_state_time = time.monotonic()

    def _on_amcl_pose(self, msg):
        p = msg.pose.pose.position
        self.amcl_pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_presence(self, msg):
        self.presence_ok = msg.data

    def _on_nav_status(self, msg):
        if msg.status_list:
            self.nav_status = msg.status_list[-1].status

    def _on_image(self, camera, msg):
        self.camera_frames[camera] = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # -- generic async-future -> sync-call bridge (calling thread) ---------------------------

    def _wait_for(self, future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not event.wait(timeout=timeout):
            return None
        return future.result()

    def _require_presence(self):
        if not self.presence_ok:
            raise PresenceLost(
                "no presence heartbeat - hold/tap the presence button (see mcp_presence.yaml) "
                "before commanding motion")

    # -- estop (never presence-gated - always callable, including to recover from PresenceLost) --

    def estop(self, buses=None, active=True):
        buses = buses or ESTOP_BUSES
        results = {}
        futures = {}
        for bus in buses:
            client = self.estop_clients[bus]
            if not client.wait_for_service(timeout_sec=3.0):
                results[bus] = "service not available"
                continue
            futures[bus] = client.call_async(SetBool.Request(data=active))
        for bus, future in futures.items():
            result = self._wait_for(future, timeout=5.0)
            results[bus] = result.message if result else "no answer"
        return results

    def stop_base(self):
        if self.nav_cancel_client.wait_for_service(timeout_sec=1.0):
            self.nav_cancel_client.call_async(CancelGoal.Request())
        for _ in range(5):
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)

    # -- MoveGroup goals (goto_position, move_to_pose, move_gripper, play_sequence) ----------

    def _run_move_group(self, request, group, timeout=60.0):
        self._require_presence()
        with self.motion_lock:
            # server_is_ready() alone is an instant, no-wait check - right after the bridge
            # starts (before this client has discovered move_action at all) it false-negatives
            # even though move_group is genuinely up, since discovery takes a moment. Bounded
            # wait instead - only blocks the calling (tool) thread via a plain sleep loop, safe
            # to call alongside the background executor spinning this node (confirmed: rclpy's
            # own wait_for_server() does no spinning of its own).
            if not self.move_group.wait_for_server(timeout_sec=5.0):
                raise MoveError("/move_action not available - is move_group running?")
            options = PlanningOptions(plan_only=False)
            goal_msg = MoveGroup.Goal(request=request, planning_options=options)
            send_future = self.move_group.send_goal_async(goal_msg)
            handle = self._wait_for(send_future, timeout=15.0)
            if handle is None or not handle.accepted:
                raise MoveError("goal not accepted")

            result_future = handle.get_result_async()
            result_event = threading.Event()
            result_future.add_done_callback(lambda f: result_event.set())
            deadline = time.monotonic() + timeout
            while not result_event.is_set():
                if not self.presence_ok:
                    handle.cancel_goal_async()
                    result_event.wait(timeout=2.0)
                    self.estop(position_store.GROUP_BUSES.get(group, ESTOP_BUSES))
                    raise PresenceLost("presence lost mid-motion - goal cancelled, bus e-stopped")
                if time.monotonic() > deadline:
                    handle.cancel_goal_async()
                    raise MoveError(f"timed out after {timeout:.0f}s, goal cancelled")
                result_event.wait(timeout=0.1)

            result = result_future.result()
            code = result.result.error_code.val
            if code != MoveItErrorCodes.SUCCESS:
                raise MoveError(f"MoveIt error code {code}")
            return True

    def goto_position(self, name, scaling=0.0):
        entry = position_store.get(name)
        request = joint_goal_request(entry["group"], entry["joints"], scaling)
        self._run_move_group(request, entry["group"])
        return f"reached '{name}' ({entry['group']})"

    def move_to_pose(self, arm, x, y, z, frame="base_footprint", scaling=0.0):
        group = f"{arm}_arm"
        tip_link = position_store.GROUP_TIP_LINK[group]
        request = position_goal_request(group, tip_link, frame, (x, y, z), scaling)
        self._run_move_group(request, group)
        return f"{group} gripper landed near ({x:.3f}, {y:.3f}, {z:.3f}) in {frame}"

    def move_gripper(self, arm, action, scaling=0.0):
        if action not in ("open", "close"):
            raise ValueError("action must be 'open' or 'close'")
        group = f"{arm}_gripper"
        joint = position_store.GROUP_JOINTS[group][0]
        value = position_store.GRIPPER_OPEN if action == "open" else position_store.GRIPPER_CLOSED
        request = joint_goal_request(group, {joint: value}, scaling)
        self._run_move_group(request, group)
        return f"{arm} gripper {action}d"

    def play_sequence(self, names, loop="1", scaling=0.0):
        entries = [(name, position_store.get(name)) for name in names]
        forever = loop == "forever"
        count = 0 if forever else int(loop)
        if not forever and count < 1:
            raise ValueError("loop must be a positive integer or 'forever'")
        iteration = 0
        while forever or iteration < count:
            iteration += 1
            for name, entry in entries:
                request = joint_goal_request(entry["group"], entry["joints"], scaling)
                self._run_move_group(request, entry["group"])
        return f"played {' -> '.join(names)} x{iteration}"

    def save_position(self, name, group):
        joints = position_store.GROUP_JOINTS[group]
        deadline = time.time() + 5
        while not all(j in self.joint_positions for j in joints) and time.time() < deadline:
            time.sleep(0.1)
        missing = [j for j in joints if j not in self.joint_positions]
        if missing:
            raise MoveError(f"no /joint_states for: {', '.join(missing)}")
        positions = position_store.load()
        positions[name] = {
            "group": group,
            "joints": {j: round(float(self.joint_positions[j]), 5) for j in joints},
        }
        position_store.save(positions)
        return f"saved '{name}' ({group})"

    # -- navigation (goto_waypoint) -----------------------------------------------------------

    def goto_waypoint(self, name, timeout=120.0):
        self._require_presence()
        w = waypoint_store.get(name)
        with self.motion_lock:
            if not self.nav_action.wait_for_server(timeout_sec=10.0):
                raise MoveError("Nav2 is not up (no navigate_to_pose action server)")

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = w["x"], w["y"]
            goal.pose.orientation.z = math.sin(w["yaw"] / 2)
            goal.pose.orientation.w = math.cos(w["yaw"] / 2)
            self.nav_status = None
            self.goal_pub.publish(goal)

            t0 = time.monotonic()
            active = False
            while True:
                if not self.presence_ok:
                    self.stop_base()
                    raise PresenceLost("presence lost mid-navigation - goal cancelled")
                active = active or self.nav_status in (
                    GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
                if active and self.nav_status in NAV_TERMINAL:
                    reason = NAV_TERMINAL[self.nav_status]
                    break
                if not active and time.monotonic() - t0 > 5.0:
                    self.stop_base()
                    raise MoveError("Nav2 did not accept the goal within 5s")
                if time.monotonic() - t0 > timeout:
                    self.stop_base()
                    raise MoveError(f"timed out after {timeout:.0f}s, goal cancelled")
                time.sleep(0.05)
            if reason != "succeeded":
                raise MoveError(f"navigation {reason}")
            return f"arrived at '{name}'"

    def save_waypoint(self, name):
        deadline = time.time() + 5
        while self.amcl_pose is None and time.time() < deadline:
            time.sleep(0.1)
        if self.amcl_pose is None:
            raise MoveError("no /amcl_pose - is navigation.launch.py running with localization:=true?")
        x, y, yaw = self.amcl_pose
        waypoints = waypoint_store.load()
        waypoints[name] = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}
        waypoint_store.save(waypoints)
        return f"saved '{name}': x={x:.3f} y={y:.3f} yaw={yaw:.3f} rad"

    # -- read-only introspection (never presence-gated) --------------------------------------

    def list_positions(self):
        return position_store.load()

    def list_waypoints(self):
        return waypoint_store.load()

    def get_joint_states(self, group=None):
        joints = position_store.GROUP_JOINTS[group] if group else list(self.joint_positions)
        return {j: self.joint_positions.get(j) for j in joints}

    def get_robot_pose(self):
        if self.amcl_pose is None:
            return None
        x, y, yaw = self.amcl_pose
        return {"x": x, "y": y, "yaw": yaw, "frame": "map"}

    def get_presence(self):
        return self.presence_ok

    def get_status(self):
        """Best-effort liveness summary, NOT a torque-state check - sts_hardware_interface's own
        bus is owned by the running hardware interface while bringup is up, so there's no way to
        read the real torque-enable register from here without a serial-port conflict. The only
        ground truth for "is torque actually off" is tools/calibrate_arm.py status with the stack
        stopped - always use that after any estop, per standing practice, regardless of what this
        tool reports."""
        fresh = (self.last_joint_state_time is not None
                 and time.monotonic() - self.last_joint_state_time < 1.0)
        return {
            "presence_ok": self.presence_ok,
            "joint_states_fresh": fresh,
            "localized": self.amcl_pose is not None,
            "cameras_live": {c: (self.camera_frames[c] is not None) for c in CAMERAS},
        }

    def capture_image(self, camera, output_path):
        if camera not in CAMERAS:
            raise ValueError(f"camera must be one of {CAMERAS}")
        deadline = time.time() + 5
        while self.camera_frames[camera] is None and time.time() < deadline:
            time.sleep(0.1)
        frame = self.camera_frames[camera]
        if frame is None:
            raise MoveError(f"no frame from cameras/{camera}/image_raw - is bringup running with cameras:=true?")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), frame)
        return {"path": str(output_path), "width": frame.shape[1], "height": frame.shape[0]}


_bridge = None
_executor = None
_init_lock = threading.Lock()


def get_bridge():
    """Lazily start rclpy + the bridge node + its background spin thread, once per process.
    FastMCP dispatches tool calls on worker threads, so two calls can race in here on the very
    first request - the lock makes the double-init (rclpy.init() twice) impossible rather than
    just unlikely.

    The brief sleep after starting the spin thread is a one-time cost, paid only by whichever
    tool call happens to be first: DDS discovery of the rest of the ROS graph (bringup's
    /joint_states, /mcp_presence_ok, ...) isn't instant, and a caller landing before it settles
    would see empty caches - not a hang, just a wrong-looking "nothing is alive yet" answer from
    the very first call. 1.0s was enough on a laptop with nothing else competing for CPU; on a
    Jetson Orin NX also running cameras + mock hardware + move_group, discovery sometimes took
    longer than that (confirmed live: joint_states came through by 1s but /mcp_presence_ok
    hadn't yet) - 2.5s gave consistent results there. Same one-time cost either way, paid once
    per server process lifetime."""
    global _bridge, _executor
    with _init_lock:
        if _bridge is None:
            rclpy.init()
            _bridge = XleRobotBridge()
            _executor = rclpy.executors.SingleThreadedExecutor()
            _executor.add_node(_bridge)
            threading.Thread(target=_executor.spin, daemon=True).start()
            time.sleep(2.5)
        return _bridge
