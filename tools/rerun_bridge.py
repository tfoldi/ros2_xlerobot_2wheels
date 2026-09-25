#!/usr/bin/env python3
"""Streams cmd_vel/odom/wheel data and the robot model into Rerun for timeline
debugging, opens the Rerun Viewer. Complements RViz/Foxglove rather than
replacing them: scrub scalar time series (commanded vs actual velocity) next
to the robot moving in the odom frame.

The model comes from /robot_description (the same URDF robot_state_publisher
serves), placed in the odom frame from odometry, wheels turned from
/joint_states.

Camera video is taken from the ffmpeg_image_transport topics
(<camera>/image_raw/ffmpeg, H.264 or H.265 packets) and logged as an encoded
video stream, so nothing is decoded or re-encoded here. Cameras are found
while running, so they can start after the bridge.

Usage (bringup already running):
    python3 tools/rerun_bridge.py
    python3 tools/rerun_bridge.py --save run.rrd    # record to a file, no viewer
    python3 tools/rerun_bridge.py --connect rerun+http://laptop:9876/proxy
        # run on the robot, show in a viewer started elsewhere with `rerun`
"""

import argparse
import math
import os
import signal
import tempfile

import rclpy
import rerun as rr
import rerun.blueprint as rrb
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rerun.blueprint.components import PlayState
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
except ImportError:
    FFMPEGPacket = None

ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
CAMERAS = ("head", "left", "right")
FFMPEG_TYPE = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"
AV_PKT_FLAG_KEY = 1


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def blueprint(cameras):
    camera_views = [
        rrb.Spatial2DView(name=f"{name} camera", origin=f"/cameras/{name}") for name in cameras]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial3DView(
                    name="Robot", origin="/",
                    contents=[
                        "/xlerobot/**", "/transforms/**", "- /xlerobot/collision_geometries/**"],
                    spatial_information=rrb.SpatialInformation(target_frame=ODOM_FRAME)),
                rrb.Spatial2DView(
                    name="Odom path", origin="/odom", contents=["/odom/path", "/odom/pose"]),
            ),
            *([rrb.Vertical(*camera_views)] if camera_views else []),
            rrb.Vertical(
                rrb.TimeSeriesView(name="Yaw", origin="/odom/yaw"),
                rrb.TimeSeriesView(
                    name="Angular z",
                    contents=["/odom/angular_z", "/cmd_vel/angular_z", "/cmd_vel_nav/angular_z"]),
                rrb.TimeSeriesView(
                    name="Linear x",
                    contents=["/odom/linear_x", "/cmd_vel/linear_x", "/cmd_vel_nav/linear_x"]),
                rrb.TimeSeriesView(name="Wheel velocity", origin="/wheels"),
            ),
        ),
        # follow the live edge, otherwise a fresh viewer sits at the start of the
        # recording, before the first odom transform, and draws no robot
        rrb.TimePanel(timeline="ros_time", play_state=PlayState.Following),
    )


class RerunBridge(Node):
    def __init__(self):
        super().__init__("rerun_bridge")
        self.path_x = []
        self.path_y = []
        self.joints = {}
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.on_odom, 20)
        self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel", self.on_cmd_vel_final, 20)
        self.create_subscription(TwistStamped, "/cmd_vel_nav", self.on_cmd_vel_nav, 20)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 20)
        # robot_state_publisher latches the description, so match its durability.
        self.create_subscription(
            String, "/robot_description", self.on_robot_description,
            QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))
        self.video_subs = {}
        self.video_started = set()
        if FFMPEGPacket is not None:
            self.create_timer(2.0, self.find_cameras)

    def find_cameras(self):
        for topic, types in self.get_topic_names_and_types():
            if FFMPEG_TYPE in types and topic not in self.video_subs:
                # /cameras/left/image_raw/ffmpeg -> cameras/left
                path = topic.strip("/").removesuffix("/image_raw/ffmpeg").removesuffix("/ffmpeg")
                self.video_subs[topic] = self.create_subscription(
                    FFMPEGPacket, topic, lambda msg, path=path: self.on_video(msg, path),
                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
                self.get_logger().info(f"logging video from {topic} to {path}")

    def on_video(self, msg, path):
        keyframe = bool(msg.flags & AV_PKT_FLAG_KEY)
        if path not in self.video_started:
            # a stream can only start on a keyframe, it carries the codec headers
            if not keyframe:
                return
            # encoding is "codec;source_pixel_format;...", the first token is the codec
            hevc = msg.encoding.split(";")[0].lower() in ("hevc", "h265")
            rr.log(
                path, rr.VideoStream(codec=rr.VideoCodec.H265 if hevc else rr.VideoCodec.H264),
                static=True)
            self.video_started.add(path)
        rr.set_time("ros_time", timestamp=stamp_seconds(msg.header))
        rr.log(path, rr.VideoStream.from_fields(sample=bytes(msg.data), is_keyframe=keyframe))

    def on_robot_description(self, msg: String):
        if self.joints:
            return
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "robot.urdf")
            with open(path, "w") as f:
                f.write(msg.data)
            tree = rr.urdf.UrdfTree.from_file_path(path)
        tree.log_urdf_to_recording()
        self.joints = {j.name: j for j in tree.joints() if j.joint_type != "fixed"}
        self.get_logger().info(f"logged robot model, {len(self.joints)} movable joints")

    def on_odom(self, msg: Odometry):
        rr.set_time("ros_time", timestamp=stamp_seconds(msg.header))
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.path_x.append(p.x)
        self.path_y.append(p.y)
        rr.log("odom/path", rr.LineStrips2D([list(zip(self.path_x, self.path_y))]))
        rr.log("odom/pose", rr.Points2D([[p.x, p.y]]))
        rr.log("odom/yaw", rr.Scalars(yaw_from_quat(q)))
        rr.log("odom/linear_x", rr.Scalars(msg.twist.twist.linear.x))
        rr.log("odom/angular_z", rr.Scalars(msg.twist.twist.angular.z))
        rr.log(
            "transforms/base",
            rr.Transform3D(
                translation=[p.x, p.y, p.z],
                rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                parent_frame=ODOM_FRAME, child_frame=BASE_FRAME))

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
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            if name in ("left_wheel_joint", "right_wheel_joint"):
                rr.log(f"wheels/{name}/velocity", rr.Scalars(velocity))
            if name in self.joints:
                rr.log(f"transforms/{name}", self.joints[name].compute_transform(position))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", metavar="FILE.rrd", help="record to a file instead of opening the viewer")
    parser.add_argument("--connect", metavar="URL",
                        help="send to a running viewer, e.g. rerun+http://laptop:9876/proxy")
    parser.add_argument("--cameras", default=",".join(CAMERAS),
                        help="camera views in the layout, comma separated, empty for none")
    args, ros_args = parser.parse_known_args()

    # so a backgrounded run (SIGINT ignored) still flushes the recording on kill
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    rr.init("xlerobot", spawn=args.save is None and args.connect is None)
    if args.save:
        rr.save(args.save)
    elif args.connect:
        rr.connect_grpc(args.connect)
    rr.send_blueprint(blueprint([name for name in args.cameras.split(",") if name]))
    rclpy.init(args=ros_args)
    node = RerunBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # rclpy's own signal handler may have shut the context down already
            rclpy.shutdown()


if __name__ == "__main__":
    main()
