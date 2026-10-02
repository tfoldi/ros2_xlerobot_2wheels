#!/usr/bin/env python3
"""Seed AMCL's initial pose from the home marker instead of letting it guess, or needing someone
to manually place the robot at a measured spot + give it a RViz "2D Pose Estimate" by eye.

  python3 tools/home_marker_localize.py

Run this right after bringup + navigation.launch.py localization:=true come up, with the robot
able to see the home marker (tools/generate_home_marker.py) from wherever it happened to be
parked - that's the point, it does not need to be a known/marked spot, just somewhere the head
camera can see the marker from. Needs tools/home_marker_calibrate.py to have been run at least
once already (src/xlerobot_navigation/config/home_marker.yaml must exist).

Math: this uses head_camera_optical_frame -> base_footprint from TF, which is a static/kinematic
chain through the current head pan/tilt joint_states - it does NOT depend on map or AMCL being
localized at all, unlike home_marker_calibrate.py's map -> head_camera_optical_frame lookup. That
asymmetry is exactly what lets this tool bootstrap AMCL from cold: map_to_marker (saved, fixed)
composed with marker_to_camera (this frame's detection) composed with camera_to_base (always
available) gives map_to_base without ever needing AMCL's own (not-yet-correct) belief.
"""
import math
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, LookupException, TransformListener

from home_marker_common import detect_marker_pose, load_marker_pose, matrix_to_xy_yaw, rvec_tvec_to_matrix, transform_to_matrix

HEAD_CAMERA_FRAME = "head_camera_optical_frame"
BASE_FRAME = "base_footprint"
SAMPLES = 10

# Tighter than AMCL's usual cold-start spread (it defaults to several metres / a full turn) -
# this pose comes from a direct visual measurement, not a guess, so AMCL's particle filter
# should converge in just a few scans instead of needing to explore the whole map.
COVARIANCE_XY = 0.05 ** 2
COVARIANCE_YAW = math.radians(5) ** 2


class HomeMarkerLocalize(Node):
    def __init__(self):
        super().__init__("home_marker_localize")
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.frame = None
        self.create_subscription(CameraInfo, "cameras/head/camera_info", self.on_camera_info, 10)
        self.create_subscription(Image, "cameras/head/image_raw", self.on_image, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def on_camera_info(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    t_map_marker = load_marker_pose()

    rclpy.init()
    node = HomeMarkerLocalize()

    deadline = time.time() + 10
    while (node.camera_matrix is None or node.frame is None) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.camera_matrix is None or node.frame is None:
        sys.exit("no camera_info/image_raw from cameras/head - is bringup running with cameras:=true?")

    t_map_base_samples = []
    deadline = time.time() + 15
    while len(t_map_base_samples) < SAMPLES and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        pose = detect_marker_pose(node.frame, node.camera_matrix, node.dist_coeffs)
        if pose is None:
            continue
        rvec, tvec = pose
        try:
            transform = node.tf_buffer.lookup_transform(HEAD_CAMERA_FRAME, BASE_FRAME, rclpy.time.Time())
        except LookupException as e:
            sys.exit(f"no TF {HEAD_CAMERA_FRAME} -> {BASE_FRAME} yet: {e} - is bringup running "
                      "with with_head:=true and publishing joint_states?")
        t_cam_marker = rvec_tvec_to_matrix(rvec, tvec)
        t_marker_cam = np.linalg.inv(t_cam_marker)
        t_cam_base = transform_to_matrix(transform.transform)
        t_map_base_samples.append(t_map_marker @ t_marker_cam @ t_cam_base)

    if not t_map_base_samples:
        sys.exit("never saw the home marker in the head camera in 15s - point the robot at it "
                  "(see where it's mounted in src/xlerobot_navigation/config/home_marker.yaml)")

    x, y, yaw = matrix_to_xy_yaw(np.mean(np.stack(t_map_base_samples), axis=0))

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    msg.pose.covariance[0] = COVARIANCE_XY
    msg.pose.covariance[7] = COVARIANCE_XY
    msg.pose.covariance[35] = COVARIANCE_YAW

    # AMCL only picks up the first message on a topic it's still subscribing to - give it a
    # moment, then publish a few times instead of trusting it caught the very first one.
    time.sleep(1.0)
    for _ in range(5):
        node.initialpose_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f"published /initialpose: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} deg "
          f"from {len(t_map_base_samples)} samples")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
