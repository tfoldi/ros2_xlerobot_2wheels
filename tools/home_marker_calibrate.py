#!/usr/bin/env python3
"""Measure the home marker's fixed pose in the map frame, once, and save it for
home_marker_localize.py to use at every future cold start.

  python3 tools/home_marker_calibrate.py

Run this ONCE, with:
  - navigation running against the saved map (localization:=true, not slam:=true - a fresh SLAM
    session has no stable map frame yet, see [[project-nav2-and-jetson-status]]),
  - AMCL already well-localized (drive it around a bit, or give it a good manual "2D Pose
    Estimate" in RViz first - this script trusts whatever map->base_footprint TF currently says,
    garbage in, garbage out),
  - the printed home marker (tools/generate_home_marker.py) visible in the head camera, at
    whatever wall position it's going to live at permanently.

Re-run it any time the marker moves (or the saved map changes) - home_marker_localize.py always
uses whatever this last wrote to src/xlerobot_navigation/config/home_marker.yaml.
"""
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, LookupException, TransformListener

from home_marker_common import (
    CONFIG_PATH, HOME_MARKER_ID, detect_marker_pose, matrix_to_xyz_quat, rvec_tvec_to_matrix,
    save_marker_pose, transform_to_matrix,
)

SAMPLES = 15
HEAD_CAMERA_FRAME = "head_camera_optical_frame"


class HomeMarkerCalibrate(Node):
    def __init__(self):
        super().__init__("home_marker_calibrate")
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.frame = None
        self.create_subscription(CameraInfo, "cameras/head/camera_info", self.on_camera_info, 10)
        self.create_subscription(Image, "cameras/head/image_raw", self.on_image, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_camera_info(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    rclpy.init()
    node = HomeMarkerCalibrate()

    deadline = time.time() + 10
    while (node.camera_matrix is None or node.frame is None) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.camera_matrix is None or node.frame is None:
        sys.exit("no camera_info/image_raw from cameras/head - is bringup running with cameras:=true?")

    t_map_marker_samples = []
    deadline = time.time() + 15
    while len(t_map_marker_samples) < SAMPLES and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        pose = detect_marker_pose(node.frame, node.camera_matrix, node.dist_coeffs)
        if pose is None:
            continue
        rvec, tvec = pose
        try:
            transform = node.tf_buffer.lookup_transform("map", HEAD_CAMERA_FRAME, rclpy.time.Time())
        except LookupException as e:
            sys.exit(f"no TF map -> {HEAD_CAMERA_FRAME} yet: {e} - is localization:=true running "
                      "and AMCL publishing map -> odom?")
        t_map_cam = transform_to_matrix(transform.transform)
        t_cam_marker = rvec_tvec_to_matrix(rvec, tvec)
        t_map_marker_samples.append(t_map_cam @ t_cam_marker)

    if not t_map_marker_samples:
        sys.exit(f"never saw marker ID {HOME_MARKER_ID} in the head camera in 15s - check it's "
                  "in view and the right size/dictionary")

    # Average translation directly; average rotation via its quaternion mean would need care
    # about sign ambiguity, so just take the last good sample's rotation - samples are a few
    # hundred ms apart with the robot stationary, rotation noise between them is negligible
    # compared to translation noise from marker-corner jitter.
    avg = np.mean(np.stack(t_map_marker_samples), axis=0)
    avg[:3, :3] = t_map_marker_samples[-1][:3, :3]

    save_marker_pose(avg)
    x, y, z, qx, qy, qz, qw = matrix_to_xyz_quat(avg)
    print(f"saved {CONFIG_PATH}")
    print(f"home marker in map frame: x={x:.3f} y={y:.3f} z={z:.3f} "
          f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}) "
          f"from {len(t_map_marker_samples)} samples")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
