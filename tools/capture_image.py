#!/usr/bin/env python3
"""Grab a single frame from one of the USB cameras and save it as a PNG.

  python3 tools/capture_image.py <head|left|right> [--output path.png]

Reads cameras/<name>/image_raw (plain sensor_msgs/Image, from src/xlerobot_bringup/launch/
cameras.launch.py's usb_cam nodes - the ffmpeg-encoded stream on the same topic tree is for
tools/rerun_bridge.py, not this). Needs the bringup running with cameras:=true.

Saves under captures/ by default (gitignored, ephemeral like trajectories/ - not curated).

Needs cv_bridge, not used elsewhere in this repo yet: sudo apt install ros-jazzy-cv-bridge
ros-jazzy-vision-opencv
"""
import argparse
import pathlib
import sys
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

CAPTURE_DIR = pathlib.Path(__file__).resolve().parent.parent / "captures"


class ImageGrabber(Node):
    def __init__(self, camera):
        super().__init__("capture_image")
        self.frame = None
        self.bridge = CvBridge()
        self.create_subscription(Image, f"cameras/{camera}/image_raw", self.on_image, 10)

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("camera", choices=["head", "left", "right"])
    parser.add_argument("--output", type=pathlib.Path, default=None)
    args = parser.parse_args()

    output = args.output or (CAPTURE_DIR / f"{args.camera}_{int(time.time())}.png")

    rclpy.init()
    node = ImageGrabber(args.camera)
    deadline = time.time() + 10
    while node.frame is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.frame is None:
        sys.exit(f"no frame from cameras/{args.camera}/image_raw - is bringup running with cameras:=true?")

    import cv2
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), node.frame)
    print(f"saved {output} ({node.frame.shape[1]}x{node.frame.shape[0]})")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
