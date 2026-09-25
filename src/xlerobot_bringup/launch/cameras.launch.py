"""USB cameras with H.264 video, from config/cameras.yaml. Included by
bringup.launch.py with cameras:=true, or run standalone.

One usb_cam node per camera whose device exists, in the namespace
cameras/<name>. image_transport's ffmpeg plugin encodes on
<name>/image_raw/ffmpeg while something subscribes to it.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _camera_nodes(context, *args, **kwargs):
    config_path = os.path.join(
        get_package_share_directory("xlerobot_bringup"), "config", "cameras.yaml")
    with open(config_path) as f:
        config = yaml.safe_load(f)
    d = config["defaults"]

    nodes = []
    for name, camera in config["cameras"].items():
        device = camera["device"]
        if not os.path.exists(device):
            print(f"[cameras.launch] {name}: {device} not found, skipping")
            continue
        cfg = {**d, **camera}
        nodes.append(Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace=f"cameras/{name}",
            parameters=[{
                "video_device": device,
                "image_width": cfg["width"],
                "image_height": cfg["height"],
                "framerate": float(cfg["fps"]),
                "pixel_format": cfg["pixel_format"],
                "camera_name": name,
                "camera_frame_id": f"{name}_camera_optical_frame",
                "camera_info_url": "",
                "image_raw.ffmpeg.encoder": cfg["encoder"],
                "image_raw.ffmpeg.encoder_av_options": cfg["encoder_options"],
                "image_raw.ffmpeg.bit_rate": cfg["bit_rate"],
                "image_raw.ffmpeg.gop_size": cfg["gop_size"],
                "image_raw.ffmpeg.max_b_frames": 0,
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_camera_nodes)])
