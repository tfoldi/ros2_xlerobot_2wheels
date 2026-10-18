#!/usr/bin/env python3
"""Check the head depth camera's mounting pose against the floor.

  python3 tools/calibrate_head_pose.py [--frames 30]

Fits the dominant floor plane in /cameras/head/depth/points (RANSAC, then least squares on
the inliers, averaged over --frames clouds) and reports, in the cloud's own optical frame
(read live from the message header, not hardcoded):

  - camera height above the floor
  - pitch (optical axis above the horizon is positive) and roll (right side of the image
    higher is positive) relative to the floor
  - plane-fit RMS over the inliers

The floor only fixes height, roll and pitch. x, y and yaw of the mount cannot be seen from a
plane and are left at their current values.

If TF base_footprint -> <optical frame> is available (bringup running, robot standing on a
level floor), it also prints the head_camera_joint origin that would make the URDF agree with
the measurement, for src/xlerobot_description/urdf/sensors.xacro (gemini branch). The current
head pan/tilt are included through TF, so any head pose works. The head_link_joint under it
stays identity. Run it on the Jetson, the cloud is ~15 MB per message.

Assumes base_footprint sits on the floor plane, so a robot on a ramp or carpet that sinks
gives a matching offset in the result.
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

CLOUD_TOPIC = "cameras/head/depth/points"
BASE_FRAME = "base_footprint"
MOUNT_FRAME = "head_camera_mount"
LINK_FRAME = "head_link"  # the driver's tree root, sits on head_camera_joint's child

MIN_RANGE = 0.3  # m
MAX_RANGE = 3.0  # m
MIN_BELOW = 0.05  # m, a point must be at least this far below the camera's horizontal plane
SUBSAMPLE_POINTS = 30000
RANSAC_ITERATIONS = 300
INLIER_THRESHOLD = 0.015  # m
MAX_TILT_FROM_DOWN = math.radians(60)  # floor normal must be within this of the camera's "up"
MIN_INLIERS = 2000
MIN_INLIER_FRACTION = 0.25  # of the cropped cloud, a floor in view fills far more than a wall does
OUTLIER_HEIGHT = 0.03  # m, frames further than this from the median height are dropped


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def matrix_to_rpy(r):
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(r[2, 0]) < 0.999999:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def transform_to_matrix(transform):
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def cloud_xyz(msg):
    """PointCloud2 -> (N, 3) float32 x, y, z, finite points only."""
    fields = {f.name: f for f in msg.fields}
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [np.float32] * 3,
        "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
        "itemsize": msg.point_step,
    })
    arr = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def fit_floor(xyz, rng):
    """Dominant plane below the camera. Returns (up, height, rms, inliers, candidates) or None.

    Optical frame is +x right, +y down, +z forward, so the camera's own "up" is -y and a floor
    point has n . p = -height for the unit normal n pointing from the floor to the camera.
    """
    dist = np.linalg.norm(xyz, axis=1)
    keep = (dist > MIN_RANGE) & (dist < MAX_RANGE) & (xyz[:, 1] > MIN_BELOW)
    pts = xyz[keep]
    if len(pts) < MIN_INLIERS:
        return None
    if len(pts) > SUBSAMPLE_POINTS:
        pts = pts[rng.choice(len(pts), SUBSAMPLE_POINTS, replace=False)]
    pts = pts.astype(np.float64)

    tri = pts[rng.integers(0, len(pts), size=(RANSAC_ITERATIONS, 3))]
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    normals = normals[norms > 1e-9] / norms[norms > 1e-9, None]
    tri = tri[norms > 1e-9]
    normals[normals[:, 1] > 0] *= -1  # point up (-y in the optical frame)
    cos_tilt = -normals[:, 1]  # n . (0, -1, 0)
    ok = cos_tilt > math.cos(MAX_TILT_FROM_DOWN)
    normals, tri = normals[ok], tri[ok]
    if len(normals) == 0:
        return None
    offsets = -np.einsum("ij,ij->i", normals, tri[:, 0])  # n . p + d = 0
    residuals = np.abs(pts @ normals.T + offsets)  # (N, K)
    counts = (residuals < INLIER_THRESHOLD).sum(axis=0)
    best = int(np.argmax(counts))
    if counts[best] < MIN_INLIERS or counts[best] < MIN_INLIER_FRACTION * len(pts):
        return None

    inl = pts[residuals[:, best] < INLIER_THRESHOLD]
    for _ in range(3):  # least squares refit, then re-gather inliers around the refit
        centroid = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
        n = vt[2]
        if n[1] > 0:
            n = -n
        d = -n @ centroid
        inl = pts[np.abs(pts @ n + d) < INLIER_THRESHOLD]
    rms = float(np.sqrt(np.mean((inl @ n + d) ** 2)))
    return n, float(d), rms, len(inl), len(pts)


def corrected_mount(t_base_cam, t_base_mount, t_link_cam, up_cam, height):
    """Mount->head_link transform that puts the camera at `height` with the measured tilt,
    keeping its current x, y and heading in base_footprint."""
    r_cur = t_base_cam[:3, :3]
    fwd = r_cur[:, 2]  # optical axis in base_footprint
    heading = math.atan2(fwd[1], fwd[0])
    nx, ny, nz = up_cam  # base z axis expressed in camera axes == camera axes' z in base
    hx = math.sqrt(max(1e-12, 1.0 - nz * nz))
    f = np.array([math.cos(heading), math.sin(heading), 0.0])
    left = np.array([-math.sin(heading), math.cos(heading), 0.0])
    ez = np.array([0.0, 0.0, 1.0])
    z_b = hx * f + nz * ez
    alpha = -nx * nz / hx
    beta = -math.sqrt(max(0.0, 1.0 - alpha * alpha - nx * nx))  # camera x points to the right
    x_b = alpha * f + beta * left + nx * ez
    y_b = np.cross(z_b, x_b)
    t_base_cam_new = np.eye(4)
    t_base_cam_new[:3, :3] = np.stack([x_b, y_b, z_b], axis=1)
    t_base_cam_new[:3, 3] = [t_base_cam[0, 3], t_base_cam[1, 3], height]
    t_base_link_new = t_base_cam_new @ np.linalg.inv(t_link_cam)
    return np.linalg.inv(t_base_mount) @ t_base_link_new


class HeadPose(Node):
    def __init__(self, frames):
        super().__init__("calibrate_head_pose")
        self.frames = frames
        self.results = []
        self.frame_id = None
        self.rng = np.random.default_rng(0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        if len(self.results) >= self.frames:
            return
        self.frame_id = msg.header.frame_id
        fit = fit_floor(cloud_xyz(msg), self.rng)
        if fit is None:
            self.get_logger().warning(
                "no dominant floor plane in this cloud, tilt the head down so the floor fills the "
                "lower part of the view", throttle_duration_sec=3.0)
            return
        self.results.append(fit)
        self.get_logger().info(
            f"frame {len(self.results)}/{self.frames}: height {fit[1]:.3f} m, "
            f"{fit[3]}/{fit[4]} inliers, rms {fit[2] * 1000:.1f} mm")


def report(node):
    res = node.results
    heights = np.array([r[1] for r in res])
    median = float(np.median(heights))
    kept = [r for r in res if abs(r[1] - median) < OUTLIER_HEIGHT]
    if not kept:
        kept = res
    ups = np.array([r[0] for r in kept])
    up = ups.mean(axis=0)
    up /= np.linalg.norm(up)
    height = float(np.mean([r[1] for r in kept]))
    height_std = float(np.std([r[1] for r in kept]))
    rms = float(np.mean([r[2] for r in kept]))
    frac = float(np.mean([r[3] / r[4] for r in kept]))
    # asin of the up vector's z/x components in the camera frame
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, up[2]))))
    roll = math.degrees(math.asin(max(-1.0, min(1.0, up[0]))))
    tilt_spread = float(np.degrees(np.mean(np.arccos(np.clip(ups @ up, -1.0, 1.0)))))

    print()
    print(f"Floor plane in {node.frame_id}, {len(kept)}/{len(res)} frames kept")
    print(f"  camera height above floor : {height:.3f} m  (std {height_std * 1000:.1f} mm)")
    print(f"  pitch, optical axis vs horizon : {pitch:+.2f} deg  (+ = looking up)")
    print(f"  roll, image right side vs left : {roll:+.2f} deg  (+ = right side higher)")
    print(f"  floor normal in camera axes    : ({up[0]:+.4f}, {up[1]:+.4f}, {up[2]:+.4f})")
    print(f"  plane-fit rms {rms * 1000:.1f} mm, inliers {frac * 100:.0f}% of the cropped cloud, "
          f"normal spread {tilt_spread:.2f} deg")

    try:
        t_base_cam = transform_to_matrix(node.tf_buffer.lookup_transform(
            BASE_FRAME, node.frame_id, rclpy.time.Time()).transform)
        t_base_mount = transform_to_matrix(node.tf_buffer.lookup_transform(
            BASE_FRAME, MOUNT_FRAME, rclpy.time.Time()).transform)
        t_link_cam = transform_to_matrix(node.tf_buffer.lookup_transform(
            LINK_FRAME, node.frame_id, rclpy.time.Time()).transform)
    except TransformException as e:
        print(f"\nTF is missing ({e}), so no URDF correction. Bring the robot up and rerun for it.")
        return

    cur_z = t_base_cam[2, 3]
    print(f"\nTF says the optical frame is {cur_z:.3f} m above {BASE_FRAME} now "
          f"(measured {height:.3f} m, diff {1000 * (height - cur_z):+.0f} mm)")
    new = corrected_mount(t_base_cam, t_base_mount, t_link_cam, up, height)
    cur = np.linalg.inv(t_base_mount) @ t_base_cam @ np.linalg.inv(t_link_cam)
    for name, m in (("current", cur), ("corrected", new)):
        r, p, y = matrix_to_rpy(m[:3, :3])
        print(f"  head_camera_joint origin, {name:9s}: xyz=\"{m[0, 3]:.4f} {m[1, 3]:.4f} {m[2, 3]:.4f}\" "
              f"rpy=\"{r:.4f} {p:.4f} {y:.4f}\"")
    print("  x, y and yaw are not measured by the floor, they are the current values carried over.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for frames")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = HeadPose(args.frames)
    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    try:
        while rclpy.ok() and len(node.results) < args.frames:
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.get_clock().now().nanoseconds / 1e9 > deadline:
                break
        # give TF a moment to fill if it is running
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.results:
            print("No floor found. Is cameras/head/depth/points publishing, and does the floor fill "
                  "the lower part of the view within 0.3-3 m?", file=sys.stderr)
            sys.exit(1)
        report(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
