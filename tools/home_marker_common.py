"""Shared constants and pose math for the home-marker AMCL seeding pair:
tools/home_marker_calibrate.py (run once, well-localized, saves the marker's map-frame pose)
and tools/home_marker_localize.py (run at cold start, publishes /initialpose from it).

See tools/generate_home_marker.py for the printable marker itself.

Needs scipy for rotation/quaternion math (present on the Jetson already, 1.11.4 - if missing
elsewhere: pip install scipy, or apt install python3-scipy to match how the rest of this repo's
tools/*.py rely on system packages rather than a venv).
"""
import math
import pathlib

import cv2.aruco as aruco
import numpy as np
import yaml

HOME_MARKER_DICT = aruco.DICT_4X4_250
HOME_MARKER_ID = 100  # outside the ChArUco board's own 0-43 ID range (tools/calibrate_camera_charuco.py)
HOME_MARKER_SIZE_M = 0.163  # measured on the actual print (target was 0.15 - printer rescaled it;
# matching the real printed size here is what matters, not the original target)

CONFIG_PATH = (pathlib.Path(__file__).resolve().parent.parent
               / "src" / "xlerobot_navigation" / "config" / "home_marker.yaml")


def detect_marker_pose(frame_bgr, camera_matrix, dist_coeffs):
    """One frame -> (rvec, tvec) of HOME_MARKER_ID in the camera's optical frame, or None.

    OpenCV's camera-optical convention (+X right, +Y down, +Z forward) matches this project's
    own camera_xyz=RDF choice for Pinhole logging (see tools/rerun_bridge.py), so no axis
    juggling is needed between this and the *_optical_frame TF frames.
    """
    gray = cv2_cvt_gray(frame_bgr)
    dictionary = aruco.getPredefinedDictionary(HOME_MARKER_DICT)
    corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    if ids is None or HOME_MARKER_ID not in ids.flatten():
        return None
    idx = list(ids.flatten()).index(HOME_MARKER_ID)
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        [corners[idx]], HOME_MARKER_SIZE_M, camera_matrix, dist_coeffs)
    return rvecs[0][0], tvecs[0][0]


def cv2_cvt_gray(frame_bgr):
    import cv2
    return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)


def rvec_tvec_to_matrix(rvec, tvec):
    """OpenCV (rvec, tvec), camera -> marker, to a 4x4 homogeneous transform."""
    import cv2
    rot, _ = cv2.Rodrigues(rvec)
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = tvec
    return m


def transform_to_matrix(transform):
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    from scipy.spatial.transform import Rotation
    t = transform.translation
    q = transform.rotation
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def matrix_to_xy_yaw(m):
    """4x4 homogeneous matrix -> (x, y, yaw), flattening to the ground plane. base_footprint's
    z/roll/pitch are ~0 by definition for a wheeled robot; any residual here is marker-detection
    noise, not signal, so it is dropped rather than published."""
    x, y = m[0, 3], m[1, 3]
    yaw = math.atan2(m[1, 0], m[0, 0])
    return x, y, yaw


def matrix_to_xyz_quat(m):
    """4x4 homogeneous matrix -> (x, y, z, qx, qy, qz, qw)."""
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(m[:3, :3]).as_quat()
    return (*m[:3, 3], *q)


def xyz_quat_to_matrix(x, y, z, qx, qy, qz, qw):
    from scipy.spatial.transform import Rotation
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    m[:3, 3] = [x, y, z]
    return m


def save_marker_pose(matrix_map_marker):
    x, y, z, qx, qy, qz, qw = matrix_to_xyz_quat(matrix_map_marker)
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(yaml.safe_dump({
        "home_marker": {
            "frame_id": "map",
            "marker_id": HOME_MARKER_ID,
            "size_m": HOME_MARKER_SIZE_M,
            "x": float(x), "y": float(y), "z": float(z),
            "qx": float(qx), "qy": float(qy), "qz": float(qz), "qw": float(qw),
        }
    }, default_flow_style=False))


def load_marker_pose():
    if not CONFIG_PATH.is_file():
        raise FileNotFoundError(
            f"{CONFIG_PATH} does not exist yet - run home_marker_calibrate.py first, "
            "while AMCL is well-localized and the marker is visible.")
    m = yaml.safe_load(CONFIG_PATH.read_text())["home_marker"]
    return xyz_quat_to_matrix(m["x"], m["y"], m["z"], m["qx"], m["qy"], m["qz"], m["qw"])
