#!/usr/bin/env python3
"""Calibrate a camera against a ChArUco board, no GUI - just move the board in front of the
camera while this runs, it prints progress as it collects samples.

  python3 tools/calibrate_camera_charuco.py head
  python3 tools/calibrate_camera_charuco.py right --samples 150

Runs equally well pointed at a remote camera over the network (e.g. this laptop watching
cameras/head/image_raw published by the Jetson) - ROS2 topics don't care which machine is
subscribing, and the laptop's CPU does the detection/calibration math far faster. Only the usb_cam
driver itself needs to run where the hardware actually is.

Board defaults: 11x8 squares, 15mm squares, 11mm ArUco markers, DICT_4X4_250 - confirmed the right
orientation for the physical board this was built against (8x11, the other way around, fits much
worse - see calibrate_from_bag.py's docstring for how that was confirmed). Override with
--squares-x/--squares-y/--square-length/--marker-length/--dict if yours differs. Collects more raw
samples than a final calibration needs (default 120) because calibrate_with_outlier_rejection
below then trims the worst-fitting ones - see that function's docstring for why that step matters,
confirmed live to be the difference between an unusable and a working calibration.

A GUI alternative exists (ros-jazzy-camera-calibration's `cameracalibrator`), but its mouse-click
button only updates visibly on the next frame redraw and gives no progress feedback while
`do_calibration()` runs in a background thread - confirmed live (repeated clicks did fire
`do_calibration()`, 31 times in one session, visible in its own log) but on a remote/under-powered
display setup this looks exactly like a dead button, and the real-time cost of each click is
invisible until everything's already piled up. This script's per-sample console progress and
explicit completion report avoid that trap entirely.

Writes a standard camera_info YAML to ~/.ros/camera_info/<camera_name>.yaml (the path
camera_info_manager / usb_cam loads from on startup) - restart the camera node to pick it up.
"""
import argparse
import pathlib
import sys
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

MIN_CORNERS_PER_SAMPLE = 8  # below this, the frame contributes too little to be worth keeping
MIN_SAMPLE_INTERVAL_S = 0.5  # don't accept two samples closer together than this - forces the
# board to actually move between samples instead of grabbing near-duplicate frames

# Outlier rejection: calibrate, drop the worst-fitting views by their individual reprojection
# error (calibrateCameraCharucoExtended's perViewErrors), recalibrate, repeat. Needed in practice,
# not just in theory - a first live attempt without this converged to a plausible-looking camera
# matrix but a stubborn ~27px overall error regardless of board orientation, ArUco dictionary
# variant, or aspect-ratio constraints tried; this same rejection loop confirmed a working subset
# of views was in there all along (27px -> 8.6px -> 3.3px -> 0.86px over 4 rounds keeping 36/72
# original views), just dragged down by some number of frames with ordinary defects (most likely
# motion blur - the board was moving continuously during capture rather than paused per-pose) that
# a single unweighted least-squares fit has no way to discount on its own. Separately: an
# unconstrained fit can also converge to a deceptively low error with a physically nonsensical
# camera matrix (confirmed live: fx=5197, fy=4247 on a 640px-wide image) if too few samples survive
# rejection to still constrain scale - CALIB_FIX_ASPECT_RATIO (fx == fy, true for any real camera)
# and a sample-count floor guard against exactly that.
OUTLIER_DROP_FRACTION = 0.2
OUTLIER_MAX_ROUNDS = 6
OUTLIER_MIN_SAMPLES = 30
OUTLIER_TARGET_ERROR_PX = 1.0
CALIB_FLAGS = cv2.CALIB_FIX_ASPECT_RATIO


def detect_charuco(frame_bgr, board, dictionary, draw=False):
    """One frame -> (corner_count, charuco_corners, charuco_ids, preview_or_None). Shared between
    the live subscriber below and calibrate_from_bag.py's offline pass over a bag recording."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    preview = frame_bgr.copy() if draw else None
    if ids is None or len(ids) == 0:
        return 0, None, None, preview
    if draw:
        aruco.drawDetectedMarkers(preview, corners, ids)
    count, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if draw and ch_corners is not None:
        aruco.drawDetectedCornersCharuco(preview, ch_corners, ch_ids)
    return count, ch_corners, ch_ids, preview


class CharucoCalibrator(Node):
    WINDOW_NAME = "calibrate_camera_charuco - live preview (no buttons, just watch)"

    def __init__(self, camera, board, dictionary, min_samples, preview_path, show_window):
        super().__init__("calibrate_camera_charuco")
        self.bridge = CvBridge()
        self.board = board
        self.dictionary = dictionary
        self.min_samples = min_samples
        self.preview_path = preview_path
        self.show_window = show_window
        self.all_corners = []
        self.all_ids = []
        self.image_size = None
        self.last_sample_time = 0.0
        self.window_failed = False
        self.create_subscription(Image, f"cameras/{camera}/image_raw", self.on_image, 5)

    def on_image(self, msg):
        if len(self.all_corners) >= self.min_samples:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image_size = (frame.shape[1], frame.shape[0])

        count, ch_corners, ch_ids, preview = detect_charuco(
            frame, self.board, self.dictionary, draw=bool(self.preview_path) or self.show_window)
        if ch_corners is not None:
            now = time.monotonic()
            if (count >= MIN_CORNERS_PER_SAMPLE
                    and now - self.last_sample_time >= MIN_SAMPLE_INTERVAL_S):
                self.all_corners.append(ch_corners)
                self.all_ids.append(ch_ids)
                self.last_sample_time = now
                print(f"sample {len(self.all_corners)}/{self.min_samples} "
                      f"({count} corners detected)")
        if self.preview_path:
            cv2.imwrite(str(self.preview_path), preview)
        if self.show_window and not self.window_failed:
            try:
                overlay = f"samples: {len(self.all_corners)}/{self.min_samples}  corners: {count}"
                cv2.putText(preview, overlay, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(self.WINDOW_NAME, preview)
                cv2.waitKey(1)  # required for the window to actually redraw - no button handling, just display
            except cv2.error as e:
                print(f"(live preview window unavailable, continuing without it: {e})")
                self.window_failed = True


def build_board(args):
    try:
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    except AttributeError:
        sys.exit(f"unknown dictionary {args.dict!r} - try e.g. DICT_4X4_250")
    board = aruco.CharucoBoard_create(
        args.squares_x, args.squares_y, args.square_length, args.marker_length, dictionary)
    return board, dictionary


def write_camera_info(path, camera_name, size, camera_matrix, dist_coeffs):
    w, h = size
    # .tolist() on the full matrix/vector already yields plain Python floats below, but indexing
    # single elements (camera_matrix[0, 0]) returns numpy.float64 scalars instead - PyYAML's
    # safe_dump doesn't know how to represent those, only real Python floats. float() each one.
    fx, fy, cx, cy = (float(camera_matrix[0, 0]), float(camera_matrix[1, 1]),
                       float(camera_matrix[0, 2]), float(camera_matrix[1, 2]))
    data = {
        "image_width": w,
        "image_height": h,
        "camera_name": camera_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": camera_matrix.flatten().tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": dist_coeffs.flatten()[:5].tolist()},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
        "projection_matrix": {"rows": 3, "cols": 4,
                               "data": [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)


def run_calibration(all_corners, all_ids, board, image_size, flags=0):
    """One-shot calibration, no outlier rejection - kept for reference/comparison, prefer
    calibrate_with_outlier_rejection below for anything real. Returns (ret, camera_matrix,
    dist_coeffs)."""
    ret, camera_matrix, dist_coeffs, _, _ = aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, None, None, flags=flags)
    return ret, camera_matrix, dist_coeffs


def calibrate_with_outlier_rejection(all_corners, all_ids, board, image_size, label=""):
    """Shared with calibrate_from_bag.py. See the OUTLIER_* constants' comment for why this is
    needed. Returns (ret, camera_matrix, dist_coeffs, n_samples_used) for whichever round looked
    best, prioritizing a round that hit the target error with the most remaining samples over a
    later round that's marginally better but thinner."""
    corners, ids = list(all_corners), list(all_ids)
    w, h = image_size
    # CALIB_FIX_ASPECT_RATIO reads the fixed fx/fy ratio from this initial guess (1.0 here) rather
    # than actually using it as a full guess - CALIB_USE_INTRINSIC_GUESS is deliberately not set.
    init_camera_matrix = np.array([[1.0, 0, w / 2], [0, 1.0, h / 2], [0, 0, 1]])
    best = None
    for round_num in range(OUTLIER_MAX_ROUNDS):
        try:
            ret, cm, dc, _, _, _, _, per_view = aruco.calibrateCameraCharucoExtended(
                corners, ids, board, image_size, init_camera_matrix.copy(), None, flags=CALIB_FLAGS)
        except Exception as e:
            print(f"{label} round {round_num}: calibration failed ({e})")
            break
        n = len(corners)
        print(f"{label} round {round_num}: {n} samples, RMS {ret:.3f} px")
        if best is None or ret < best[0]:
            best = (ret, cm, dc, n)
        if ret <= OUTLIER_TARGET_ERROR_PX or n <= OUTLIER_MIN_SAMPLES:
            break
        errs = per_view.flatten()
        keep_count = max(OUTLIER_MIN_SAMPLES, int(n * (1 - OUTLIER_DROP_FRACTION)))
        keep_set = set(np.argsort(errs)[:keep_count].tolist())
        corners = [c for i, c in enumerate(corners) if i in keep_set]
        ids = [d for i, d in enumerate(ids) if i in keep_set]
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("camera", choices=["head", "left", "right"])
    parser.add_argument("--samples", type=int, default=120,
                         help="raw samples to collect before outlier rejection trims them down")
    parser.add_argument("--squares-x", type=int, default=11,
                         help="confirmed orientation for this board - 8x11 (the other way around) "
                         "fits much worse, see calibrate_from_bag.py's docstring")
    parser.add_argument("--squares-y", type=int, default=8)
    parser.add_argument("--square-length", type=float, default=0.015)
    parser.add_argument("--marker-length", type=float, default=0.011)
    parser.add_argument("--dict", default="DICT_4X4_250")
    parser.add_argument("--preview", type=pathlib.Path, default=pathlib.Path("/tmp/charuco_preview.png"),
                         help="overwritten every frame with marker/corner overlay - set empty to disable")
    parser.add_argument("--output", type=pathlib.Path, default=None,
                         help="default: ~/.ros/camera_info/<camera>.yaml")
    parser.add_argument("--no-window", action="store_false", dest="show_window", default=True,
                         help="skip the live cv2.imshow preview window (e.g. on a headless box)")
    args = parser.parse_args()
    board, dictionary = build_board(args)
    output = args.output or (pathlib.Path.home() / ".ros" / "camera_info" / f"{args.camera}.yaml")

    rclpy.init()
    node = CharucoCalibrator(args.camera, board, dictionary, args.samples, args.preview, args.show_window)
    print(f"watching cameras/{args.camera}/image_raw - move the board around, "
          f"preview written to {args.preview}"
          + (f", live window: {node.WINDOW_NAME!r}" if args.show_window else ""))
    try:
        while rclpy.ok() and len(node.all_corners) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print(f"\nstopped early with {len(node.all_corners)} samples")
    finally:
        if args.show_window:
            cv2.destroyAllWindows()

    if len(node.all_corners) < OUTLIER_MIN_SAMPLES:
        sys.exit(f"not enough samples to calibrate reliably (need at least {OUTLIER_MIN_SAMPLES}, "
                  f"got {len(node.all_corners)}) - is the board actually in view?")

    print(f"calibrating from {len(node.all_corners)} samples, with outlier rejection...")
    result = calibrate_with_outlier_rejection(node.all_corners, node.all_ids, board, node.image_size)
    if result is None:
        sys.exit("calibration failed")
    ret, camera_matrix, dist_coeffs, n = result
    print(f"\nfinal: {n} samples after outlier rejection, "
          f"reprojection error {ret:.3f} px (lower is better, under ~0.5-1.0 is good)")
    print(f"camera matrix:\n{camera_matrix}")
    print(f"distortion coefficients: {dist_coeffs.flatten()}")

    write_camera_info(output, args.camera, node.image_size, camera_matrix, dist_coeffs)
    print(f"wrote {output} - restart the camera node to pick it up")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
