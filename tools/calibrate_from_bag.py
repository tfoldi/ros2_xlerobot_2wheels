#!/usr/bin/env python3
"""Calibrate a camera from a rosbag recording of its image topic, instead of a live feed - lets
the dataset be fixed once (recorded while watching the camera_calibration GUI's X/Y/Size/Skew
coverage bars for feedback) and the calibration math iterated on repeatedly without re-recording.

  python3 tools/calibrate_from_bag.py ~/charuco_head_bag/charuco_head_bag_0.mcap head

Reads the mcap file with a plain sequential stream reader rather than rosbag2's indexed Reader
API, since an abruptly-killed `ros2 bag record` (no SIGINT handling that actually worked here -
not investigated further) leaves the file's trailing summary/footer corrupt even though the
actual message chunks it already wrote are intact; the indexed reader needs that footer first and
refuses the whole file, the sequential one just reads records front-to-back and stops cleanly at
the truncation point.

Tries both board orientations ("8x11" is ambiguous about which axis is which - confirmed live
that getting this backwards produces a real but much worse fit, ~70px vs ~27px reprojection
error, not a total failure, which is what made it non-obvious at first) and, within each, applies
iterative outlier rejection: calibrate, drop the worst-fitting 20% of views by their individual
reprojection error (calibrateCameraCharucoExtended's perViewErrors), recalibrate, repeat. A first
attempt without this converged to a plausible-looking camera matrix but a stubborn ~27px overall
error regardless of orientation/dictionary/aspect-ratio constraints - confirmed via this same
rejection loop that a working subset of views was in there all along (27px -> 8.6px -> 3.3px ->
0.86px over 4 rounds keeping 36/72 original views), just dragged down by some number of frames
with real but ordinary defects (most likely motion blur, the board was moving continuously during
capture rather than paused per-pose) that a single unweighted least-squares fit has no way to
discount on its own.
"""
import argparse
import pathlib
import sys

import cv2
import cv2.aruco as aruco
import numpy as np
from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader
from mcap.exceptions import EndOfFile
from mcap_ros2.decoder import Decoder

from calibrate_camera_charuco import detect_charuco, write_camera_info

MIN_SAMPLE_GAP_FRAMES = 2  # frames, not wall-clock - bag has no live timing to rely on
MIN_CORNERS_INITIAL = 8  # low on purpose - outlier rejection below cleans up the rest
OUTLIER_DROP_FRACTION = 0.2
OUTLIER_MAX_ROUNDS = 6
OUTLIER_MIN_SAMPLES = 30  # stop rejecting before falling below this - a low pixel error from too
# few remaining samples is a red flag, not a win: confirmed live, unconstrained rejection down to
# 36 samples hit 0.86px RMS with a physically nonsensical camera matrix (fx=5197, fy=4247 on a
# 640px-wide image - implies a ~7deg FOV telephoto lens on an ordinary webcam). The remaining
# samples had too little pose/distance diversity left to actually constrain the fit; the low
# error was the optimizer exploiting that, not evidence of a good calibration. Keeping more
# samples preserves diversity at the cost of a higher but trustworthy error.
OUTLIER_TARGET_ERROR_PX = 1.0  # stop early once this good
CALIB_FLAGS = cv2.CALIB_FIX_ASPECT_RATIO  # fx == fy is true for any real camera with square
# pixels - without this constraint the optimizer is free to let fx/fy diverge to fit noise,
# exactly what produced the nonsensical result above.


def extract_frames(bag_path, topic, stride=1):
    """Yields BGR numpy frames from the given topic, in order, every `stride`-th decodable one.
    Stops cleanly (not an error) at the first point the file can't be read further - expected for
    a bag that was force-killed rather than cleanly closed."""
    decoder = Decoder()
    schemas, channels = {}, {}
    count = 0
    with open(bag_path, "rb") as f:
        reader = StreamReader(f)
        try:
            for record in reader.records:
                if isinstance(record, Schema):
                    schemas[record.id] = record
                elif isinstance(record, Channel):
                    channels[record.id] = record
                elif isinstance(record, Message):
                    channel = channels[record.channel_id]
                    if channel.topic != topic:
                        continue
                    count += 1
                    if count % stride:
                        continue
                    msg = decoder.decode(schemas[channel.schema_id], record)
                    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                    if msg.encoding == "rgb8":
                        frame = frame[:, :, ::-1]  # rgb8 -> bgr for cv2/aruco
                    yield frame
        except EndOfFile:
            pass
    print(f"read {count} messages on {topic} from {bag_path} (truncation at the end is expected)")


def detect_all(frames, board, dictionary):
    """One detection pass over every frame for this board orientation - expensive, run once per
    orientation and filter the (cheap, in-memory) result afterward rather than re-detecting."""
    detections = []
    image_size = None
    for i, frame in enumerate(frames):
        image_size = (frame.shape[1], frame.shape[0])
        count, ch_corners, ch_ids, _ = detect_charuco(frame, board, dictionary)
        if ch_corners is not None:
            detections.append((i, count, ch_corners, ch_ids))
        if (i + 1) % 200 == 0:
            print(f"  detected {i + 1}/{len(frames)} frames...")
    return detections, image_size


def filter_samples(detections, min_corners):
    """Cheap in-memory filter over an already-detected pass - no re-detection."""
    all_corners, all_ids = [], []
    last_accepted = -MIN_SAMPLE_GAP_FRAMES
    for i, count, ch_corners, ch_ids in detections:
        if count >= min_corners and i - last_accepted >= MIN_SAMPLE_GAP_FRAMES:
            all_corners.append(ch_corners)
            all_ids.append(ch_ids)
            last_accepted = i
    return all_corners, all_ids


def calibrate_with_outlier_rejection(all_corners, all_ids, board, image_size, label=""):
    """Calibrate, then repeatedly drop the worst-fitting views and recalibrate - see module
    docstring for why this is needed (not every view is equally good, and a handful of bad ones
    can dominate an unweighted least-squares fit). Returns (ret, camera_matrix, dist_coeffs, n)
    for whichever round looked best, prioritizing a round that hit the target error with the
    most remaining samples over a later round that's marginally better but thinner."""
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
        keep_indices = np.argsort(errs)[:keep_count]
        keep_set = set(keep_indices.tolist())
        corners = [c for i, c in enumerate(corners) if i in keep_set]
        ids = [d for i, d in enumerate(ids) if i in keep_set]
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path", type=pathlib.Path, help="path to the .mcap file")
    parser.add_argument("camera", choices=["head", "left", "right"])
    parser.add_argument("--topic", default=None, help="default: /cameras/<camera>/image_raw")
    parser.add_argument("--squares-x", type=int, default=8)
    parser.add_argument("--squares-y", type=int, default=11)
    parser.add_argument("--square-length", type=float, default=0.015)
    parser.add_argument("--marker-length", type=float, default=0.011)
    parser.add_argument("--dict", default="DICT_4X4_250")
    parser.add_argument("--stride", type=int, default=3, help="use every Nth decoded frame")
    parser.add_argument("--output", type=pathlib.Path, default=None)
    args = parser.parse_args()
    topic = args.topic or f"/cameras/{args.camera}/image_raw"
    output = args.output or (pathlib.Path.home() / ".ros" / "camera_info" / f"{args.camera}.yaml")

    try:
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, args.dict))
    except AttributeError:
        sys.exit(f"unknown dictionary {args.dict!r}")

    frames = list(extract_frames(args.bag_path, topic, args.stride))
    if not frames:
        sys.exit(f"no frames decoded on {topic} - wrong topic name, or bag truncated before any message?")

    orientations = [(args.squares_x, args.squares_y), (args.squares_y, args.squares_x)]
    best_overall = None
    for sx, sy in orientations:
        board = aruco.CharucoBoard_create(sx, sy, args.square_length, args.marker_length, dictionary)
        print(f"\ndetecting for {sx}x{sy} orientation ({len(frames)} frames)...")
        detections, image_size = detect_all(frames, board, dictionary)
        all_corners, all_ids = filter_samples(detections, MIN_CORNERS_INITIAL)
        print(f"  {len(all_corners)} usable samples before outlier rejection")
        if len(all_corners) < OUTLIER_MIN_SAMPLES:
            print(f"  too few samples, skipping {sx}x{sy}")
            continue
        result = calibrate_with_outlier_rejection(all_corners, all_ids, board, image_size, label=f"[{sx}x{sy}]")
        if result is None:
            continue
        ret, camera_matrix, dist_coeffs, n = result
        if best_overall is None or ret < best_overall[0]:
            best_overall = (ret, camera_matrix, dist_coeffs, image_size, sx, sy, n)

    if best_overall is None:
        sys.exit("no orientation produced a usable calibration")

    ret, camera_matrix, dist_coeffs, image_size, sx, sy, n = best_overall
    print(f"\nbest: {sx}x{sy} board, {n} samples after outlier rejection, "
          f"reprojection error {ret:.3f} px (lower is better, under ~0.5-1.0 is good)")
    print(f"camera matrix:\n{camera_matrix}")
    print(f"distortion coefficients: {dist_coeffs.flatten()}")

    write_camera_info(output, args.camera, image_size, camera_matrix, dist_coeffs)
    print(f"wrote {output} - restart the camera node to pick it up")


if __name__ == "__main__":
    main()
