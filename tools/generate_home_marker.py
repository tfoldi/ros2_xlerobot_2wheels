#!/usr/bin/env python3
"""Generate the printable ArUco home marker used by home_marker_calibrate.py /
home_marker_localize.py to seed AMCL's initial pose at cold start.

  python3 tools/generate_home_marker.py --output /tmp/home_marker.png

DICT_4X4_250, ID 100 - same dictionary tools/calibrate_camera_charuco.py's ChArUco board uses,
but an ID well outside that board's own range (an 11x8 ChAruco board uses IDs 0-43), so the two
are never confused if both end up in frame at some point.

Print at the exact physical size in HOME_MARKER_SIZE_M (home_marker_common.py) - printer
"fit to page"/"scale to fit" settings often resize it slightly, so measure the printed square
with a ruler and adjust HOME_MARKER_SIZE_M if it's off by more than a couple mm. Pose accuracy
scales directly with getting this measurement right.
"""
import argparse
import pathlib

import cv2
import cv2.aruco as aruco

from home_marker_common import HOME_MARKER_DICT, HOME_MARKER_ID, HOME_MARKER_SIZE_M

PIXELS = 1000  # print resolution, not physical size - that's HOME_MARKER_SIZE_M


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("/tmp/home_marker.png"))
    args = parser.parse_args()

    dictionary = aruco.getPredefinedDictionary(HOME_MARKER_DICT)
    marker = aruco.drawMarker(dictionary, HOME_MARKER_ID, PIXELS)

    # White border so the detector's quiet-zone margin isn't right at the paper edge.
    border = PIXELS // 8
    canvas = cv2.copyMakeBorder(marker, border, border, border, border, cv2.BORDER_CONSTANT, value=255)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), canvas)
    print(f"saved {args.output} ({canvas.shape[1]}x{canvas.shape[0]} px)")
    print(f"print so the BLACK marker square (not counting the white border) measures exactly "
          f"{HOME_MARKER_SIZE_M * 100:.0f} cm per side - verify with a ruler after printing.")
    print(f"dictionary: DICT_4X4_250, marker ID: {HOME_MARKER_ID}")


if __name__ == "__main__":
    main()
