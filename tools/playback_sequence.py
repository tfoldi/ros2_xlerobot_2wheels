#!/usr/bin/env python3
"""Move through a sequence of saved named positions in order, via MoveIt (collision-aware).

  python3 tools/playback_sequence.py pos1 pos2 pos3 [--loop 3] [--loop forever] [--scaling 0.3]

Each position can belong to a different MoveIt group (they don't have to match). Ctrl-C stops
cleanly after the move in progress finishes. See tools/save_position.py to create positions.
"""
import argparse
import sys

import rclpy

import position_store
from goto_position import GotoPosition


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("names", nargs="+")
    parser.add_argument("--loop", default="1", help="repeat count, or 'forever'")
    parser.add_argument("--scaling", type=float, default=0.0, help="0 = joint_limits.yaml default")
    args = parser.parse_args()

    entries = [(name, position_store.get(name)) for name in args.names]
    all_joints = [j for _, entry in entries for j in entry["joints"]]

    forever = args.loop == "forever"
    count = 0 if forever else int(args.loop)
    if not forever and count < 1:
        sys.exit("--loop must be a positive integer or 'forever'")

    rclpy.init()
    node = GotoPosition()
    node.wait_ready(all_joints)

    try:
        iteration = 0
        while forever or iteration < count:
            iteration += 1
            print(f"--- pass {iteration}{'' if not forever else ' (forever)'} ---")
            for name, entry in entries:
                print(f"moving to '{name}' ({entry['group']})")
                node.move(entry["group"], entry["joints"], args.scaling)
    except KeyboardInterrupt:
        print("\nstopped")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
