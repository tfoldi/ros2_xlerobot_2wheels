#!/usr/bin/env python3
"""Save the current joint positions of a MoveIt group as a named position.

  python3 tools/save_position.py <name> --group right_arm

Overwrites an existing entry of the same name. See tools/position_store.py for the storage
format and tools/list_positions.py / tools/goto_position.py to inspect and use saved positions.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import position_store


class Reader(Node):
    def __init__(self):
        super().__init__("save_position")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("--group", required=True, choices=sorted(position_store.GROUP_JOINTS))
    args = parser.parse_args()
    joints = position_store.GROUP_JOINTS[args.group]

    rclpy.init()
    node = Reader()
    deadline = time.time() + 10
    while not all(j in node.positions for j in joints) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    missing = [j for j in joints if j not in node.positions]
    if missing:
        sys.exit(f"no /joint_states for: {', '.join(missing)} - is bringup running?")

    positions = position_store.load()
    positions[args.name] = {
        "group": args.group,
        "joints": {j: round(float(node.positions[j]), 5) for j in joints},
    }
    position_store.save(positions)
    print(f"saved '{args.name}' ({args.group}): "
          + ", ".join(f"{j}={v:.4f}" for j, v in positions[args.name]["joints"].items()))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
