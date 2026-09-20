#!/usr/bin/env python3
"""Scan a Feetech TTL bus and report which servo IDs answer. Useful for
sorting out which cable is bus1 and which is bus2 before writing udev rules,
see the README section on bus layout.

Needs a Feetech SDK on the python path, either lerobot's own
(lerobot.motors.feetech) or the standalone scservo_sdk. Not wired to either
yet, this is a placeholder for the real scan loop.

Usage:
    python3 check_feetech_bus.py /dev/ttyACM0
"""

import sys

EXPECTED_IDS = list(range(1, 11))  # 1-8 arm/head, 9-10 wheels, per bus this
                                    # obviously overcounts, just for scan range


def scan(port: str):
    print(f"scanning {port} for IDs {EXPECTED_IDS[0]}-{EXPECTED_IDS[-1]}")
    # TODO: open the bus with the Feetech SDK and ping each ID, print which
    # ones answer along with their reported model. Left unimplemented until
    # we've settled which SDK path this stack actually uses (see ADR-0001,
    # sts_hardware_interface likely bundles its own).
    raise NotImplementedError("wire this up once sts_hardware_interface is in src/")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <port>")
        sys.exit(1)
    scan(sys.argv[1])
