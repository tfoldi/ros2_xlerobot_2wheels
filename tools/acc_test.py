#!/usr/bin/env python3
"""Raw-protocol test: does the Feetech ACC register value change how fast a servo actually moves?
Bypasses ROS entirely. Uses calibrate_arm.py's Bus class. One servo, one-shot moves, logs
PRESENT_POSITION over time after each command."""
import sys
import time

sys.path.insert(0, "/home/tfoldi/Developer/ros2_xlerobot_2wheels/tools")
from calibrate_arm import Bus, PRESENT_POSITION  # noqa: E402

ACCELERATION = (41, 1)
GOAL_POSITION = (42, 2)
GOAL_TIME = (44, 2)
GOAL_SPEED = (46, 2)

PORT = "/dev/xle-bus2"
MOTOR_ID = 1  # right_arm_shoulder_pan
SPEED = 2000  # raw steps/s, out of max 3400 - testing whether speed itself scales real motion
DELTA_STEPS = 120  # ~0.18 rad


def move(bus, acc, target, speed):
    bus.write(MOTOR_ID, ACCELERATION, acc)
    bus.write(MOTOR_ID, GOAL_TIME, 0)
    bus.write(MOTOR_ID, GOAL_SPEED, speed)
    bus.write(MOTOR_ID, GOAL_POSITION, target, sign_bit=15)


def log_for(bus, seconds, interval=0.1):
    t0 = time.time()
    rows = []
    while time.time() - t0 < seconds:
        pos = bus.position(MOTOR_ID)
        rows.append((time.time() - t0, pos))
        time.sleep(interval)
    return rows


def main():
    bus = Bus(PORT)
    start = bus.position(MOTOR_ID)
    print(f"start raw position: {start}")

    acc = 100
    speeds = [300, 800, 1500, 2000, 3000]
    pos = start
    for i, speed in enumerate(speeds):
        target = pos + DELTA_STEPS if i % 2 == 0 else pos - DELTA_STEPS
        print(f"\n--- ACC={acc}, speed={speed}, target={target} ---")
        move(bus, acc, target, speed)
        rows = log_for(bus, 2.5)
        for t, p in rows:
            print(f"{t:5.2f} {p:6d} (error {p - target:+d})")
        pos = rows[-1][1]

    bus.close()


if __name__ == "__main__":
    main()
