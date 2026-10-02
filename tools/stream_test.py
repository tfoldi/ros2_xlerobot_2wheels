#!/usr/bin/env python3
"""Does sending a train of small incremental WritePosEx commands every 10ms (like our driver's
write() loop does) reproduce the slow crawl, vs sending one command for the whole move?
Bypasses ROS. Same servo/registers as acc_test.py."""
import sys
import time

sys.path.insert(0, "/home/tfoldi/Developer/ros2_xlerobot_2wheels/tools")
from calibrate_arm import Bus, PRESENT_POSITION  # noqa: E402

ACCELERATION = (41, 1)
GOAL_POSITION = (42, 2)
GOAL_TIME = (44, 2)
GOAL_SPEED = (46, 2)

PORT = "/dev/xle-bus2"
MOTOR_ID = 1
ACC = 100
SPEED = 2000
DELTA_STEPS = 120
STEP_PER_CYCLE = 1  # matches a slow (~0.15 rad/s) MoveIt-planned trajectory's real per-cycle step
CYCLE_S = 0.01


def main():
    bus = Bus(PORT)
    start = bus.position(MOTOR_ID)
    target = start + DELTA_STEPS
    print(f"start={start} target={target}, streaming {STEP_PER_CYCLE}-step increments every "
          f"{CYCLE_S*1000:.0f}ms, ACC={ACC} SPEED={SPEED}")

    bus.write(MOTOR_ID, ACCELERATION, ACC)
    bus.write(MOTOR_ID, GOAL_TIME, 0)
    bus.write(MOTOR_ID, GOAL_SPEED, SPEED)

    t0 = time.time()
    setpoint = start
    next_cycle = t0
    next_print = t0
    while True:
        now = time.time()
        if now >= next_cycle:
            setpoint = min(setpoint + STEP_PER_CYCLE, target)
            bus.write(MOTOR_ID, GOAL_POSITION, setpoint, sign_bit=15)
            next_cycle += CYCLE_S
        pos = bus.position(MOTOR_ID)
        t = now - t0
        if now >= next_print:
            print(f"{t:5.2f} setpoint={setpoint:6d} actual={pos:6d} error={pos - target:+d}")
            next_print += 0.2
        if t > 8.0:
            break

    bus.close()


if __name__ == "__main__":
    main()
