#!/usr/bin/env python3
"""Does repeating the IDENTICAL position command on alternating 10ms cycles (same average
advance rate as test A, but writing the same value twice before advancing) reproduce the
slowdown, unlike test A's strictly-monotonic-every-cycle stream?"""
import sys
import time

sys.path.insert(0, "/home/tfoldi/Developer/ros2_xlerobot_2wheels/tools")
from calibrate_arm import Bus  # noqa: E402

ACCELERATION = (41, 1)
GOAL_POSITION = (42, 2)
GOAL_TIME = (44, 2)
GOAL_SPEED = (46, 2)

PORT = "/dev/xle-bus2"
MOTOR_ID = 1
DELTA_STEPS = 120

bus = Bus(PORT)
start = bus.position(MOTOR_ID)
target = start + DELTA_STEPS
print(f"start={start} target={target}: 100Hz writes, alternating repeat/advance-by-2 "
      f"(same 1 step/cycle average as test A)")

t0 = time.time()
setpoint = start
next_write = t0
next_print = t0
cycle = 0
while True:
    now = time.time()
    if now >= next_write:
        if cycle % 2 == 1:
            setpoint = min(setpoint + 2, target)
        # else: repeat the same setpoint unchanged (no advance this cycle)
        cycle += 1
        bus.write(MOTOR_ID, ACCELERATION, 100)
        bus.write(MOTOR_ID, GOAL_TIME, 0)
        bus.write(MOTOR_ID, GOAL_SPEED, 600)
        bus.write(MOTOR_ID, GOAL_POSITION, setpoint, sign_bit=15)
        next_write += 0.01
    pos = bus.position(MOTOR_ID)
    t = now - t0
    if now >= next_print:
        print(f"{t:5.2f} setpoint={setpoint:6d} actual={pos:6d} error={pos - target:+d}")
        next_print += 0.3
    if t > 6.0:
        break
print(f"final error: {bus.position(MOTOR_ID) - target:+d} steps after 6.0s")
bus.close()
