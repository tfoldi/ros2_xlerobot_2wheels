#!/usr/bin/env python3
"""Isolate Speed=600 (matches real driver's servo_speed_limit_steps) vs Speed=2000,
same ACC=100, same 1-step/cycle streaming pattern as raw test A."""
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


def stream_test(bus, label, speed):
    start = bus.position(MOTOR_ID)
    target = start + DELTA_STEPS
    print(f"\n--- {label}: ACC=100, Speed={speed}, 100Hz, 1 step/write ---")
    t0 = time.time()
    setpoint = start
    next_write = t0
    next_print = t0
    while True:
        now = time.time()
        if now >= next_write:
            setpoint = min(setpoint + 1, target)
            bus.write(MOTOR_ID, ACCELERATION, 100)
            bus.write(MOTOR_ID, GOAL_TIME, 0)
            bus.write(MOTOR_ID, GOAL_SPEED, speed)
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


bus = Bus(PORT)
stream_test(bus, "Speed=600 (matches real driver)", 600)
bus.close()
