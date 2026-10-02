#!/usr/bin/env python3
"""Opus-proposed discriminating test matrix for the streaming-throughput mystery.
Raw protocol, ROS bypassed entirely. Single servo: right_arm_shoulder_pan (bus2 id 1).

A: WritePosEx individual, ACC=100, 100Hz, 1 step/write        (baseline - reproduce ~50 steps/s)
B: WritePosEx individual, ACC=254, 100Hz, 1 step/write        (restart model predicts ~127 steps/s)
C: WritePosEx individual, ACC=100, 50Hz, 2 steps/write        (restart model predicts ~100 steps/s)
D: Goal_Position-only write (reg 42), ACC/Speed set ONCE, 100Hz, 1 step/write  (the LeRobot-style question)
F: One-shot, large move (~1.5 rad), ACC=100 and ACC=254        (real cruise speed - sets true max_velocity)
"""
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
DELTA_STEPS = 120  # ~0.18 rad, matches prior tests for comparability
RAD_PER_STEP = 6.2832 / 4096


def stream_test(bus, label, acc, write_hz, step_per_write, goal_only=False):
    start = bus.position(MOTOR_ID)
    target = start + DELTA_STEPS
    period = 1.0 / write_hz
    print(f"\n--- {label}: ACC={acc}, {write_hz}Hz, {step_per_write} step/write, "
          f"goal_only={goal_only} ---")
    if goal_only:
        bus.write(MOTOR_ID, ACCELERATION, acc)
        bus.write(MOTOR_ID, GOAL_TIME, 0)
        bus.write(MOTOR_ID, GOAL_SPEED, 2000)
    t0 = time.time()
    setpoint = start
    next_write = t0
    next_print = t0
    settle_start = None
    while True:
        now = time.time()
        if now >= next_write:
            setpoint = min(setpoint + step_per_write, target)
            if goal_only:
                bus.write(MOTOR_ID, GOAL_POSITION, setpoint, sign_bit=15)
            else:
                bus.write(MOTOR_ID, ACCELERATION, acc)
                bus.write(MOTOR_ID, GOAL_TIME, 0)
                bus.write(MOTOR_ID, GOAL_SPEED, 2000)
                bus.write(MOTOR_ID, GOAL_POSITION, setpoint, sign_bit=15)
            next_write += period
        pos = bus.position(MOTOR_ID)
        t = now - t0
        if now >= next_print:
            print(f"{t:5.2f} setpoint={setpoint:6d} actual={pos:6d} error={pos - target:+d}")
            next_print += 0.3
        if setpoint == target and pos == target and settle_start is None:
            settle_start = t
        if t > 6.0:
            break
    final_error = bus.position(MOTOR_ID) - target
    print(f"final error: {final_error:+d} steps after 6.0s")
    return bus.position(MOTOR_ID)


def one_shot(bus, label, acc, speed, delta_steps):
    start = bus.position(MOTOR_ID)
    target = start + delta_steps
    print(f"\n--- {label}: one-shot ACC={acc} Speed={speed} delta={delta_steps} steps "
          f"({delta_steps*RAD_PER_STEP:.3f} rad) ---")
    bus.write(MOTOR_ID, ACCELERATION, acc)
    bus.write(MOTOR_ID, GOAL_TIME, 0)
    bus.write(MOTOR_ID, GOAL_SPEED, speed)
    bus.write(MOTOR_ID, GOAL_POSITION, target, sign_bit=15)
    t0 = time.time()
    next_print = t0
    prev = start
    prev_t = 0.0
    peak_vel_steps_s = 0.0
    while True:
        now = time.time()
        pos = bus.position(MOTOR_ID)
        t = now - t0
        if t - prev_t > 0.05:
            vel = abs(pos - prev) / (t - prev_t)
            peak_vel_steps_s = max(peak_vel_steps_s, vel)
            prev, prev_t = pos, t
        if now >= next_print:
            print(f"{t:5.2f} actual={pos:6d} error={pos - target:+d}")
            next_print += 0.3
        if t > 8.0:
            break
    final_error = bus.position(MOTOR_ID) - target
    print(f"final error: {final_error:+d} steps, peak observed velocity ~{peak_vel_steps_s:.0f} "
          f"steps/s ({peak_vel_steps_s*RAD_PER_STEP:.3f} rad/s)")
    return bus.position(MOTOR_ID)


SAFE_RAW_MIN = 800   # right_arm_shoulder_pan center_step=2075, limit ~+-2.26 rad (~1471 steps)
SAFE_RAW_MAX = 3300  # this window stays well clear of both ends


def main():
    bus = Bus(PORT)

    pos = stream_test(bus, "A (baseline)", acc=100, write_hz=100, step_per_write=1)
    pos = stream_test(bus, "B (higher ACC)", acc=254, write_hz=100, step_per_write=1)
    pos = stream_test(bus, "C (slower write, bigger step)", acc=100, write_hz=50, step_per_write=2)
    pos = stream_test(bus, "D (goal-position-only writes)", acc=100, write_hz=100, step_per_write=1,
                       goal_only=True)

    # F: large one-shot move, direction chosen to stay inside the safe window
    current = bus.position(MOTOR_ID)
    delta1 = 980 if current - 980 >= SAFE_RAW_MIN else -980
    if not (SAFE_RAW_MIN <= current + delta1 <= SAFE_RAW_MAX):
        print(f"SKIPPING F: current={current} is too close to a limit for a safe 980-step move")
    else:
        pos = one_shot(bus, "F1 (large move, ACC=100)", acc=100, speed=3000, delta_steps=delta1)
        delta2 = -delta1
        if SAFE_RAW_MIN <= pos + delta2 <= SAFE_RAW_MAX:
            one_shot(bus, "F2 (large move, ACC=254)", acc=254, speed=3000, delta_steps=delta2)
        else:
            print("SKIPPING F2: would exceed safe window")

    bus.close()


if __name__ == "__main__":
    main()
