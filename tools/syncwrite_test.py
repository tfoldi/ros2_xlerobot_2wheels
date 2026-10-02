#!/usr/bin/env python3
"""Replicate the real driver's exact SyncWritePosEx bundling: all 6 right-arm servos in ONE
packet every cycle, only ID 1 actually moving (matching servo_sync_deltas_==0 for the rest,
Speed=0 for them per the real Pass2 logic, ACC=100 for all per the fix). Isolates whether
bundling multiple servos in one SyncWrite (vs a single servo alone) reproduces the slowdown."""
import sys
import time

sys.path.insert(0, "/home/tfoldi/Developer/ros2_xlerobot_2wheels/tools")
from calibrate_arm import Bus  # noqa: E402

INST_SYNC_WRITE = 0x83
ADDR_ACC = 41
DATA_LEN = 7  # ACC(1) + Position(2) + Time(2) + Speed(2), matches WritePosEx layout

PORT = "/dev/xle-bus2"
IDS = [1, 2, 3, 4, 5, 6]
MOVER_ID = 1
DELTA_STEPS = 120


def encode_signed(value, sign_bit=15):
    if value < 0:
        return (1 << sign_bit) | (-value)
    return value


def sync_write_posex(bus, ids, positions, speeds, accs):
    n = len(ids)
    length = n * (DATA_LEN + 1) + 4
    body = bytes([0xFE, length, INST_SYNC_WRITE, ADDR_ACC, DATA_LEN])
    for i in range(n):
        enc = encode_signed(positions[i])
        params = bytes([
            accs[i],
            enc & 0xFF, (enc >> 8) & 0xFF,
            0, 0,  # time, unused
            speeds[i] & 0xFF, (speeds[i] >> 8) & 0xFF,
        ])
        body += bytes([ids[i]]) + params
    checksum = (~sum(body)) & 0xFF
    packet = b"\xff\xff" + body + bytes([checksum])
    bus.ser.write(packet)


def main():
    bus = Bus(PORT)
    current = {i: bus.position(i) for i in IDS}
    start = current[MOVER_ID]
    target = start + DELTA_STEPS
    print(f"mover id={MOVER_ID} start={start} target={target}, others held at current, "
          f"ACC=100 all, Speed=600 for mover / 0 for others, 100Hz")

    t0 = time.time()
    setpoint = start
    next_write = t0
    next_print = t0
    while True:
        now = time.time()
        if now >= next_write:
            setpoint = min(setpoint + 1, target)
            positions = [current[i] for i in IDS]
            positions[IDS.index(MOVER_ID)] = setpoint
            speeds = [0] * len(IDS)
            speeds[IDS.index(MOVER_ID)] = 600
            accs = [100] * len(IDS)
            sync_write_posex(bus, IDS, positions, speeds, accs)
            next_write += 0.01
        pos = bus.position(MOVER_ID)
        t = now - t0
        if now >= next_print:
            print(f"{t:5.2f} setpoint={setpoint:6d} actual={pos:6d} error={pos - target:+d}")
            next_print += 0.3
        if t > 6.0:
            break
    print(f"final error: {bus.position(MOVER_ID) - target:+d} steps after 6.0s")
    bus.close()


if __name__ == "__main__":
    main()
