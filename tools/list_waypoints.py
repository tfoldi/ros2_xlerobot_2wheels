#!/usr/bin/env python3
"""List saved named waypoints (see tools/save_waypoint.py)."""
import math

import waypoint_store


def main():
    waypoints = waypoint_store.load()
    if not waypoints:
        print("no saved waypoints yet")
        return
    for name in sorted(waypoints):
        w = waypoints[name]
        print(f"{name}: x={w['x']:.3f} y={w['y']:.3f} yaw={math.degrees(w['yaw']):.1f} deg")


if __name__ == "__main__":
    main()
