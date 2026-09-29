#!/usr/bin/env python3
"""List saved named positions (see tools/save_position.py)."""
import position_store


def main():
    positions = position_store.load()
    if not positions:
        print("no saved positions yet")
        return
    for name in sorted(positions):
        entry = positions[name]
        joints = ", ".join(f"{j}={v:.4f}" for j, v in entry["joints"].items())
        print(f"{name} ({entry['group']}): {joints}")


if __name__ == "__main__":
    main()
