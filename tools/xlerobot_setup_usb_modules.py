#!/usr/bin/env python3
# Device detection and udev rule generation ported from RoboCrew
# (github.com/Grigorij-Dudnik/RoboCrew, robocrew_setup_usb_modules.py and
# robocrew_generate_udev_rules.py), MIT licensed:
#
#   MIT License
#   Copyright (c) 2025 Grigorij Dudnik
#   Permission is hereby granted, free of charge, to any person obtaining a
#   copy of this software and associated documentation files (the
#   "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to
#   the following conditions:
#   The above copyright notice and this permission notice shall be included
#   in all copies or substantial portions of the Software.
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
#   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
#   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
#   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# The WiFi-priority feature in the original robocrew_setup_usb_modules.py
# is not carried over, it has nothing to do with USB device naming.
"""Interactively assigns stable /dev names to the XLeRobot's USB serial
buses, so /dev/ttyACM0 and /dev/ttyACM1 swapping on reboot stops mattering.
See README.md's bus layout section.

Usage:
    sudo python3 tools/xlerobot_setup_usb_modules.py

Prompts to disconnect all devices, then connect each of xle-bus1 and
xle-bus2 in turn. Writes /etc/udev/rules.d/99-xlerobot.rules and reloads
udev. Also scans /dev/v4l/by-path and /dev/snd/controlC* so the same tool
can assign a stable name to a camera or microphone later - type 'a' at the
end to add one beyond the two default bus aliases.
"""

import glob
import os
import subprocess
import sys
import time
from pathlib import Path

MODE = os.environ.get("MODE", "0660")
GROUP = os.environ.get("GROUP", "dialout")
RULES_PATH = "/etc/udev/rules.d/99-xlerobot.rules"

DEFAULT_ALIASES = ["xle-bus1", "xle-bus2"]


def udevadm(args):
    try:
        out = subprocess.run(
            ["udevadm", *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except FileNotFoundError as exc:
        raise SystemExit("udevadm binary is required to run this script") from exc
    return out.stdout.strip()


def get_props(node):
    props = {}
    for line in udevadm(["info", "-q", "property", "-n", node]).splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            props[key] = value
    return props


def get_camera_index(kernel):
    idx_file = Path("/sys/class/video4linux") / kernel / "index"
    if idx_file.is_file():
        return idx_file.read_text(encoding="ascii", errors="ignore").strip()
    return None


def scan(pattern, subsystem, devices, serial_counts, camera_ids):
    for device in sorted(glob.glob(pattern)):
        real_device = os.path.realpath(device)
        sysfs_path = udevadm(["info", "-q", "path", "-n", real_device])
        if not sysfs_path:
            continue

        props = get_props(real_device)
        vendor_id = props.get("ID_VENDOR_ID", "").lower()
        if not vendor_id:
            continue
        product_id = props.get("ID_MODEL_ID", "").lower()
        cam_key = f"{vendor_id}:{product_id}"

        if subsystem == "video4linux":
            camera_ids.add(cam_key)
            idx = get_camera_index(os.path.basename(real_device))
            if idx and idx != "0":
                continue
        elif cam_key in camera_ids:
            continue

        phys_dir = os.path.dirname(os.path.dirname(sysfs_path))
        phys_path = os.path.basename(phys_dir).split(":", 1)[0]
        id_path = props.get("ID_PATH") or phys_path
        serial_short = props.get("ID_SERIAL_SHORT")
        serial = serial_short or props.get("ID_SERIAL") or None

        devices.append(
            {
                "kernel": os.path.basename(real_device),
                "serial": serial,
                "serial_is_short": bool(serial_short),
                "vendor": vendor_id,
                "product": product_id,
                "id_path": id_path,
                "phys": phys_path,
                "subsystem": subsystem,
            }
        )

        if serial:
            serial_counts[serial] = serial_counts.get(serial, 0) + 1


def capture_devices():
    devices = []
    serial_counts = {}
    camera_ids = set()
    scan("/dev/v4l/by-path/*", "video4linux", devices, serial_counts, camera_ids)
    scan("/dev/serial/by-path/*", "tty", devices, serial_counts, camera_ids)
    scan("/dev/snd/controlC*", "sound", devices, serial_counts, camera_ids)
    return devices


def device_key(dev):
    return (dev["subsystem"], dev["kernel"], dev["phys"], dev.get("serial"))


def wait_for_device(known):
    print("Waiting for newly connected device...")
    while True:
        for dev in capture_devices():
            key = device_key(dev)
            if key not in known:
                return dev, key
        time.sleep(1)


def build_rule(dev, alias):
    rule = f'SUBSYSTEM=="{dev["subsystem"]}"'
    if dev["subsystem"] == "video4linux":
        rule += ', ATTR{index}=="0"'
    id_path = dev.get("id_path") or dev["phys"]
    rule += f', ENV{{ID_PATH}}=="{id_path}"'
    serial = dev.get("serial")
    if serial and serial != "00000000":
        if dev.get("serial_is_short"):
            rule += f', ATTRS{{serial}}=="{serial}"'
        else:
            rule += f', ENV{{ID_SERIAL}}=="{serial}"'
    rule += f', MODE="{MODE}", GROUP="{GROUP}", SYMLINK+="{alias}"'
    return rule


def ensure_root():
    if os.geteuid() == 0:
        return
    print("Writing udev rules needs root, requesting sudo...")
    cmd = ["sudo", "-E", sys.executable, *sys.argv]
    os.execvp("sudo", cmd)


def main():
    ensure_root()
    input("Disconnect both XLeRobot USB buses, then press Enter to continue.")

    known = {device_key(dev) for dev in capture_devices()}
    assignments = []

    for alias in DEFAULT_ALIASES:
        resp = input(f"Connect the USB cable for '{alias}' and press Enter (or 's' to skip): ").strip().lower()
        if resp == "s":
            continue
        dev, key = wait_for_device(known)
        assignments.append({"alias": alias, "device": dev})
        known.add(key)

    while True:
        resp = input("Both buses assigned. Type 'a' to add another device (camera, mic), or press Enter to finish: ").strip().lower()
        if resp != "a":
            break
        alias = input("Enter alias for the new device: ").strip()
        if not alias:
            print("Alias cannot be empty. Skipping.")
            continue
        dev, key = wait_for_device(known)
        assignments.append({"alias": alias, "device": dev})
        known.add(key)

    if not assignments:
        print("No devices registered, nothing to write.")
        return

    rules = [build_rule(entry["device"], entry["alias"]) for entry in assignments]
    with open(RULES_PATH, "w", encoding="ascii") as fh:
        fh.write("\n\n".join(rules) + "\n")
    print(f"Saved {len(rules)} rule(s) to {RULES_PATH}.")
    subprocess.run(["udevadm", "control", "--reload-rules"], check=False)
    subprocess.run(["udevadm", "trigger"], check=False)
    print("Done. /dev/" + " and /dev/".join(a["alias"] for a in assignments) + " should now be stable across reboots.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
