#!/usr/bin/env python3
"""Generate MoveIt's self-collision matrix for the arms, head and cart.

The Setup Assistant's own method, without its GUI: start move_group without a matrix, ask its
/check_state_validity service about random poses of the arms and the head, and disable the link
pairs that

  adjacent  are joined by a joint,
  default   collide in the zero, the rest or the folded pose (the folded pose is how the real arms
            are parked, MoveIt has to accept it as a start state),
  always    collide in at least 95 % of the poses,
  never     did not collide in any pose.

Pairs that collide in some poses stay enabled, those are the ones the planner has to avoid. Writes
src/xlerobot_moveit_config/config/disabled_collisions.xacro. Regenerate after the geometry, the
joint limits or the rest pose change. Needs the workspace built and sourced.

  python3 tools/generate_moveit_collisions.py [--samples 20000]
"""

import argparse
import itertools
import os
import pathlib
import random
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.srv import GetStateValidity

REPO = pathlib.Path(__file__).resolve().parent.parent
OUT = REPO / "src/xlerobot_moveit_config/config/disabled_collisions.xacro"
URDF_XACRO = REPO / "src/xlerobot_description/urdf/xlerobot.urdf.xacro"
DIMENSIONS = REPO / "src/xlerobot_description/config/robot_dimensions.yaml"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def load_urdf():
    text = subprocess.run(
        ["xacro", str(URDF_XACRO), "with_arms:=true", "with_head:=true", "use_ros2_control:=false"],
        check=True, capture_output=True, text=True).stdout
    return ET.fromstring(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--samples", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    import yaml
    arms = yaml.safe_load(DIMENSIONS.read_text())["xlerobot"]["arms"]
    rest, folded = arms["rest_pose"], arms["folded_pose"]

    robot = load_urdf()
    with_collision = sorted(l.get("name") for l in robot.findall("link") if l.find("collision") is not None)
    adjacent = set()
    limits = {}
    for j in robot.findall("joint"):
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        adjacent.add(frozenset((parent, child)))
        if j.get("type") == "revolute":
            lim = j.find("limit")
            limits[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    moved = [n for n in limits if n.startswith(("left_arm_", "right_arm_", "head_"))]
    all_joints = [j.get("name") for j in robot.findall("joint") if j.get("type") in ("revolute", "continuous")]
    print(f"{len(with_collision)} links with collision geometry, {len(moved)} joints sampled")

    move_group = subprocess.Popen(
        ["ros2", "launch", "xlerobot_moveit_config", "move_group.launch.py", "disabled_collisions:=false"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        rclpy.init()
        node = rclpy.create_node("generate_moveit_collisions")
        client = node.create_client(GetStateValidity, "/check_state_validity")
        if not client.wait_for_service(timeout_sec=60.0):
            sys.exit("move_group did not come up")
        time.sleep(3.0)  # the planning scene needs a moment after the service appears

        def check(positions):
            """positions: joint name -> value, the joints not given are 0. Returns the colliding link pairs."""
            req = GetStateValidity.Request()
            req.robot_state.joint_state.name = all_joints
            req.robot_state.joint_state.position = [float(positions.get(n, 0.0)) for n in all_joints]
            future = client.call_async(req)
            return future

        def pairs_of(future):
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            res = future.result()
            if res is None:
                alive = "still running" if move_group.poll() is None else f"exited with {move_group.returncode}"
                sys.exit(f"no answer from /check_state_validity, move_group is {alive}")
            return {frozenset((c.contact_body_1, c.contact_body_2)) for c in res.contacts}

        def pose_of(values):
            state = {}
            for side in ("left", "right"):
                for j in ARM_JOINTS:
                    state[f"{side}_arm_{j}_joint"] = values[j]
            return state

        default_pairs = {}
        for name, state in (("zero", {}), ("rest", pose_of(rest)), ("folded", pose_of(folded))):
            pairs = pairs_of(check(state))
            default_pairs[name] = pairs
            print(f"{name} pose: {len(pairs)} colliding pairs")

        rng = random.Random(args.seed)
        counts = {}
        started = time.time()
        batch = 1  # concurrent requests to move_group stall it
        done = 0
        while done < args.samples:
            futures = []
            for _ in range(min(batch, args.samples - done)):
                futures.append(check({n: rng.uniform(*limits[n]) for n in moved}))
            for f in futures:
                for p in pairs_of(f):
                    counts[p] = counts.get(p, 0) + 1
            done += len(futures)
            if done % 2000 < batch:
                print(f"  {done}/{args.samples} poses, {time.time() - started:.0f} s, {len(counts)} pairs seen colliding", flush=True)

        entries = {}
        for a, b in itertools.combinations(with_collision, 2):
            pair = frozenset((a, b))
            n = counts.get(pair, 0)
            if pair in adjacent:
                reason = "Adjacent"
            elif any(pair in colliding for colliding in default_pairs.values()):
                reason = "Default"
            elif n >= 0.95 * args.samples:
                reason = "Always"
            elif n == 0:
                reason = "Never"
            else:
                continue
            entries[tuple(sorted((a, b)))] = reason

        # collisions in the rest pose that are not just neighbouring links are worth a look
        for name in default_pairs:
            odd = sorted(tuple(sorted(p)) for p in default_pairs[name] if p not in adjacent)
            if odd:
                print(f"{name} pose, colliding pairs that are not adjacent (disabled as Default, check them):")
                for a, b in odd:
                    print(f"    {a} - {b}")

        lines = ['<?xml version="1.0"?>',
                 "<!-- Generated by tools/generate_moveit_collisions.py, do not edit. "
                 f"{args.samples} random poses, seed {args.seed}. -->",
                 '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']
        for (a, b), reason in sorted(entries.items()):
            lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>')
        lines += ["</robot>", ""]
        OUT.write_text("\n".join(lines))
        kept = [(tuple(sorted(p)), n) for p, n in counts.items() if tuple(sorted(p)) not in entries]
        by_reason = {}
        for r in entries.values():
            by_reason[r] = by_reason.get(r, 0) + 1
        print(f"wrote {OUT.relative_to(REPO)}: {len(entries)} pairs disabled {by_reason}, "
              f"{len(kept)} pairs stay enabled")
        for (a, b), n in sorted(kept, key=lambda x: -x[1])[:12]:
            print(f"    kept: {a} - {b}  ({100.0 * n / args.samples:.1f} % of poses)")
        node.destroy_node()
        rclpy.shutdown()
    finally:
        try:
            os.killpg(move_group.pid, 2)
        except ProcessLookupError:
            pass
        move_group.wait(timeout=20)


if __name__ == "__main__":
    main()
