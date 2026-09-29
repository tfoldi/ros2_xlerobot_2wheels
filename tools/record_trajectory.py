#!/usr/bin/env python3
"""Record a hand-guided (kinesthetic teaching) trajectory: cuts torque on the relevant bus(es),
streams /joint_states while you move the arm by hand, and saves it on Ctrl-C.

  python3 tools/record_trajectory.py <name> --group right_arm [--rate 10]

Torque comes back on (holding wherever the arm ends up) as soon as recording stops, same as
tools/estop.py --release. Cutting torque means the arm goes limp and WILL fall under its own
weight and whatever you're holding - support it exactly like any other torque_off:=false bringup,
just for the opposite reason (a sudden drop instead of a sudden move). Replay with
tools/replay_trajectory.py - that sends the raw recording back with no collision checking, since
it's just resending exactly what you did.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import yaml

import position_store


class Recorder(Node):
    def __init__(self, buses):
        super().__init__("record_trajectory")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.estop_clients = {b: self.create_client(SetBool, f"/emergency_stop_{b}") for b in buses}

    def on_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def set_estop(self, active):
        futures = {}
        for bus, client in self.estop_clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                sys.exit(f"/emergency_stop_{bus} not available - is bringup running?")
            futures[bus] = client.call_async(SetBool.Request(data=active))
        for bus, future in futures.items():
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result()
            ok = bool(result and result.success)
            print(f"{bus}: {'torque off' if active else 'torque on'} - {result.message if result else 'no answer'}")
            if not ok:
                sys.exit(f"failed to {'stop' if active else 'release'} {bus}, aborting")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("--group", required=True, choices=sorted(position_store.GROUP_JOINTS))
    parser.add_argument("--rate", type=float, default=10.0, help="Hz")
    args = parser.parse_args()
    joints = position_store.GROUP_JOINTS[args.group]
    buses = position_store.GROUP_BUSES[args.group]

    # rclpy's own default SIGINT handler races with our except KeyboardInterrupt below - if it
    # invalidates the context first, the torque-release call after Ctrl-C fails outright ("rcl
    # node's context is invalid"), which for this script means torque can stay OFF with no
    # recovery. Disabling it so only our own handling controls shutdown ordering.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Recorder(buses)
    deadline = time.time() + 10
    while not all(j in node.positions for j in joints) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    missing = [j for j in joints if j not in node.positions]
    if missing:
        sys.exit(f"no /joint_states for: {', '.join(missing)} - is bringup running?")

    print(f"cutting torque on {', '.join(buses)} - support the arm now")
    node.set_estop(True)

    print("recording... move the arm by hand, Ctrl-C to stop")
    samples = []
    t0 = time.time()
    period = 1.0 / args.rate
    next_sample = t0
    try:
        while True:
            now = time.time()
            rclpy.spin_once(node, timeout_sec=max(0.0, next_sample - now))
            now = time.time()
            if now >= next_sample:
                samples.append({
                    "time_from_start": round(now - t0, 3),
                    "positions": [round(float(node.positions[j]), 5) for j in joints],
                })
                next_sample += period
    except KeyboardInterrupt:
        pass

    print(f"\n{len(samples)} samples over {samples[-1]['time_from_start']:.1f}s" if samples else "\nno samples recorded")
    print("releasing torque, holding current position")
    node.set_estop(False)

    if samples:
        position_store.TRAJECTORY_DIR.mkdir(parents=True, exist_ok=True)
        with open(position_store.trajectory_path(args.name), "w") as f:
            yaml.safe_dump({"group": args.group, "joints": joints, "points": samples}, f, sort_keys=False)
        print(f"saved to {position_store.trajectory_path(args.name)}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
