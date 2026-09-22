#!/usr/bin/env python3
"""Runs the Isaac Sim twin headless with the timeline actually playing, so
its ROS 2 bridge stays live for a bringup running against it. Unlike
launch_gui.py, which opens a stage and waits for a human to press Play,
this presses Play itself and keeps stepping until told to stop - it's the
form meant for driving from ROS 2 without a display, e.g. on the Jetson
or in CI.

    source ~/venvs/isaacsim/bin/activate
    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
    python sim/run_headless.py [path/to/stage.usd]

Defaults to sim/output/xlerobot_twin.usd. Stop with Ctrl+C or plain `kill
<pid>` (not -9) - see launch_gui.py's docstring for why a flag checked in
the main loop is used instead of a signal handler that raises.
"""

import os
import signal
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_STAGE_PATH = os.path.join(REPO_ROOT, "sim", "output", "xlerobot_twin.usd")

_shutdown_requested = False


def _request_shutdown(signum, frame):
    global _shutdown_requested
    _shutdown_requested = True


def main():
    given_path = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STAGE_PATH)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True})

    signal.signal(signal.SIGTERM, _request_shutdown)

    try:
        import isaacsim.core.experimental.utils.app as app_utils
        import omni.usd
        from isaacsim.core.simulation_manager import SimulationManager

        app_utils.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

        if not os.path.isfile(given_path):
            print(f"ERROR: {given_path} not found, build it first with build_isaac_scene.py")
            sys.exit(1)

        print(f"Opening {given_path}")
        omni.usd.get_context().open_stage(given_path)
        simulation_app.update()

        # Same physics dt as build_isaac_scene.py's own --test-frames step,
        # so a headless run behaves the same as that verification path.
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
        simulation_app.update()

        app_utils.play()
        print("Playing. ROS 2 topics are live. Ctrl+C or `kill <pid>` to stop.")

        while simulation_app.is_running() and not _shutdown_requested:
            simulation_app.update()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
