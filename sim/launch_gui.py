#!/usr/bin/env python3
"""Opens a USD stage in the interactive Isaac Sim GUI, for manual inspection
and driving. Not headless: requires a display.

    source ~/venvs/isaacsim/bin/activate
    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
    python sim/launch_gui.py [path/to/stage.usd]

Defaults to sim/output/xlerobot_twin.usd if no path is given. Stays open
until the window is closed.
"""

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_STAGE_PATH = os.path.join(REPO_ROOT, "sim", "output", "xlerobot_twin.usd")


def main():
    stage_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STAGE_PATH
    stage_path = os.path.abspath(stage_path)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    import isaacsim.core.experimental.utils.app as app_utils
    import omni.usd

    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    if os.path.isfile(stage_path):
        print(f"Opening {stage_path}")
        omni.usd.get_context().open_stage(stage_path)
    else:
        print(f"{stage_path} not found. Opening an empty stage instead.")

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
