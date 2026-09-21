#!/usr/bin/env python3
"""Opens a USD stage, or a URDF file (converted first), in the interactive
Isaac Sim GUI, for manual inspection and driving. Not headless: requires a
display.

    source ~/venvs/isaacsim/bin/activate
    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
    python sim/launch_gui.py [path/to/stage.usd | path/to/robot.urdf]

Defaults to sim/output/xlerobot_twin.usd if no path is given. A .urdf path
is converted to USD first (into sim/output/, alongside build_isaac_scene.py's
own output) rather than opened directly - USD cannot open URDF files, only
the importer can read them. Stays open until the window is closed.
"""

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_STAGE_PATH = os.path.join(REPO_ROOT, "sim", "output", "xlerobot_twin.usd")
USD_OUTPUT_DIR = os.path.join(REPO_ROOT, "sim", "output")


def convert_urdf(urdf_path):
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    os.makedirs(USD_OUTPUT_DIR, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=urdf_path,
        usd_path=USD_OUTPUT_DIR,
        merge_fixed_joints=False,
        collision_from_visuals=True,
        fix_base=False,
    )
    usd_path = URDFImporter().import_urdf(config)
    print(f"Converted {urdf_path} -> {usd_path}")
    return usd_path


def main():
    given_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STAGE_PATH
    given_path = os.path.abspath(given_path)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    import isaacsim.core.experimental.utils.app as app_utils
    import omni.usd

    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    if not os.path.isfile(given_path):
        print(f"{given_path} not found. Opening an empty stage instead.")
    elif given_path.lower().endswith(".urdf"):
        stage_path = convert_urdf(given_path)
        omni.usd.get_context().open_stage(stage_path)
    else:
        print(f"Opening {given_path}")
        omni.usd.get_context().open_stage(given_path)

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
