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

Closing the window does not reliably stop the underlying process for a
plain SimulationApp script like this one (a Kit/Omniverse limitation, the
window-close handler isn't wired to process exit outside the full editor
app). Use Ctrl+C in the terminal, or `kill <pid>`, not `kill -9`: SIGKILL
skips Python cleanup entirely and can leave GPU memory held by the dead
process until reboot.

A SIGTERM handler that raises KeyboardInterrupt does not work here: Kit's
extension loader wraps each extension's import in a broad except clause
(to keep one broken extension from taking down the app) and that swallows
the interrupt if it lands mid-load, which is likely since extension setup
takes a few seconds. Instead the handler only sets a flag, and the main
loop below checks it once per frame - by that point control is back in
our own code, past any of Kit's internal except blocks, so shutdown always
reaches the try/finally and simulation_app.close() actually runs.
"""

import os
import signal
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_STAGE_PATH = os.path.join(REPO_ROOT, "sim", "output", "xlerobot_twin.usd")
USD_OUTPUT_DIR = os.path.join(REPO_ROOT, "sim", "output")

# Not part of this repo - github.com/serionist/omniverse-mcp, cloned
# separately. Registered here (add_path + enable) rather than through
# Window > Extensions in the UI: that dialog has had focus-stealing issues
# on this machine where typing in the search/path field goes to the main
# window instead, making the manual one-time setup impossible to complete.
# Doing it in code sidesteps the UI entirely and needs no interaction.
OMNIVERSE_MCP_EXTENSION_PATH = os.path.expanduser("~/Developer/omniverse-mcp/extension")
OMNIVERSE_MCP_EXTENSION_NAME = "isaacsim.mcp.bridge"


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


_shutdown_requested = False


def _request_shutdown(signum, frame):
    global _shutdown_requested
    _shutdown_requested = True


def main():
    given_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STAGE_PATH
    given_path = os.path.abspath(given_path)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    # Must be installed after SimulationApp() is constructed: Kit registers
    # its own SIGTERM handler as part of startup and a handler set
    # beforehand gets overwritten by it.
    signal.signal(signal.SIGTERM, _request_shutdown)

    try:
        import isaacsim.core.experimental.utils.app as app_utils
        import omni.ext
        import omni.kit.app
        import omni.usd

        app_utils.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

        if os.path.isdir(OMNIVERSE_MCP_EXTENSION_PATH):
            omni.kit.app.get_app().get_extension_manager().add_path(OMNIVERSE_MCP_EXTENSION_PATH)
            simulation_app.update()
            enabled = app_utils.enable_extension(OMNIVERSE_MCP_EXTENSION_NAME)
            simulation_app.update()
            print(f"Omniverse MCP Bridge: {'enabled' if enabled else 'FAILED to enable'}")
        else:
            print(f"Omniverse MCP Bridge not found at {OMNIVERSE_MCP_EXTENSION_PATH}, skipping")

        if not os.path.isfile(given_path):
            print(f"{given_path} not found. Opening an empty stage instead.")
        elif given_path.lower().endswith(".urdf"):
            stage_path = convert_urdf(given_path)
            omni.usd.get_context().open_stage(stage_path)
        else:
            print(f"Opening {given_path}")
            omni.usd.get_context().open_stage(given_path)

        while simulation_app.is_running() and not _shutdown_requested:
            simulation_app.update()
    finally:
        # Always runs: normal exit (is_running() went False), Ctrl+C
        # (KeyboardInterrupt, Python's default SIGINT behavior), or SIGTERM
        # (the flag above, checked once we're back in our own loop). This is
        # the actual fix for Isaac Sim "not quitting properly" - without it,
        # any interrupt skips this call and Kit's GPU/CUDA context never
        # tears down cleanly.
        simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
