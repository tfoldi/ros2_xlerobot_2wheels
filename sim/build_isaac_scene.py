#!/usr/bin/env python3
"""Builds the XLeRobot Isaac Sim twin: imports the URDF, wires a ROS 2
bridge Action Graph matching xlerobot.ros2_control_sim.xacro's topic names,
saves the stage. See docs/sim-twin.md for the full story.

Run with the Isaac Sim venv, not the system/ROS Python:
    source ~/venvs/isaacsim/bin/activate
    python sim/build_isaac_scene.py

Base + head only for this first twin, no arms. Arms are still stub box
geometry (phase 3 hasn't started), nothing to gain simulating them yet.
xlerobot.ros2_control_sim.xacro already supports with_arms:=true whenever
that changes, this script would just need the same joint list extended.

Node names and the enable_extension/app_utils pattern below are taken
directly from Isaac Sim 6's own shipped reference
(isaacsim/skills/isaac-sim-ros2-bridge/scripts/multi_robot_namespacing.py),
not guessed - that script targets Nav2 fleet demos and publishes ground
truth odom/TF from the chassis pose directly, which we deliberately do NOT
want as our primary odom source here, diff_drive_controller computing
odometry FROM joint states is the actual thing under test. Only took the
joint state pub/sub + articulation controller pattern from it, not the
odom/TF part.

Confirmed working end to end by actually running it, not just rendering
clean: URDF imports, ROS2 bridge extension loads, graph builds with no
OmniGraphError, stage saves. See docs/sim-twin.md for what that run found
along the way (env var ordering for the bridge extension, node names that
turned out different from search results / older doc snapshots).
"""

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(REPO_ROOT, "sim", "xlerobot_import.urdf")
USD_OUTPUT_DIR = os.path.join(REPO_ROOT, "sim", "output")
STAGE_PATH = os.path.join(USD_OUTPUT_DIR, "xlerobot_twin.usd")

ROBOT_PRIM_PATH = "/World/xlerobot"
# PhysicsArticulationRootAPI actually lands here, not on ROBOT_PRIM_PATH -
# confirmed by reading the importer's own generated payloads/Physics/physics.usda
# after the first two attempts guessed wrong (the bare wrapper prim, then a
# "Physics" prim that turned out to hold joint prims, not the articulation
# root). This matches our own base_footprint -> base_link naming exactly.
ARTICULATION_ROOT_PATH = ROBOT_PRIM_PATH + "/Geometry/base_footprint/base_link"

# Matches xlerobot.ros2_control_sim.xacro's joint_state_topic_hardware_interface
# params exactly. If those change, change these too.
JOINT_COMMANDS_TOPIC = "/isaac_joint_commands"
JOINT_STATES_TOPIC = "/isaac_joint_states"

# Wheels are velocity-driven (diff_drive_controller), head is position-driven
# (matches operating_mode in xlerobot.ros2_control.xacro on the real robot).
VELOCITY_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
POSITION_JOINTS = ["head_pan_joint", "head_tilt_joint"]


def import_urdf():
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    os.makedirs(USD_OUTPUT_DIR, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=URDF_PATH,
        usd_path=USD_OUTPUT_DIR,
        merge_fixed_joints=False,
        collision_from_visuals=True,
        fix_base=False,  # mobile base, must NOT be fixed to the world
        joint_drive_type="force",
        # Matches each joint's real operating mode. Everything not matched
        # by these two patterns (nothing else exists in the base+head-only
        # import) keeps whatever the importer defaults to.
        joint_target_type={
            "left_wheel_joint": "velocity",
            "right_wheel_joint": "velocity",
            "head_pan_joint": "position",
            "head_tilt_joint": "position",
        },
        run_asset_transformer=True,
    )
    importer = URDFImporter()
    result_path = importer.import_urdf(config)
    print(f"URDF imported: {result_path}")
    return result_path


def add_robot_to_stage(imported_usd_path):
    import omni.usd
    from pxr import Sdf, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    xform = stage.DefinePrim("/World", "Xform")
    robot_prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    robot_prim.GetReferences().AddReference(imported_usd_path)

    # The importer's own USD uses a "Physics" variant set (physics/physx/
    # mujoco/none) for the physics representation. Referencing the file
    # doesn't select a variant on its own - found this out the hard way,
    # first run had no active variant at all, so no ArticulationRootAPI
    # anywhere, and PhysX's tensor plugin correctly reported no articulation
    # at this prim. "physics" carries both PhysicsArticulationRootAPI and
    # the Newton-specific schemas together, that's the one PhysX's own
    # tensor interface (what IsaacArticulationController runs on) needs.
    variant_sets = robot_prim.GetVariantSets()
    if variant_sets.HasVariantSet("Physics"):
        variant_sets.GetVariantSet("Physics").SetVariantSelection("physics")
    else:
        print(f"WARNING: no 'Physics' variant set found on {ROBOT_PRIM_PATH}, "
              "articulation will likely not work, check the importer output")

    # basic ground plane + light, otherwise nothing to stand on and nothing
    # to see - Kit 105+ has no default stage light, an empty scene renders
    # black.
    from isaacsim.core.api.objects import GroundPlane

    GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)

    light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1000.0)

    print(f"Robot referenced at {ROBOT_PRIM_PATH}, ground plane and light added")

    # Verify the fix actually worked instead of trusting it silently -
    # exactly the kind of thing that already burned two runs tonight.
    # HasAPI is the real check; the schema wrapper constructor above
    # doesn't tell you whether the API is actually applied, importing it
    # was only needed to name the schema class to check for.
    target_prim = stage.GetPrimAtPath(ARTICULATION_ROOT_PATH)
    if target_prim.IsValid() and target_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"Confirmed: ArticulationRootAPI present at {ARTICULATION_ROOT_PATH}")
    else:
        print(f"WARNING: still no ArticulationRootAPI at {ARTICULATION_ROOT_PATH}, "
              "the ArticulationController node will not find anything to drive")


def build_ros2_bridge_graph():
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph_path = "/ROS2_Bridge_xlerobot"

    # Ground-truth odom (isaacsim.ros2.bridge.ROS2PublishOdometry) was here as
    # a comparison-only bonus, dropped for now: its input attribute isn't
    # named inputs:chassisPrim on this Isaac Sim 6.1 build, confirmed by
    # actually running it (OmniGraphError, attribute doesn't exist), and
    # every core node below already builds and sets clean. Not worth another
    # ~100s relaunch cycle to guess the right name for a bonus feature,
    # check the node's real schema in-process before re-adding.
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]

    values = [
        ("PublishClock.inputs:topicName", "/clock"),
        ("SubscribeJointState.inputs:topicName", JOINT_COMMANDS_TOPIC),
        # Both of these need the real articulation root, not the wrapper
        # prim - see ARTICULATION_ROOT_PATH's own comment.
        ("ArticulationController.inputs:targetPrim", ARTICULATION_ROOT_PATH),
        ("PublishJointState.inputs:topicName", JOINT_STATES_TOPIC),
        ("PublishJointState.inputs:targetPrim", ARTICULATION_ROOT_PATH),
    ]

    connections = [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
    ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connections},
    )
    print(f"ROS 2 bridge graph created at {graph_path}")
    print(f"  subscribes {JOINT_COMMANDS_TOPIC} -> drives the articulation")
    print(f"  publishes  {JOINT_STATES_TOPIC} <- articulation joint state")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--test-frames", type=int, default=0,
                         help="If > 0, press Play and step this many frames before saving, "
                              "to verify the graph actually evaluates without erroring.")
    args = parser.parse_args()

    if not os.path.isfile(URDF_PATH):
        print(f"ERROR: {URDF_PATH} not found. Render it first, see the module docstring.")
        sys.exit(1)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import isaacsim.core.experimental.utils.app as app_utils
    import omni.usd

    omni.usd.get_context().new_stage()
    simulation_app.update()

    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    imported_usd_path = import_urdf()
    simulation_app.update()

    add_robot_to_stage(imported_usd_path)
    simulation_app.update()

    build_ros2_bridge_graph()
    simulation_app.update()

    if args.test_frames > 0:
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
        simulation_app.update()
        app_utils.play()
        print(f"Play pressed, stepping {args.test_frames} frames to verify the graph evaluates...")
        for i in range(args.test_frames):
            simulation_app.update()
        app_utils.stop()
        print("Stepped without a hard crash. Does not mean the topics look right on the ROS side,"
              " check that separately once the graph is confirmed built.")

    os.makedirs(USD_OUTPUT_DIR, exist_ok=True)
    omni.usd.get_context().save_as_stage(STAGE_PATH)
    print(f"Stage saved: {STAGE_PATH}")

    simulation_app.close()


if __name__ == "__main__":
    main()
