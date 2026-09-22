#!/usr/bin/env python3
"""Builds the XLeRobot Isaac Sim twin: imports the URDF, wires a ROS 2
bridge Action Graph matching xlerobot.ros2_control_sim.xacro's topic names,
saves the stage. See docs/sim-twin.md.

Run with the Isaac Sim venv, not the system/ROS Python:
    source ~/venvs/isaacsim/bin/activate
    python sim/build_isaac_scene.py

Base + head only, no arms: arms are still stub box geometry (phase 3
hasn't started). xlerobot.ros2_control_sim.xacro supports with_arms:=true;
extending this script's joint list is the only change needed.

Node names and the enable_extension/app_utils pattern follow Isaac Sim 6's
own shipped reference (isaacsim/skills/isaac-sim-ros2-bridge/scripts/multi_robot_namespacing.py).
That script targets Nav2 fleet demos and publishes ground-truth odom/TF
from the chassis pose directly; only the joint-state pub/sub and
articulation-controller pattern is reused here, not the odom/TF part,
since diff_drive_controller computing odometry from joint states is the
thing under test.
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
# params exactly. If those change, change these too. Commands are split by
# interface type (see that xacro's comment for why); states stay on one
# shared topic since the read side tolerates joint names it doesn't own.
WHEEL_COMMANDS_TOPIC = "/isaac_wheel_commands"
POSITION_COMMANDS_TOPIC = "/isaac_position_commands"
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
        # xlerobot.urdf.xacro emits package://xlerobot_description/... mesh
        # URIs (needed for RViz/robot_state_publisher on the real bringup).
        # The importer's own package:// auto-discovery climbs up from the
        # URDF's directory looking for a bare "meshes/" folder, which never
        # matches our src/xlerobot_description layout - this mapping is the
        # documented, explicit way around that instead of relying on it.
        ros_package_paths=[{
            "name": "xlerobot_description",
            "path": os.path.join(REPO_ROOT, "src", "xlerobot_description"),
        }],
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
    # mujoco/none) for the physics representation, not selected by default
    # when the file is referenced. "physics" carries both
    # PhysicsArticulationRootAPI and the Newton-specific schemas together,
    # which is what PhysX's tensor interface (IsaacArticulationController)
    # needs.
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

    # HasAPI is the real check; the schema wrapper constructor above
    # doesn't tell you whether the API is actually applied, importing it
    # is only needed to name the schema class to check for. Asserting
    # this here instead of trusting the reference silently matters: the
    # wrapper prim path looks like a plausible target and is not.
    target_prim = stage.GetPrimAtPath(ARTICULATION_ROOT_PATH)
    if target_prim.IsValid() and target_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"Confirmed: ArticulationRootAPI present at {ARTICULATION_ROOT_PATH}")
    else:
        print(f"WARNING: still no ArticulationRootAPI at {ARTICULATION_ROOT_PATH}, "
              "the ArticulationController node will not find anything to drive")


# stiffness, damping, max_force. The URDF importer creates each joint's
# PhysX drive with zero gain whenever the source URDF has no <dynamics
# damping=".../> - it says so directly ("actuator will be created without
# gain parameters"). A drive with zero gain applies zero corrective torque,
# so a commanded position/velocity has no physical effect no matter what
# ros2_control sends - confirmed by watching /isaac_wheel_commands carry
# the right numbers while the wheels barely moved and drifted unevenly.
# Velocity-driven wheels only need damping (the term that converts a
# velocity error into torque); position-driven joints need both. These
# values are picked to make closed-loop control visibly work, not tuned
# against the real servo's torque curve - revisit once that matters.
JOINT_DRIVE_GAINS = {
    "left_wheel_joint": (0.0, 1e5, 1e4),
    "right_wheel_joint": (0.0, 1e5, 1e4),
    "head_pan_joint": (1e4, 1e3, 1e3),
    "head_tilt_joint": (1e4, 1e3, 1e3),
}


def configure_joint_drives():
    import omni.usd
    from pxr import UsdPhysics

    stage = omni.usd.get_context().get_stage()
    for joint_name, (stiffness, damping, max_force) in JOINT_DRIVE_GAINS.items():
        joint_path = f"{ROBOT_PRIM_PATH}/Physics/{joint_name}"
        prim = stage.GetPrimAtPath(joint_path)
        if not prim.IsValid():
            print(f"WARNING: joint prim not found at {joint_path}, drive gain not set")
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr(stiffness)
        drive.CreateDampingAttr(damping)
        drive.CreateMaxForceAttr(max_force)
    print("Joint drive gains set (replacing the importer's zero-gain defaults)")


def capture_screenshot(path):
    """One-off diagnostic capture, not part of the normal build - lets a
    mesh/offset change (like base_link's raskogbody.stl placement) be
    checked by actually looking at it instead of trusting the arithmetic
    that produced it."""
    import omni.kit.viewport.utility as vp_util
    from isaacsim.core.rendering_manager import ViewportManager

    ViewportManager.set_camera_view(
        camera="/OmniverseKit_Persp",
        eye=[0.9, -0.9, 0.6],
        target=[0.0, 0.0, 0.3],
    )
    viewport = vp_util.get_active_viewport()
    return vp_util.capture_viewport_to_file(viewport, path)


def build_ros2_bridge_graph():
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph_path = "/ROS2_Bridge_xlerobot"

    # Ground-truth odom (isaacsim.ros2.bridge.ROS2PublishOdometry) is
    # deliberately not wired here: it would publish from the chassis pose
    # directly, duplicating what diff_drive_controller is meant to produce
    # from joint states. Its input attribute names also differ from the
    # inputs:chassisPrim used by older reference material - check the
    # node's schema in-process before adding it back.
    #
    # Two separate Subscribe/ArticulationController pairs, one per command
    # interface type, not one shared pair: joint_state_topic_hardware_interface's
    # write() only fills position/velocity/effort for the command interfaces
    # a joint actually declares, so a single combined topic covering both
    # velocity-commanded wheels and position-commanded head/arm joints comes
    # out with name/position/velocity arrays of different lengths.
    # ROS2SubscribeJointState correctly rejects that ("size of position,
    # velocity and effort arrays match the number of actuators") instead of
    # guessing which value belongs to which joint. Splitting by interface
    # type keeps each topic's arrays internally consistent. Both
    # ArticulationController instances target the same articulation and
    # drive disjoint joint sets, so they don't fight each other.
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("SubscribeWheelCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationControllerWheels", "isaacsim.core.nodes.IsaacArticulationController"),
        ("SubscribePositionCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationControllerPosition", "isaacsim.core.nodes.IsaacArticulationController"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]

    values = [
        ("PublishClock.inputs:topicName", "/clock"),
        ("SubscribeWheelCommand.inputs:topicName", WHEEL_COMMANDS_TOPIC),
        ("SubscribePositionCommand.inputs:topicName", POSITION_COMMANDS_TOPIC),
        # All three need the real articulation root, not the wrapper prim -
        # see ARTICULATION_ROOT_PATH's own comment.
        ("ArticulationControllerWheels.inputs:targetPrim", ARTICULATION_ROOT_PATH),
        ("ArticulationControllerPosition.inputs:targetPrim", ARTICULATION_ROOT_PATH),
        ("PublishJointState.inputs:topicName", JOINT_STATES_TOPIC),
        ("PublishJointState.inputs:targetPrim", ARTICULATION_ROOT_PATH),
    ]

    connections = [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("OnPlaybackTick.outputs:tick", "SubscribeWheelCommand.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationControllerWheels.inputs:execIn"),
        ("SubscribeWheelCommand.outputs:jointNames", "ArticulationControllerWheels.inputs:jointNames"),
        ("SubscribeWheelCommand.outputs:positionCommand", "ArticulationControllerWheels.inputs:positionCommand"),
        ("SubscribeWheelCommand.outputs:velocityCommand", "ArticulationControllerWheels.inputs:velocityCommand"),
        ("OnPlaybackTick.outputs:tick", "SubscribePositionCommand.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationControllerPosition.inputs:execIn"),
        ("SubscribePositionCommand.outputs:jointNames", "ArticulationControllerPosition.inputs:jointNames"),
        ("SubscribePositionCommand.outputs:positionCommand", "ArticulationControllerPosition.inputs:positionCommand"),
        ("SubscribePositionCommand.outputs:velocityCommand", "ArticulationControllerPosition.inputs:velocityCommand"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        # Without this, timeStamp defaults to 0.0 and every published message
        # trips Isaac's "Frame Timestamp is invalid" warning on every tick -
        # noisy but not otherwise harmful, since the payload itself still
        # goes out with real data. PublishClock needs the same wiring, done
        # above; this one was missing it.
        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
    ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connections},
    )
    print(f"ROS 2 bridge graph created at {graph_path}")
    print(f"  subscribes {WHEEL_COMMANDS_TOPIC} + {POSITION_COMMANDS_TOPIC} -> drives the articulation")
    print(f"  publishes  {JOINT_STATES_TOPIC} <- articulation joint state")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--test-frames", type=int, default=0,
                         help="If > 0, press Play and step this many frames before saving, "
                              "to verify the graph actually evaluates without erroring.")
    parser.add_argument("--screenshot", type=str, default=None,
                         help="Path to save a viewport screenshot to after building, "
                              "for visually checking mesh placement.")
    args = parser.parse_args()

    if not os.path.isfile(URDF_PATH):
        print(f"ERROR: {URDF_PATH} not found. Render it first, see the module docstring.")
        sys.exit(1)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    try:
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

        configure_joint_drives()
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

        if args.screenshot:
            capture_screenshot(args.screenshot)
            for _ in range(30):
                simulation_app.update()
            print(f"Screenshot saved: {args.screenshot}")

        os.makedirs(USD_OUTPUT_DIR, exist_ok=True)
        omni.usd.get_context().save_as_stage(STAGE_PATH)
        print(f"Stage saved: {STAGE_PATH}")
    finally:
        # Always runs, including on Ctrl+C mid-import: an interrupted run
        # otherwise skips this and Kit's GPU/CUDA context never tears down
        # cleanly, same issue launch_gui.py had.
        simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
