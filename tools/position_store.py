"""Shared load/save helpers for named saved positions, used by save_position.py,
goto_position.py, list_positions.py and playback_sequence.py.

Storage: src/xlerobot_bringup/config/saved_positions.yaml, tracked in git like any other
bringup config (sync it to the Jetson the same way as everything else, not gitignored like
bags/ or tools/*_test.py scratch scripts - these are curated, meant to be kept).

Format:
  positions:
    <name>:
      group: <MoveIt planning group name>
      joints: {<joint_name>: <radians>, ...}
"""
import pathlib

import yaml

# MoveIt SRDF group -> its joints (xlerobot.srdf.xacro). Kept here rather than queried live so
# these tools work without a running move_group, only real ros2_control joint state/action topics.
GROUP_JOINTS = {
    "left_arm": ["left_arm_shoulder_pan_joint", "left_arm_shoulder_lift_joint",
                 "left_arm_elbow_flex_joint", "left_arm_wrist_flex_joint", "left_arm_wrist_roll_joint"],
    "right_arm": ["right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint",
                  "right_arm_elbow_flex_joint", "right_arm_wrist_flex_joint", "right_arm_wrist_roll_joint"],
    "left_gripper": ["left_arm_gripper_joint"],
    "right_gripper": ["right_arm_gripper_joint"],
    "head": ["head_pan_joint", "head_tilt_joint"],
}
GROUP_JOINTS["both_arms"] = GROUP_JOINTS["left_arm"] + GROUP_JOINTS["right_arm"]

# MoveIt group -> its SRDF chain tip link (xlerobot.srdf.xacro), for Cartesian/pose-goal tools
# (tools/move_to_pose.py) that target a point in space rather than a joint configuration.
GROUP_TIP_LINK = {
    "left_arm": "left_arm_gripper_link",
    "right_arm": "right_arm_gripper_link",
}

# rad, {prefix}_arm_gripper_joint. Matches the SRDF's own "open"/"closed" named group states
# and tools/moveit_smoke_test.py - not derived from the joint's full range (robot_dimensions.yaml
# arms.joints.gripper), those are the values actually used to open/close the jaw in practice.
GRIPPER_OPEN = -0.7
GRIPPER_CLOSED = 0.35

# Matches the corresponding ros2_control controller for each group, for tools that need to
# talk to a FollowJointTrajectory action directly instead of through MoveIt (record/replay).
GROUP_CONTROLLER = {
    "left_arm": "left_arm_controller",
    "right_arm": "right_arm_controller",
    "left_gripper": "left_gripper_controller",
    "right_gripper": "right_gripper_controller",
    "head": "head_controller",
}

STORE_PATH = (pathlib.Path(__file__).resolve().parent.parent
              / "src" / "xlerobot_bringup" / "config" / "saved_positions.yaml")

# Which servo bus(es) a group's joints live on, for tools/record_trajectory.py to know which
# bus(es) to e-stop (cut torque) for teaching. left_arm/left_gripper/head are all on bus1,
# right_arm/right_gripper on bus2 (see xlerobot.ros2_control.xacro).
GROUP_BUSES = {
    "left_arm": ["xlerobot_bus1"],
    "left_gripper": ["xlerobot_bus1"],
    "head": ["xlerobot_bus1"],
    "right_arm": ["xlerobot_bus2"],
    "right_gripper": ["xlerobot_bus2"],
    "both_arms": ["xlerobot_bus1", "xlerobot_bus2"],
}

# Recorded trajectories: many-sample captures, not curated like saved_positions.yaml - kept
# alongside bags/ in spirit (gitignored, see .gitignore), not committed.
TRAJECTORY_DIR = pathlib.Path(__file__).resolve().parent.parent / "trajectories"


def trajectory_path(name):
    return TRAJECTORY_DIR / f"{name}.yaml"


def load():
    if not STORE_PATH.exists():
        return {}
    with open(STORE_PATH) as f:
        data = yaml.safe_load(f) or {}
    return data.get("positions", {})


def save(positions):
    STORE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(STORE_PATH, "w") as f:
        yaml.safe_dump({"positions": positions}, f, sort_keys=True, default_flow_style=False)


class NotFoundError(LookupError):
    """A regular exception, not SystemExit - tools/mcp_server/bridge.py embeds this module in a
    long-lived process where SystemExit would kill the whole server on a typo'd name. CLI
    scripts catch this in their own main() and sys.exit(str(e)) themselves."""


def get(name):
    positions = load()
    if name not in positions:
        available = ", ".join(sorted(positions)) or "(none saved yet)"
        raise NotFoundError(f"no saved position named '{name}'. Available: {available}")
    return positions[name]
