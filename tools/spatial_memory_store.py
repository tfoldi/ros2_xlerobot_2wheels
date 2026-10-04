"""Shared load/save helpers for the agent's own spatial-memory ("remembered objects") cache.
Same load()/save()/get() shape as position_store.py/waypoint_store.py, but this store is
agent-written working memory, not hand-curated config, so it lives under captures/ (gitignored,
ephemeral, like tools/capture_image.py's own convention) rather than
src/xlerobot_bringup/config/.

Format:
  objects:
    <label>:
      description: <free text, overwritten by the most recent remember_object call>
      created_at: <iso8601 utc, first observation>
      updated_at: <iso8601 utc, most recent observation>
      observations: [<observation>, ...]
      position_estimate: <see spatial_memory_math.estimate_position()>

  <observation>:
    camera: head|left|right
    frame_id: <optical frame used>
    image_path: <path under captures/, as returned by capture_image>
    timestamp: <iso8601 utc>
    image_size: {width: int, height: int}
    pixel: {x: int, y: int}                 # defaults to image center if not given
    camera_pose_map: {x,y,z,qx,qy,qz,qw} | null   # map frame, at capture time
    robot_pose_map: {x,y,yaw} | null              # base pose, at capture time
    ray_camera_frame: {x,y,z} | null        # pixel unprojected via PinholeCameraModel, camera
                                             # optical frame, cached so later recompute (e.g. a
                                             # different plane_height_m) needs no ROS at all
    pose_available: bool    # == camera_pose_map is not null
    ray_available: bool     # == ray_camera_frame is not null (camera_info was cached in time)
"""
import pathlib

import yaml

STORE_PATH = pathlib.Path(__file__).resolve().parent.parent / "captures" / "spatial_memory.yaml"


class NotFoundError(LookupError):
    """Same reasoning as position_store's own not-found error - a regular exception, not
    SystemExit, since bridge.py embeds this in a long-lived server process."""


def load():
    if not STORE_PATH.exists():
        return {}
    with open(STORE_PATH) as f:
        data = yaml.safe_load(f) or {}
    return data.get("objects", {})


def save(objects):
    STORE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(STORE_PATH, "w") as f:
        yaml.safe_dump({"objects": objects}, f, sort_keys=True, default_flow_style=False)


def get(label):
    objects = load()
    if label not in objects:
        available = ", ".join(sorted(objects)) or "(none remembered yet)"
        raise NotFoundError(f"no remembered object named '{label}'. Available: {available}")
    return objects[label]


def delete(label):
    objects = load()
    entry = get(label)  # raises NotFoundError with the same message as above
    del objects[label]
    save(objects)
    return entry
