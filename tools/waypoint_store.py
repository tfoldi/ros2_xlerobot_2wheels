"""Shared load/save helpers for named base waypoints, used by save_waypoint.py,
goto_waypoint.py and list_waypoints.py. Same shape as position_store.py, but for map-frame
base poses instead of arm joint configurations - the two aren't interchangeable (a waypoint has
no joints, a saved position has no x/y/yaw).

Storage: src/xlerobot_bringup/config/saved_waypoints.yaml, tracked in git like
saved_positions.yaml (curated, meant to be kept, not gitignored).

Format:
  waypoints:
    <name>:
      x: <metres, map frame>
      y: <metres, map frame>
      yaw: <radians, map frame>
"""
import pathlib

import yaml

STORE_PATH = (pathlib.Path(__file__).resolve().parent.parent
              / "src" / "xlerobot_bringup" / "config" / "saved_waypoints.yaml")


def load():
    if not STORE_PATH.exists():
        return {}
    with open(STORE_PATH) as f:
        data = yaml.safe_load(f) or {}
    return data.get("waypoints", {})


def save(waypoints):
    STORE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(STORE_PATH, "w") as f:
        yaml.safe_dump({"waypoints": waypoints}, f, sort_keys=True, default_flow_style=False)


class NotFoundError(LookupError):
    """A regular exception, not SystemExit - tools/mcp_server/bridge.py embeds this module in a
    long-lived process where SystemExit would kill the whole server on a typo'd name. CLI
    scripts catch this in their own main() and sys.exit(str(e)) themselves."""


def get(name):
    waypoints = load()
    if name not in waypoints:
        available = ", ".join(sorted(waypoints)) or "(none saved yet)"
        raise NotFoundError(f"no saved waypoint named '{name}'. Available: {available}")
    return waypoints[name]
