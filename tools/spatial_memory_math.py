"""Pure math for turning a (pixel, camera pose) observation into a 3D point estimate, with no
depth sensor. No live ROS node required - CameraInfo/pose inputs are plain data, so this is
testable standalone with synthetic values before any camera/TF integration.

Single observation: intersect the pixel's ray with an assumed horizontal plane (a guess, labeled
as such). Two or more observations of the same label from sufficiently different viewpoints:
triangulate a real 3D point instead, no plane assumption needed. See estimate_position() for how
the two are chosen between.
"""
import datetime

import numpy as np
from image_geometry import PinholeCameraModel
from scipy.spatial.transform import Rotation

DEFAULT_PLANE_HEIGHT_M = 0.75  # "typical tabletop/desk height" - a guess, not measured; override
# per remember_object call for floor-level/shelf objects etc.

# Below this angle (deg) between any two observations' ray directions, triangulation is
# numerically "present" but meaningless - depth uncertainty for fixed pixel noise scales roughly
# as 1/sin(parallax), so near-parallel rays blow up. 8 deg is a judgment call (this camera's fx
# ~332 at 640x480 gives ~0.17 deg/pixel, so a few pixels of detection noise already dominates
# below this), not an empirical fit - revisit once real captures exist.
MIN_PARALLAX_DEG = 8.0


def _now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds")


def _point_dict(p):
    return {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}


def pixel_to_ray_camera_frame(camera_info_msg, pixel_x, pixel_y):
    """(pixel_x, pixel_y) -> unit-ish direction vector in the camera's optical frame (+X right,
    +Y down, +Z forward). Undistorts via the camera's real calibration, not a naive
    pinhole-only inverse."""
    model = PinholeCameraModel()
    model.fromCameraInfo(camera_info_msg)
    x, y, z = model.projectPixelTo3dRay((pixel_x, pixel_y))
    return np.array([x, y, z], dtype=float)


def ray_to_map_frame(ray_camera, camera_pose_map):
    """Rotate+place a camera-frame ray into map frame using a cached camera_pose_map (NOT a live
    TF lookup - this replays a past observation's recorded pose, so it works purely from the
    YAML, no ROS needed). Returns (origin, unit_direction), both (3,) arrays, map frame."""
    origin = np.array([camera_pose_map["x"], camera_pose_map["y"], camera_pose_map["z"]])
    rot = Rotation.from_quat([camera_pose_map["qx"], camera_pose_map["qy"],
                              camera_pose_map["qz"], camera_pose_map["qw"]])
    direction = rot.apply(np.asarray(ray_camera, dtype=float))
    return origin, direction / np.linalg.norm(direction)


def intersect_ray_with_plane(origin, direction, plane_z):
    """None if the ray is ~parallel to the plane or the plane is behind the ray's origin."""
    EPS_Z = 1e-3
    if abs(direction[2]) < EPS_Z:
        return None
    s = (plane_z - origin[2]) / direction[2]
    if s <= 0:
        return None
    return origin + s * direction


def max_pairwise_parallax_deg(directions):
    """Largest angle (deg) between any two unit directions - the usable parallax across all
    observations. 0.0 for fewer than 2 directions."""
    best = 0.0
    for i in range(len(directions)):
        for j in range(i + 1, len(directions)):
            cos_a = np.clip(abs(np.dot(directions[i], directions[j])), -1.0, 1.0)
            best = max(best, np.degrees(np.arccos(cos_a)))
    return best


def triangulate_rays(origins, directions):
    """Least-squares closest point to N>=2 lines (point-origin + unit direction each), via the
    standard normal-equations form:
        A = sum_i (I - d_i d_i^T),   b = sum_i (I - d_i d_i^T) o_i,   X = A^+ b
    (I - d d^T) is the projector onto the plane perpendicular to d, so this is minimizing the sum
    of squared perpendicular distances from X to every line - the generalization of "closest
    point between two skew lines" to N>=2, and reduces to exactly that formula for N==2, so there
    is no separate 2-ray special case. Returns (point (3,), rms_residual_m) - rms_residual_m is
    the root-mean-square perpendicular distance from the solved point back to each input line, a
    direct measure of how self-consistent the observations were (near 0 for clean geometry)."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for o, d in zip(origins, directions):
        proj = np.eye(3) - np.outer(d, d)
        A += proj
        b += proj @ o
    point, *_ = np.linalg.lstsq(A, b, rcond=None)
    residuals = [np.linalg.norm((np.eye(3) - np.outer(d, d)) @ (point - o))
                 for o, d in zip(origins, directions)]
    return point, float(np.sqrt(np.mean(np.square(residuals))))


def estimate_position(observations, plane_height_m=DEFAULT_PLANE_HEIGHT_M):
    """observations: the raw list of per-observation dicts for one label, as stored in YAML
    (spatial_memory_store's schema) - this function does its own filtering to pose_available and
    ray_available ones, so callers (bridge.remember_object, recall_objects) never need to
    pre-filter. Pure numpy/scipy - safe to call with zero ROS running, straight off the YAML.

    Returns a position_estimate dict with provenance one of:
      "none"                                - no usable observation at all
      "single-observation-plane-assumption" - exactly one usable ray, or >1 but insufficient
                                               parallax to triangulate (see MIN_PARALLAX_DEG)
      "triangulated"                        - >=2 usable rays with sufficient parallax
    """
    usable = [o for o in observations if o["pose_available"] and o["ray_available"]]
    if not usable:
        return {"provenance": "none", "point": None, "observations_used": 0,
                "note": "no observation yet has both a camera pose and a camera_info-derived "
                        "ray - check AMCL localization and that cameras:=true publishes "
                        "camera_info", "computed_at": _now_iso()}

    rays = [ray_to_map_frame(
        (o["ray_camera_frame"]["x"], o["ray_camera_frame"]["y"], o["ray_camera_frame"]["z"]),
        o["camera_pose_map"]) for o in usable]
    origins, directions = zip(*rays)
    parallax = max_pairwise_parallax_deg(directions) if len(usable) > 1 else 0.0

    if len(usable) >= 2 and parallax >= MIN_PARALLAX_DEG:
        point, residual = triangulate_rays(origins, directions)
        confidence = "high" if residual < 0.03 else "medium" if residual < 0.08 else "low"
        return {"provenance": "triangulated", "point": _point_dict(point),
                "observations_used": len(usable), "max_parallax_deg": round(parallax, 1),
                "residual_m": round(residual, 4), "confidence": confidence,
                "note": f"triangulated from {len(usable)} observations, {parallax:.1f} deg "
                        f"max parallax, {residual * 1000:.0f}mm rms residual",
                "computed_at": _now_iso()}

    # fall back to plane-assumption, using the most recent usable observation's ray
    origin, direction = origins[-1], directions[-1]
    point = intersect_ray_with_plane(origin, direction, plane_height_m)
    note = (f"{len(usable)} observation(s), insufficient parallax ({parallax:.1f} deg < "
            f"{MIN_PARALLAX_DEG} deg) for triangulation - " if len(usable) > 1 else "") + \
        f"single-ray estimate assuming a horizontal plane at z={plane_height_m:.2f}m"
    if point is None:
        return {"provenance": "single-observation-plane-assumption", "point": None,
                "observations_used": len(usable), "plane_height_m": plane_height_m,
                "confidence": "low",
                "note": note + " - ray is near-parallel to the plane or points away from it, "
                        "no intersection", "computed_at": _now_iso()}
    return {"provenance": "single-observation-plane-assumption", "point": _point_dict(point),
            "observations_used": len(usable), "plane_height_m": plane_height_m,
            "confidence": "low", "note": note, "computed_at": _now_iso()}
