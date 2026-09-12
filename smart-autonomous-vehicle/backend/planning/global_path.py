"""
Global reference path generation along road corridor.
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any, List
import numpy as np
from scipy.ndimage import gaussian_filter1d

try:
    from ..config import Config, get_config
    from ..perception.drivable_area import DrivableArea
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
        from backend.perception.drivable_area import DrivableArea
    except (ImportError, ValueError):
        from config import Config, get_config
        from perception.drivable_area import DrivableArea


@dataclass
class GlobalPath:
    waypoints: np.ndarray    # [N, 2] array of [x, y] coordinates
    headings: np.ndarray     # [N] target heading angles (rad)
    velocities: np.ndarray   # [N] target speeds (m/s)
    curvatures: np.ndarray   # [N] path curvature kappa (1/m)
    length: float            # Total path length (m)
    valid: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "waypoints": [[round(pt[0], 2), round(pt[1], 2)] for pt in self.waypoints],
            "headings": [round(h, 3) for h in self.headings],
            "velocities": [round(v, 2) for v in self.velocities],
            "length": round(self.length, 2),
            "valid": self.valid,
        }


def generate_global_path(
    start_pose: Tuple[float, float, float],
    goal_pose: Tuple[float, float, float],
    drivable_area: Optional[DrivableArea] = None,
    cfg: Optional[Config] = None,
) -> GlobalPath:
    """
    Generates a baseline reference global path from start to destination along road centerline.
    """
    if cfg is None:
        cfg = get_config()

    ds = cfg.planner.waypoint_spacing
    nominal_speed = cfg.vehicle.nominal_speed

    x0, y0, _ = start_pose
    xg, yg, _ = goal_pose

    x_samples = np.arange(x0, xg + ds / 2, ds)
    if len(x_samples) == 0 or x_samples[-1] < xg:
        x_samples = np.append(x_samples, xg)

    if drivable_area is not None and hasattr(drivable_area, "left_bound_func"):
        y_left = drivable_area.left_bound_func(x_samples)
        y_right = drivable_area.right_bound_func(x_samples)
        y_center = (y_left + y_right) / 2.0
    else:
        y_center = np.linspace(y0, yg, len(x_samples))

    num_blend = min(15, len(x_samples))
    blend_weights = np.linspace(1.0, 0.0, num_blend)
    y_center[:num_blend] = blend_weights * y0 + (1.0 - blend_weights) * y_center[:num_blend]
    y_center[-1] = yg

    if len(y_center) >= 5:
        y_center = gaussian_filter1d(y_center, sigma=2.0)

    waypoints = np.column_stack((x_samples, y_center))

    dx = np.gradient(waypoints[:, 0])
    dy = np.gradient(waypoints[:, 1])
    headings = np.arctan2(dy, dx)

    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvatures = (dx * ddy - dy * ddx) / ((dx**2 + dy**2) ** 1.5 + 1e-6)

    velocities = np.full(len(waypoints), nominal_speed, dtype=float)
    dist_to_goal = np.linalg.norm(waypoints - np.array([xg, yg]), axis=1)
    decel_mask = dist_to_goal <= 8.0
    velocities[decel_mask] = np.maximum(1.5, nominal_speed * (dist_to_goal[decel_mask] / 8.0))
    velocities[-1] = 0.0

    diffs = np.diff(waypoints, axis=0)
    path_length = float(np.sum(np.linalg.norm(diffs, axis=1)))

    return GlobalPath(
        waypoints=waypoints,
        headings=headings,
        velocities=velocities,
        curvatures=curvatures,
        length=path_length,
        valid=True,
    )
