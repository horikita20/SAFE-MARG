"""
Drivable road corridor and unstructured margin extraction module.
Handles roads without reliable lane markings.
"""

from dataclasses import dataclass
from typing import Tuple, List, Callable, Optional, Dict, Any
import numpy as np

try:
    from ..config import Config, get_config
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
    except (ImportError, ValueError):
        from config import Config, get_config


@dataclass
class DrivableArea:
    left_boundary: np.ndarray    # [N, 2] coordinates [x, y] of left margin
    right_boundary: np.ndarray   # [N, 2] coordinates [x, y] of right margin
    centerline: np.ndarray       # [N, 2] coordinates [x, y] of road center
    road_width: float            # Average road width (meters)
    surface_type: str            # "asphalt" | "unpaved_gravel" | "patchy_rural"
    left_bound_func: Callable[[np.ndarray], np.ndarray]
    right_bound_func: Callable[[np.ndarray], np.ndarray]

    def in_drivable_area(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return (y <= self.left_bound_func(x)) & (y >= self.right_bound_func(x))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "road_width": round(self.road_width, 2),
            "surface_type": self.surface_type,
            "left_boundary": [[round(pt[0], 2), round(pt[1], 2)] for pt in self.left_boundary[::5]],
            "right_boundary": [[round(pt[0], 2), round(pt[1], 2)] for pt in self.right_boundary[::5]],
            "centerline": [[round(pt[0], 2), round(pt[1], 2)] for pt in self.centerline[::5]],
        }


def detect_drivable_area(
    scenario: Any,
    ego_pose: Tuple[float, float, float],
    cfg: Optional[Config] = None,
) -> DrivableArea:
    """
    Extracts road margins and drivable area corridor for unstructured roads.
    """
    if cfg is None:
        cfg = get_config()

    x_min = cfg.grid.x_min
    x_max = cfg.grid.x_max
    num_pts = 100
    x_samples = np.linspace(x_min, x_max, num_pts)

    if hasattr(scenario, "road") and scenario.road is not None:
        road = scenario.road
        left_bound_func = road.left_bound_func
        right_bound_func = road.right_bound_func
        surface_type = getattr(road, "surface_type", "patchy_rural")
    elif isinstance(scenario, dict) and "road" in scenario:
        road = scenario["road"]
        left_bound_func = road["left_bound_func"]
        right_bound_func = road["right_bound_func"]
        surface_type = road.get("surface_type", "patchy_rural")
    else:
        half_w = 3.5
        left_bound_func = lambda x: half_w * np.ones_like(x)
        right_bound_func = lambda x: -half_w * np.ones_like(x)
        surface_type = "patchy_rural"

    y_left = left_bound_func(x_samples)
    y_right = right_bound_func(x_samples)
    y_center = (y_left + y_right) / 2.0

    left_bound = np.column_stack((x_samples, y_left))
    right_bound = np.column_stack((x_samples, y_right))
    centerline = np.column_stack((x_samples, y_center))
    road_width = float(np.mean(y_left - y_right))

    return DrivableArea(
        left_boundary=left_bound,
        right_boundary=right_bound,
        centerline=centerline,
        road_width=road_width,
        surface_type=surface_type,
        left_bound_func=left_bound_func,
        right_bound_func=right_bound_func,
    )
