"""
Dynamic obstacle local reactive avoidance module.
Predicts hazard motion and generates smooth lateral detour bypass.
"""

from typing import Tuple, List, Optional, Any
import numpy as np
from scipy.ndimage import gaussian_filter1d

try:
    from ..config import Config, get_config
    from ..perception.object_detector import DetectedObject
    from ..perception.occupancy_grid import OccupancyGrid
    from .adaptive_planner import PlannedPath
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
        from backend.perception.object_detector import DetectedObject
        from backend.perception.occupancy_grid import OccupancyGrid
        from backend.planning.adaptive_planner import PlannedPath
    except (ImportError, ValueError):
        from config import Config, get_config
        from perception.object_detector import DetectedObject
        from perception.occupancy_grid import OccupancyGrid
        from planning.adaptive_planner import PlannedPath


def dynamic_obstacle_avoidance(
    active_path: Any,
    current_pose: Tuple[float, float, float],
    current_velocity: float,
    detected_objects: List[DetectedObject],
    occ_grid: Optional[OccupancyGrid] = None,
    cfg: Optional[Config] = None,
) -> PlannedPath:
    """
    Computes a smooth local lateral detour around dynamic hazards.
    """
    if cfg is None:
        cfg = get_config()

    if not detected_objects or active_path is None or len(getattr(active_path, "waypoints", [])) == 0:
        return active_path

    ego_pos = np.array(current_pose[:2], dtype=float)
    waypoints = np.copy(active_path.waypoints)
    num_wp = len(waypoints)

    threat_obj: Optional[DetectedObject] = None
    min_d = float("inf")
    for obj in detected_objects:
        d = float(np.linalg.norm(np.array(obj.world_position) - ego_pos))
        if d < min_d and d < 18.0:
            min_d = d
            threat_obj = obj

    if threat_obj is None:
        return active_path

    obs_y = threat_obj.world_position[1]
    obs_vy = threat_obj.world_velocity[1]

    if obs_vy > 0.3:
        bypass_dir = -1.0  # Moving left -> bypass right
    elif obs_vy < -0.3:
        bypass_dir = 1.0   # Moving right -> bypass left
    else:
        bypass_dir = -1.0 if obs_y >= 0 else 1.0

    avoid_radius = threat_obj.radius + cfg.risk.lateral_clearance + 0.8
    detour_mag = bypass_dir * avoid_radius
    threat_pos = np.array(threat_obj.world_position)

    new_waypoints = np.copy(waypoints)
    for w in range(num_wp):
        wp = waypoints[w]
        dist_to_obs = float(np.linalg.norm(wp - threat_pos))
        if dist_to_obs < (avoid_radius * 2.5):
            shift = np.exp(-(dist_to_obs**2) / (2 * (avoid_radius * 0.9) ** 2))
            new_y = wp[1] + detour_mag * shift
            if occ_grid is not None:
                new_y = np.clip(new_y, occ_grid.y_min + 0.8, occ_grid.y_max - 0.8)
            new_waypoints[w, 1] = new_y

    new_waypoints[0, 1] = ego_pos[1]

    if len(new_waypoints) >= 5:
        new_waypoints[:, 1] = gaussian_filter1d(new_waypoints[:, 1], sigma=1.5)

    dx = np.gradient(new_waypoints[:, 0])
    dy = np.gradient(new_waypoints[:, 1])
    headings = np.arctan2(dy, dx)

    velocities = np.copy(active_path.velocities)
    velocities = np.maximum(2.0, velocities * 0.75)

    return PlannedPath(
        waypoints=new_waypoints,
        headings=headings,
        velocities=velocities,
        cost=getattr(active_path, "cost", 10.0),
        valid=True,
    )
