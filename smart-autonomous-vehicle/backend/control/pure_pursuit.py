"""
Geometric Pure Pursuit steering controller with speed-adaptive lookahead.
"""

from typing import Tuple, Optional
import numpy as np

try:
    from ..config import Config, get_config
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
    except (ImportError, ValueError):
        from config import Config, get_config


def pure_pursuit_controller(
    current_pose: Tuple[float, float, float],
    current_velocity: float,
    waypoints: np.ndarray,
    cfg: Optional[Config] = None,
) -> Tuple[float, Tuple[float, float], float]:
    """
    Computes steering angle command, lookahead target point, and cross-track error.
    """
    if cfg is None:
        cfg = get_config()

    wheelbase = cfg.vehicle.wheelbase
    max_steer = cfg.vehicle.max_steer_angle
    min_ld = cfg.control.min_lookahead
    max_ld = cfg.control.max_lookahead
    k_ld = cfg.control.lookahead_gain

    if waypoints is None or len(waypoints) < 2:
        return 0.0, (current_pose[0], current_pose[1]), 0.0

    ego_x, ego_y, ego_theta = current_pose

    ld = float(np.clip(k_ld * current_velocity, min_ld, max_ld))

    dx = waypoints[:, 0] - ego_x
    dy = waypoints[:, 1] - ego_y
    dists = np.sqrt(dx**2 + dy**2)

    closest_idx = int(np.argmin(dists))
    cross_track_error = float(dists[closest_idx])

    target_idx = closest_idx
    for i in range(closest_idx, len(waypoints)):
        if dists[i] >= ld:
            target_idx = i
            break
        target_idx = i

    target_pt = (float(waypoints[target_idx, 0]), float(waypoints[target_idx, 1]))

    vec_x = target_pt[0] - ego_x
    vec_y = target_pt[1] - ego_y

    local_x = np.cos(ego_theta) * vec_x + np.sin(ego_theta) * vec_y
    local_y = -np.sin(ego_theta) * vec_x + np.cos(ego_theta) * vec_y

    actual_ld = max(1.0, float(np.hypot(local_x, local_y)))
    steer_cmd = float(np.arctan2(2.0 * wheelbase * local_y, actual_ld**2))
    steer_cmd = float(np.clip(steer_cmd, -max_steer, max_steer))

    return steer_cmd, target_pt, cross_track_error
