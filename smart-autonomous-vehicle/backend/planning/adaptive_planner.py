"""
Adaptive Kinematic A* Path Planner.
Searches collision-free trajectory with turning radius constraints and obstacle clearance costs.
"""

from dataclasses import dataclass
from typing import Tuple, Optional, List, Dict, Any
import heapq
import numpy as np
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter1d

try:
    from ..config import Config, get_config
    from ..perception.occupancy_grid import OccupancyGrid
    from .global_path import GlobalPath
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
        from backend.perception.occupancy_grid import OccupancyGrid
        from backend.planning.global_path import GlobalPath
    except (ImportError, ValueError):
        from config import Config, get_config
        from perception.occupancy_grid import OccupancyGrid
        from planning.global_path import GlobalPath


@dataclass
class PlannedPath:
    waypoints: np.ndarray    # [N, 2] array of [x, y] coordinates
    headings: np.ndarray     # [N] target heading angles (rad)
    velocities: np.ndarray   # [N] target speeds (m/s)
    cost: float              # Total traversal cost
    valid: bool              # True if feasible path found

    def to_dict(self) -> Dict[str, Any]:
        return {
            "waypoints": [[round(pt[0], 2), round(pt[1], 2)] for pt in self.waypoints],
            "headings": [round(h, 3) for h in self.headings],
            "velocities": [round(v, 2) for v in self.velocities],
            "cost": round(self.cost, 2),
            "valid": self.valid,
        }


def adaptive_path_planner(
    current_pose: Tuple[float, float, float],
    goal_pose: Tuple[float, float, float],
    occ_grid: OccupancyGrid,
    cfg: Optional[Config] = None,
    global_path: Optional[GlobalPath] = None,
) -> PlannedPath:
    """
    Computes a safe, smoothed path using Kinematic Grid A*.
    """
    if cfg is None:
        cfg = get_config()

    x_min = occ_grid.x_min
    x_max = occ_grid.x_max
    y_min = occ_grid.y_min
    y_max = occ_grid.y_max
    res = occ_grid.resolution
    grid = occ_grid.grid
    ny, nx = grid.shape

    start_pos = np.array(current_pose[:2], dtype=float)
    goal_pos = np.array(goal_pose[:2], dtype=float)

    s_iy, s_ix = occ_grid.world_to_grid(start_pos[0], start_pos[1])
    g_iy, g_ix = occ_grid.world_to_grid(goal_pos[0], goal_pos[1])

    motions = [
        (1, 0, 1.0, 0.0),    # Forward
        (1, 1, 1.414, 0.4),  # Forward-Left
        (1, -1, 1.414, 0.4), # Forward-Right
        (0, 1, 1.2, 0.8),    # Left
        (0, -1, 1.2, 0.8),   # Right
        (2, 1, 2.236, 0.2),  # Smooth shallow left
        (2, -1, 2.236, 0.2), # Smooth shallow right
    ]

    g_score = np.full((ny, nx), np.inf, dtype=np.float64)
    g_score[s_iy, s_ix] = 0.0

    parent_x = np.zeros((ny, nx), dtype=np.int32)
    parent_y = np.zeros((ny, nx), dtype=np.int32)
    closed_set = np.zeros((ny, nx), dtype=bool)

    h_start = float(np.linalg.norm(start_pos - goal_pos))
    open_heap = [(h_start, (s_iy, s_ix))]
    in_open_set = np.zeros((ny, nx), dtype=bool)
    in_open_set[s_iy, s_ix] = True

    max_iter = cfg.planner.max_iterations
    iterations = 0
    goal_reached = False
    closest_node = (s_iy, s_ix)
    min_h = h_start

    while open_heap and iterations < max_iter:
        iterations += 1
        _, (cy, cx) = heapq.heappop(open_heap)
        in_open_set[cy, cx] = False
        closed_set[cy, cx] = True

        curr_world = np.array([x_min + cx * res, y_min + cy * res])
        dist_to_goal = float(np.linalg.norm(curr_world - goal_pos))

        if dist_to_goal < min_h:
            min_h = dist_to_goal
            closest_node = (cy, cx)

        if (cx >= g_ix - 1 and abs(cy - g_iy) <= 2) or dist_to_goal <= 1.5:
            closest_node = (cy, cx)
            goal_reached = True
            break

        for dx, dy, step_cost, steer_penalty in motions:
            nx_pos = cx + dx
            ny_pos = cy + dy

            if nx_pos < 0 or nx_pos >= nx or ny_pos < 0 or ny_pos >= ny:
                continue
            if closed_set[ny_pos, nx_pos]:
                continue

            cell_occ = grid[ny_pos, nx_pos]
            if cell_occ >= 0.95:  # Block only hard obstacle cores
                continue

            world_ny = y_min + ny_pos * res
            cost_obstacle = 20.0 * (cell_occ ** 2)  # Quadratic penalty for proximity
            cost_center_bias = 0.15 * abs(world_ny)
            total_step = step_cost * res + steer_penalty + cost_obstacle + cost_center_bias

            tentative_g = g_score[cy, cx] + total_step

            if tentative_g < g_score[ny_pos, nx_pos]:
                parent_x[ny_pos, nx_pos] = cx
                parent_y[ny_pos, nx_pos] = cy
                g_score[ny_pos, nx_pos] = tentative_g

                n_world = np.array([x_min + nx_pos * res, y_min + ny_pos * res])
                h = float(np.linalg.norm(n_world - goal_pos))
                f = tentative_g + 1.1 * h

                if not in_open_set[ny_pos, nx_pos]:
                    heapq.heappush(open_heap, (f, (ny_pos, nx_pos)))
                    in_open_set[ny_pos, nx_pos] = True

    path_nodes = [closest_node]
    curr = closest_node
    while curr != (s_iy, s_ix):
        py = int(parent_y[curr[0], curr[1]])
        px = int(parent_x[curr[0], curr[1]])
        if px == 0 and py == 0 and curr != (s_iy, s_ix):
            break
        curr = (py, px)
        path_nodes.append(curr)

    path_nodes.reverse()

    if len(path_nodes) < 2:
        x_pts = np.linspace(start_pos[0], min(start_pos[0] + 10.0, goal_pos[0]), 20)
        y_pts = np.full_like(x_pts, start_pos[1])
        raw_waypoints = np.column_stack((x_pts, y_pts))
        valid = False
    else:
        raw_waypoints = np.array([[x_min + cx * res, y_min + cy * res] for cy, cx in path_nodes])
        valid = goal_reached or (min_h < 6.0)

    waypoints = smooth_path(raw_waypoints, start_pos, cfg)

    nominal_speed = cfg.vehicle.nominal_speed
    velocities = np.full(len(waypoints), nominal_speed, dtype=float)
    dist_g = np.linalg.norm(waypoints - goal_pos, axis=1)
    decel_idx = dist_g <= 6.0
    velocities[decel_idx] = np.maximum(1.5, nominal_speed * (dist_g[decel_idx] / 6.0))
    velocities[-1] = 0.0

    dx = np.gradient(waypoints[:, 0])
    dy = np.gradient(waypoints[:, 1])
    headings = np.arctan2(dy, dx)

    return PlannedPath(
        waypoints=waypoints,
        headings=headings,
        velocities=velocities,
        cost=float(g_score[closest_node[0], closest_node[1]]),
        valid=valid,
    )


def smooth_path(raw_waypoints: np.ndarray, start_pos: np.ndarray, cfg: Config) -> np.ndarray:
    if len(raw_waypoints) <= 3:
        return raw_waypoints

    diffs = np.diff(raw_waypoints, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_dist = np.insert(np.cumsum(seg_lengths), 0, 0.0)
    total_dist = cum_dist[-1]

    if total_dist < 0.5:
        return raw_waypoints

    ds = cfg.planner.waypoint_spacing
    uniform_dist = np.arange(0, total_dist + ds / 2, ds)

    fx = interp1d(cum_dist, raw_waypoints[:, 0], kind="linear", fill_value="extrapolate")
    fy = interp1d(cum_dist, raw_waypoints[:, 1], kind="linear", fill_value="extrapolate")

    interp_x = fx(uniform_dist)
    interp_y = fy(uniform_dist)

    smoothed_y = gaussian_filter1d(interp_y, sigma=2.0)
    smoothed_y[0] = start_pos[1]
    interp_x[0] = start_pos[0]

    return np.column_stack((interp_x, smoothed_y))
