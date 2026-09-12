"""
3-State Spatial Occupancy Grid with Obstacle Safety Inflation.
States: 0.0 (FREE SPACE), 1.0 (OBSTACLE/BLOCKED), 0.5 (UNKNOWN).
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional, Callable, Dict, Any
import numpy as np

try:
    from ..config import Config, get_config
    from .object_detector import DetectedObject
    from .drivable_area import DrivableArea
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
        from backend.perception.object_detector import DetectedObject
        from backend.perception.drivable_area import DrivableArea
    except (ImportError, ValueError):
        from config import Config, get_config
        from perception.object_detector import DetectedObject
        from perception.drivable_area import DrivableArea


@dataclass
class OccupancyGrid:
    grid: np.ndarray             # [Ny, Nx] float array in [0.0, 1.0]
    x_vec: np.ndarray            # 1D array of X coordinates (m)
    y_vec: np.ndarray            # 1D array of Y coordinates (m)
    resolution: float            # Grid cell resolution (m/cell)
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def world_to_grid(self, xw: float, yw: float) -> Tuple[int, int]:
        ny, nx = self.grid.shape
        ix = int(np.clip(round((xw - self.x_min) / self.resolution), 0, nx - 1))
        iy = int(np.clip(round((yw - self.y_min) / self.resolution), 0, ny - 1))
        return iy, ix

    def is_occupied(self, xw: float, yw: float) -> bool:
        if xw < self.x_min or xw > self.x_max or yw < self.y_min or yw > self.y_max:
            return True
        iy, ix = self.world_to_grid(xw, yw)
        return bool(self.grid[iy, ix] >= 0.95)

    def get_cost(self, xw: float, yw: float) -> float:
        if xw < self.x_min or xw > self.x_max or yw < self.y_min or yw > self.y_max:
            return 1000.0
        iy, ix = self.world_to_grid(xw, yw)
        return float(1.0 + 10.0 * self.grid[iy, ix])

    def to_dict(self) -> Dict[str, Any]:
        sub_grid = self.grid[::4, ::4]
        return {
            "resolution": self.resolution,
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
            "shape": list(self.grid.shape),
            "grid_downsampled": [[round(float(val), 2) for val in row] for row in sub_grid],
        }


def generate_occupancy_grid(
    detected_objects: List[DetectedObject],
    drivable_area: DrivableArea,
    cfg: Optional[Config] = None,
    ego_pose: Optional[Tuple[float, float, float]] = None,
) -> OccupancyGrid:
    """
    Creates a spatial occupancy grid with safety inflation around detected hazards.
    """
    if cfg is None:
        cfg = get_config()

    x_min = cfg.grid.x_min
    x_max = cfg.grid.x_max
    y_min = cfg.grid.y_min
    y_max = cfg.grid.y_max
    res = cfg.grid.resolution
    val_free = cfg.grid.val_free
    val_obs = cfg.grid.val_obstacle

    x_vec = np.arange(x_min, x_max + res / 2, res)
    y_vec = np.arange(y_min, y_max + res / 2, res)

    x_mesh, y_mesh = np.meshgrid(x_vec, y_vec)
    ny, nx = x_mesh.shape

    grid = np.full((ny, nx), val_free, dtype=np.float64)

    if drivable_area is not None:
        in_road = drivable_area.in_drivable_area(x_mesh, y_mesh)
        grid[~in_road] = val_obs

    for obj in detected_objects:
        ox, oy = obj.world_position
        if str(obj.object_class).lower() == "pothole":
            infl_radius = obj.radius + cfg.grid.pothole_inflation_radius
        else:
            infl_radius = obj.radius + cfg.grid.inflation_radius

        dist_sq = (x_mesh - ox) ** 2 + (y_mesh - oy) ** 2

        obs_core = dist_sq <= (obj.radius**2)
        obs_infl = dist_sq <= (infl_radius**2)

        grid[obs_infl] = np.maximum(grid[obs_infl], 0.70)  # Inflation cost zone
        grid[obs_core] = 1.0                              # Hard obstacle core

    return OccupancyGrid(
        grid=grid,
        x_vec=x_vec,
        y_vec=y_vec,
        resolution=res,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
    )
