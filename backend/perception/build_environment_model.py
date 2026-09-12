"""Build the validated model consumed by planning and decision modules."""
from __future__ import annotations
import time
from collections.abc import Iterable
import numpy as np
from backend.models import EnvironmentModel, Obstacle

def build_environment_model(occupancy_grid: np.ndarray, obstacles: Iterable[Obstacle], *,
                            ego_pose: tuple[float, float, float] = (0., 0., 0.), ego_velocity: float = 0.,
                            timestamp: float | None = None) -> EnvironmentModel:
    grid = np.asarray(occupancy_grid)
    if grid.ndim != 2 or grid.size == 0:
        raise ValueError("occupancy_grid must be a non-empty 2-D array")
    if not np.isin(grid, (0, 1)).all():
        raise ValueError("occupancy_grid values must be 0 (blocked) or 1 (free)")
    if len(ego_pose) != 3 or ego_velocity < 0:
        raise ValueError("invalid ego_pose or ego_velocity")
    return EnvironmentModel(time.time() if timestamp is None else float(timestamp), grid.astype(np.uint8, copy=True),
        list(obstacles), tuple(map(float, ego_pose)), float(ego_velocity))
