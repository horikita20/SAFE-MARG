"""Single source of truth for all Safe Marg backend data contracts."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal
import numpy as np
from numpy.typing import NDArray

ObstacleClass = Literal["pedestrian", "vehicle", "animal", "pothole", "debris", "unknown"]

@dataclass(frozen=True, slots=True)
class Obstacle:
    id: int
    obj_class: ObstacleClass
    bbox: tuple[float, float, float, float]
    position: tuple[float, float]
    velocity: tuple[float, float]
    distance: float
    confidence: float

@dataclass(frozen=True, slots=True)
class EnvironmentModel:
    timestamp: float
    occupancy_grid: NDArray[np.uint8]  # 0=blocked, 1=free
    obstacles: list[Obstacle]
    ego_pose: tuple[float, float, float]
    ego_velocity: float

@dataclass(frozen=True, slots=True)
class PlannedPath:
    waypoints: NDArray[np.float64]  # Nx2 grid coordinates, x then y
    velocities: NDArray[np.float64]
    cost: float
    valid: bool
