"""A dependency-light Hybrid A* planner for binary Safe Marg occupancy grids.

Grid convention: ``grid[y, x]`` is 1 for free and 0 for blocked. Coordinates
returned in ``PlannedPath.waypoints`` are ``(x, y)`` grid-cell centres.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
from math import cos, hypot, pi, sin
from typing import Iterator
import numpy as np

from backend.models import PlannedPath

@dataclass(frozen=True, slots=True)
class PlannerConfig:
    turning_radius_cells: float = 3.0
    step_size_cells: float = 1.0
    heading_bins: int = 24
    goal_tolerance_cells: float = 1.5
    obstacle_clearance_cells: float = 0.0
    nominal_speed_mps: float = 4.0

class HybridAStarPlanner:
    """Searches forward bicycle-model motion primitives over a binary grid."""
    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()
        if self.config.turning_radius_cells <= 0 or self.config.step_size_cells <= 0:
            raise ValueError("turning_radius_cells and step_size_cells must be positive")
        if self.config.heading_bins < 8 or self.config.nominal_speed_mps <= 0:
            raise ValueError("heading_bins must be >= 8 and nominal_speed_mps positive")

    def plan(self, occupancy_grid: np.ndarray, start: tuple[float, float, float], goal: tuple[float, float]) -> PlannedPath:
        grid = self._validate_grid(occupancy_grid)
        sx, sy, stheta = start
        gx, gy = goal
        if not self._is_free(grid, sx, sy) or not self._is_free(grid, gx, gy):
            return self._invalid_path()
        start_key = self._key(sx, sy, stheta)
        queue: list[tuple[float, int, float, float, float, float]] = []
        serial = 0
        heapq.heappush(queue, (self._heuristic(sx, sy, gx, gy), serial, 0., sx, sy, self._normalise(stheta)))
        parent: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start_key: None}
        states: dict[tuple[int, int, int], tuple[float, float, float]] = {start_key: (sx, sy, self._normalise(stheta))}
        costs: dict[tuple[int, int, int], float] = {start_key: 0.}
        while queue:
            _, _, current_cost, x, y, theta = heapq.heappop(queue)
            key = self._key(x, y, theta)
            if current_cost != costs.get(key):
                continue
            if hypot(x - gx, y - gy) <= self.config.goal_tolerance_cells:
                return self._make_path(key, parent, states, current_cost)
            for nx, ny, ntheta, primitive_cost in self._successors(x, y, theta):
                if not self._segment_is_free(grid, x, y, nx, ny):
                    continue
                next_key, next_cost = self._key(nx, ny, ntheta), current_cost + primitive_cost
                if next_cost >= costs.get(next_key, float("inf")):
                    continue
                costs[next_key], parent[next_key], states[next_key] = next_cost, key, (nx, ny, ntheta)
                serial += 1
                heapq.heappush(queue, (next_cost + self._heuristic(nx, ny, gx, gy), serial, next_cost, nx, ny, ntheta))
        return self._invalid_path()

    def _successors(self, x: float, y: float, theta: float) -> Iterator[tuple[float, float, float, float]]:
        step = self.config.step_size_cells
        for curvature in (-1. / self.config.turning_radius_cells, 0., 1. / self.config.turning_radius_cells):
            next_theta = self._normalise(theta + curvature * step)
            midpoint = theta + curvature * step / 2.
            yield x + step * cos(midpoint), y + step * sin(midpoint), next_theta, step + (.15 if curvature else 0.)

    def _segment_is_free(self, grid: np.ndarray, x0: float, y0: float, x1: float, y1: float) -> bool:
        for fraction in np.linspace(0., 1., max(2, int(np.ceil(hypot(x1 - x0, y1 - y0) * 2)))):
            if not self._is_free(grid, x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction):
                return False
        return True

    def _is_free(self, grid: np.ndarray, x: float, y: float) -> bool:
        xi, yi = int(round(x)), int(round(y))
        if not (0 <= yi < grid.shape[0] and 0 <= xi < grid.shape[1]) or grid[yi, xi] != 1:
            return False
        clearance = int(np.ceil(self.config.obstacle_clearance_cells))
        return not clearance or bool(np.all(grid[max(0, yi-clearance):min(grid.shape[0], yi+clearance+1), max(0, xi-clearance):min(grid.shape[1], xi+clearance+1)] == 1))

    def _key(self, x: float, y: float, theta: float) -> tuple[int, int, int]:
        heading = int(round(self._normalise(theta) / (2*pi) * self.config.heading_bins)) % self.config.heading_bins
        return int(round(x)), int(round(y)), heading
    @staticmethod
    def _heuristic(x: float, y: float, gx: float, gy: float) -> float: return hypot(gx-x, gy-y)
    @staticmethod
    def _normalise(theta: float) -> float: return theta % (2*pi)
    @staticmethod
    def _validate_grid(grid: np.ndarray) -> np.ndarray:
        grid = np.asarray(grid, dtype=np.uint8)
        if grid.ndim != 2 or grid.size == 0 or not np.isin(grid, (0, 1)).all():
            raise ValueError("occupancy_grid must be a non-empty binary 2-D array")
        return grid
    def _make_path(self, goal_key: tuple[int, int, int], parent: dict, states: dict, cost: float) -> PlannedPath:
        keys, key = [], goal_key
        while key is not None:
            keys.append(key); key = parent[key]
        waypoints = np.array([(states[k][0], states[k][1]) for k in reversed(keys)], dtype=np.float64)
        return PlannedPath(waypoints, np.full(len(waypoints), self.config.nominal_speed_mps), float(cost), True)
    def _invalid_path(self) -> PlannedPath:
        return PlannedPath(np.empty((0, 2)), np.empty(0), float("inf"), False)
