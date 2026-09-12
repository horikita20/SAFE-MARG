"""Executable collision-free Hybrid A* demonstration."""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from backend.planning.hybrid_astar import HybridAStarPlanner, PlannerConfig

grid = np.ones((30, 30), dtype=np.uint8)
grid[8:23, 13:17] = 0
path = HybridAStarPlanner(PlannerConfig(turning_radius_cells=3., heading_bins=32)).plan(grid, (3., 15., 0.), (26., 15.))
assert path.valid and len(path.waypoints) > 2
assert all(grid[int(round(y)), int(round(x))] == 1 for x, y in path.waypoints)
print("HYBRID A* DEMO: PASS")
print(f"waypoints={len(path.waypoints)} cost={path.cost:.2f} final={path.waypoints[-1].round(2).tolist()}")
