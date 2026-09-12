"""Dependency-light executable verification for the first Safe Marg Python slice."""
from __future__ import annotations

import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from backend.perception.build_environment_model import build_environment_model
from backend.perception.detect_free_space import FreeSpaceDetector
from backend.perception.detect_obstacles import ObstacleDetector, RawDetection


def fake_yolo(_: np.ndarray) -> list[RawDetection]:
    return [
        RawDetection("person", .94, (280., 290., 340., 470.)),
        RawDetection("cow", .72, (50., 320., 160., 475.)),
        RawDetection("car", .10, (400., 300., 550., 470.)),
    ]


def synthetic_road() -> np.ndarray:
    frame = np.full((480, 640, 3), (255, 150, 60), dtype=np.uint8)  # sky
    frame[216:, :] = (100, 100, 100)  # grey road
    frame[330:410, 270:370] = (15, 15, 15)  # un-drivable obstacle
    return frame


def main() -> None:
    frame = synthetic_road()
    obstacles = ObstacleDetector(fake_yolo).detect(frame)
    grid = FreeSpaceDetector(grid_shape=(16, 16)).detect(frame)
    environment = build_environment_model(grid, obstacles, ego_velocity=2.5, timestamp=1.0)

    assert [o.obj_class for o in obstacles] == ["pedestrian", "animal"]
    assert all(o.distance > 0 for o in obstacles)
    assert grid.shape == (16, 16) and set(np.unique(grid)).issubset({0, 1})
    assert grid[-1, 0] == 1 and grid[0, 0] == 0
    assert environment.timestamp == 1.0 and environment.ego_velocity == 2.5

    print("PERCEPTION DEMO: PASS")
    print(f"obstacles={len(obstacles)} classes={[o.obj_class for o in obstacles]}")
    print(f"grid_shape={grid.shape} free_cells={int(grid.sum())} blocked_cells={int(grid.size - grid.sum())}")
    print(f"environment timestamp={environment.timestamp:.1f} ego_velocity={environment.ego_velocity:.1f} m/s")


if __name__ == "__main__":
    main()
