"""Conservative classical-CV drivable-area segmentation.

Uses OpenCV when available; the NumPy path keeps the hackathon demo runnable
on a minimal Python installation.
"""
from __future__ import annotations
import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

class FreeSpaceDetector:
    def __init__(self, *, grid_shape: tuple[int, int] = (80, 80), road_start_ratio: float = .45) -> None:
        if min(grid_shape) <= 0 or not 0 < road_start_ratio < 1:
            raise ValueError("invalid grid_shape or road_start_ratio")
        self.grid_shape, self.road_start_ratio = grid_shape, road_start_ratio

    def detect(self, frame: np.ndarray) -> np.ndarray:
        if frame.ndim != 3 or frame.shape[2] not in (3, 4):
            raise ValueError("frame must be HxWx3 or HxWx4")
        h, w = frame.shape[:2]
        bgr = frame[:, :, :3].astype(np.int16)
        saturation_proxy = bgr.max(axis=2) - bgr.min(axis=2)
        brightness = bgr.mean(axis=2)
        # Indian asphalt/dirt generally has less colour than sky/foliage; dark regions stay blocked.
        free = (saturation_proxy < 145) & (brightness >= 35) & (brightness <= 235)
        free[:int(h * self.road_start_ratio), :] = False
        if cv2 is not None:
            mask = (free.astype(np.uint8) * 255)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            free = mask.astype(bool)
        return self._to_grid(free)

    def _to_grid(self, free: np.ndarray) -> np.ndarray:
        h, w = free.shape
        rows, cols = self.grid_shape
        row_edges = np.linspace(0, h, rows + 1, dtype=int)
        col_edges = np.linspace(0, w, cols + 1, dtype=int)
        grid = np.zeros((rows, cols), dtype=np.uint8)
        for r in range(rows):
            for c in range(cols):
                cell = free[row_edges[r]:row_edges[r + 1], col_edges[c]:col_edges[c + 1]]
                grid[r, c] = np.uint8(cell.size and cell.mean() >= .5)
        return grid
