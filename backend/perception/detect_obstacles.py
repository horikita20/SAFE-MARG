"""YOLO-compatible obstacle detection and image-to-ego projection."""
from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any, Iterable, Protocol
import numpy as np
from backend.models import Obstacle, ObstacleClass

CLASS_MAP: dict[str, ObstacleClass] = {
    "person": "pedestrian", "pedestrian": "pedestrian", "car": "vehicle",
    "bus": "vehicle", "truck": "vehicle", "motorcycle": "vehicle",
    "bicycle": "vehicle", "rickshaw": "vehicle", "auto-rickshaw": "vehicle",
    "cow": "animal", "dog": "animal", "cat": "animal", "horse": "animal",
    "pothole": "pothole", "debris": "debris",
}

@dataclass(frozen=True, slots=True)
class RawDetection:
    class_name: str
    confidence: float
    xyxy: tuple[float, float, float, float]

class Detector(Protocol):
    def __call__(self, frame: np.ndarray) -> Iterable[RawDetection]: ...

class ObstacleDetector:
    """Normalises an injected detector or Ultralytics YOLO into ``Obstacle`` values."""
    def __init__(self, model: Detector | Any | None = None, *, confidence_threshold: float = .35,
                 focal_length_px: float = 700., camera_height_m: float = 1.4) -> None:
        if not 0 <= confidence_threshold <= 1 or focal_length_px <= 0 or camera_height_m <= 0:
            raise ValueError("invalid detector calibration or confidence threshold")
        self.model, self.confidence_threshold = model, confidence_threshold
        self.focal_length_px, self.camera_height_m = focal_length_px, camera_height_m

    def detect(self, frame: np.ndarray) -> list[Obstacle]:
        if frame.ndim != 3 or frame.shape[2] not in (3, 4):
            raise ValueError("frame must be HxWx3 or HxWx4")
        obstacles: list[Obstacle] = []
        for item in self._run_model(frame):
            x1, y1, x2, y2 = item.xyxy
            if item.confidence < self.confidence_threshold or x2 <= x1 or y2 <= y1:
                continue
            bbox = (float(x1), float(y1), float(x2 - x1), float(y2 - y1))
            position = self._project_to_ego(bbox, frame.shape[:2])
            obstacles.append(Obstacle(len(obstacles), CLASS_MAP.get(item.class_name.lower(), "unknown"),
                bbox, position, (0., 0.), hypot(*position), float(item.confidence)))
        return obstacles

    def _run_model(self, frame: np.ndarray) -> list[RawDetection]:
        if self.model is None:
            try:
                from ultralytics import YOLO
            except ImportError as error:
                raise RuntimeError("Live detection needs 'ultralytics'; inject a detector for offline testing.") from error
            self.model = YOLO("yolov8n.pt")
        if callable(self.model) and not hasattr(self.model, "predict"):
            return list(self.model(frame))
        result = self.model.predict(frame, verbose=False)[0]
        return [RawDetection(str(result.names[int(c)]), float(s), tuple(map(float, box)))
                for box, s, c in zip(result.boxes.xyxy.cpu().tolist(), result.boxes.conf.cpu().tolist(), result.boxes.cls.cpu().tolist())]

    def _project_to_ego(self, bbox: tuple[float, float, float, float], frame_shape: tuple[int, int]) -> tuple[float, float]:
        height, width = frame_shape
        x, y, w, h = bbox
        below_horizon = max(1., min(height - 1., y + h) - height / 2.)
        forward = self.camera_height_m * self.focal_length_px / below_horizon
        lateral = ((x + w / 2.) - width / 2.) * forward / self.focal_length_px
        return float(forward), float(lateral)
