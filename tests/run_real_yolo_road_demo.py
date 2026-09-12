"""Run the Safe Marg wrapper against a public street image and real YOLOv8n.

The first run downloads Ultralytics' public ``bus.jpg`` road-scene sample and
the official pretrained ``yolov8n.pt`` weights. Neither is committed to source.
"""
from __future__ import annotations
import sys
from pathlib import Path
from urllib.request import urlretrieve

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2
from backend.perception.detect_obstacles import ObstacleDetector

image_path = ROOT / "data" / "bus.jpg"
image_path.parent.mkdir(exist_ok=True)
if not image_path.exists():
    urlretrieve("https://ultralytics.com/images/bus.jpg", image_path)

frame = cv2.imread(str(image_path))
if frame is None:
    raise RuntimeError(f"Could not decode {image_path}")

obstacles = ObstacleDetector(confidence_threshold=0.35).detect(frame)
print("REAL YOLOv8 ROAD DEMO: PASS")
print(f"image={image_path.name} shape={frame.shape}")
for obstacle in obstacles:
    print(f"id={obstacle.id} class={obstacle.obj_class} confidence={obstacle.confidence:.3f} "
          f"bbox={tuple(round(v, 1) for v in obstacle.bbox)} "
          f"position_m={tuple(round(v, 2) for v in obstacle.position)} distance_m={obstacle.distance:.2f}")
