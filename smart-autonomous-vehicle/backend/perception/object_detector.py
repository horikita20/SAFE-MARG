"""
Multi-class object detection module for Indian Road environments.
Simulates sensor detection with Field-Of-View (FOV) filtering, range limits, and realistic noise.
Ready for YOLO/ONNX integration in future phases.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional, Callable, Dict, Any
import numpy as np

try:
    from ..config import Config, get_config
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
    except (ImportError, ValueError):
        from config import Config, get_config


@dataclass
class DetectedObject:
    id: int
    object_class: str
    position: Tuple[float, float]       # [x, y] in ego vehicle frame (m, x=fwd, y=left)
    world_position: Tuple[float, float] # [xw, yw] in world coordinates (m)
    velocity: Tuple[float, float]       # [vx vy] in ego frame (m/s)
    world_velocity: Tuple[float, float] # [vxw, vyw] in world frame (m/s)
    distance: float                     # Euclidean distance from ego (m)
    bbox: Tuple[float, float, float, float] # [xMin, yMin, width, length] (m)
    radius: float                       # Enclosing radius for collision checking (m)
    confidence: float                   # Detection confidence score [0, 1]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "class": self.object_class,
            "position": [round(self.position[0], 2), round(self.position[1], 2)],
            "world_position": [round(self.world_position[0], 2), round(self.world_position[1], 2)],
            "velocity": [round(self.velocity[0], 2), round(self.velocity[1], 2)],
            "world_velocity": [round(self.world_velocity[0], 2), round(self.world_velocity[1], 2)],
            "distance": round(self.distance, 2),
            "bbox": [round(b, 2) for b in self.bbox],
            "radius": round(self.radius, 2),
            "confidence": round(self.confidence, 2),
        }


def get_default_radius(object_class: str) -> float:
    radii = {
        "pedestrian": 0.45,
        "cattle": 1.10,
        "auto-rickshaw": 1.20,
        "vehicle": 1.50,
        "motorcycle": 0.60,
        "pothole": 0.70,
        "debris": 0.50,
    }
    return radii.get(object_class.lower(), 0.80)


def detect_objects(
    scenario_actors: List[Any],
    ego_pose: Tuple[float, float, float],
    cfg: Optional[Config] = None,
    t: float = 0.0,
) -> List[DetectedObject]:
    """
    Detects objects within sensor field of view and range relative to ego vehicle.
    """
    if cfg is None:
        cfg = get_config()

    ego_x, ego_y, ego_theta = ego_pose
    max_range = cfg.sensors.max_range
    fov_angle = cfg.sensors.fov_angle
    noise_pos = cfg.sensors.pos_noise_std
    noise_vel = cfg.sensors.vel_noise_std

    cos_t = np.cos(ego_theta)
    sin_t = np.sin(ego_theta)
    r_world_to_ego = np.array([[cos_t, sin_t], [-sin_t, cos_t]])

    detected_objects: List[DetectedObject] = []

    for actor in scenario_actors:
        if hasattr(actor, "trajectory_func") and actor.trajectory_func is not None:
            world_pos, world_vel = actor.trajectory_func(t)
        elif hasattr(actor, "position"):
            world_pos = actor.position
            world_vel = getattr(actor, "velocity", (0.0, 0.0))
        elif isinstance(actor, dict):
            if "trajectory_func" in actor and actor["trajectory_func"]:
                world_pos, world_vel = actor["trajectory_func"](t)
            else:
                world_pos = actor["position"]
                world_vel = actor.get("velocity", (0.0, 0.0))
        else:
            continue

        world_pos = np.array(world_pos, dtype=float)
        world_vel = np.array(world_vel, dtype=float)

        rel_pos_world = world_pos - np.array([ego_x, ego_y])
        rel_pos_ego = r_world_to_ego @ rel_pos_world

        dist = float(np.linalg.norm(rel_pos_ego))
        bearing = float(np.arctan2(rel_pos_ego[1], rel_pos_ego[0]))

        actor_class = getattr(actor, "object_class", getattr(actor, "class_name", "unknown")) if not isinstance(actor, dict) else actor.get("class", "unknown")
        actor_id = getattr(actor, "id", 0) if not isinstance(actor, dict) else actor.get("id", 0)
        actor_radius = getattr(actor, "radius", None) if not isinstance(actor, dict) else actor.get("radius", None)

        if actor_radius is None:
            actor_radius = get_default_radius(actor_class)

        is_pothole = str(actor_class).lower() == "pothole"
        eff_max_range = min(20.0, max_range) if is_pothole else max_range

        if dist <= eff_max_range and abs(bearing) <= (fov_angle / 2.0):
            noisy_pos = rel_pos_ego + np.random.randn(2) * noise_pos
            rel_vel_ego = (r_world_to_ego @ world_vel) + np.random.randn(2) * noise_vel
            if is_pothole or float(np.linalg.norm(world_vel)) < 0.05:
                rel_vel_ego = np.array([0.0, 0.0])

            confidence = max(0.6, min(1.0, 1.0 - (dist / (eff_max_range * 1.5))))
            bbox = (
                float(world_pos[0] - actor_radius),
                float(world_pos[1] - actor_radius),
                float(2 * actor_radius),
                float(2 * actor_radius),
            )

            detected_objects.append(
                DetectedObject(
                    id=int(actor_id),
                    object_class=str(actor_class),
                    position=(float(noisy_pos[0]), float(noisy_pos[1])),
                    world_position=(float(world_pos[0]), float(world_pos[1])),
                    velocity=(float(rel_vel_ego[0]), float(rel_vel_ego[1])),
                    world_velocity=(float(world_vel[0]), float(world_vel[1])),
                    distance=dist,
                    bbox=bbox,
                    radius=float(actor_radius),
                    confidence=float(confidence),
                )
            )

    return detected_objects
