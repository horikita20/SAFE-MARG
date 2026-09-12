"""
Master perception pipeline combining detection, road margins, and occupancy grid.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional, Any, Dict

try:
    from ..config import Config, get_config
    from .object_detector import detect_objects, DetectedObject
    from .drivable_area import detect_drivable_area, DrivableArea
    from .occupancy_grid import generate_occupancy_grid, OccupancyGrid
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
        from backend.perception.object_detector import detect_objects, DetectedObject
        from backend.perception.drivable_area import detect_drivable_area, DrivableArea
        from backend.perception.occupancy_grid import generate_occupancy_grid, OccupancyGrid
    except (ImportError, ValueError):
        from config import Config, get_config
        from perception.object_detector import detect_objects, DetectedObject
        from perception.drivable_area import detect_drivable_area, DrivableArea
        from perception.occupancy_grid import generate_occupancy_grid, OccupancyGrid


@dataclass
class PerceptionOutput:
    timestamp: float
    objects: List[DetectedObject]
    drivable_area: DrivableArea
    occupancy_grid: OccupancyGrid
    ego_pose: Tuple[float, float, float]
    ego_velocity: float
    nearest_obstacle: Optional[DetectedObject]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": round(self.timestamp, 2),
            "objects": [obj.to_dict() for obj in self.objects],
            "drivable_area": self.drivable_area.to_dict(),
            "occupancy_grid": self.occupancy_grid.to_dict(),
            "ego_pose": [round(p, 3) for p in self.ego_pose],
            "ego_velocity": round(self.ego_velocity, 2),
            "nearest_obstacle": self.nearest_obstacle.to_dict() if self.nearest_obstacle else None,
        }


def perception_pipeline(
    scenario: Any,
    ego_pose: Tuple[float, float, float],
    ego_velocity: float,
    t: float = 0.0,
    cfg: Optional[Config] = None,
) -> PerceptionOutput:
    """
    Executes the full perception stack for a single timestep.
    """
    if cfg is None:
        cfg = get_config()

    actors = getattr(scenario, "actors", []) if not isinstance(scenario, dict) else scenario.get("actors", [])

    detected_objects = detect_objects(actors, ego_pose, cfg, t)
    drivable_area = detect_drivable_area(scenario, ego_pose, cfg)
    occupancy_grid = generate_occupancy_grid(detected_objects, drivable_area, cfg, ego_pose)

    nearest_obstacle: Optional[DetectedObject] = None
    if detected_objects:
        nearest_obstacle = min(detected_objects, key=lambda obj: obj.distance)

    return PerceptionOutput(
        timestamp=t,
        objects=detected_objects,
        drivable_area=drivable_area,
        occupancy_grid=occupancy_grid,
        ego_pose=ego_pose,
        ego_velocity=ego_velocity,
        nearest_obstacle=nearest_obstacle,
    )
