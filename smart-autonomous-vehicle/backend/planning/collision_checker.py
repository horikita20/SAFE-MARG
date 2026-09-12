"""
Collision Risk Engine & Time-To-Collision (TTC) Evaluator.
"""

from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict, Any
import numpy as np

try:
    from ..config import Config, get_config
    from ..perception.object_detector import DetectedObject
    from .adaptive_planner import PlannedPath
    from .global_path import GlobalPath
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
        from backend.perception.object_detector import DetectedObject
        from backend.planning.adaptive_planner import PlannedPath
        from backend.planning.global_path import GlobalPath
    except (ImportError, ValueError):
        from config import Config, get_config
        from perception.object_detector import DetectedObject
        from planning.adaptive_planner import PlannedPath
        from planning.global_path import GlobalPath


@dataclass
class CollisionReport:
    risk_level: str                 # "SAFE" | "WARNING" | "CRITICAL"
    min_distance: float             # Proximity to nearest hazard (m)
    min_ttc: float                  # Minimum Time-To-Collision (s) (inf if none)
    rel_velocity: float             # Relative closing velocity (m/s)
    critical_obstacle: Optional[DetectedObject]
    needs_replan: bool              # Flag to trigger replanning
    reason: str                     # Human-readable explanation string

    def to_dict(self) -> Dict[str, Any]:
        return {
            "risk_level": self.risk_level,
            "min_distance": round(self.min_distance, 2) if not np.isinf(self.min_distance) else 999.0,
            "min_ttc": round(self.min_ttc, 2) if not np.isinf(self.min_ttc) else 999.0,
            "rel_velocity": round(self.rel_velocity, 2),
            "critical_obstacle": self.critical_obstacle.to_dict() if self.critical_obstacle else None,
            "needs_replan": self.needs_replan,
            "reason": self.reason,
        }


def collision_checker(
    current_pose: Tuple[float, float, float],
    current_velocity: float,
    current_path: Any,
    detected_objects: List[DetectedObject],
    cfg: Optional[Config] = None,
) -> CollisionReport:
    """
    Assesses collision risk, calculates TTC, and identifies whether path replanning is needed.
    """
    if cfg is None:
        cfg = get_config()

    ttc_crit = cfg.risk.ttc_critical
    ttc_warn = cfg.risk.ttc_warning
    dist_crit = cfg.risk.dist_critical
    dist_warn = cfg.risk.dist_warning
    corridor_w = cfg.risk.lateral_clearance

    if not detected_objects or current_path is None or len(getattr(current_path, "waypoints", [])) == 0:
        return CollisionReport(
            risk_level="SAFE",
            min_distance=float("inf"),
            min_ttc=float("inf"),
            rel_velocity=0.0,
            critical_obstacle=None,
            needs_replan=False,
            reason="Path is clear and safe.",
        )

    ego_pos = np.array(current_pose[:2], dtype=float)
    ego_theta = current_pose[2]
    ego_vel_vec = np.array([current_velocity * np.cos(ego_theta), current_velocity * np.sin(ego_theta)])

    waypoints = current_path.waypoints
    min_dist = float("inf")
    min_ttc = float("inf")
    closing_speed = 0.0
    highest_risk = "SAFE"
    critical_obj: Optional[DetectedObject] = None
    needs_replan = False
    reason_str = "Path clear."

    for obj in detected_objects:
        obs_pos = np.array(obj.world_position, dtype=float)
        obs_vel = np.array(obj.world_velocity, dtype=float)
        obs_radius = obj.radius

        dist = float(np.linalg.norm(obs_pos - ego_pos))
        if dist < min_dist:
            min_dist = dist

        rel_pos = obs_pos - ego_pos
        rel_vel = ego_vel_vec - obs_vel
        curr_closing_speed = float(np.dot(rel_pos, rel_vel) / (dist + 1e-5))

        ttc = float("inf")
        if curr_closing_speed > 0.1:
            ttc = max(0.0, (dist - (obs_radius + cfg.vehicle.length / 2.0)) / curr_closing_speed)

        if ttc < min_ttc:
            min_ttc = ttc
            closing_speed = curr_closing_speed

        # Check if obstacle intersects planned corridor ahead
        path_intersects = False
        for wp in waypoints[:40]:
            dist_wp_to_ego = float(np.linalg.norm(wp - ego_pos))
            if dist_wp_to_ego > 25.0:
                break
            if float(np.linalg.norm(wp - obs_pos)) <= (obs_radius + corridor_w):
                path_intersects = True
                break

        # Risk state assignment
        obj_risk = "SAFE"
        if (ttc <= ttc_crit) or (path_intersects and dist <= (dist_crit * 1.5)):
            obj_risk = "CRITICAL"
            needs_replan = True
        elif dist <= dist_crit and path_intersects:
            obj_risk = "CRITICAL"
            needs_replan = True
        elif dist <= dist_warn or ttc <= ttc_warn or path_intersects:
            obj_risk = "WARNING"
            if path_intersects:
                needs_replan = True

        # If vehicle has stopped and detour path is clear around obstacle, allow crawl bypass
        if current_velocity < 0.3 and not path_intersects and obj_risk == "CRITICAL":
            obj_risk = "WARNING"

        if obj_risk == "CRITICAL":
            highest_risk = "CRITICAL"
            critical_obj = obj
            closing_speed = curr_closing_speed
            reason_str = f"{obj.object_class.upper()} detected {dist:.1f}m ahead (TTC: {min(99.9, ttc):.1f}s) - Collision imminent!"
            break
        elif obj_risk == "WARNING" and highest_risk != "CRITICAL":
            highest_risk = "WARNING"
            critical_obj = obj
            closing_speed = curr_closing_speed
            reason_str = f"{obj.object_class.upper()} in proximity ({dist:.1f}m, TTC: {min(99.9, ttc):.1f}s) - Caution required."

    return CollisionReport(
        risk_level=highest_risk,
        min_distance=min_dist,
        min_ttc=min_ttc,
        rel_velocity=closing_speed,
        critical_obstacle=critical_obj,
        needs_replan=needs_replan,
        reason=reason_str,
    )
