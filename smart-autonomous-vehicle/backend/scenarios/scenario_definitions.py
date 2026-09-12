"""
7 Realistic Indian Road Driving Scenarios for SIH26037.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Callable, Optional, Dict, Any
import numpy as np

try:
    from ..config import Config, get_config
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
    except (ImportError, ValueError):
        from config import Config, get_config


@dataclass
class Actor:
    id: int
    object_class: str
    position: Tuple[float, float]
    velocity: Tuple[float, float] = (0.0, 0.0)
    radius: float = 0.8
    trajectory_func: Optional[Callable[[float], Tuple[Tuple[float, float], Tuple[float, float]]]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "class": self.object_class,
            "position": [round(self.position[0], 2), round(self.position[1], 2)],
            "velocity": [round(self.velocity[0], 2), round(self.velocity[1], 2)],
            "radius": round(self.radius, 2),
        }


@dataclass
class RoadGeometry:
    left_bound_func: Callable[[np.ndarray], np.ndarray]
    right_bound_func: Callable[[np.ndarray], np.ndarray]
    surface_type: str = "patchy_rural"


@dataclass
class Scenario:
    id: int
    name: str
    description: str
    start_pose: Tuple[float, float, float]
    goal_pose: Tuple[float, float, float]
    road: RoadGeometry
    actors: List[Actor] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "start_pose": [round(p, 2) for p in self.start_pose],
            "goal_pose": [round(p, 2) for p in self.goal_pose],
            "surface_type": self.road.surface_type,
            "num_actors": len(self.actors),
        }


# =========================================================================
# Scenario 1: Narrow Unmarked Rural Road
# =========================================================================
def scenario_narrow_road(cfg: Config) -> Scenario:
    left_bound = lambda x: 3.25 - 1.0 * ((x >= 25) & (x <= 45))
    right_bound = lambda x: -3.25 + 1.0 * ((x >= 25) & (x <= 45))

    actors = [
        Actor(id=101, object_class="debris", position=(22.0, 1.8), velocity=(0.0, 0.0), radius=0.8),
        Actor(id=102, object_class="debris", position=(35.0, -1.8), velocity=(0.0, 0.0), radius=0.7),
        Actor(id=103, object_class="vehicle", position=(48.0, 1.6), velocity=(0.0, 0.0), radius=0.6),
    ]

    return Scenario(
        id=1,
        name="Narrow Unmarked Rural Road",
        description="Narrow rural road with no lane markings, shoulder debris, and a bottleneck.",
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(65.0, 0.0, 0.0),
        road=RoadGeometry(left_bound, right_bound, "unpaved_gravel"),
        actors=actors,
    )


# =========================================================================
# Scenario 2: Potholes on Path
# =========================================================================
def scenario_pothole(cfg: Config) -> Scenario:
    left_bound = lambda x: 3.5 * np.ones_like(x)
    right_bound = lambda x: -3.5 * np.ones_like(x)

    actors = [
        Actor(id=201, object_class="pothole", position=(20.0, -0.4), velocity=(0.0, 0.0), radius=0.75),
        Actor(id=202, object_class="pothole", position=(36.0, 0.5), velocity=(0.0, 0.0), radius=0.85),
        Actor(id=203, object_class="pothole", position=(50.0, -0.8), velocity=(0.0, 0.0), radius=0.70),
    ]

    return Scenario(
        id=2,
        name="Potholes on Driving Path",
        description="Deep potholes situated on the vehicle centerline requiring lateral avoidance.",
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(65.0, 0.0, 0.0),
        road=RoadGeometry(left_bound, right_bound, "potholed_asphalt"),
        actors=actors,
    )


# =========================================================================
# Scenario 3: Cattle Crossing Road (Flagship SIH Demo)
# =========================================================================
def scenario_cattle(cfg: Config) -> Scenario:
    left_bound = lambda x: 3.8 * np.ones_like(x)
    right_bound = lambda x: -3.8 * np.ones_like(x)

    cattle_start_x = 28.0
    cattle_start_y = -3.5
    cattle_speed_y = 0.65

    def cow_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        curr_y = min(4.2, cattle_start_y + cattle_speed_y * t)
        curr_vy = cattle_speed_y if (cattle_start_y + cattle_speed_y * t < 4.2) else 0.0
        return (cattle_start_x + 0.05 * t, curr_y), (0.05, curr_vy)

    actors = [
        Actor(
            id=301,
            object_class="cattle",
            position=(cattle_start_x, cattle_start_y),
            velocity=(0.05, cattle_speed_y),
            radius=1.1,
            trajectory_func=cow_traj,
        ),
        Actor(id=302, object_class="debris", position=(55.0, 2.0), velocity=(0.0, 0.0), radius=0.7),
    ]

    return Scenario(
        id=3,
        name="Cattle Crossing Road (SIH Flagship Demo)",
        description="A stray cow slowly walks across the road from the right shoulder, forcing risk escalation & replanning.",
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(70.0, 0.0, 0.0),
        road=RoadGeometry(left_bound, right_bound, "patchy_rural"),
        actors=actors,
    )


# =========================================================================
# Scenario 4: Pedestrian Crossing
# =========================================================================
def scenario_pedestrian(cfg: Config) -> Scenario:
    left_bound = lambda x: 3.6 * np.ones_like(x)
    right_bound = lambda x: -3.6 * np.ones_like(x)

    ped_x = 25.0
    ped_speed = -1.2

    def ped_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        dt_walk = max(0.0, t - 1.0)
        curr_y = max(-4.2, 3.4 + ped_speed * dt_walk)
        curr_vy = ped_speed if (t >= 1.0 and (3.4 + ped_speed * dt_walk > -4.2)) else 0.0
        return (ped_x, curr_y), (0.0, curr_vy)

    actors = [
        Actor(id=401, object_class="debris", position=(22.0, 2.5), velocity=(0.0, 0.0), radius=0.8),
        Actor(
            id=402,
            object_class="pedestrian",
            position=(ped_x, 3.4),
            velocity=(0.0, ped_speed),
            radius=0.5,
            trajectory_func=ped_traj,
        ),
    ]

    return Scenario(
        id=4,
        name="Pedestrian Entering Road",
        description="A pedestrian steps out from behind a parked cart into the vehicle path at x=25m.",
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(65.0, 0.0, 0.0),
        road=RoadGeometry(left_bound, right_bound, "asphalt"),
        actors=actors,
    )


# =========================================================================
# Scenario 5: Auto-Rickshaw Obstruction
# =========================================================================
def scenario_auto_rickshaw(cfg: Config) -> Scenario:
    left_bound = lambda x: 4.0 * np.ones_like(x)
    right_bound = lambda x: -4.0 * np.ones_like(x)

    rickshaw_x0 = 18.0
    def rickshaw_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        curr_x = min(34.0, rickshaw_x0 + 2.0 * t)
        curr_vx = 2.0 if (rickshaw_x0 + 2.0 * t < 34.0) else 0.0
        return (curr_x, 0.2), (curr_vx, 0.0)

    def bike_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        curr_x = max(0.0, 60.0 - 4.5 * t)
        return (curr_x, 2.6), (-4.5, 0.0)

    actors = [
        Actor(id=501, object_class="auto-rickshaw", position=(rickshaw_x0, 0.2), velocity=(2.0, 0.0), radius=1.2, trajectory_func=rickshaw_traj),
        Actor(id=502, object_class="motorcycle", position=(60.0, 2.6), velocity=(-4.5, 0.0), radius=0.6, trajectory_func=bike_traj),
    ]

    return Scenario(
        id=5,
        name="Auto-Rickshaw Obstruction",
        description="A 3-wheeled auto-rickshaw stops abruptly in the center of the lane with an oncoming motorcycle.",
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(70.0, 0.0, 0.0),
        road=RoadGeometry(left_bound, right_bound, "asphalt"),
        actors=actors,
    )


# =========================================================================
# Scenario 6: Unsignalized Intersection
# =========================================================================
def scenario_intersection(cfg: Config) -> Scenario:
    left_bound = lambda x: 3.5 + 4.0 * ((x >= 28) & (x <= 44))
    right_bound = lambda x: -3.5 - 4.0 * ((x >= 28) & (x <= 44))

    def moto_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        curr_y = max(-7.0, 6.0 - 3.2 * t)
        curr_vy = -3.2 if (6.0 - 3.2 * t > -7.0) else 0.0
        return (35.0, curr_y), (0.0, curr_vy)

    actors = [
        Actor(id=601, object_class="motorcycle", position=(35.0, 6.0), velocity=(0.0, -3.2), radius=0.65, trajectory_func=moto_traj),
        Actor(id=602, object_class="vehicle", position=(42.0, -4.5), velocity=(0.0, 0.0), radius=1.4),
    ]

    return Scenario(
        id=6,
        name="Unsignalized Intersection",
        description="Unsignalized rural intersection where lateral cross-traffic cuts through without right-of-way.",
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(70.0, 0.0, 0.0),
        road=RoadGeometry(left_bound, right_bound, "asphalt"),
        actors=actors,
    )


# =========================================================================
# Scenario 7: Mixed Chaotic Traffic
# =========================================================================
def scenario_mixed_traffic(cfg: Config) -> Scenario:
    left_bound = lambda x: 4.2 * np.ones_like(x)
    right_bound = lambda x: -4.2 * np.ones_like(x)

    def cow_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        curr_y = min(4.5, -3.8 + 0.65 * t)
        curr_vy = 0.65 if (-3.8 + 0.65 * t < 4.5) else 0.0
        return (30.0 + 0.05 * t, curr_y), (0.05, curr_vy)

    def auto_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        curr_x = min(60.0, 38.0 + 1.8 * t)
        curr_vx = 1.8 if (38.0 + 1.8 * t < 60.0) else 0.0
        return (curr_x, -1.2), (curr_vx, 0.0)

    def ped_traj(t: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        dt_p = max(0.0, t - 3.0)
        curr_y = max(-4.5, 3.8 - 1.1 * dt_p)
        curr_vy = -1.1 if (t >= 3.0 and (3.8 - 1.1 * dt_p > -4.5)) else 0.0
        return (55.0, curr_y), (0.0, curr_vy)

    actors = [
        Actor(id=701, object_class="pothole", position=(16.0, -0.6), velocity=(0.0, 0.0), radius=0.75),
        Actor(id=702, object_class="cattle", position=(30.0, -3.8), velocity=(0.05, 0.65), radius=1.1, trajectory_func=cow_traj),
        Actor(id=703, object_class="auto-rickshaw", position=(38.0, -1.2), velocity=(1.8, 0.0), radius=1.2, trajectory_func=auto_traj),
        Actor(id=704, object_class="pedestrian", position=(55.0, 3.8), velocity=(0.0, -1.1), radius=0.5, trajectory_func=ped_traj),
    ]

    return Scenario(
        id=7,
        name="Chaotic Mixed Indian Traffic",
        description="Multiple concurrent hazards: walking cattle, sudden pedestrian, auto-rickshaw, and potholes.",
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(75.0, 0.0, 0.0),
        road=RoadGeometry(left_bound, right_bound, "patchy_rural"),
        actors=actors,
    )


def create_indian_road_scenario(scenario_id: int = 3, cfg: Optional[Config] = None) -> Scenario:
    if cfg is None:
        cfg = get_config()

    scenarios = {
        1: scenario_narrow_road,
        2: scenario_pothole,
        3: scenario_cattle,
        4: scenario_pedestrian,
        5: scenario_auto_rickshaw,
        6: scenario_intersection,
        7: scenario_mixed_traffic,
    }
    return scenarios.get(int(scenario_id), scenario_cattle)(cfg)


def get_all_scenario_metadata(cfg: Optional[Config] = None) -> List[Dict[str, Any]]:
    if cfg is None:
        cfg = get_config()
    return [create_indian_road_scenario(i, cfg).to_dict() for i in range(1, 8)]
