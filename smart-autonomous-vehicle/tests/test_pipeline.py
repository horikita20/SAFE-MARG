"""
Comprehensive automated unit and scenario integration test suite for Python stack.
"""

import sys
from pathlib import Path
import numpy as np

# Add backend directory to sys.path
backend_dir = Path(__file__).resolve().parent.parent / "backend"
sys.path.insert(0, str(backend_dir))

from config import get_config
from perception.object_detector import detect_objects, get_default_radius, DetectedObject
from perception.drivable_area import detect_drivable_area
from perception.occupancy_grid import generate_occupancy_grid
from perception.perception_pipeline import perception_pipeline
from planning.global_path import generate_global_path
from planning.adaptive_planner import adaptive_path_planner
from planning.collision_checker import collision_checker
from planning.dynamic_avoidance import dynamic_obstacle_avoidance
from control.pure_pursuit import pure_pursuit_controller
from control.vehicle_controller import vehicle_controller
from control.kinematic_bicycle import vehicle_model
from scenarios.scenario_definitions import create_indian_road_scenario
from simulation.simulation_engine import SimulationEngine


def test_config():
    cfg = get_config()
    assert cfg.vehicle.wheelbase == 2.7
    assert cfg.grid.resolution == 0.25
    assert cfg.risk.ttc_critical == 1.8


def test_perception_and_occupancy_grid():
    cfg = get_config()
    scenario = create_indian_road_scenario(3, cfg)  # Cattle Crossing
    ego_pose = (0.0, 0.0, 0.0)

    perc = perception_pipeline(scenario, ego_pose, 5.0, 0.0, cfg)
    assert len(perc.objects) >= 1
    assert perc.occupancy_grid.grid.shape[0] > 10
    assert perc.occupancy_grid.grid.shape[1] > 10
    assert np.any(perc.occupancy_grid.grid >= 0.85)  # Obstacle present


def test_global_path():
    cfg = get_config()
    scenario = create_indian_road_scenario(1, cfg)
    drivable = detect_drivable_area(scenario, scenario.start_pose, cfg)
    g_path = generate_global_path(scenario.start_pose, scenario.goal_pose, drivable, cfg)

    assert len(g_path.waypoints) > 10
    assert g_path.length > 30.0
    assert g_path.valid is True


def test_adaptive_a_star_planner():
    cfg = get_config()
    scenario = create_indian_road_scenario(2, cfg)  # Potholes
    perc = perception_pipeline(scenario, (0.0, 0.0, 0.0), 5.0, 0.0, cfg)
    plan = adaptive_path_planner((0.0, 0.0, 0.0), scenario.goal_pose, perc.occupancy_grid, cfg)

    assert plan.valid is True
    assert len(plan.waypoints) >= 2


def test_collision_checker():
    cfg = get_config()

    dummy_cow = DetectedObject(
        id=1,
        object_class="cattle",
        position=(3.0, 0.0),
        world_position=(3.0, 0.0),
        velocity=(0.0, 0.0),
        world_velocity=(0.0, 0.0),
        distance=3.0,
        bbox=(2.0, -1.0, 2.0, 2.0),
        radius=1.1,
        confidence=0.95,
    )

    class DummyPath:
        waypoints = np.column_stack((np.linspace(0, 20, 40), np.zeros(40)))

    report = collision_checker((0.0, 0.0, 0.0), 6.0, DummyPath(), [dummy_cow], cfg)
    assert report.risk_level == "CRITICAL"
    assert report.needs_replan is True


def test_pure_pursuit_controller():
    cfg = get_config()
    waypoints = np.column_stack((np.linspace(0, 30, 60), np.full(60, 2.0)))
    steer_cmd, target_pt, cte = pure_pursuit_controller((0.0, 0.0, 0.0), 5.0, waypoints, cfg)

    assert steer_cmd > 0  # Should steer left toward y=2.0
    assert abs(cte - 2.0) < 0.2


def test_vehicle_model_dynamics():
    cfg = get_config()
    pose0 = (0.0, 0.0, 0.0)
    v0 = 5.0
    steer = 0.1
    accel = 1.0
    dt = 0.05

    pose1, v1, yaw_rate, vertices = vehicle_model(pose0, v0, steer, accel, dt, cfg)
    assert pose1[0] > pose0[0]  # Forward movement
    assert pose1[2] > pose0[2]  # Heading angle change
    assert v1 > v0
    assert len(vertices) == 4


def run_scenario_test(scenario_id: int):
    """Verifies that the given scenario completes with 0 collisions and reaches destination."""
    cfg = get_config()
    engine = SimulationEngine(scenario_id=scenario_id, cfg=cfg)
    metrics, _ = engine.run_full()

    assert metrics.goal_reached is True, f"Scenario {scenario_id} did not reach goal"
    assert metrics.collision_count == 0, f"Scenario {scenario_id} had {metrics.collision_count} collisions"
    assert metrics.min_obstacle_dist >= 1.0, f"Scenario {scenario_id} safe distance violated: {metrics.min_obstacle_dist}m"
    return metrics


if __name__ == "__main__":
    print("=" * 65)
    print("   SMART INDIA HACKATHON 2026 (SIH26037) - PYTHON TEST SUITE   ")
    print("=" * 65)

    test_config()
    print(" [PASS] Unit Test 1: Configuration Parameters")
    test_perception_and_occupancy_grid()
    print(" [PASS] Unit Test 2: Perception & Inflated Occupancy Grid")
    test_global_path()
    print(" [PASS] Unit Test 3: Global Reference Path Generator")
    test_adaptive_a_star_planner()
    print(" [PASS] Unit Test 4: Adaptive A* Motion Planner")
    test_collision_checker()
    print(" [PASS] Unit Test 5: Collision Risk Engine & TTC")
    test_pure_pursuit_controller()
    print(" [PASS] Unit Test 6: Pure Pursuit Steering Controller")
    test_vehicle_model_dynamics()
    print(" [PASS] Unit Test 7: Kinematic Bicycle Vehicle Dynamics")

    print("\n--- End-to-End Simulation Benchmarks (All 7 Scenarios) ---")
    for s_id in range(1, 8):
        m = run_scenario_test(s_id)
        print(f" [PASS] Scenario {s_id}: {m.scenario_name:<32} | Reached Goal: YES | Collisions: {m.collision_count} | Min Clear: {m.min_obstacle_dist:.2f}m")

    print("\n" + "=" * 65)
    print(" >>> ALL 7 SCENARIOS & SUBSYSTEM TESTS PASSED (100%) <<< ")
    print("=" * 65)
