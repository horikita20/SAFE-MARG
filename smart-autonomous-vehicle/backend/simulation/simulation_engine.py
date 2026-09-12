"""
Closed-Loop Autonomous Vehicle Simulation Engine for SIH26037.
Pipeline:
  Perception -> Occupancy Grid -> Risk Assessment -> Adaptive Replanning -> Pure Pursuit -> Kinematic Model
"""

from dataclasses import dataclass, field
from typing import Tuple, List, Optional, Dict, Any
import numpy as np

try:
    from ..config import Config, get_config
    from ..scenarios.scenario_definitions import create_indian_road_scenario, Scenario
    from ..perception.perception_pipeline import perception_pipeline, PerceptionOutput
    from ..planning.global_path import generate_global_path, GlobalPath
    from ..planning.adaptive_planner import adaptive_path_planner, PlannedPath
    from ..planning.collision_checker import collision_checker, CollisionReport
    from ..planning.dynamic_avoidance import dynamic_obstacle_avoidance
    from ..control.pure_pursuit import pure_pursuit_controller
    from ..control.vehicle_controller import vehicle_controller
    from ..control.kinematic_bicycle import vehicle_model
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
        from backend.scenarios.scenario_definitions import create_indian_road_scenario, Scenario
        from backend.perception.perception_pipeline import perception_pipeline, PerceptionOutput
        from backend.planning.global_path import generate_global_path, GlobalPath
        from backend.planning.adaptive_planner import adaptive_path_planner, PlannedPath
        from backend.planning.collision_checker import collision_checker, CollisionReport
        from backend.planning.dynamic_avoidance import dynamic_obstacle_avoidance
        from backend.control.pure_pursuit import pure_pursuit_controller
        from backend.control.vehicle_controller import vehicle_controller
        from backend.control.kinematic_bicycle import vehicle_model
    except (ImportError, ValueError):
        from config import Config, get_config
        from scenarios.scenario_definitions import create_indian_road_scenario, Scenario
        from perception.perception_pipeline import perception_pipeline, PerceptionOutput
        from planning.global_path import generate_global_path, GlobalPath
        from planning.adaptive_planner import adaptive_path_planner, PlannedPath
        from planning.collision_checker import collision_checker, CollisionReport
        from planning.dynamic_avoidance import dynamic_obstacle_avoidance
        from control.pure_pursuit import pure_pursuit_controller
        from control.vehicle_controller import vehicle_controller
        from control.kinematic_bicycle import vehicle_model


@dataclass
class SimulationState:
    t: float
    sim_step: int
    ego_pose: Tuple[float, float, float]
    ego_velocity: float
    steer_angle_cmd: float
    accel_cmd: float
    footprint_vertices: List[Tuple[float, float]]
    perception: PerceptionOutput
    collision_report: CollisionReport
    active_path: Any
    global_path: GlobalPath
    target_lookahead_point: Tuple[float, float]
    cross_track_error: float
    replan_count: int
    collision_count: int
    goal_reached: bool
    dist_to_goal: float
    trajectory_history: List[Tuple[float, float]]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": round(self.t, 2),
            "sim_step": self.sim_step,
            "ego_pose": [round(p, 3) for p in self.ego_pose],
            "ego_velocity": round(self.ego_velocity, 2),
            "speed_kmh": round(self.ego_velocity * 3.6, 1),
            "steer_angle_deg": round(float(np.rad2deg(self.steer_angle_cmd)), 2),
            "accel_cmd": round(self.accel_cmd, 2),
            "footprint_vertices": [[round(pt[0], 2), round(pt[1], 2)] for pt in self.footprint_vertices],
            "perception": self.perception.to_dict(),
            "collision_report": self.collision_report.to_dict(),
            "active_path": self.active_path.to_dict() if hasattr(self.active_path, "to_dict") else None,
            "global_path": self.global_path.to_dict(),
            "target_lookahead": [round(self.target_lookahead_point[0], 2), round(self.target_lookahead_point[1], 2)],
            "cross_track_error": round(self.cross_track_error, 3),
            "replan_count": self.replan_count,
            "collision_count": self.collision_count,
            "goal_reached": self.goal_reached,
            "dist_to_goal": round(self.dist_to_goal, 2),
            "trajectory_history": [[round(pt[0], 2), round(pt[1], 2)] for pt in self.trajectory_history[::2]],
        }


@dataclass
class PerformanceMetrics:
    scenario_id: int
    scenario_name: str
    goal_reached: bool
    collision_count: int
    min_obstacle_dist: float
    path_length: float
    replan_count: int
    avg_speed_kmh: float
    mean_cross_track_err: float
    sim_duration_sec: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "scenario_name": self.scenario_name,
            "goal_reached": self.goal_reached,
            "collision_count": self.collision_count,
            "min_obstacle_dist": round(self.min_obstacle_dist, 2),
            "path_length": round(self.path_length, 2),
            "replan_count": self.replan_count,
            "avg_speed_kmh": round(self.avg_speed_kmh, 2),
            "mean_cross_track_err": round(self.mean_cross_track_err, 3),
            "sim_duration_sec": round(self.sim_duration_sec, 2),
        }


class SimulationEngine:
    def __init__(self, scenario_id: int = 3, cfg: Optional[Config] = None):
        self.cfg = cfg if cfg is not None else get_config()
        self.scenario_id = scenario_id
        self.scenario = create_indian_road_scenario(scenario_id, self.cfg)
        self.reset()

    def reset(self, scenario_id: Optional[int] = None):
        if scenario_id is not None:
            self.scenario_id = scenario_id
            self.scenario = create_indian_road_scenario(scenario_id, self.cfg)

        self.t = 0.0
        self.sim_step = 0
        self.last_replan_time = -1.0
        self.current_pose = self.scenario.start_pose
        self.current_velocity = 0.0
        self.steer_angle_cmd = 0.0
        self.accel_cmd = 0.0
        self.footprint_vertices = []

        self.global_path = generate_global_path(
            self.scenario.start_pose, self.scenario.goal_pose, None, self.cfg
        )
        self.active_path = self.global_path

        self.trajectory_history = [self.current_pose[:2]]
        self.replan_count = 0
        self.collision_count = 0
        self.min_obstacle_dist = float("inf")
        self.goal_reached = False
        self.time_log = []
        self.vel_log = []
        self.cross_track_log = []

    def step(self) -> SimulationState:
        dt = self.cfg.sim.dt
        self.sim_step += 1
        self.t = (self.sim_step - 1) * dt

        dist_to_goal = float(np.linalg.norm(np.array(self.current_pose[:2]) - np.array(self.scenario.goal_pose[:2])))
        if dist_to_goal <= self.cfg.sim.goal_radius:
            self.goal_reached = True

        perception_out = perception_pipeline(
            self.scenario, self.current_pose, self.current_velocity, self.t, self.cfg
        )
        detected_objects = perception_out.objects
        occ_grid = perception_out.occupancy_grid

        collision_report = collision_checker(
            self.current_pose, self.current_velocity, self.active_path, detected_objects, self.cfg
        )
        risk_level = collision_report.risk_level

        if collision_report.min_distance < self.min_obstacle_dist:
            self.min_obstacle_dist = collision_report.min_distance

        if collision_report.min_distance < 0.9:
            self.collision_count += 1

        # Replan with hysteresis (max once every 0.3s)
        if collision_report.needs_replan and not self.goal_reached and (self.t - self.last_replan_time >= 0.3):
            self.replan_count += 1
            self.last_replan_time = self.t
            avoid_path = dynamic_obstacle_avoidance(
                self.active_path, self.current_pose, self.current_velocity, detected_objects, occ_grid, self.cfg
            )
            test_rep = collision_checker(
                self.current_pose, self.current_velocity, avoid_path, detected_objects, self.cfg
            )
            if test_rep.risk_level == "CRITICAL":
                new_plan = adaptive_path_planner(
                    self.current_pose, self.scenario.goal_pose, occ_grid, self.cfg, self.global_path
                )
                if new_plan.valid:
                    self.active_path = new_plan
                else:
                    self.active_path = avoid_path
            else:
                self.active_path = avoid_path

        steer_cmd, target_point, cross_track_err = pure_pursuit_controller(
            self.current_pose, self.current_velocity, self.active_path.waypoints, self.cfg
        )
        self.steer_angle_cmd = steer_cmd

        target_vel = self.cfg.vehicle.nominal_speed
        if hasattr(self.active_path, "velocities") and len(self.active_path.velocities) > 0:
            target_vel = float(self.active_path.velocities[0])
        if self.goal_reached:
            target_vel = 0.0

        throttle_cmd, brake_cmd, accel_cmd = vehicle_controller(
            target_vel, self.current_velocity, risk_level, self.cfg
        )
        self.accel_cmd = accel_cmd

        next_pose, next_vel, yaw_rate, footprint_vertices = vehicle_model(
            self.current_pose, self.current_velocity, self.steer_angle_cmd, self.accel_cmd, dt, self.cfg
        )

        self.current_pose = next_pose
        self.current_velocity = next_vel
        self.footprint_vertices = footprint_vertices
        self.trajectory_history.append(self.current_pose[:2])

        self.time_log.append(self.t)
        self.vel_log.append(self.current_velocity)
        self.cross_track_log.append(cross_track_err)

        return SimulationState(
            t=self.t,
            sim_step=self.sim_step,
            ego_pose=self.current_pose,
            ego_velocity=self.current_velocity,
            steer_angle_cmd=self.steer_angle_cmd,
            accel_cmd=self.accel_cmd,
            footprint_vertices=self.footprint_vertices,
            perception=perception_out,
            collision_report=collision_report,
            active_path=self.active_path,
            global_path=self.global_path,
            target_lookahead_point=target_point,
            cross_track_error=cross_track_err,
            replan_count=self.replan_count,
            collision_count=self.collision_count,
            goal_reached=self.goal_reached,
            dist_to_goal=dist_to_goal,
            trajectory_history=self.trajectory_history,
        )

    def run_full(self) -> Tuple[PerformanceMetrics, List[Dict[str, Any]]]:
        self.reset()
        history = []
        max_time = self.cfg.sim.max_time
        dt = self.cfg.sim.dt

        while self.t <= max_time and not self.goal_reached:
            state = self.step()
            history.append(state.to_dict())

        diffs = np.diff(np.array(self.trajectory_history), axis=0)
        total_path_len = float(np.sum(np.linalg.norm(diffs, axis=1)))
        avg_speed = float(np.mean(self.vel_log) * 3.6) if self.vel_log else 0.0
        mean_cross_track = float(np.mean(self.cross_track_log)) if self.cross_track_log else 0.0

        metrics = PerformanceMetrics(
            scenario_id=self.scenario_id,
            scenario_name=self.scenario.name,
            goal_reached=self.goal_reached,
            collision_count=self.collision_count,
            min_obstacle_dist=self.min_obstacle_dist,
            path_length=total_path_len,
            replan_count=self.replan_count,
            avg_speed_kmh=avg_speed,
            mean_cross_track_err=mean_cross_track,
            sim_duration_sec=self.t,
        )
        return metrics, history
