"""
Configuration parameters for Smart Autonomous Vehicle (SIH26037).
Centralized dataclasses for vehicle dynamics, grid, sensors, planner, risk, control, and UI theme.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class VehicleConfig:
    wheelbase: float = 2.7            # Wheelbase L (meters)
    track_width: float = 1.6          # Track width (meters)
    length: float = 4.4               # Overall vehicle length (meters)
    width: float = 1.8                # Overall vehicle width (meters)
    rear_axle_to_bumper: float = 1.0  # Rear axle to rear bumper (meters)
    max_steer_angle: float = float(np.deg2rad(35))  # Max front wheel steer angle (rad)
    max_steer_rate: float = float(np.deg2rad(40))   # Max steering rate (rad/s)
    max_speed: float = 12.0           # Max forward velocity (m/s) (~43.2 km/h)
    nominal_speed: float = 7.0        # Target cruising speed (m/s) (~25.2 km/h)
    min_speed: float = 0.0            # Min forward velocity (m/s)
    max_accel: float = 2.5            # Max forward acceleration (m/s^2)
    max_decel: float = 6.0            # Max emergency braking (m/s^2)
    comfort_decel: float = 2.0        # Nominal deceleration (m/s^2)


@dataclass
class GridConfig:
    x_min: float = 0.0                # Minimum X (meters)
    x_max: float = 80.0               # Maximum X (meters)
    y_min: float = -10.0              # Minimum Y (meters, right)
    y_max: float = 10.0               # Maximum Y (meters, left)
    resolution: float = 0.25          # Grid cell resolution (m/cell)
    inflation_radius: float = 1.2     # Safety inflation buffer around obstacles (m)
    pothole_inflation_radius: float = 0.8  # Safety inflation for potholes (m)
    val_free: float = 0.0             # FREE SPACE
    val_obstacle: float = 1.0         # OBSTACLE / BLOCKED
    val_unknown: float = 0.5          # UNKNOWN


@dataclass
class SensorConfig:
    max_range: float = 45.0           # Max perception range (meters)
    fov_angle: float = float(np.deg2rad(110))  # Horizontal field of view (rad)
    update_rate: float = 20.0         # Perception rate (Hz)
    pos_noise_std: float = 0.05       # Measurement noise std (m)
    vel_noise_std: float = 0.10       # Velocity noise std (m/s)


@dataclass
class PlannerConfig:
    planner_type: str = "adaptive_a_star"
    waypoint_spacing: float = 0.5     # Spacing between planned waypoints (m)
    replan_distance: float = 12.0     # Lookahead distance for replan trigger (m)
    safety_margin: float = 0.6        # Extra lateral clearance (m)
    steering_cost_weight: float = 1.2 # Turning angle penalty weight
    max_iterations: int = 8000        # Max A* search iterations


@dataclass
class RiskConfig:
    ttc_critical: float = 1.8         # Time-To-Collision threshold for CRITICAL (seconds)
    ttc_warning: float = 3.5          # Time-To-Collision threshold for WARNING (seconds)
    dist_critical: float = 3.5        # Proximity threshold for CRITICAL (meters)
    dist_warning: float = 8.0         # Proximity threshold for WARNING (meters)
    lateral_clearance: float = 1.4    # Corridor width (meters)


@dataclass
class ControlConfig:
    min_lookahead: float = 2.5        # Minimum Pure Pursuit lookahead (m)
    max_lookahead: float = 7.0        # Maximum Pure Pursuit lookahead (m)
    lookahead_gain: float = 0.5       # Ld = max(min_lookahead, min(max_lookahead, gain * v))
    speed_kp: float = 1.2             # Proportional speed PID gain
    speed_ki: float = 0.05            # Integral gain
    speed_kd: float = 0.02            # Derivative gain


@dataclass
class SimConfig:
    dt: float = 0.05                  # Simulation timestep (20 Hz)
    max_time: float = 45.0            # Max timeout (seconds)
    goal_radius: float = 2.0          # Goal arrival distance tolerance (meters)


@dataclass
class Config:
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    grid: GridConfig = field(default_factory=GridConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    risk: RiskConfig = field(default_factory=RiskConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    sim: SimConfig = field(default_factory=SimConfig)


def get_config() -> Config:
    return Config()
