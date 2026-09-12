"""
2D Kinematic Bicycle Model with Runge-Kutta (RK2) numerical integration.
"""

from typing import Tuple, Optional, List
import numpy as np

try:
    from ..config import Config, get_config
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
    except (ImportError, ValueError):
        from config import Config, get_config


def vehicle_model(
    current_pose: Tuple[float, float, float],
    current_velocity: float,
    steer_angle_cmd: float,
    accel_cmd: float,
    dt: float,
    cfg: Optional[Config] = None,
) -> Tuple[Tuple[float, float, float], float, float, List[Tuple[float, float]]]:
    """
    Simulates vehicle motion over timestep dt using Kinematic Bicycle equations.
    """
    if cfg is None:
        cfg = get_config()

    wheelbase = cfg.vehicle.wheelbase
    max_steer = cfg.vehicle.max_steer_angle
    max_speed = cfg.vehicle.max_speed
    min_speed = cfg.vehicle.min_speed
    v_length = cfg.vehicle.length
    v_width = cfg.vehicle.width
    rear_to_bumper = cfg.vehicle.rear_axle_to_bumper

    x0, y0, theta0 = current_pose
    v0 = current_velocity

    delta = float(np.clip(steer_angle_cmd, -max_steer, max_steer))
    a = accel_cmd

    # RK2 Midpoint Integration
    dx1 = v0 * np.cos(theta0)
    dy1 = v0 * np.sin(theta0)
    dtheta1 = (v0 / wheelbase) * np.tan(delta)
    dv1 = a

    x_mid = x0 + 0.5 * dt * dx1
    y_mid = y0 + 0.5 * dt * dy1
    theta_mid = theta0 + 0.5 * dt * dtheta1
    v_mid = float(np.clip(v0 + 0.5 * dt * dv1, min_speed, max_speed))

    dx_mid = v_mid * np.cos(theta_mid)
    dy_mid = v_mid * np.sin(theta_mid)
    dtheta_mid = (v_mid / wheelbase) * np.tan(delta)
    dv_mid = a

    x1 = x0 + dt * dx_mid
    y1 = y0 + dt * dy_mid
    theta1 = theta0 + dt * dtheta_mid
    v1 = float(np.clip(v0 + dt * dv_mid, min_speed, max_speed))

    theta1 = float(np.arctan2(np.sin(theta1), np.cos(theta1)))

    next_pose = (float(x1), float(y1), float(theta1))
    next_vel = float(v1)
    yaw_rate = float((v1 / wheelbase) * np.tan(delta))

    front_dist = v_length - rear_to_bumper
    rear_dist = -rear_to_bumper
    half_w = v_width / 2.0

    local_corners = np.array([
        [front_dist, half_w],   # Front-Left
        [front_dist, -half_w],  # Front-Right
        [rear_dist, -half_w],   # Rear-Right
        [rear_dist, half_w],    # Rear-Left
    ])

    cos_t = np.cos(theta1)
    sin_t = np.sin(theta1)
    r = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    rot_corners = (r @ local_corners.T).T + np.array([x1, y1])

    footprint_vertices = [(float(pt[0]), float(pt[1])) for pt in rot_corners]

    return next_pose, next_vel, yaw_rate, footprint_vertices
