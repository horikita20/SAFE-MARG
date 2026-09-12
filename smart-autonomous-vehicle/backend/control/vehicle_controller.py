"""
Longitudinal speed and emergency brake controller.
"""

from typing import Tuple, Optional
import numpy as np

try:
    from ..config import Config, get_config
except (ImportError, ValueError):
    try:
        from backend.config import Config, get_config
    except (ImportError, ValueError):
        from config import Config, get_config


def vehicle_controller(
    target_velocity: float,
    current_velocity: float,
    risk_level: str,
    cfg: Optional[Config] = None,
) -> Tuple[float, float, float]:
    """
    Computes throttle, brake, and net acceleration commands.
    """
    if cfg is None:
        cfg = get_config()

    max_accel = cfg.vehicle.max_accel
    max_decel = cfg.vehicle.max_decel
    kp = cfg.control.speed_kp

    if str(risk_level).upper() == "CRITICAL":
        return 0.0, 1.0, -max_decel

    eff_target_vel = target_velocity
    if str(risk_level).upper() == "WARNING":
        eff_target_vel = min(target_velocity, 3.5)

    vel_error = eff_target_vel - current_velocity
    desired_accel = kp * vel_error

    if desired_accel >= 0:
        throttle_cmd = min(1.0, desired_accel / max_accel)
        brake_cmd = 0.0
        accel_cmd = min(max_accel, desired_accel)
    else:
        throttle_cmd = 0.0
        brake_cmd = min(1.0, abs(desired_accel) / max_decel)
        accel_cmd = max(-max_decel, desired_accel)

    return float(throttle_cmd), float(brake_cmd), float(accel_cmd)
