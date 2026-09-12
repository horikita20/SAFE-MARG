function [throttleCmd, brakeCmd, accelCmd] = vehicleController(targetVelocity, currentVelocity, riskLevel, cfg)
% VEHICLECONTROLLER Longitudinal velocity & emergency braking controller for autonomous vehicle.
%
% Inputs:
%   targetVelocity  - Target forward velocity from path planner (m/s)
%   currentVelocity - Current forward speed (m/s)
%   riskLevel       - "SAFE" | "WARNING" | "CRITICAL"
%   cfg             - System configuration struct
%
% Outputs:
%   throttleCmd - Throttle command in [0, 1]
%   brakeCmd    - Brake command in [0, 1]
%   accelCmd    - Net acceleration command (m/s^2)

    if nargin < 4 || isempty(cfg)
        cfg = config();
    end

    maxAccel = cfg.vehicle.maxAccel;
    maxDecel = cfg.vehicle.maxDecel;
    kp = cfg.control.speedKp;

    % Hard override for CRITICAL collision risk
    if string(riskLevel) == "CRITICAL"
        throttleCmd = 0.0;
        brakeCmd    = 1.0;
        accelCmd    = -maxDecel;
        return;
    end

    % Adjust target velocity for WARNING risk
    effTargetVel = targetVelocity;
    if string(riskLevel) == "WARNING"
        effTargetVel = min(targetVelocity, 3.5); % Slow down to cautious crawling speed
    end

    % Velocity error
    velError = effTargetVel - currentVelocity;

    % Proportional acceleration control
    desiredAccel = kp * velError;

    if desiredAccel >= 0
        % Accelerating
        throttleCmd = min(1.0, desiredAccel / maxAccel);
        brakeCmd = 0.0;
        accelCmd = min(maxAccel, desiredAccel);
    else
        % Braking / Coasting
        throttleCmd = 0.0;
        brakeCmd = min(1.0, abs(desiredAccel) / maxDecel);
        accelCmd = max(-maxDecel, desiredAccel);
    end
end
