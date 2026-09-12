function [nextPose, nextVel, yawRate, footprintVertices] = vehicleModel(currentPose, currentVelocity, steerAngleCmd, accelCmd, dt, cfg)
% VEHICLEMODEL Discrete Kinematic Bicycle Vehicle Dynamics Model.
%
% State:
%   Pose     = [x y theta] (m, m, rad)
%   Velocity = v (m/s)
%
% Inputs:
%   currentPose     - [x y theta] Current pose at time t
%   currentVelocity - Current speed v at time t (m/s)
%   steerAngleCmd   - Front wheel steer angle delta (rad)
%   accelCmd        - Longitudinal acceleration a (m/s^2)
%   dt              - Integration timestep (seconds)
%   cfg             - System configuration struct
%
% Outputs:
%   nextPose          - [x y theta] Updated vehicle pose at t + dt
%   nextVel           - Updated vehicle speed at t + dt (m/s)
%   yawRate           - Vehicle yaw rate omega = dtheta/dt (rad/s)
%   footprintVertices - [4 x 2] coordinates of 4 vehicle corners in world frame

    if nargin < 6 || isempty(cfg)
        cfg = config();
    end

    L = cfg.vehicle.wheelbase;
    maxSteer = cfg.vehicle.maxSteerAngle;
    maxSpeed = cfg.vehicle.maxSpeed;
    minSpeed = cfg.vehicle.minSpeed;
    vLength  = cfg.vehicle.length;
    vWidth   = cfg.vehicle.width;
    rearToBumper = cfg.vehicle.rearAxleToBumper;

    x0 = currentPose(1);
    y0 = currentPose(2);
    theta0 = currentPose(3);
    v0 = currentVelocity;

    % Clamp inputs
    delta = max(-maxSteer, min(maxSteer, steerAngleCmd));
    a = accelCmd;

    % RK2 (Midpoint) numerical integration of kinematic bicycle equations
    % 1. Derivatives at start of step
    dx1 = v0 * cos(theta0);
    dy1 = v0 * sin(theta0);
    dtheta1 = (v0 / L) * tan(delta);
    dv1 = a;

    % 2. State at midpoint
    x_mid = x0 + 0.5 * dt * dx1;
    y_mid = y0 + 0.5 * dt * dy1;
    theta_mid = theta0 + 0.5 * dt * dtheta1;
    v_mid = max(minSpeed, min(maxSpeed, v0 + 0.5 * dt * dv1));

    % 3. Derivatives at midpoint
    dx_mid = v_mid * cos(theta_mid);
    dy_mid = v_mid * sin(theta_mid);
    dtheta_mid = (v_mid / L) * tan(delta);
    dv_mid = a;

    % 4. Step update
    x1 = x0 + dt * dx_mid;
    y1 = y0 + dt * dy_mid;
    theta1 = theta0 + dt * dtheta_mid;
    v1 = max(minSpeed, min(maxSpeed, v0 + dt * dv_mid));

    % Normalize angle to [-pi, pi]
    theta1 = atan2(sin(theta1), cos(theta1));

    nextPose = [x1, y1, theta1];
    nextVel  = v1;
    yawRate  = (v1 / L) * tan(delta);

    % Compute 4 corner vertices of vehicle box in world frame
    % Vehicle origin is at rear axle center
    frontDist = vLength - rearToBumper;
    rearDist  = -rearToBumper;
    halfW     = vWidth / 2.0;

    localCorners = [
        frontDist,  halfW;  % Front-Left
        frontDist, -halfW;  % Front-Right
        rearDist,  -halfW;  % Rear-Right
        rearDist,   halfW   % Rear-Left
    ];

    R = [cos(theta1), -sin(theta1);
         sin(theta1),  cos(theta1)];

    footprintVertices = (R * localCorners')' + [x1, y1];
end
