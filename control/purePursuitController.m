function [steerCmd, targetPoint, crossTrackError] = purePursuitController(currentPose, currentVelocity, waypoints, cfg)
% PUREPURSUITCONTROLLER Geometric path-following Pure Pursuit steering controller.
%
% Inputs:
%   currentPose     - [x y theta] Current ego pose (world frame, meters and rad)
%   currentVelocity - Current ego forward speed (m/s)
%   waypoints       - [N x 2] Array of path waypoints [x, y]
%   cfg             - System configuration struct
%
% Outputs:
%   steerCmd        - Commanded front wheel steering angle (rad)
%   targetPoint     - [1 x 2] Selected lookahead target waypoint [x, y]
%   crossTrackError - Distance from ego vehicle to closest path point (m)

    if nargin < 4 || isempty(cfg)
        cfg = config();
    end

    L = cfg.vehicle.wheelbase;
    maxSteer = cfg.vehicle.maxSteerAngle;
    minLd = cfg.control.minLookahead;
    maxLd = cfg.control.maxLookahead;
    k_ld  = cfg.control.lookaheadGain;

    if isempty(waypoints) || size(waypoints, 1) < 2
        steerCmd = 0.0;
        targetPoint = currentPose(1:2);
        crossTrackError = 0.0;
        return;
    end

    egoX = currentPose(1);
    egoY = currentPose(2);
    egoTheta = currentPose(3);

    % Speed-adaptive lookahead distance
    Ld = max(minLd, min(maxLd, k_ld * currentVelocity));

    % Distances from vehicle to all waypoints
    dx = waypoints(:, 1) - egoX;
    dy = waypoints(:, 2) - egoY;
    dists = sqrt(dx.^2 + dy.^2);

    [minDist, closestIdx] = min(dists);
    crossTrackError = minDist;

    % Search forward from closest index for waypoint at lookahead distance Ld
    targetIdx = closestIdx;
    for i = closestIdx:size(waypoints, 1)
        if dists(i) >= Ld
            targetIdx = i;
            break;
        end
        targetIdx = i; % Fallback to last waypoint if end of path reached
    end

    targetPoint = waypoints(targetIdx, :);

    % Transform target waypoint into vehicle reference frame
    % Vehicle frame: x is forward along vehicle heading, y is lateral left
    vecX = targetPoint(1) - egoX;
    vecY = targetPoint(2) - egoY;

    localX =  cos(egoTheta) * vecX + sin(egoTheta) * vecY;
    localY = -sin(egoTheta) * vecX + cos(egoTheta) * vecY;

    % Pure Pursuit curvature formula: kappa = 2 * localY / (Ld^2)
    % Steering angle: delta = atan(kappa * L) = atan(2 * L * localY / (Ld^2))
    actualLd = max(1.0, norm([localX, localY]));
    steerCmd = atan2(2.0 * L * localY, actualLd^2);

    % Clamp steering angle to physical vehicle limit
    steerCmd = max(-maxSteer, min(maxSteer, steerCmd));
end
