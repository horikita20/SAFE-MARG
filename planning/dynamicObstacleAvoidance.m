function safePath = dynamicObstacleAvoidance(activePath, currentPose, currentVelocity, detectedObjects, occGridStruct, cfg)
% DYNAMICOBSTACLEAVOIDANCE Computes local reactive avoidance maneuver for dynamic hazards.
%
% Inputs:
%   activePath      - Active PlannedPath struct
%   currentPose     - [x y theta] Vehicle pose
%   currentVelocity - Vehicle speed (m/s)
%   detectedObjects - Array of detected object structs
%   occGridStruct   - Current occupancy grid struct
%   cfg             - System configuration struct
%
% Output:
%   safePath - Updated PlannedPath with local avoidance detour

    if nargin < 6 || isempty(cfg)
        cfg = config();
    end

    safePath = activePath;
    if isempty(detectedObjects) || isempty(activePath) || ~isfield(activePath, 'waypoints') || isempty(activePath.waypoints)
        return;
    end

    egoPos = currentPose(1:2);
    waypoints = activePath.waypoints;
    numWp = size(waypoints, 1);

    % Identify nearest dynamic obstacle threatening the corridor
    threatObj = [];
    minDist = inf;
    for i = 1:numel(detectedObjects)
        obj = detectedObjects(i);
        d = norm(obj.worldPosition - egoPos);
        if d < minDist && d < 18.0
            minDist = d;
            threatObj = obj;
        end
    end

    if isempty(threatObj)
        return;
    end

    % Predict dynamic obstacle trajectory over 3 seconds
    dt = 0.2;
    horizon = 3.0;
    predSteps = round(horizon / dt);
    predObstaclePositions = zeros(predSteps, 2);
    for s = 1:predSteps
        t_pred = s * dt;
        predObstaclePositions(s, :) = threatObj.worldPosition + threatObj.worldVelocity * t_pred;
    end

    % Determine bypass side (left or right) based on obstacle lateral velocity and road boundaries
    obsY = threatObj.worldPosition(2);
    obsVy = threatObj.worldVelocity(2);

    % If obstacle moving towards +Y (left), bypass on right (-Y), and vice versa
    if obsVy > 0.3
        bypassDir = -1.0; % Go right
    elseif obsVy < -0.3
        bypassDir = 1.0;  % Go left
    else
        % Static or longitudinal: bypass to the side with more road margin
        if obsY >= 0
            bypassDir = -1.0; % Obstacle is on left, pass on right
        else
            bypassDir = 1.0;  % Obstacle is on right, pass on left
        end
    end

    % Apply smooth lateral Gaussian perturbation to waypoints near obstacle
    avoidRadius = threatObj.radius + cfg.risk.lateralClearance + 0.8;
    detourMag = bypassDir * avoidRadius;

    newWaypoints = waypoints;
    for w = 1:numWp
        wp = waypoints(w, :);
        distToObs = norm(wp - threatObj.worldPosition);
        if distToObs < (avoidRadius * 2.5)
            % Bell-curve lateral shift
            shiftFactor = exp(-(distToObs^2) / (2 * (avoidRadius * 0.9)^2));
            newY = wp(2) + detourMag * shiftFactor;
            
            % Clamp to road boundaries
            if ~isempty(occGridStruct) && isfield(occGridStruct, 'yMax')
                newY = max(occGridStruct.yMin + 0.8, min(occGridStruct.yMax - 0.8, newY));
            end
            newWaypoints(w, 2) = newY;
        end
    end

    % Ensure vehicle start pose is pinned
    newWaypoints(1, 2) = egoPos(2);

    % Smooth the modified path
    if size(newWaypoints, 1) >= 5
        newWaypoints(:, 2) = smoothdata(newWaypoints(:, 2), 'gaussian', 5);
    end

    dx = gradient(newWaypoints(:, 1));
    dy = gradient(newWaypoints(:, 2));
    headings = atan2(dy, dx);

    % Slow down during avoidance maneuver
    velocities = activePath.velocities;
    velocities = max(2.0, velocities * 0.75); % 25% speed reduction for safety

    safePath.waypoints = newWaypoints;
    safePath.headings = headings;
    safePath.velocities = velocities;
    safePath.valid = true;
end
