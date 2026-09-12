function globalPath = generateGlobalPath(startPose, goalPose, drivableArea, cfg)
% GENERATEGLOBALPATH Generates a smooth reference global path along the road corridor.
%
% Inputs:
%   startPose    - [x y theta] Vehicle start pose
%   goalPose     - [x y theta] Target destination pose
%   drivableArea - Drivable area struct (from detectDrivableArea)
%   cfg          - Configuration struct
%
% Output:
%   globalPath - Struct containing:
%                .waypoints  - [N x 2] coordinates [x, y] along reference path
%                .headings   - [N x 1] target yaw angles (radians)
%                .velocities - [N x 1] target speeds (m/s)
%                .curvatures - [N x 1] path curvature kappa (1/m)
%                .length     - Total path length (meters)

    if nargin < 4 || isempty(cfg)
        cfg = config();
    end

    ds = cfg.planner.waypointSpacing;
    nominalSpeed = cfg.vehicle.nominalSpeed;

    x0 = startPose(1);
    y0 = startPose(2);
    xg = goalPose(1);
    yg = goalPose(2);

    % Sample points from start to goal along road centerline
    xSamples = (x0:ds:xg)';
    if isempty(xSamples) || xSamples(end) < xg
        xSamples = [xSamples; xg];
    end

    if ~isempty(drivableArea) && isfield(drivableArea, 'leftBoundFunc') && isfield(drivableArea, 'rightBoundFunc')
        yLeft = drivableArea.leftBoundFunc(xSamples);
        yRight = drivableArea.rightBoundFunc(xSamples);
        yCenter = (yLeft + yRight) / 2.0;
    else
        yCenter = linspace(y0, yg, numel(xSamples))';
    end

    % Smooth transition from starting lateral offset to centerline
    numBlend = min(15, numel(xSamples));
    blendWeights = linspace(1, 0, numBlend)';
    yCenter(1:numBlend) = blendWeights * y0 + (1 - blendWeights) .* yCenter(1:numBlend);
    yCenter(end) = yg;

    rawWaypoints = [xSamples, yCenter];

    % Smooth waypoints using moving average / spline
    if size(rawWaypoints, 1) >= 5
        smoothedY = smoothdata(rawWaypoints(:, 2), 'gaussian', 5);
        waypoints = [rawWaypoints(:, 1), smoothedY];
    else
        waypoints = rawWaypoints;
    end

    % Compute headings, curvatures, and path length
    dx = gradient(waypoints(:, 1));
    dy = gradient(waypoints(:, 2));
    headings = atan2(dy, dx);

    ddx = gradient(dx);
    ddy = gradient(dy);
    curvatures = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^(1.5) + 1e-6);

    % Speed profile (slow down near sharp turns and final goal)
    velocities = nominalSpeed * ones(size(waypoints, 1), 1);
    
    % Deceleration near goal
    goalDecelDist = 8.0;
    distToGoal = sqrt((waypoints(:, 1) - xg).^2 + (waypoints(:, 2) - yg).^2);
    decelMask = distToGoal <= goalDecelDist;
    velocities(decelMask) = max(1.5, nominalSpeed * (distToGoal(decelMask) / goalDecelDist));
    velocities(end) = 0.0;

    % Path length
    diffs = diff(waypoints, 1, 1);
    pathLength = sum(sqrt(sum(diffs.^2, 2)));

    globalPath = struct(...
        'waypoints', waypoints, ...
        'headings', headings, ...
        'velocities', velocities, ...
        'curvatures', curvatures, ...
        'length', pathLength, ...
        'valid', true);
end
