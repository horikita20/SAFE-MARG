function plannedPath = adaptivePathPlanner(currentPose, goalPose, occGridStruct, cfg, globalPath)
% ADAPTIVEPATHPLANNER Safe adaptive path planner supporting Hybrid A* and robust pure-MATLAB fallback.
%
% Inputs:
%   currentPose   - [x y theta] Current vehicle pose (meters, rad)
%   goalPose      - [x y theta] Target destination pose (meters, rad)
%   occGridStruct - Occupancy grid struct (from generateOccupancyGrid)
%   cfg           - System configuration struct
%   globalPath    - Baseline global reference path (optional)
%
% Output:
%   plannedPath - Struct matching PS26037 data contracts:
%                 .waypoints  - [N x 2] planned path coordinates [x, y]
%                 .velocities - [N x 1] target speeds at waypoints (m/s)
%                 .headings   - [N x 1] target yaw angles (rad)
%                 .cost       - Total path traversal cost
%                 .valid      - Logical true if path found, false if blocked

    if nargin < 4 || isempty(cfg)
        cfg = config();
    end

    % 1. Check if Navigation Toolbox Hybrid A* is available
    hasNavToolbox = false;
    try
        hasNavToolbox = ~isempty(which('plannerHybridAStar')) || ~isempty(which('hybridAStarPathPlanner'));
    catch
        hasNavToolbox = false;
    end

    plannedPath = [];
    if hasNavToolbox
        try
            plannedPath = planHybridAStarToolbox(currentPose, goalPose, occGridStruct, cfg);
        catch ME
            % Fall back gracefully to custom adaptive A*
            plannedPath = [];
        end
    end

    % 2. Fallback: Fast vector-optimized Kinematic Adaptive A* Planner
    if isempty(plannedPath) || ~isfield(plannedPath, 'valid') || ~plannedPath.valid
        plannedPath = planCustomAdaptiveAStar(currentPose, goalPose, occGridStruct, cfg, globalPath);
    end
end

% -------------------------------------------------------------------------
% Fallback 1: Custom Vectorized Kinematic Grid A* Planner
% -------------------------------------------------------------------------
function plannedPath = planCustomAdaptiveAStar(currentPose, goalPose, occGridStruct, cfg, globalPath)
    xMin = occGridStruct.xMin;
    xMax = occGridStruct.xMax;
    yMin = occGridStruct.yMin;
    yMax = occGridStruct.yMax;
    res  = occGridStruct.resolution;
    grid = occGridStruct.grid;
    [Ny, Nx] = size(grid);

    startPos = currentPose(1:2);
    startTheta = currentPose(3);
    goalPos = goalPose(1:2);

    % Convert start and goal to grid coordinates
    sIx = max(1, min(Nx, round((startPos(1) - xMin) / res) + 1));
    sIy = max(1, min(Ny, round((startPos(2) - yMin) / res) + 1));
    gIx = max(1, min(Nx, round((goalPos(1) - xMin) / res) + 1));
    gIy = max(1, min(Ny, round((goalPos(2) - yMin) / res) + 1));

    % 8-connected grid motion primitives [dx, dy, stepCost, steerPenalty]
    motions = [
        1,  0, 1.0,  0.0;    % Forward
        1,  1, 1.414, 0.4;   % Forward-Left
        1, -1, 1.414, 0.4;   % Forward-Right
        0,  1, 1.2,   0.8;   % Left
        0, -1, 1.2,   0.8;   % Right
        2,  1, 2.236, 0.2;   % Smooth shallow left
        2, -1, 2.236, 0.2    % Smooth shallow right
    ];

    % Open list and cost matrices
    gScore = inf(Ny, Nx);
    fScore = inf(Ny, Nx);
    parentX = zeros(Ny, Nx, 'int16');
    parentY = zeros(Ny, Nx, 'int16');
    closedSet = false(Ny, Nx);

    gScore(sIy, sIx) = 0;
    fScore(sIy, sIx) = norm(startPos - goalPos);

    % Priority queue using linear indexing
    openSet = false(Ny, Nx);
    openSet(sIy, sIx) = true;

    maxIter = cfg.planner.maxIterations;
    iter = 0;
    goalReached = false;
    closestNode = [sIy, sIx];
    minH = fScore(sIy, sIx);

    while any(openSet(:)) && iter < maxIter
        iter = iter + 1;

        % Find open node with minimum fScore
        fTemp = fScore;
        fTemp(~openSet) = inf;
        [minF, minLinearIdx] = min(fTemp(:));

        if isinf(minF)
            break;
        end

        [cy, cx] = ind2sub([Ny, Nx], minLinearIdx);
        openSet(cy, cx) = false;
        closedSet(cy, cx) = true;

        currWorldX = xMin + (cx - 1) * res;
        currWorldY = yMin + (cy - 1) * res;

        % Distance to goal
        distToGoal = norm([currWorldX, currWorldY] - goalPos);
        if distToGoal < minH
            minH = distToGoal;
            closestNode = [cy, cx];
        end

        % Goal check
        if (cx >= gIx - 1 && abs(cy - gIy) <= 2) || distToGoal <= 1.0
            closestNode = [cy, cx];
            goalReached = true;
            break;
        end

        % Expand neighbors
        for m = 1:size(motions, 1)
            nx = cx + motions(m, 1);
            ny = cy + motions(m, 2);

            % Boundary check
            if nx < 1 || nx > Nx || ny < 1 || ny > Ny || closedSet(ny, nx)
                continue;
            end

            % Obstacle / cost check
            cellOcc = grid(ny, nx);
            if cellOcc >= 0.85 % Obstacle or high inflation zone
                continue;
            end

            % Obstacle proximity penalty + road centerline bias
            worldNY = yMin + (ny - 1) * res;
            roadCenterDist = abs(worldNY);
            costObstacle = 15.0 * cellOcc;
            costCenterBias = 0.15 * roadCenterDist;
            stepCost = motions(m, 3) * res + motions(m, 4) + costObstacle + costCenterBias;

            tentativeG = gScore(cy, cx) + stepCost;

            if tentativeG < gScore(ny, nx)
                parentX(ny, nx) = cx;
                parentY(ny, nx) = cy;
                gScore(ny, nx) = tentativeG;

                % Euclidean heuristic
                nWorldX = xMin + (nx - 1) * res;
                nWorldY = yMin + (ny - 1) * res;
                h = norm([nWorldX, nWorldY] - goalPos);
                fScore(ny, nx) = tentativeG + 1.1 * h;
                openSet(ny, nx) = true;
            end
        end
    end

    % Reconstruct path from goal/closestNode back to start
    pathIndices = closestNode;
    curr = closestNode;
    while ~(curr(1) == sIy && curr(2) == sIx)
        py = parentY(curr(1), curr(2));
        px = parentX(curr(1), curr(2));
        if py == 0 || px == 0
            break;
        end
        curr = [py, px];
        pathIndices = [curr; pathIndices]; %#ok<AGROW>
    end

    if size(pathIndices, 1) < 2
        % Emergency straight fallback if search completely failed
        xPts = linspace(startPos(1), min(startPos(1) + 10, goalPos(1)), 20)';
        yPts = linspace(startPos(2), startPos(2), 20)';
        rawWaypoints = [xPts, yPts];
        valid = false;
    else
        % Convert grid indices to world coordinates
        rawWaypoints = [xMin + (pathIndices(:, 2) - 1) * res, yMin + (pathIndices(:, 1) - 1) * res];
        valid = goalReached || (minH < 5.0);
    end

    % Smooth path using spline interpolation and moving average
    waypoints = smoothPath(rawWaypoints, startPos, goalPos, cfg);

    % Compute velocities along path
    nominalSpeed = cfg.vehicle.nominalSpeed;
    velocities = nominalSpeed * ones(size(waypoints, 1), 1);
    
    % Decelerate near end of path
    decelDist = 6.0;
    dGoal = sqrt(sum((waypoints - goalPos).^2, 2));
    decelIdx = dGoal <= decelDist;
    velocities(decelIdx) = max(1.5, nominalSpeed * (dGoal(decelIdx) / decelDist));
    velocities(end) = 0.0;

    dx = gradient(waypoints(:, 1));
    dy = gradient(waypoints(:, 2));
    headings = atan2(dy, dx);

    plannedPath = struct(...
        'waypoints', waypoints, ...
        'velocities', velocities, ...
        'headings', headings, ...
        'cost', double(gScore(closestNode(1), closestNode(2))), ...
        'valid', valid);
end

% -------------------------------------------------------------------------
% Fallback 2: Navigation Toolbox Hybrid A* (if toolbox is licensed & available)
% -------------------------------------------------------------------------
function plannedPath = planHybridAStarToolbox(currentPose, goalPose, occGridStruct, cfg)
    res = occGridStruct.resolution;
    grid = occGridStruct.grid;
    
    % Build binary occupancy map
    costmap = binaryOccupancyMap(grid >= 0.85, 1 / res);
    costmap.GridLocationInWorld = [occGridStruct.xMin, occGridStruct.yMin];

    % Hybrid A* Planner configuration
    planner = plannerHybridAStar(costmap, ...
        'MinTurningRadius', cfg.vehicle.wheelbase / tan(cfg.vehicle.maxSteerAngle), ...
        'MotionPrimitiveLength', 1.0);

    startState = [currentPose(1), currentPose(2), currentPose(3)];
    goalState  = [goalPose(1), goalPose(2), goalPose(3)];

    refPath = plan(planner, startState, goalState);

    if ~isempty(refPath) && refPath.NumStates > 1
        states = refPath.States;
        waypoints = states(:, 1:2);
        headings = states(:, 3);
        velocities = cfg.vehicle.nominalSpeed * ones(size(waypoints, 1), 1);
        velocities(end) = 0.0;

        plannedPath = struct(...
            'waypoints', waypoints, ...
            'velocities', velocities, ...
            'headings', headings, ...
            'cost', double(refPath.PathCost), ...
            'valid', true);
    else
        plannedPath = struct('valid', false);
    end
end

% -------------------------------------------------------------------------
% Helper: Path Smoothing
% -------------------------------------------------------------------------
function smoothedWaypoints = smoothPath(rawWaypoints, startPos, goalPos, cfg)
    numPts = size(rawWaypoints, 1);
    if numPts <= 3
        smoothedWaypoints = rawWaypoints;
        return;
    end

    % Resample to equidistant points
    diffs = diff(rawWaypoints, 1, 1);
    segLengths = sqrt(sum(diffs.^2, 2));
    cumDist = [0; cumsum(segLengths)];
    totalDist = cumDist(end);

    if totalDist < 0.5
        smoothedWaypoints = rawWaypoints;
        return;
    end

    ds = cfg.planner.waypointSpacing;
    uniformDist = 0:ds:totalDist;
    if uniformDist(end) < totalDist
        uniformDist = [uniformDist, totalDist];
    end

    interpX = interp1(cumDist, rawWaypoints(:, 1), uniformDist, 'pchip');
    interpY = interp1(cumDist, rawWaypoints(:, 2), uniformDist, 'pchip');

    % Apply gaussian smoothing filter to lateral coordinate
    smoothedY = smoothdata(interpY, 'gaussian', 7);
    
    % Clamp start point to current pose
    smoothedY(1) = startPos(2);
    interpX(1) = startPos(1);

    smoothedWaypoints = [interpX(:), smoothedY(:)];
end
