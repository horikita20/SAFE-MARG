function perceptionOut = perceptionPipeline(scenario, egoPose, egoVelocity, t, cfg)
% PERCEPTIONPIPELINE Master perception fusion pipeline for Smart Autonomous Vehicle.
%
% Inputs:
%   scenario    - Active scenario struct containing road and dynamic actors
%   egoPose     - Current vehicle pose [x y theta] (m, rad)
%   egoVelocity - Current vehicle speed (m/s)
%   t           - Current simulation timestamp (s)
%   cfg         - System configuration struct
%
% Output:
%   perceptionOut - Comprehensive perception output struct:
%                   .timestamp       - Simulation timestamp (s)
%                   .objects         - Detected objects array (classes, positions, velocities)
%                   .drivableArea    - Drivable corridor boundaries & centerline
%                   .occupancyGrid   - 3-state inflated occupancy grid struct
%                   .egoPose         - Ego pose [x y theta]
%                   .egoVelocity     - Ego speed (m/s)
%                   .nearestObstacle - Struct of closest obstacle to ego

    if nargin < 5 || isempty(cfg)
        cfg = config();
    end

    % 1. Detect objects (with noise and FOV simulation)
    actors = [];
    if isfield(scenario, 'actors')
        actors = scenario.actors;
    end
    detectedObjects = detectObjects(actors, egoPose, cfg, t);

    % 2. Extract drivable area and road margins
    drivableArea = detectDrivableArea(scenario, egoPose, cfg);

    % 3. Generate inflated 3-state occupancy grid
    occGrid = generateOccupancyGrid(detectedObjects, drivableArea, cfg, egoPose);

    % 4. Find nearest obstacle
    nearestObstacle = struct('id', 0, 'class', "none", 'distance', inf, 'position', [inf inf]);
    if ~isempty(detectedObjects)
        dists = [detectedObjects.distance];
        [minDist, minIdx] = min(dists);
        nearestObstacle = detectedObjects(minIdx);
        nearestObstacle.distance = minDist;
    end

    perceptionOut = struct(...
        'timestamp', t, ...
        'objects', detectedObjects, ...
        'drivableArea', drivableArea, ...
        'occupancyGrid', occGrid, ...
        'egoPose', egoPose, ...
        'egoVelocity', egoVelocity, ...
        'nearestObstacle', nearestObstacle);
end
