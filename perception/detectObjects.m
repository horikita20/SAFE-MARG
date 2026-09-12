function detectedObjects = detectObjects(scenarioActors, egoPose, cfg, t)
% DETECTOBJECTS Detects objects in the environment relative to ego vehicle.
%
% Inputs:
%   scenarioActors - Array of actor/obstacle structs from the scenario
%   egoPose        - [x y theta] Current ego pose (world frame, meters and rad)
%   cfg            - System configuration struct
%   t              - Current simulation timestamp (s)
%
% Output:
%   detectedObjects - Array of detected object structs containing:
%                     .id            - Persistent object ID
%                     .class         - "pedestrian"|"vehicle"|"motorcycle"|"auto-rickshaw"|
%                                      "cattle"|"pothole"|"debris"|"unknown"
%                     .position      - [x y] in ego vehicle frame (meters, x=fwd, y=left)
%                     .worldPosition - [xw yw] in world coordinates (meters)
%                     .velocity      - [vx vy] in m/s (ego frame)
%                     .worldVelocity - [vxw vyw] in m/s (world frame)
%                     .distance      - Straight-line distance from ego (meters)
%                     .bbox          - [x y w h] 2D bounding box (meters in world or pixels)
%                     .radius        - Enclosing obstacle radius (meters)
%                     .confidence    - Detection score [0, 1]

    if nargin < 3 || isempty(cfg)
        cfg = config();
    end
    if nargin < 4 || isempty(t)
        t = 0.0;
    end

    maxRange = cfg.sensors.maxRange;
    fovAngle = cfg.sensors.fovAngle;
    noisePos = cfg.sensors.posNoiseStd;
    noiseVel = cfg.sensors.velNoiseStd;

    egoX = egoPose(1);
    egoY = egoPose(2);
    egoTheta = egoPose(3);

    % Rotation matrix from world to ego vehicle frame
    % ego frame: +x points along heading, +y points 90 deg left
    R_worldToEgo = [cos(egoTheta), sin(egoTheta);
                   -sin(egoTheta), cos(egoTheta)];

    numActors = numel(scenarioActors);
    detectedObjects = repmat(struct(...
        'id', 0, ...
        'class', "unknown", ...
        'position', [0.0, 0.0], ...
        'worldPosition', [0.0, 0.0], ...
        'velocity', [0.0, 0.0], ...
        'worldVelocity', [0.0, 0.0], ...
        'distance', 0.0, ...
        'bbox', [0.0, 0.0, 0.0, 0.0], ...
        'radius', 0.5, ...
        'confidence', 1.0), 1, 0);

    for i = 1:numActors
        actor = scenarioActors(i);

        % Compute current world position of actor at time t
        if isfield(actor, 'trajectoryFunc') && ~isempty(actor.trajectoryFunc)
            [worldPos, worldVel] = actor.trajectoryFunc(t);
        elseif isfield(actor, 'position')
            worldPos = actor.position;
            if isfield(actor, 'velocity')
                worldVel = actor.velocity;
            else
                worldVel = [0.0, 0.0];
            end
        else
            continue;
        end

        % Transform world position to ego vehicle frame
        relPosWorld = worldPos - [egoX, egoY];
        relPosEgo = (R_worldToEgo * relPosWorld(:))';

        % Distance and relative bearing
        dist = norm(relPosEgo);
        bearing = atan2(relPosEgo(2), relPosEgo(1));

        % Check if within sensor field-of-view and range
        % (Potholes are detected within shorter range ~20m)
        isPothole = isfield(actor, 'class') && (string(actor.class) == "pothole");
        effMaxRange = maxRange;
        if isPothole
            effMaxRange = min(20.0, maxRange);
        end

        if dist <= effMaxRange && abs(bearing) <= (fovAngle / 2)
            % Add small synthetic measurement noise
            noisyEgoPos = relPosEgo + randn(1, 2) * noisePos;
            
            % Transform world velocity to ego frame
            relVelEgo = (R_worldToEgo * worldVel(:))' + randn(1, 2) * noiseVel;
            if isPothole || norm(worldVel) < 0.05
                relVelEgo = [0.0, 0.0];
            end

            % Object radius
            if isfield(actor, 'radius') && ~isempty(actor.radius)
                rad = actor.radius;
            else
                rad = getDefaultRadius(actor.class);
            end

            % Class name
            if isfield(actor, 'class')
                cName = string(actor.class);
            else
                cName = "unknown";
            end

            % Confidence score (decays slightly with distance)
            conf = max(0.6, min(1.0, 1.0 - (dist / (effMaxRange * 1.5))));

            % Bounding box in world frame [xMin, yMin, width, length]
            bbox = [worldPos(1) - rad, worldPos(2) - rad, 2 * rad, 2 * rad];

            det = struct(...
                'id', double(actor.id), ...
                'class', cName, ...
                'position', noisyEgoPos, ...
                'worldPosition', worldPos, ...
                'velocity', relVelEgo, ...
                'worldVelocity', worldVel, ...
                'distance', dist, ...
                'bbox', bbox, ...
                'radius', rad, ...
                'confidence', conf);

            detectedObjects(end + 1) = det; %#ok<AGROW>
        end
    end
end

function rad = getDefaultRadius(className)
    switch lower(string(className))
        case "pedestrian"
            rad = 0.45;
        case "cattle"
            rad = 1.10;
        case "auto-rickshaw"
            rad = 1.20;
        case "vehicle"
            rad = 1.50;
        case "motorcycle"
            rad = 0.60;
        case "pothole"
            rad = 0.70;
        case "debris"
            rad = 0.50;
        otherwise
            rad = 0.80;
    end
end
