function collisionReport = collisionChecker(currentPose, currentVelocity, currentPath, detectedObjects, cfg)
% COLLISIONCHECKER Assesses collision risk and computes Time-To-Collision (TTC) along vehicle path.
%
% Risk Levels:
%   "SAFE"     - No obstacles threatening vehicle path
%   "WARNING"  - Obstacle approaching corridor or within warning threshold
%   "CRITICAL" - Imminent collision risk (TTC < threshold or obstacle blocking path)
%
% Inputs:
%   currentPose     - [x y theta] Current ego pose (world frame)
%   currentVelocity - Current ego speed (m/s)
%   currentPath     - Active PlannedPath struct (containing .waypoints)
%   detectedObjects - Array of detected object structs
%   cfg             - System configuration struct
%
% Output:
%   collisionReport - Struct containing:
%                     .riskLevel        - "SAFE" | "WARNING" | "CRITICAL"
%                     .minDistance      - Minimum distance to obstacle (m)
%                     .minTTC           - Minimum Time-To-Collision (s) (inf if none)
%                     .relVelocity      - Relative closing velocity (m/s)
%                     .criticalObstacle - Struct of threatening obstacle
%                     .needsReplan      - Logical flag indicating replanning required
%                     .reason           - Human-readable explanation string

    if nargin < 5 || isempty(cfg)
        cfg = config();
    end

    ttcCrit = cfg.risk.ttcCritical;
    ttcWarn = cfg.risk.ttcWarning;
    distCrit = cfg.risk.distCritical;
    distWarn = cfg.risk.distWarning;
    corridorWidth = cfg.risk.lateralClearance;

    numObs = numel(detectedObjects);

    % Default SAFE report
    collisionReport = struct(...
        'riskLevel', "SAFE", ...
        'minDistance', inf, ...
        'minTTC', inf, ...
        'relVelocity', 0.0, ...
        'criticalObstacle', struct('id', 0, 'class', "none"), ...
        'needsReplan', false, ...
        'reason', "Path is clear and safe.");

    if numObs == 0 || isempty(currentPath) || ~isfield(currentPath, 'waypoints') || isempty(currentPath.waypoints)
        return;
    end

    egoPos = currentPose(1:2);
    egoHeading = currentPose(3);
    egoVelVec = [currentVelocity * cos(egoHeading), currentVelocity * sin(egoHeading)];

    minDist = inf;
    minTTC = inf;
    closingSpeed = 0.0;
    highestRisk = "SAFE";
    criticalObj = struct('id', 0, 'class', "none");
    needsReplan = false;
    reasonStr = "Path clear";

    waypoints = currentPath.waypoints;
    numWp = size(waypoints, 1);

    % Check each detected obstacle
    for i = 1:numObs
        obj = detectedObjects(i);
        obsPos = obj.worldPosition;
        obsVel = obj.worldVelocity;
        obsRadius = obj.radius;
        dist = norm(obsPos - egoPos);

        if dist < minDist
            minDist = dist;
        end

        % Relative position and closing velocity
        relPos = obsPos - egoPos;
        relVel = egoVelVec - obsVel; % Positive means ego is closing in on obstacle
        currClosingSpeed = dot(relPos, relVel) / (dist + 1e-5);

        % Compute Time-To-Collision (TTC)
        ttc = inf;
        if currClosingSpeed > 0.1
            ttc = (dist - (obsRadius + cfg.vehicle.length / 2)) / currClosingSpeed;
            if ttc < 0
                ttc = 0.0;
            end
        end

        if ttc < minTTC
            minTTC = ttc;
            closingSpeed = currClosingSpeed;
        end

        % Check if obstacle intersects the active planned path corridor ahead
        % Lookahead along waypoints within next 15 meters
        pathIntersects = false;
        for w = 1:min(numWp, 40)
            wp = waypoints(w, :);
            distWpToEgo = norm(wp - egoPos);
            if distWpToEgo > 25.0
                break;
            end

            distToPath = norm(wp - obsPos);
            if distToPath <= (obsRadius + corridorWidth)
                pathIntersects = true;
                break;
            end
        end

        % Risk Evaluation Logic
        objRisk = "SAFE";
        if dist <= distCrit || ttc <= ttcCrit || (pathIntersects && dist <= (distCrit * 2.0))
            objRisk = "CRITICAL";
            needsReplan = true;
        elseif dist <= distWarn || ttc <= ttcWarn || pathIntersects
            objRisk = "WARNING";
            if pathIntersects
                needsReplan = true;
            end
        end

        % Update highest risk found
        if (objRisk == "CRITICAL")
            highestRisk = "CRITICAL";
            criticalObj = obj;
            closingSpeed = currClosingSpeed;
            reasonStr = sprintf("%s detected %.1fm ahead (TTC: %.1fs) - Collision imminent!", ...
                upper(obj.class), dist, min(99.9, ttc));
            break; % Critical is maximum priority
        elseif (objRisk == "WARNING") && (highestRisk ~= "CRITICAL")
            highestRisk = "WARNING";
            criticalObj = obj;
            closingSpeed = currClosingSpeed;
            reasonStr = sprintf("%s in proximity (%.1fm, TTC: %.1fs) - Cautious navigation", ...
                upper(obj.class), dist, min(99.9, ttc));
        end
    end

    collisionReport.riskLevel = highestRisk;
    collisionReport.minDistance = minDist;
    collisionReport.minTTC = minTTC;
    collisionReport.relVelocity = closingSpeed;
    collisionReport.criticalObstacle = criticalObj;
    collisionReport.needsReplan = needsReplan;
    collisionReport.reason = reasonStr;
end
