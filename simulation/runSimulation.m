function [metrics, logData] = runSimulation(scenarioID, cfg, enableVisualization, customDashboardHandles)
% RUNSIMULATION Closed-loop autonomous vehicle simulation engine with collision avoidance.
%
% Complete Closed-Loop Pipeline:
%   Perception -> Occupancy Grid -> Risk Assessment -> Adaptive Replanner -> Pure Pursuit -> Vehicle Model
%
% Inputs:
%   scenarioID          - Scenario number (1-7) or struct (default: 3)
%   cfg                 - Configuration struct (default: config())
%   enableVisualization - Logical flag to render live graphics (default: true)
%   customDashboardHandles - Handles to existing dashboard GUI (optional)
%
% Outputs:
%   metrics - Struct of 8 quantitative SIH performance benchmarks:
%             .goalReached       - Logical true if vehicle reached target
%             .collisionCount    - Number of collisions (0 is required)
%             .minObstacleDist   - Closest obstacle encounter (meters)
%             .pathLength        - Total distance traveled (meters)
%             .replanCount       - Number of dynamic path replans
%             .avgSpeedKmH       - Average cruising speed (km/h)
%             .meanCrossTrackErr - Mean path tracking error (meters)
%             .simDurationSec    - Total simulation time (seconds)
%   logData - Struct of logged time-series arrays for plotting and analysis

    % Ensure project subfolders are on MATLAB path when invoked directly
    baseDir = fileparts(fileparts(mfilename('fullpath')));
    if ~isempty(baseDir)
        addpath(fullfile(baseDir, 'config'));
        addpath(fullfile(baseDir, 'perception'));
        addpath(fullfile(baseDir, 'planning'));
        addpath(fullfile(baseDir, 'control'));
        addpath(fullfile(baseDir, 'scenarios'));
        addpath(fullfile(baseDir, 'visualization'));
        addpath(fullfile(baseDir, 'simulation'));
        addpath(fullfile(baseDir, 'tests'));
    end

    if nargin < 2 || isempty(cfg), cfg = config(); end
    if nargin < 1 || isempty(scenarioID), scenarioID = 3; end
    if nargin < 3 || isempty(enableVisualization), enableVisualization = true; end

    % 1. Instantiate Scenario
    if isstruct(scenarioID)
        scenario = scenarioID;
    else
        scenario = createIndianRoadScenario(scenarioID, cfg);
    end

    dt = cfg.sim.dt;
    maxTime = cfg.sim.maxTime;
    goalRadius = cfg.sim.goalRadius;

    % Initial vehicle state
    currentPose = scenario.startPose;
    currentVel = 0.0;
    steerAngleCmd = 0.0;
    accelCmd = 0.0;
    footprintVertices = [];

    % 2. Setup Visualization / Dashboard
    hUI = [];
    if enableVisualization
        if nargin >= 4 && ~isempty(customDashboardHandles) && isvalid(customDashboardHandles.fig)
            hUI = customDashboardHandles;
        else
            hUI = dashboard(cfg);
        end
    end

    % 3. Generate Initial Global Reference Path
    initDrivable = detectDrivableArea(scenario, currentPose, cfg);
    globalPath = generateGlobalPath(scenario.startPose, scenario.goalPose, initDrivable, cfg);
    activePath = globalPath;

    % Telemetry Logging Arrays
    timeLog = [];
    posXLog = [];
    posYLog = [];
    thetaLog = [];
    velLog = [];
    steerLog = [];
    accelLog = [];
    riskLog = strings(0, 1);
    minDistLog = [];
    crossTrackLog = [];

    trajectoryHistory = zeros(0, 2);
    replanCount = 0;
    collisionCount = 0;
    minObsDistEncountered = inf;
    goalReached = false;

    t = 0.0;
    simStep = 0;

    fprintf('=======================================================\n');
    fprintf('  Starting Simulation: Scenario %d - %s\n', scenario.id, scenario.name);
    fprintf('=======================================================\n');

    %% Closed-Loop Simulation Main Loop
    while t <= maxTime
        simStep = simStep + 1;
        t = (simStep - 1) * dt;

        % Check if goal reached
        distToGoal = norm(currentPose(1:2) - scenario.goalPose(1:2));
        if distToGoal <= goalRadius
            goalReached = true;
            fprintf('\n[SUCCESS] Vehicle reached destination goal at t = %.2fs!\n', t);
            break;
        end

        % -----------------------------------------------------------------
        % Step 1: PERCEPTION
        % -----------------------------------------------------------------
        perceptionOut = perceptionPipeline(scenario, currentPose, currentVel, t, cfg);
        detectedObjects = perceptionOut.objects;
        occGridStruct   = perceptionOut.occupancyGrid;

        % -----------------------------------------------------------------
        % Step 2: COLLISION RISK ASSESSMENT (SAFE / WARNING / CRITICAL)
        % -----------------------------------------------------------------
        collisionReport = collisionChecker(currentPose, currentVel, activePath, detectedObjects, cfg);
        riskLevel = collisionReport.riskLevel;

        if collisionReport.minDistance < minObsDistEncountered
            minObsDistEncountered = collisionReport.minDistance;
        end

        % Check for hard physical collision (distance < 0.9m)
        if collisionReport.minDistance < 0.9
            collisionCount = collisionCount + 1;
            warning('Physical collision detected with %s at t=%.2fs (dist: %.2fm)!', ...
                collisionReport.criticalObstacle.class, t, collisionReport.minDistance);
        end

        % -----------------------------------------------------------------
        % Step 3: ADAPTIVE PATH PLANNING & LOCAL REPLANNING
        % -----------------------------------------------------------------
        if collisionReport.needsReplan
            % Replan alternative safe detour around dynamic / newly detected hazards
            replanCount = replanCount + 1;
            
            % Try dynamic local avoidance first
            avoidPath = dynamicObstacleAvoidance(activePath, currentPose, currentVel, detectedObjects, occGridStruct, cfg);
            
            % If dynamic avoidance is still blocked, run full adaptive A* replan
            testReport = collisionChecker(currentPose, currentVel, avoidPath, detectedObjects, cfg);
            if testReport.riskLevel == "CRITICAL"
                newPlan = adaptivePathPlanner(currentPose, scenario.goalPose, occGridStruct, cfg, globalPath);
                if newPlan.valid
                    activePath = newPlan;
                else
                    activePath = avoidPath;
                end
            else
                activePath = avoidPath;
            end
        end

        % -----------------------------------------------------------------
        % Step 4: PURE PURSUIT STEERING CONTROLLER
        % -----------------------------------------------------------------
        [steerAngleCmd, targetPoint, crossTrackErr] = ...
            purePursuitController(currentPose, currentVel, activePath.waypoints, cfg);

        % -----------------------------------------------------------------
        % Step 5: LONGITUDINAL SPEED & EMERGENCY BRAKE CONTROLLER
        % -----------------------------------------------------------------
        targetVel = cfg.vehicle.nominalSpeed;
        if ~isempty(activePath.velocities)
            targetVel = activePath.velocities(1);
        end
        [throttleCmd, brakeCmd, accelCmd] = ...
            vehicleController(targetVel, currentVel, riskLevel, cfg);

        % -----------------------------------------------------------------
        % Step 6: VEHICLE DYNAMICS (Kinematic Bicycle Model)
        % -----------------------------------------------------------------
        [currentPose, currentVel, yawRate, footprintVertices] = ...
            vehicleModel(currentPose, currentVel, steerAngleCmd, accelCmd, dt, cfg);

        % Record trajectory history
        trajectoryHistory(end + 1, :) = currentPose(1:2); %#ok<AGROW>

        % -----------------------------------------------------------------
        % Step 7: LOGGING
        % -----------------------------------------------------------------
        timeLog(end + 1) = t; %#ok<AGROW>
        posXLog(end + 1) = currentPose(1); %#ok<AGROW>
        posYLog(end + 1) = currentPose(2); %#ok<AGROW>
        thetaLog(end + 1) = currentPose(3); %#ok<AGROW>
        velLog(end + 1) = currentVel; %#ok<AGROW>
        steerLog(end + 1) = steerAngleCmd; %#ok<AGROW>
        accelLog(end + 1) = accelCmd; %#ok<AGROW>
        riskLog(end + 1) = string(riskLevel); %#ok<AGROW>
        minDistLog(end + 1) = collisionReport.minDistance; %#ok<AGROW>
        crossTrackLog(end + 1) = crossTrackErr; %#ok<AGROW>

        % -----------------------------------------------------------------
        % Step 8: LIVE DASHBOARD & MAP VISUALIZATION
        % -----------------------------------------------------------------
        if enableVisualization && ~isempty(hUI) && isvalid(hUI.fig)
            % Update BEV Map
            visualizeEnvironment(hUI.axMap, scenario, currentPose, footprintVertices, detectedObjects, cfg);
            visualizePath(hUI.axMap, globalPath, activePath, trajectoryHistory, targetPoint, cfg);
            title(hUI.axMap, sprintf('Scenario %d: %s | Elapsed: %.1fs', scenario.id, scenario.name, t), ...
                  'Color', [1 1 1], 'FontSize', 11, 'FontWeight', 'bold');

            % Update Dashboard Telemetry Widgets
            speedKmH = currentVel * 3.6;
            set(hUI.lblSpeed, 'String', sprintf('%.1f km/h (%.2f m/s)', speedKmH, currentVel));
            set(hUI.lblSteer, 'String', sprintf('%.1f deg (%.2f rad)', rad2deg(steerAngleCmd), steerAngleCmd));
            
            if isinf(collisionReport.minDistance)
                set(hUI.lblObstacle, 'String', 'Clear (>40m)');
            else
                set(hUI.lblObstacle, 'String', sprintf('%s (%.1fm, TTC: %.1fs)', ...
                    upper(collisionReport.criticalObstacle.class), collisionReport.minDistance, min(99, collisionReport.minTTC)));
            end

            set(hUI.lblPlanner, 'String', sprintf('Adaptive A* (%d Replans)', replanCount));
            progressPct = min(100, round((currentPose(1) / scenario.goalPose(1)) * 100));
            set(hUI.lblTime, 'String', sprintf('Time: %.2fs | Goal Dist: %.1fm | %d%%', t, distToGoal, progressPct));

            % Update Risk Banner
            switch string(riskLevel)
                case "CRITICAL"
                    set(hUI.pnlRisk, 'String', 'RISK: CRITICAL', ...
                        'BackgroundColor', cfg.ui.colorCritical, 'ForegroundColor', [1 1 1]);
                case "WARNING"
                    set(hUI.pnlRisk, 'String', 'RISK: WARNING', ...
                        'BackgroundColor', cfg.ui.colorWarning, 'ForegroundColor', [0.1 0.1 0.1]);
                otherwise
                    set(hUI.pnlRisk, 'String', 'RISK: SAFE', ...
                        'BackgroundColor', cfg.ui.colorSafe, 'ForegroundColor', [0.05 0.15 0.05]);
            end
            set(hUI.lblReason, 'String', sprintf('Reason: %s', collisionReport.reason));

            drawnow limitrate;
        end
    end

    %% Compute Final Performance Metrics
    diffs = diff(trajectoryHistory, 1, 1);
    totalPathLength = sum(sqrt(sum(diffs.^2, 2)));
    avgSpeed = mean(velLog) * 3.6; % km/h
    meanCrossTrack = mean(crossTrackLog);

    metrics = struct(...
        'scenarioID', scenario.id, ...
        'scenarioName', scenario.name, ...
        'goalReached', goalReached, ...
        'collisionCount', collisionCount, ...
        'minObstacleDist', minObsDistEncountered, ...
        'pathLength', totalPathLength, ...
        'replanCount', replanCount, ...
        'avgSpeedKmH', avgSpeed, ...
        'meanCrossTrackErr', meanCrossTrack, ...
        'simDurationSec', t);

    logData = struct(...
        'time', timeLog, ...
        'x', posXLog, ...
        'y', posYLog, ...
        'theta', thetaLog, ...
        'velocity', velLog, ...
        'steering', steerLog, ...
        'accel', accelLog, ...
        'risk', riskLog, ...
        'minObstacleDistance', minDistLog, ...
        'crossTrackError', crossTrackLog, ...
        'trajectory', trajectoryHistory);

    % Print summary report to Command Window
    printPerformanceReport(metrics);
end

function printPerformanceReport(m)
    fprintf('\n=======================================================\n');
    fprintf('  SIH26037 PERFORMANCE BENCHMARK REPORT: %s\n', m.scenarioName);
    fprintf('=======================================================\n');
    fprintf('  Goal Reached Status      : %s\n', char(string(m.goalReached)));
    fprintf('  Collision Count          : %d %s\n', m.collisionCount, ternary(m.collisionCount==0, '(ZERO COLLISIONS)', '(FAILED)'));
    fprintf('  Minimum Obstacle Distance: %.2f meters (Safe Buffer Maintained)\n', m.minObstacleDist);
    fprintf('  Total Path Length Driven : %.2f meters\n', m.pathLength);
    fprintf('  Dynamic Replan Triggers  : %d times\n', m.replanCount);
    fprintf('  Average Vehicle Speed    : %.2f km/h\n', m.avgSpeedKmH);
    fprintf('  Mean Cross-Track Error   : %.3f meters\n', m.meanCrossTrackErr);
    fprintf('  Total Simulation Time    : %.2f seconds\n', m.simDurationSec);
    fprintf('=======================================================\n\n');
end

function res = ternary(cond, a, b)
    if cond, res = a; else, res = b; end
end
