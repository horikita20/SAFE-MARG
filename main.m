function main()
% MAIN Top-level launcher for SIH26037 Smart Autonomous Vehicle navigation system.
%
% Problem Statement:
%   SIH26037 — Adaptive Path Planning and Collision Avoidance for Autonomous
%   Vehicles on Unstructured Indian Roads.
%
% Usage:
%   main()               % Launches interactive menu and SIH demo
%   runSimulation(3)     % Direct launch of Scenario 3 (Cattle Crossing Demo)

    clc;
    close all;

    % Setup project paths
    baseDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(baseDir, 'config'));
    addpath(fullfile(baseDir, 'perception'));
    addpath(fullfile(baseDir, 'planning'));
    addpath(fullfile(baseDir, 'control'));
    addpath(fullfile(baseDir, 'scenarios'));
    addpath(fullfile(baseDir, 'visualization'));
    addpath(fullfile(baseDir, 'simulation'));
    addpath(fullfile(baseDir, 'tests'));

    cfg = config();

    fprintf('========================================================================\n');
    fprintf('   SMART INDIA HACKATHON 2026 (SIH26037)                                \n');
    fprintf('   Smart Autonomous Vehicle — Adaptive Indian Road Navigation           \n');
    fprintf('   All-MATLAB Perception, Planning, Dynamics & Collision Avoidance Stack \n');
    fprintf('========================================================================\n\n');

    fprintf('Available Scenarios:\n');
    fprintf('  [1] Narrow Unmarked Rural Road (Shoulder Obstacles & Bottleneck)\n');
    fprintf('  [2] Potholes on Driving Path (Road Surface Depression Avoidance)\n');
    fprintf('  [3] Cattle Crossing Road (Flagship SIH Demonstration)\n');
    fprintf('  [4] Pedestrian Entering Road (Sudden Cross-Pedestrian)\n');
    fprintf('  [5] Auto-Rickshaw Obstruction (Overtaking Stopped 3-Wheeler)\n');
    fprintf('  [6] Unsignalized Rural Intersection (Crossing Traffic)\n');
    fprintf('  [7] Chaotic Mixed Traffic (Multi-hazard stress test)\n');
    fprintf('  [8] Run Complete Automated Test Suite (All 7 Scenarios)\n');
    fprintf('  [0] Exit\n\n');

    % Prompt user selection (defaults to 3 for instant demo)
    choice = input('Select Scenario to run [Default: 3 (Cattle Crossing)]: ', 's');
    if isempty(choice)
        selectedScenario = 3;
    else
        selectedScenario = str2double(choice);
    end

    if selectedScenario == 0
        fprintf('Exiting Smart Autonomous Vehicle system.\n');
        return;
    elseif selectedScenario == 8
        testPipeline();
        return;
    elseif isnan(selectedScenario) || selectedScenario < 1 || selectedScenario > 7
        fprintf('Invalid selection. Defaulting to Scenario 3 (Cattle Crossing Demo).\n');
        selectedScenario = 3;
    end

    % Run Closed-Loop Simulation
    [metrics, logData] = runSimulation(selectedScenario, cfg, true);

    % Plot Comprehensive Post-Simulation Metrics Dashboard
    plotPerformanceTelemetry(logData, metrics, cfg);
end

% -------------------------------------------------------------------------
% Helper: Comprehensive Post-Simulation Telemetry Plotter
% -------------------------------------------------------------------------
function plotPerformanceTelemetry(logData, metrics, cfg)
    if isempty(logData) || isempty(logData.time)
        return;
    end

    t = logData.time;
    vKmH = logData.velocity * 3.6;
    steerDeg = rad2deg(logData.steering);
    minDist = logData.minObstacleDistance;
    cte = logData.crossTrackError;

    figTel = figure('Name', sprintf('Performance Telemetry - Scenario %d: %s', metrics.scenarioID, metrics.scenarioName), ...
                    'Color', [0.12, 0.14, 0.18], 'Position', [120, 120, 1050, 680]);

    % Subplot 1: Vehicle Speed Profile
    subplot(2, 2, 1);
    plot(t, vKmH, 'Color', [0.0, 0.85, 1.0], 'LineWidth', 2.0);
    grid on;
    set(gca, 'Color', [0.18, 0.20, 0.25], 'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
    xlabel('Time (s)', 'Color', [0.8 0.8 0.8]);
    ylabel('Speed (km/h)', 'Color', [0.8 0.8 0.8]);
    title('Longitudinal Velocity Profile', 'Color', [1 1 1], 'FontWeight', 'bold');

    % Subplot 2: Front Wheel Steering Angle
    subplot(2, 2, 2);
    plot(t, steerDeg, 'Color', [1.0, 0.75, 0.1], 'LineWidth', 2.0);
    grid on;
    set(gca, 'Color', [0.18, 0.20, 0.25], 'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
    xlabel('Time (s)', 'Color', [0.8 0.8 0.8]);
    ylabel('Steer Angle (deg)', 'Color', [0.8 0.8 0.8]);
    title('Steering Actuation (Pure Pursuit)', 'Color', [1 1 1], 'FontWeight', 'bold');

    % Subplot 3: Minimum Obstacle Distance vs Time
    subplot(2, 2, 3);
    validDist = minDist;
    validDist(isinf(validDist)) = 40.0; % Cap inf for clean visualization
    plot(t, validDist, 'Color', [0.95, 0.40, 0.40], 'LineWidth', 2.0);
    yline(cfg.risk.distCritical, '--r', 'Critical Buffer (3.5m)', 'LineWidth', 1.5);
    yline(cfg.risk.distWarning, '--y', 'Warning Buffer (8.0m)', 'LineWidth', 1.2);
    grid on;
    set(gca, 'Color', [0.18, 0.20, 0.25], 'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
    xlabel('Time (s)', 'Color', [0.8 0.8 0.8]);
    ylabel('Clearance Distance (m)', 'Color', [0.8 0.8 0.8]);
    title('Obstacle Clearance Distance', 'Color', [1 1 1], 'FontWeight', 'bold');

    % Subplot 4: Cross-Track Error
    subplot(2, 2, 4);
    plot(t, cte, 'Color', [0.2, 0.9, 0.4], 'LineWidth', 2.0);
    grid on;
    set(gca, 'Color', [0.18, 0.20, 0.25], 'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
    xlabel('Time (s)', 'Color', [0.8 0.8 0.8]);
    ylabel('Tracking Error (m)', 'Color', [0.8 0.8 0.8]);
    title('Cross-Track Path Tracking Error', 'Color', [1 1 1], 'FontWeight', 'bold');
end
