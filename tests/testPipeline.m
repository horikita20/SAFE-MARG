function results = testPipeline()
% TESTPIPELINE Comprehensive automated test runner for SIH26037 Autonomous Vehicle.
%
% Runs:
%   Part 1: Subsystem Unit Tests (Config, Perception, Grid, Planner, Control, Model)
%   Part 2: End-to-End Batch Simulation across ALL 7 Indian Road Scenarios
%
% Usage:
%   results = testPipeline()

    % Add project folders to MATLAB path
    baseDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(baseDir, 'config'));
    addpath(fullfile(baseDir, 'perception'));
    addpath(fullfile(baseDir, 'planning'));
    addpath(fullfile(baseDir, 'control'));
    addpath(fullfile(baseDir, 'scenarios'));
    addpath(fullfile(baseDir, 'visualization'));
    addpath(fullfile(baseDir, 'simulation'));
    addpath(fullfile(baseDir, 'tests'));

    fprintf('========================================================================\n');
    fprintf('   SIH26037 AUTOMATED VERIFICATION & TEST SUITE                         \n');
    fprintf('========================================================================\n\n');

    %% PART 1: SUBSYSTEM UNIT TESTS
    unitTests = {
        @testConfigParams, ...
        @testPerceptionAndGrid, ...
        @testGlobalPathGeneration, ...
        @testAdaptiveAStarPlanner, ...
        @testCollisionChecker, ...
        @testPurePursuitController, ...
        @testVehicleModelDynamics ...
    };

    numUnits = numel(unitTests);
    unitPassed = 0;
    fprintf('--- Part 1: Subsystem Unit Tests (%d tests) ---\n', numUnits);

    for i = 1:numUnits
        testFn = unitTests{i};
        fnName = func2str(testFn);
        fprintf('  [TEST %d/%d] %s...', i, numUnits, fnName);
        try
            testFn();
            fprintf(' [PASSED]\n');
            unitPassed = unitPassed + 1;
        catch ME
            fprintf(' [FAILED]: %s\n', ME.message);
        end
    end

    %% PART 2: END-TO-END SCENARIO BENCHMARKS (ALL 7 SCENARIOS)
    fprintf('\n--- Part 2: End-to-End Scenario Verification (Scenarios 1-7) ---\n');
    cfg = config();
    scenarioResults = struct('id', {}, 'name', {}, 'goalReached', {}, 'collisions', {}, 'minDist', {}, 'replans', {});

    allPassed = (unitPassed == numUnits);

    for sID = 1:7
        fprintf('  [SIMULATING SCENARIO %d]...', sID);
        try
            metrics = runSimulation(sID, cfg, false); % Headless fast simulation
            scenarioResults(sID).id = metrics.scenarioID;
            scenarioResults(sID).name = metrics.scenarioName;
            scenarioResults(sID).goalReached = metrics.goalReached;
            scenarioResults(sID).collisions = metrics.collisionCount;
            scenarioResults(sID).minDist = metrics.minObstacleDist;
            scenarioResults(sID).replans = metrics.replanCount;

            if metrics.goalReached && metrics.collisionCount == 0
                fprintf(' [SUCCESS] Reached Goal | 0 Collisions | Min Clear: %.2fm\n', metrics.minObstacleDist);
            else
                fprintf(' [FAILED] Goal: %s, Collisions: %d\n', string(metrics.goalReached), metrics.collisionCount);
                allPassed = false;
            end
        catch ME
            fprintf(' [ERROR]: %s\n', ME.message);
            allPassed = false;
        end
    end

    %% PRINT FINAL SUMMARY TABLE
    fprintf('\n========================================================================\n');
    fprintf('   FINAL VERIFICATION SUMMARY TABLE                                    \n');
    fprintf('========================================================================\n');
    fprintf('  ID | Scenario Name                   | Goal Reached | Collisions | Min Dist\n');
    fprintf('------------------------------------------------------------------------\n');
    for k = 1:numel(scenarioResults)
        r = scenarioResults(k);
        fprintf('  %2d | %-31s | %-12s | %10d | %6.2fm\n', ...
            r.id, r.name, string(r.goalReached), r.collisions, r.minDist);
    end
    fprintf('========================================================================\n');
    if allPassed
        fprintf('   >>> OVERALL RESULT: ALL TESTS & SCENARIOS PASSED (100%%) <<<\n');
    else
        fprintf('   >>> OVERALL RESULT: SOME TESTS/SCENARIOS FAILED <<<\n');
    end
    fprintf('========================================================================\n\n');

    results = scenarioResults;
end

% -------------------------------------------------------------------------
% Unit Test 1: Config
% -------------------------------------------------------------------------
function testConfigParams()
    cfg = config();
    assert(isfield(cfg, 'vehicle') && isfield(cfg, 'grid') && isfield(cfg, 'planner') && isfield(cfg, 'risk'), ...
        'Config missing core categories');
    assert(cfg.vehicle.wheelbase > 0, 'Invalid wheelbase');
    assert(cfg.grid.resolution > 0, 'Invalid grid resolution');
end

% -------------------------------------------------------------------------
% Unit Test 2: Perception & Occupancy Grid
% -------------------------------------------------------------------------
function testPerceptionAndGrid()
    cfg = config();
    scenario = scenario_cattle(cfg);
    egoPose = [0.0, 0.0, 0.0];
    
    perc = perceptionPipeline(scenario, egoPose, 5.0, 0.0, cfg);
    assert(isfield(perc, 'objects') && isfield(perc, 'occupancyGrid'), 'Missing perception fields');
    assert(numel(perc.objects) >= 1, 'Expected at least 1 detected obstacle');
    
    grid = perc.occupancyGrid.grid;
    assert(isa(grid, 'double'), 'Occupancy grid must be double');
    assert(any(grid(:) == cfg.grid.valObstacle), 'Expected obstacle cells in grid');
end

% -------------------------------------------------------------------------
% Unit Test 3: Global Path Generation
% -------------------------------------------------------------------------
function testGlobalPathGeneration()
    cfg = config();
    scenario = scenario_narrowRoad(cfg);
    drivable = detectDrivableArea(scenario, [0 0 0], cfg);
    
    gPath = generateGlobalPath(scenario.startPose, scenario.goalPose, drivable, cfg);
    assert(isfield(gPath, 'waypoints') && size(gPath.waypoints, 1) > 10, 'Invalid global path waypoints');
    assert(gPath.length > 30.0, 'Global path length too short');
end

% -------------------------------------------------------------------------
% Unit Test 4: Adaptive A* Path Planner
% -------------------------------------------------------------------------
function testAdaptiveAStarPlanner()
    cfg = config();
    scenario = scenario_pothole(cfg);
    perc = perceptionPipeline(scenario, [0 0 0], 5.0, 0.0, cfg);
    
    plan = adaptivePathPlanner([0 0 0], [40 0 0], perc.occupancyGrid, cfg);
    assert(isfield(plan, 'waypoints') && size(plan.waypoints, 1) >= 2, 'A* planner failed to generate path');
    assert(plan.valid, 'Planner reported invalid path');
end

% -------------------------------------------------------------------------
% Unit Test 5: Collision Checker
% -------------------------------------------------------------------------
function testCollisionChecker()
    cfg = config();
    % Place obstacle directly 3m ahead
    dummyObs = struct('id', 1, 'class', "cattle", 'worldPosition', [3.0, 0.0], ...
                      'worldVelocity', [0 0], 'radius', 1.0);
    path = struct('waypoints', [linspace(0, 20, 40)', zeros(40, 1)]);
    
    report = collisionChecker([0 0 0], 6.0, path, dummyObs, cfg);
    assert(report.riskLevel == "CRITICAL", 'Expected CRITICAL risk for obstacle 3m ahead');
    assert(report.needsReplan == true, 'Expected needsReplan = true for CRITICAL risk');
end

% -------------------------------------------------------------------------
% Unit Test 6: Pure Pursuit Controller
% -------------------------------------------------------------------------
function testPurePursuitController()
    cfg = config();
    % Straight path along y = 2.0m, vehicle at y = 0.0m
    waypoints = [linspace(0, 30, 60)', 2.0 * ones(60, 1)];
    
    [steerCmd, targetPt, cte] = purePursuitController([0 0 0], 5.0, waypoints, cfg);
    assert(steerCmd > 0, 'Vehicle to the right of path should steer LEFT (steerCmd > 0)');
    assert(abs(cte - 2.0) < 0.2, 'Cross track error mismatch');
end

% -------------------------------------------------------------------------
% Unit Test 7: Vehicle Model Dynamics
% -------------------------------------------------------------------------
function testVehicleModelDynamics()
    cfg = config();
    pose0 = [0.0, 0.0, 0.0];
    v0 = 5.0;
    steer = 0.1; % Steer left
    accel = 1.0; % Accelerate
    dt = 0.05;

    [pose1, v1, yawRate, vertices] = vehicleModel(pose0, v0, steer, accel, dt, cfg);
    assert(pose1(1) > pose0(1), 'Vehicle must move forward');
    assert(pose1(3) > pose0(3), 'Steering left must increase heading theta');
    assert(v1 > v0, 'Positive acceleration must increase velocity');
    assert(isequal(size(vertices), [4 2]), 'Vehicle footprint must have 4 vertices');
end
