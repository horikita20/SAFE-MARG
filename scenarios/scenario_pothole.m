function scenario = scenario_pothole(cfg)
% SCENARIO_POTHOLE Scenario 2: Potholes on current vehicle path.
%
% Vehicle encounters road depressions/potholes directly in its lane and must maneuver around them safely.

    if nargin < 1 || isempty(cfg), cfg = config(); end

    scenario = struct();
    scenario.id = 2;
    scenario.name = "Potholes on Driving Path";
    scenario.description = "Deep potholes directly situated on the vehicle centerline requiring lateral avoidance.";
    scenario.startPose = [0.0, 0.0, 0.0];
    scenario.goalPose  = [65.0, 0.0, 0.0];

    % Standard 7m wide rural road
    scenario.road = struct(...
        'leftBoundFunc', @(x) 3.5 * ones(size(x)), ...
        'rightBoundFunc', @(x) -3.5 * ones(size(x)), ...
        'surfaceType', "potholed_asphalt");

    actors = [];

    % 1. Pothole 1 at x = 20.0m on center-right
    actors(1).id = 201;
    actors(1).class = "pothole";
    actors(1).position = [20.0, -0.4];
    actors(1).velocity = [0.0, 0.0];
    actors(1).radius = 0.75;
    actors(1).trajectoryFunc = [];

    % 2. Pothole 2 at x = 36.0m on center-left
    actors(2).id = 202;
    actors(2).class = "pothole";
    actors(2).position = [36.0, 0.5];
    actors(2).velocity = [0.0, 0.0];
    actors(2).radius = 0.85;
    actors(2).trajectoryFunc = [];

    % 3. Small cluster pothole at x = 50.0m
    actors(3).id = 203;
    actors(3).class = "pothole";
    actors(3).position = [50.0, -0.8];
    actors(3).velocity = [0.0, 0.0];
    actors(3).radius = 0.70;
    actors(3).trajectoryFunc = [];

    scenario.actors = actors;
end
