function scenario = scenario_mixedTraffic(cfg)
% SCENARIO_MIXEDTRAFFIC Scenario 7: Mixed unstructured traffic with multiple concurrent hazards.
%
% Stress test scenario:
% Combines moving cattle, crossing pedestrian, overtaking auto-rickshaw, and road potholes.

    if nargin < 1 || isempty(cfg), cfg = config(); end

    scenario = struct();
    scenario.id = 7;
    scenario.name = "Chaotic Mixed Indian Traffic";
    scenario.description = "Multiple concurrent hazards: walking cattle, sudden pedestrian, auto-rickshaw, and road surface potholes.";
    scenario.startPose = [0.0, 0.0, 0.0];
    scenario.goalPose  = [75.0, 0.0, 0.0];

    scenario.road = struct(...
        'leftBoundFunc', @(x) 4.2 * ones(size(x)), ...
        'rightBoundFunc', @(x) -4.2 * ones(size(x)), ...
        'surfaceType', "patchy_rural");

    actors = [];

    % 1. Pothole at x = 16.0m
    actors(1).id = 701;
    actors(1).class = "pothole";
    actors(1).position = [16.0, -0.6];
    actors(1).velocity = [0.0, 0.0];
    actors(1).radius = 0.75;
    actors(1).trajectoryFunc = [];

    % 2. Cattle crossing slowly at x = 30.0m
    actors(2).id = 702;
    actors(2).class = "cattle";
    actors(2).radius = 1.1;
    cowSpeed = 0.55;
    actors(2).trajectoryFunc = @(t) deal(...
        [30.0 + 0.05 * t, min(2.5, -3.8 + cowSpeed * t)], ...
        [0.05, (-3.8 + cowSpeed * t < 2.5) * cowSpeed]);
    actors(2).position = [30.0, -3.8];
    actors(2).velocity = [0.05, cowSpeed];

    % 3. Slow auto-rickshaw ahead at x = 45.0m
    actors(3).id = 703;
    actors(3).class = "auto-rickshaw";
    actors(3).radius = 1.2;
    actors(3).trajectoryFunc = @(t) deal(...
        [min(60.0, 38.0 + 1.8 * t), -1.2], ...
        [(38.0 + 1.8 * t < 60.0) * 1.8, 0.0]);
    actors(3).position = [38.0, -1.2];
    actors(3).velocity = [1.8, 0.0];

    % 4. Pedestrian on left at x = 55.0m
    actors(4).id = 704;
    actors(4).class = "pedestrian";
    actors(4).radius = 0.5;
    actors(4).trajectoryFunc = @(t) deal(...
        [55.0, max(0.5, 3.8 - 0.9 * max(0, t - 3.0))], ...
        [0.0, ((t >= 3.0) && (3.8 - 0.9 * (t - 3.0) > 0.5)) * -0.9]);
    actors(4).position = [55.0, 3.8];
    actors(4).velocity = [0.0, -0.9];

    scenario.actors = actors;
end
