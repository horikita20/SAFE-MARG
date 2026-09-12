function scenario = scenario_cattle(cfg)
% SCENARIO_CATTLE Scenario 3: Cattle (Cow) suddenly crossing the road.
%
% Flagship Smart India Hackathon demo:
% A cow enters the road from the right shoulder at slow walking speed (0.8 m/s),
% walks across the vehicle path, causing risk to flip to WARNING/CRITICAL,
% triggering real-time replanning, speed reduction, and successful avoidance.

    if nargin < 1 || isempty(cfg), cfg = config(); end

    scenario = struct();
    scenario.id = 3;
    scenario.name = "Cattle Crossing Road (SIH Flagship Demo)";
    scenario.description = "A stray cow slowly walks across the road from the right shoulder, forcing emergency risk evaluation and replanning.";
    scenario.startPose = [0.0, 0.0, 0.0];
    scenario.goalPose  = [70.0, 0.0, 0.0];

    % Road with slight widening
    scenario.road = struct(...
        'leftBoundFunc', @(x) 3.8 * ones(size(x)), ...
        'rightBoundFunc', @(x) -3.8 * ones(size(x)), ...
        'surfaceType', "patchy_rural");

    actors = [];

    % 1. Moving Cattle (starts at x=28m, y=-3.5m at t=0, moves at vy = +0.7 m/s across road)
    actors(1).id = 301;
    actors(1).class = "cattle";
    actors(1).radius = 1.1;
    
    % Dynamic trajectory function: [pos(t), vel(t)]
    cattleStartX = 28.0;
    cattleStartY = -3.5;
    cattleSpeedY = 0.65; % walks from y = -3.5 to y = +2.5
    
    actors(1).trajectoryFunc = @(t) deal(...
        [cattleStartX + 0.05 * t, min(2.5, cattleStartY + cattleSpeedY * t)], ...
        [0.05, (cattleStartY + cattleSpeedY * t < 2.5) * cattleSpeedY]);
    
    actors(1).position = [cattleStartX, cattleStartY];
    actors(1).velocity = [0.05, cattleSpeedY];

    % 2. Static road barrier further ahead
    actors(2).id = 302;
    actors(2).class = "debris";
    actors(2).position = [55.0, 2.0];
    actors(2).velocity = [0.0, 0.0];
    actors(2).radius = 0.7;
    actors(2).trajectoryFunc = [];

    scenario.actors = actors;
end
