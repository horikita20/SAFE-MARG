function scenario = scenario_pedestrian(cfg)
% SCENARIO_PEDESTRIAN Scenario 4: Pedestrian unexpectedly entering road.
%
% A pedestrian steps out from behind a parked handcart on the left shoulder and crosses the road.

    if nargin < 1 || isempty(cfg), cfg = config(); end

    scenario = struct();
    scenario.id = 4;
    scenario.name = "Pedestrian Entering Road";
    scenario.description = "A pedestrian steps out from the left margin into the vehicle path at x=24m.";
    scenario.startPose = [0.0, 0.0, 0.0];
    scenario.goalPose  = [65.0, 0.0, 0.0];

    scenario.road = struct(...
        'leftBoundFunc', @(x) 3.6 * ones(size(x)), ...
        'rightBoundFunc', @(x) -3.6 * ones(size(x)), ...
        'surfaceType', "asphalt");

    actors = [];

    % 1. Parked cart obscuring sightline
    actors(1).id = 401;
    actors(1).class = "debris";
    actors(1).position = [22.0, 2.5];
    actors(1).velocity = [0.0, 0.0];
    actors(1).radius = 0.8;
    actors(1).trajectoryFunc = [];

    % 2. Moving pedestrian stepping onto road (starts at t=1.5s, crosses from y=3.0m to y=-1.5m)
    actors(2).id = 402;
    actors(2).class = "pedestrian";
    actors(2).radius = 0.5;
    
    pedX = 25.0;
    pedSpeed = -1.1; % Walking left to right
    actors(2).trajectoryFunc = @(t) deal(...
        [pedX, max(-2.0, 3.2 + pedSpeed * max(0, t - 1.0))], ...
        [0.0, ((t >= 1.0) && (3.2 + pedSpeed * (t - 1.0) > -2.0)) * pedSpeed]);
    
    actors(2).position = [pedX, 3.2];
    actors(2).velocity = [0.0, pedSpeed];

    scenario.actors = actors;
end
