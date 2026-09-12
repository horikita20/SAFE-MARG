function scenario = scenario_autoRickshaw(cfg)
% SCENARIO_AUTORICKSHAW Scenario 5: Slow/stopped auto-rickshaw creating obstruction.
%
% An iconic Indian auto-rickshaw stops abruptly in the center of the lane to pick up a passenger,
% requiring ego vehicle to perform safe overtake.

    if nargin < 1 || isempty(cfg), cfg = config(); end

    scenario = struct();
    scenario.id = 5;
    scenario.name = "Auto-Rickshaw Obstruction";
    scenario.description = "A three-wheeled auto-rickshaw decelerates and stops in the middle of the lane.";
    scenario.startPose = [0.0, 0.0, 0.0];
    scenario.goalPose  = [70.0, 0.0, 0.0];

    scenario.road = struct(...
        'leftBoundFunc', @(x) 4.0 * ones(size(x)), ...
        'rightBoundFunc', @(x) -4.0 * ones(size(x)), ...
        'surfaceType', "asphalt");

    actors = [];

    % 1. Auto-rickshaw moving slowly ahead at vx = 2.0 m/s then halting at x = 32m
    actors(1).id = 501;
    actors(1).class = "auto-rickshaw";
    actors(1).radius = 1.2;

    rickshawStartX = 18.0;
    rickshawStartY = 0.2;
    actors(1).trajectoryFunc = @(t) deal(...
        [min(34.0, rickshawStartX + 2.0 * t), rickshawStartY], ...
        [(rickshawStartX + 2.0 * t < 34.0) * 2.0, 0.0]);
    
    actors(1).position = [rickshawStartX, rickshawStartY];
    actors(1).velocity = [2.0, 0.0];

    % 2. Approaching oncoming motorcycle on the opposite lane (y = 2.5m, moving at -4 m/s)
    actors(2).id = 502;
    actors(2).class = "motorcycle";
    actors(2).radius = 0.6;
    actors(2).trajectoryFunc = @(t) deal(...
        [max(0.0, 60.0 - 4.5 * t), 2.6], ...
        [-4.5, 0.0]);
    actors(2).position = [60.0, 2.6];
    actors(2).velocity = [-4.5, 0.0];

    scenario.actors = actors;
end
