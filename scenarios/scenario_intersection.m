function scenario = scenario_intersection(cfg)
% SCENARIO_INTERSECTION Scenario 6: Unsignalized rural intersection.
%
% Vehicle approaches an unsignalized 4-way / T-junction with crossing motorcycle and slow tractor.

    if nargin < 1 || isempty(cfg), cfg = config(); end

    scenario = struct();
    scenario.id = 6;
    scenario.name = "Unsignalized Intersection";
    scenario.description = "Unsignalized rural intersection where lateral cross-traffic cuts through without right-of-way.";
    scenario.startPose = [0.0, 0.0, 0.0];
    scenario.goalPose  = [70.0, 0.0, 0.0];

    % Road with widening at junction x = [30, 45]
    leftBound = @(x) 3.5 + 4.0 * (x >= 28 & x <= 44);
    rightBound = @(x) -3.5 - 4.0 * (x >= 28 & x <= 44);

    scenario.road = struct(...
        'leftBoundFunc', leftBound, ...
        'rightBoundFunc', rightBound, ...
        'surfaceType', "asphalt");

    actors = [];

    % 1. Motorcycle crossing from top (+Y) to bottom (-Y) at junction x = 34.0m
    actors(1).id = 601;
    actors(1).class = "motorcycle";
    actors(1).radius = 0.65;
    
    motoCrossX = 35.0;
    motoSpeedY = -3.2;
    actors(1).trajectoryFunc = @(t) deal(...
        [motoCrossX, max(-6.0, 6.0 + motoSpeedY * t)], ...
        [0.0, (6.0 + motoSpeedY * t > -6.0) * motoSpeedY]);
    actors(1).position = [motoCrossX, 6.0];
    actors(1).velocity = [0.0, motoSpeedY];

    % 2. Static broken-down vehicle on far right corner
    actors(2).id = 602;
    actors(2).class = "vehicle";
    actors(2).position = [42.0, -4.5];
    actors(2).velocity = [0.0, 0.0];
    actors(2).radius = 1.4;
    actors(2).trajectoryFunc = [];

    scenario.actors = actors;
end
