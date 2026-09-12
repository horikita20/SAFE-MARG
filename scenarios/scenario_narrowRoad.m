function scenario = scenario_narrowRoad(cfg)
% SCENARIO_NARROWROAD Scenario 1: Narrow unmarked rural road with road margin obstacles.
%
% Road narrows from 6.5m to 4.2m with parked handcarts and dirt debris on the shoulders.

    if nargin < 1 || isempty(cfg), cfg = config(); end

    scenario = struct();
    scenario.id = 1;
    scenario.name = "Narrow Unmarked Rural Road";
    scenario.description = "Narrow rural road with no lane markings, shoulder debris, and parked handcarts.";
    scenario.startPose = [0.0, 0.0, 0.0];
    scenario.goalPose  = [65.0, 0.0, 0.0];

    % Road with bottleneck narrowing at x = 25m to 45m
    leftBound  = @(x) 3.25 - 1.0 * (x >= 25 & x <= 45);
    rightBound = @(x) -3.25 + 1.0 * (x >= 25 & x <= 45);

    scenario.road = struct(...
        'leftBoundFunc', leftBound, ...
        'rightBoundFunc', rightBound, ...
        'surfaceType', "unpaved_gravel");

    % Static obstacles on road margins
    actors = [];

    % 1. Parked handcart near bottleneck
    actors(1).id = 101;
    actors(1).class = "debris";
    actors(1).position = [22.0, 1.8];
    actors(1).velocity = [0.0, 0.0];
    actors(1).radius = 0.8;
    actors(1).trajectoryFunc = [];

    % 2. Static pile of stones/rubble on right margin
    actors(2).id = 102;
    actors(2).class = "debris";
    actors(2).position = [35.0, -1.8];
    actors(2).velocity = [0.0, 0.0];
    actors(2).radius = 0.7;
    actors(2).trajectoryFunc = [];

    % 3. Parked bicycle further ahead
    actors(3).id = 103;
    actors(3).class = "vehicle";
    actors(3).position = [48.0, 1.6];
    actors(3).velocity = [0.0, 0.0];
    actors(3).radius = 0.6;
    actors(3).trajectoryFunc = [];

    scenario.actors = actors;
end
