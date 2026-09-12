function drivableArea = detectDrivableArea(scenario, egoPose, cfg)
% DETECTDRIVABLEAREA Estimates drivable road corridor and boundaries on unstructured roads.
%
% Inputs:
%   scenario - Road scenario definition struct
%   egoPose  - [x y theta] Vehicle pose
%   cfg      - System configuration struct
%
% Output:
%   drivableArea - Struct containing:
%                  .leftBoundary     - [N x 2] points defining left road edge
%                  .rightBoundary    - [N x 2] points defining right road edge
%                  .centerline       - [N x 2] points along road center
%                  .roadWidth        - Estimated road width (m)
%                  .surfaceType      - "asphalt" | "unpaved_gravel" | "patchy_rural"
%                  .inDrivableArea   - Function handle: @(x,y) checks if (x,y) is within road

    if nargin < 3 || isempty(cfg)
        cfg = config();
    end

    xMin = cfg.grid.xMin;
    xMax = cfg.grid.xMax;
    numPts = 100;
    xSamples = linspace(xMin, xMax, numPts)';

    if isfield(scenario, 'road')
        road = scenario.road;
        leftBoundFunc = road.leftBoundFunc;
        rightBoundFunc = road.rightBoundFunc;
        surfaceType = road.surfaceType;
    else
        % Default straight rural road (width 7.0m, centerline at y=0)
        halfWidth = 3.5;
        leftBoundFunc = @(x) halfWidth * ones(size(x));
        rightBoundFunc = @(x) -halfWidth * ones(size(x));
        surfaceType = "patchy_rural";
    end

    yLeft = leftBoundFunc(xSamples);
    yRight = rightBoundFunc(xSamples);
    yCenter = (yLeft + yRight) / 2.0;

    leftBoundary = [xSamples, yLeft];
    rightBoundary = [xSamples, yRight];
    centerline = [xSamples, yCenter];
    roadWidth = mean(yLeft - yRight);

    % Checker function: tests whether [x, y] falls between left and right boundaries
    inDrivableArea = @(x, y) (y <= leftBoundFunc(x)) & (y >= rightBoundFunc(x));

    drivableArea = struct(...
        'leftBoundary', leftBoundary, ...
        'rightBoundary', rightBoundary, ...
        'centerline', centerline, ...
        'roadWidth', roadWidth, ...
        'surfaceType', string(surfaceType), ...
        'inDrivableArea', inDrivableArea, ...
        'leftBoundFunc', leftBoundFunc, ...
        'rightBoundFunc', rightBoundFunc);
end
