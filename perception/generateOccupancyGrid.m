function occGridStruct = generateOccupancyGrid(detectedObjects, drivableArea, cfg, egoPose)
% GENERATEOCCUPANCYGRID Creates a 3-state spatial occupancy grid with obstacle inflation.
%
% Grid State Encoding:
%   0.0 = FREE SPACE
%   1.0 = OBSTACLE / BLOCKED / OFF-ROAD
%   0.5 = UNKNOWN / UNOBSERVED
%
% Inputs:
%   detectedObjects - Array of detected obstacle structs
%   drivableArea    - Drivable corridor struct (from detectDrivableArea)
%   cfg             - Configuration struct
%   egoPose         - Current vehicle pose [x y theta]
%
% Output:
%   occGridStruct - Struct containing:
%                   .grid          - [Ny x Nx] double matrix with values in [0, 1]
%                   .xVector       - 1 x Nx vector of X coordinates (meters)
%                   .yVector       - 1 x Ny vector of Y coordinates (meters)
%                   .XMesh         - Ny x Nx meshgrid of X
%                   .YMesh         - Ny x Nx meshgrid of Y
%                   .resolution    - Grid cell size (m/cell)
%                   .isOccupied    - Function handle: @(x,y) true if cell >= 0.8
%                   .getCost       - Function handle: @(x,y) returns traversal cost

    if nargin < 3 || isempty(cfg)
        cfg = config();
    end

    xMin = cfg.grid.xMin;
    xMax = cfg.grid.xMax;
    yMin = cfg.grid.yMin;
    yMax = cfg.grid.yMax;
    res  = cfg.grid.resolution;
    valFree = cfg.grid.valFree;
    valObs  = cfg.grid.valObstacle;
    valUnk  = cfg.grid.valUnknown;

    xVec = xMin:res:xMax;
    yVec = yMin:res:yMax;
    Nx = numel(xVec);
    Ny = numel(yVec);

    [XGrid, YGrid] = meshgrid(xVec, yVec);

    % Initialize grid: Default to free space within sensor perception horizon, unknown outside
    grid = valFree * ones(Ny, Nx);

    % 1. Mark non-drivable / off-road regions as OBSTACLE (1.0)
    if ~isempty(drivableArea) && isfield(drivableArea, 'inDrivableArea')
        inRoad = drivableArea.inDrivableArea(XGrid, YGrid);
        grid(~inRoad) = valObs;
    end

    % 2. Mark detected obstacles with safety inflation
    numObs = numel(detectedObjects);
    for k = 1:numObs
        obj = detectedObjects(k);
        obsX = obj.worldPosition(1);
        obsY = obj.worldPosition(2);
        
        % Base radius + safety inflation margin
        if obj.class == "pothole"
            inflRadius = obj.radius + cfg.grid.potholeInflationRadius;
        else
            inflRadius = obj.radius + cfg.grid.inflationRadius;
        end

        % Compute Euclidean distance from each grid cell to obstacle center
        distSq = (XGrid - obsX).^2 + (YGrid - obsY).^2;
        
        % Hard obstacle core + inflation zone
        obsCoreMask = distSq <= (obj.radius)^2;
        obsInflMask = distSq <= (inflRadius)^2;

        grid(obsInflMask) = max(grid(obsInflMask), 0.85); % High cost inflation zone
        grid(obsCoreMask) = valObs;                        % Hard obstacle core (1.0)
    end

    % 3. Helper querying functions
    % worldToGrid index conversion
    worldToGrid = @(xw, yw) deal(...
        max(1, min(Ny, round((yw - yMin) / res) + 1)), ...
        max(1, min(Nx, round((xw - xMin) / res) + 1)));

    % Check if given world coordinate is occupied
    isOccupied = @(xw, yw) checkOccupied(xw, yw, grid, xMin, xMax, yMin, yMax, res, valObs);

    % Traversal cost lookup (higher cost near obstacles)
    getCost = @(xw, yw) getTraversalCost(xw, yw, grid, xMin, xMax, yMin, yMax, res);

    occGridStruct = struct(...
        'grid', grid, ...
        'xVector', xVec, ...
        'yVector', yVec, ...
        'XMesh', XGrid, ...
        'YMesh', YGrid, ...
        'resolution', res, ...
        'xMin', xMin, ...
        'xMax', xMax, ...
        'yMin', yMin, ...
        'yMax', yMax, ...
        'worldToGrid', worldToGrid, ...
        'isOccupied', isOccupied, ...
        'getCost', getCost);
end

function occ = checkOccupied(xw, yw, grid, xMin, xMax, yMin, yMax, res, valObs)
    % Bounds check
    if xw < xMin || xw > xMax || yw < yMin || yw > yMax
        occ = true;
        return;
    end
    [Ny, Nx] = size(grid);
    iy = max(1, min(Ny, round((yw - yMin) / res) + 1));
    ix = max(1, min(Nx, round((xw - xMin) / res) + 1));
    occ = grid(iy, ix) >= 0.8;
end

function cost = getTraversalCost(xw, yw, grid, xMin, xMax, yMin, yMax, res)
    if xw < xMin || xw > xMax || yw < yMin || yw > yMax
        cost = 1000.0;
        return;
    end
    [Ny, Nx] = size(grid);
    iy = max(1, min(Ny, round((yw - yMin) / res) + 1));
    ix = max(1, min(Nx, round((xw - xMin) / res) + 1));
    cost = 1.0 + 10.0 * grid(iy, ix);
end
