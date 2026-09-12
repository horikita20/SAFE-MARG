function ax = visualizeEnvironment(ax, scenario, egoPose, footprintVertices, detectedObjects, cfg)
% VISUALIZEENVIRONMENT Multi-layer bird's-eye-view road environment and obstacle renderer.
%
% Inputs:
%   ax                - MATLAB Axes handle to draw on
%   scenario          - Active scenario definition struct
%   egoPose           - Current vehicle pose [x y theta]
%   footprintVertices - [4 x 2] coordinates of 4 vehicle corners
%   detectedObjects   - Array of detected obstacle structs
%   cfg               - System configuration struct

    if nargin < 6 || isempty(cfg), cfg = config(); end
    if isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Smart Autonomous Vehicle - Indian Road Navigation', ...
                     'Color', cfg.ui.bgDark, 'Position', [100, 100, 1100, 650]);
        ax = axes('Parent', fig, 'Color', cfg.ui.bgDark);
    end

    cla(ax);
    hold(ax, 'on');

    xMin = cfg.grid.xMin;
    xMax = cfg.grid.xMax;
    yMin = cfg.grid.yMin;
    yMax = cfg.grid.yMax;

    % 1. Draw Road Shoulders (Off-road dirt/gravel background)
    fill(ax, [xMin, xMax, xMax, xMin], [yMin, yMin, yMax, yMax], ...
         cfg.ui.shoulderColor, 'EdgeColor', 'none', 'FaceAlpha', 0.9);

    % 2. Draw Drivable Road Surface
    numPts = 120;
    xRoad = linspace(xMin, xMax, numPts)';
    if isfield(scenario, 'road')
        yL = scenario.road.leftBoundFunc(xRoad);
        yR = scenario.road.rightBoundFunc(xRoad);
    else
        yL = 3.5 * ones(size(xRoad));
        yR = -3.5 * ones(size(xRoad));
    end

    roadPolyX = [xRoad; flipud(xRoad)];
    roadPolyY = [yL; flipud(yR)];
    fill(ax, roadPolyX, roadPolyY, cfg.ui.roadColor, 'EdgeColor', [0.8, 0.8, 0.8], 'LineWidth', 1.5);

    % Centerline (subtle dashed)
    yCenter = (yL + yR) / 2.0;
    plot(ax, xRoad, yCenter, '--', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.0);

    % 3. Draw Destination Goal Marker
    if isfield(scenario, 'goalPose')
        gx = scenario.goalPose(1);
        gy = scenario.goalPose(2);
        plot(ax, gx, gy, 'p', 'MarkerSize', 14, 'MarkerFaceColor', cfg.ui.goalColor, ...
             'MarkerEdgeColor', [1 1 1], 'LineWidth', 1.5);
        drawCircle(ax, gx, gy, 1.5, [1.0, 0.3, 0.3], ':', 1.2);
        text(ax, gx, gy + 1.8, 'GOAL', 'Color', [1, 0.4, 0.4], 'FontWeight', 'bold', ...
             'FontSize', 9, 'HorizontalAlignment', 'center');
    end

    % 4. Draw Detected Obstacles & Dynamic Agents
    numObs = numel(detectedObjects);
    for k = 1:numObs
        obj = detectedObjects(k);
        ox = obj.worldPosition(1);
        oy = obj.worldPosition(2);
        r  = obj.radius;
        cName = lower(string(obj.class));

        % Determine color and shape based on class
        switch cName
            case "cattle"
                col = cfg.ui.colorCattle;
                lbl = "COW";
                drawOrientedBox(ax, ox, oy, 2.0, 1.1, 0, col);
            case "pedestrian"
                col = cfg.ui.colorPedestrian;
                lbl = "PEDESTRIAN";
                drawCircle(ax, ox, oy, r, col, '-', 2.0);
                fill(ax, ox + r*cos(0:0.3:2*pi), oy + r*sin(0:0.3:2*pi), col, 'FaceAlpha', 0.8);
            case "auto-rickshaw"
                col = cfg.ui.colorAutoRickshaw;
                lbl = "AUTO";
                drawOrientedBox(ax, ox, oy, 2.4, 1.4, 0, col);
            case "vehicle"
                col = cfg.ui.colorVehicle;
                lbl = "VEHICLE";
                drawOrientedBox(ax, ox, oy, 3.8, 1.8, 0, col);
            case "motorcycle"
                col = cfg.ui.colorMotorcycle;
                lbl = "MOTORCYCLE";
                drawOrientedBox(ax, ox, oy, 1.8, 0.7, 0, col);
            case "pothole"
                col = cfg.ui.colorPothole;
                lbl = "POTHOLE";
                drawCircle(ax, ox, oy, r, [0.9, 0.2, 0.2], '-', 2.0);
                fill(ax, ox + r*cos(0:0.3:2*pi), oy + r*sin(0:0.3:2*pi), col, 'FaceAlpha', 0.95);
            case "debris"
                col = cfg.ui.colorDebris;
                lbl = "DEBRIS";
                drawOrientedBox(ax, ox, oy, 1.4, 1.2, pi/6, col);
            otherwise
                col = cfg.ui.colorUnknown;
                lbl = "OBSTACLE";
                drawCircle(ax, ox, oy, r, col, '-', 1.5);
        end

        % Draw Safety Inflation Ring (translucent buffer zone)
        drawCircle(ax, ox, oy, r + cfg.grid.inflationRadius, col, ':', 1.0);

        % Velocity vector for dynamic obstacles
        if norm(obj.worldVelocity) > 0.1
            quiver(ax, ox, oy, obj.worldVelocity(1)*1.5, obj.worldVelocity(2)*1.5, ...
                   0, 'Color', [1.0, 0.9, 0.2], 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
        end

        % Label
        text(ax, ox, oy + r + 0.9, sprintf('%s (%.1fm)', lbl, obj.distance), ...
             'Color', col, 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end

    % 5. Draw Ego Vehicle (Cyan chassis + heading indicator)
    if ~isempty(footprintVertices)
        fill(ax, footprintVertices(:, 1), footprintVertices(:, 2), cfg.ui.egoColor, ...
             'EdgeColor', [1 1 1], 'LineWidth', 1.8, 'FaceAlpha', 0.9);
        
        % Heading arrow from rear axle to front
        ex = egoPose(1);
        ey = egoPose(2);
        eth = egoPose(3);
        quiver(ax, ex, ey, 2.5*cos(eth), 2.5*sin(eth), 0, ...
               'Color', [0.1, 0.1, 0.1], 'LineWidth', 2.5, 'MaxHeadSize', 0.7);
        text(ax, ex, ey - 1.8, 'EGO VEHICLE', 'Color', cfg.ui.egoColor, ...
             'FontWeight', 'bold', 'FontSize', 8, 'HorizontalAlignment', 'center');
    end

    % Axis styling
    axis(ax, 'equal');
    xlim(ax, [max(0, egoPose(1) - 10), min(xMax, max(45, egoPose(1) + 40))]);
    ylim(ax, [yMin - 1.5, yMax + 1.5]);
    grid(ax, 'on');
    set(ax, 'GridColor', cfg.ui.gridColor, 'GridAlpha', 0.6);
    xlabel(ax, 'Longitudinal Distance X (meters)', 'Color', [0.8, 0.8, 0.8]);
    ylabel(ax, 'Lateral Position Y (meters)', 'Color', [0.8, 0.8, 0.8]);
    set(ax, 'XColor', [0.7, 0.7, 0.7], 'YColor', [0.7, 0.7, 0.7]);
end

function drawOrientedBox(ax, cx, cy, length, width, angle, color)
    hw = width / 2;
    hl = length / 2;
    localPts = [-hl, -hw; hl, -hw; hl, hw; -hl, hw];
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    rotPts = (R * localPts')' + [cx, cy];
    fill(ax, rotPts(:, 1), rotPts(:, 2), color, 'EdgeColor', [1 1 1], 'LineWidth', 1.2, 'FaceAlpha', 0.85);
end

function drawCircle(ax, cx, cy, r, color, lineStyle, lineWidth)
    theta = linspace(0, 2*pi, 40);
    x = cx + r * cos(theta);
    y = cy + r * sin(theta);
    plot(ax, x, y, lineStyle, 'Color', color, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
end
