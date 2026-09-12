function ax = visualizePath(ax, globalPath, activePath, trajectoryHistory, targetLookaheadPoint, cfg)
% VISUALIZEPATH Overlays global path, active planned path, vehicle trajectory, and lookahead point.
%
% Inputs:
%   ax                   - MATLAB Axes handle
%   globalPath           - Reference baseline path struct
%   activePath           - Currently active PlannedPath struct
%   trajectoryHistory    - [N x 2] array of past vehicle positions [x, y]
%   targetLookaheadPoint - [1 x 2] Selected Pure Pursuit target point
%   cfg                  - System configuration struct

    if nargin < 6 || isempty(cfg), cfg = config(); end
    if isempty(ax) || ~isvalid(ax), return; end

    hold(ax, 'on');

    % 1. Draw Global Reference Path (Dashed gray)
    if ~isempty(globalPath) && isfield(globalPath, 'waypoints') && ~isempty(globalPath.waypoints)
        plot(ax, globalPath.waypoints(:, 1), globalPath.waypoints(:, 2), ...
             '--', 'Color', cfg.ui.pathGlobal, 'LineWidth', 1.5, 'DisplayName', 'Global Path');
    end

    % 2. Draw Active Planned Path (Solid Bright Green with Waypoint markers)
    if ~isempty(activePath) && isfield(activePath, 'waypoints') && ~isempty(activePath.waypoints)
        wp = activePath.waypoints;
        plot(ax, wp(:, 1), wp(:, 2), '-', 'Color', cfg.ui.pathActive, ...
             'LineWidth', 2.8, 'DisplayName', 'Active Planned Path');
        plot(ax, wp(:, 1), wp(:, 2), '.', 'Color', [0.8, 1.0, 0.8], ...
             'MarkerSize', 8, 'HandleVisibility', 'off');
    end

    % 3. Draw Vehicle Trajectory History (Cyan breadcrumb trail)
    if ~isempty(trajectoryHistory) && size(trajectoryHistory, 1) > 1
        plot(ax, trajectoryHistory(:, 1), trajectoryHistory(:, 2), ...
             '-', 'Color', [cfg.ui.egoColor, 0.7], 'LineWidth', 1.8, 'DisplayName', 'Driven Trajectory');
    end

    % 4. Draw Pure Pursuit Lookahead Target Point (Pulsing yellow marker)
    if ~isempty(targetLookaheadPoint)
        plot(ax, targetLookaheadPoint(1), targetLookaheadPoint(2), 'o', ...
             'MarkerSize', 9, 'MarkerFaceColor', [1.0, 0.9, 0.1], ...
             'MarkerEdgeColor', [0.1, 0.1, 0.1], 'LineWidth', 1.5, 'DisplayName', 'Lookahead Target');
    end
end
