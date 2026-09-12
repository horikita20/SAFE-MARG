function scenario = createIndianRoadScenario(scenarioID, cfg)
% CREATEINDIANROADSCENARIO Factory function to load and instantiate Indian road driving scenarios.
%
% Supported Scenarios:
%   1 or "narrow_road"   - Narrow unmarked rural road with shoulder obstacles
%   2 or "pothole"       - Potholes directly on current vehicle path
%   3 or "cattle"        - Cattle (cow) suddenly crossing the road (SIH primary demo)
%   4 or "pedestrian"    - Pedestrian crossing unexpectedly from road margin
%   5 or "autorickshaw"  - Stopped / slow auto-rickshaw blocking lane
%   6 or "intersection"  - Unsignalized rural T-junction / intersection
%   7 or "mixed_traffic" - Highly chaotic mixed traffic (cattle, auto, pedestrian, potholes)
%
% Inputs:
%   scenarioID - Scenario number (1-7) or name string
%   cfg        - System configuration struct
%
% Output:
%   scenario - Fully configured scenario struct containing:
%              .id          - Scenario identifier
%              .name        - Human-readable scenario title
%              .description - Detailed scenario context
%              .startPose   - [x y theta] Vehicle start pose
%              .goalPose    - [x y theta] Target destination pose
%              .road        - Road geometry and boundary definitions
%              .actors      - Array of dynamic and static obstacles

    if nargin < 2 || isempty(cfg)
        cfg = config();
    end
    if nargin < 1 || isempty(scenarioID)
        scenarioID = 3; % Default to classic SIH Cattle Crossing demo
    end

    % Normalize scenario ID
    switch lower(string(scenarioID))
        case {"1", "narrow_road", "narrowroad", "rural"}
            scenario = scenario_narrowRoad(cfg);
        case {"2", "pothole", "potholes"}
            scenario = scenario_pothole(cfg);
        case {"3", "cattle", "cattle_crossing", "cow"}
            scenario = scenario_cattle(cfg);
        case {"4", "pedestrian", "pedestrian_crossing"}
            scenario = scenario_pedestrian(cfg);
        case {"5", "autorickshaw", "auto_rickshaw", "rickshaw"}
            scenario = scenario_autoRickshaw(cfg);
        case {"6", "intersection", "unsignalized_intersection"}
            scenario = scenario_intersection(cfg);
        case {"7", "mixed_traffic", "mixedtraffic", "mixed"}
            scenario = scenario_mixedTraffic(cfg);
        otherwise
            warning('Unknown scenario "%s". Loading default Cattle Crossing scenario.', string(scenarioID));
            scenario = scenario_cattle(cfg);
    end
end
