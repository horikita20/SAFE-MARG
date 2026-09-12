function cfg = config()
% CONFIG Central configuration parameters for Smart Autonomous Vehicle (SIH26037).
%
% Returns a struct with configuration categories:
%   - vehicle: Kinematic bicycle parameters, physical dimensions, limits
%   - grid: Occupancy grid dimensions, resolution, inflation radius
%   - sensors: Sensor range, field of view, noise
%   - planner: A* and Hybrid A* parameters, replanning thresholds
%   - risk: Collision checking and Time-To-Collision (TTC) thresholds
%   - control: Pure pursuit lookahead and PID gains
%   - sim: Simulation step size, duration, realtime playback
%   - ui: Colors, labels, display parameters

    cfg = struct();

    %% Vehicle Physical & Kinematic Parameters
    cfg.vehicle = struct();
    cfg.vehicle.wheelbase      = 2.7;        % Wheelbase L (meters)
    cfg.vehicle.trackWidth     = 1.6;        % Track width (meters)
    cfg.vehicle.length         = 4.4;        % Overall vehicle length (meters)
    cfg.vehicle.width          = 1.8;        % Overall vehicle width (meters)
    cfg.vehicle.rearAxleToBumper = 1.0;     % Rear axle to rear bumper (meters)
    cfg.vehicle.maxSteerAngle  = deg2rad(35);% Max front wheel steer angle (rad) (~35 deg)
    cfg.vehicle.maxSteerRate   = deg2rad(40);% Max steering rate (rad/s)
    cfg.vehicle.maxSpeed       = 12.0;       % Max forward velocity (m/s) (~43.2 km/h for urban/rural)
    cfg.vehicle.nominalSpeed   = 7.0;        % Target cruising speed (m/s) (~25.2 km/h)
    cfg.vehicle.minSpeed       = 0.0;        % Min forward velocity (m/s)
    cfg.vehicle.maxAccel       = 2.5;        % Max forward acceleration (m/s^2)
    cfg.vehicle.maxDecel       = 6.0;        % Max emergency deceleration (m/s^2)
    cfg.vehicle.comfortDecel   = 2.0;        % Nominal deceleration (m/s^2)

    %% Occupancy Grid Parameters
    cfg.grid = struct();
    cfg.grid.xMin       = 0.0;               % Minimum X (meters)
    cfg.grid.xMax       = 80.0;              % Maximum X (meters)
    cfg.grid.yMin       = -10.0;             % Minimum Y (meters, right side)
    cfg.grid.yMax       = 10.0;              % Maximum Y (meters, left side)
    cfg.grid.resolution = 0.25;              % Grid cell size (meters per cell)
    cfg.grid.inflationRadius = 1.2;          % Safety inflation buffer around obstacles (meters)
    cfg.grid.potholeInflationRadius = 0.8;   % Safety inflation for potholes
    % Grid state values:
    cfg.grid.valFree     = 0.0;              % FREE SPACE
    cfg.grid.valObstacle = 1.0;              % OBSTACLE / BLOCKED
    cfg.grid.valUnknown  = 0.5;              % UNKNOWN

    %% Sensor Simulation Parameters
    cfg.sensors = struct();
    cfg.sensors.maxRange     = 45.0;         % Max sensor perception range (meters)
    cfg.sensors.fovAngle     = deg2rad(110); % Horizontal field of view (rad)
    cfg.sensors.updateRate   = 20.0;         % Perception update rate (Hz)
    cfg.sensors.posNoiseStd  = 0.05;         % Position detection noise std (m)
    cfg.sensors.velNoiseStd  = 0.1;          % Velocity detection noise std (m/s)

    %% Path Planner Parameters
    cfg.planner = struct();
    cfg.planner.type            = "adaptive"; % "hybridAStar" | "adaptiveAStar" | "gridAStar"
    cfg.planner.waypointSpacing = 0.5;        % Spacing between planned waypoints (m)
    cfg.planner.replanDistance  = 12.0;       % Lookahead distance for dynamic obstacle replan trigger (m)
    cfg.planner.safetyMargin    = 0.6;        % Extra lateral clearance (m)
    cfg.planner.steeringCostWeight = 1.2;     % Penalty weight for sharp turns
    cfg.planner.reversalPenalty    = 5.0;     % Penalty for reverse motion
    cfg.planner.maxIterations      = 8000;    % Max search iterations
    cfg.planner.smoothWeightData   = 0.4;     % Path smoothing data weight
    cfg.planner.smoothWeightSmooth = 0.6;     % Path smoothing curvature weight

    %% Collision & Risk Assessment Parameters
    cfg.risk = struct();
    cfg.risk.ttcCritical = 1.8;              % Time-To-Collision threshold for CRITICAL (seconds)
    cfg.risk.ttcWarning  = 3.5;              % Time-To-Collision threshold for WARNING (seconds)
    cfg.risk.distCritical= 3.5;              % Proximity threshold for CRITICAL (meters)
    cfg.risk.distWarning = 8.0;              % Proximity threshold for WARNING (meters)
    cfg.risk.lateralClearance = 1.4;         % Critical lateral corridor width (meters)

    %% Vehicle Controller Parameters
    cfg.control = struct();
    cfg.control.minLookahead = 2.5;          % Minimum Pure Pursuit lookahead distance (m)
    cfg.control.maxLookahead = 7.0;          % Maximum Pure Pursuit lookahead distance (m)
    cfg.control.lookaheadGain= 0.5;          % Lookahead = max(minLookahead, lookaheadGain * v)
    cfg.control.speedKp      = 1.2;          % Proportional gain for speed PID
    cfg.control.speedKi      = 0.05;         % Integral gain for speed PID
    cfg.control.speedKd      = 0.02;         % Derivative gain for speed PID

    %% Simulation Settings
    cfg.sim = struct();
    cfg.sim.dt          = 0.05;              % Simulation timestep (seconds) (20 Hz)
    cfg.sim.maxTime     = 30.0;              % Max simulation timeout (seconds)
    cfg.sim.goalRadius  = 1.5;               % Goal arrival distance tolerance (meters)
    cfg.sim.realtimeFactor = 1.0;            % Playback speed factor (1.0 = real-time)

    %% UI & Visualization Color Palette (Modern Dark / Tactical Theme)
    cfg.ui = struct();
    cfg.ui.figSize       = [1200, 750];
    cfg.ui.bgDark        = [0.10, 0.12, 0.15];
    cfg.ui.roadColor     = [0.22, 0.24, 0.27];
    cfg.ui.shoulderColor = [0.45, 0.38, 0.28];
    cfg.ui.gridColor     = [0.18, 0.20, 0.24];
    cfg.ui.egoColor      = [0.00, 0.85, 1.00]; % Cyan
    cfg.ui.pathGlobal    = [0.60, 0.60, 0.60]; % Gray dashed
    cfg.ui.pathActive    = [0.15, 0.90, 0.35]; % Bright Green
    cfg.ui.pathAvoid     = [1.00, 0.70, 0.10]; % Amber
    cfg.ui.goalColor     = [1.00, 0.25, 0.25]; % Red target
    
    % Object class colors:
    cfg.ui.colorPedestrian   = [1.00, 0.35, 0.60]; % Pinkish red
    cfg.ui.colorCattle       = [0.95, 0.60, 0.10]; % Brownish orange
    cfg.ui.colorAutoRickshaw = [1.00, 0.90, 0.00]; % Auto Yellow
    cfg.ui.colorVehicle      = [0.30, 0.60, 1.00]; % Blue
    cfg.ui.colorMotorcycle   = [0.70, 0.40, 0.95]; % Purple
    cfg.ui.colorPothole      = [0.08, 0.08, 0.08]; % Deep dark pit
    cfg.ui.colorDebris       = [0.85, 0.45, 0.20]; % Rust brown
    cfg.ui.colorUnknown      = [0.50, 0.50, 0.50]; % Neutral gray

    % Risk colors:
    cfg.ui.colorSafe     = [0.10, 0.80, 0.30]; % Green
    cfg.ui.colorWarning  = [1.00, 0.75, 0.00]; % Yellow/Orange
    cfg.ui.colorCritical = [1.00, 0.15, 0.15]; % Red
end
