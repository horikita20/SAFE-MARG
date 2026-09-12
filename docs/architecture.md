# System Architecture — Smart Autonomous Vehicle (SIH26037)

## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

---

## 1. High-Level System Architecture

The system operates as a closed-loop perceive-plan-act pipeline running at $20\,\text{Hz}$ ($\Delta t = 0.05\,\text{s}$), designed exclusively in MATLAB/Simulink with zero external framework dependencies:

```
+-------------------------------------------------------------------------------+
|                        SYNTHETIC DRIVING SCENARIO                             |
|           (Rural Roads, Cattle, Pedestrians, Potholes, Auto-Rickshaws)        |
+---------------------------------------+---------------------------------------+
                                        |
                                        v
+-------------------------------------------------------------------------------+
|                             PERCEPTION SUBSYSTEM                              |
|  - detectObjects: Multi-class obstacle detection with FOV & sensor noise      |
|  - detectDrivableArea: Unstructured road boundary & margin extraction         |
|  - generateOccupancyGrid: 3-State spatial grid (FREE, OBSTACLE, UNKNOWN)      |
|  - Obstacle Safety Inflation (1.2m buffer around detected hazard cores)       |
+---------------------------------------+---------------------------------------+
                                        |
                                        v
+-------------------------------------------------------------------------------+
|                    COLLISION RISK & ASSESSMENT ENGINE                         |
|  - Time-To-Collision (TTC) & closing velocity calculation                     |
|  - Spatial corridor threat intersection checking                              |
|  - 3-Tier Tactical Risk State Machine: [SAFE] -> [WARNING] -> [CRITICAL]      |
+---------------------------------------+---------------------------------------+
                                        |
                                        v
+-------------------------------------------------------------------------------+
|                     ADAPTIVE PATH PLANNING SUBSYSTEM                          |
|  - generateGlobalPath: Road midline reference trajectory                      |
|  - adaptivePathPlanner: Hybrid A* (with robust vector-optimized pure-MATLAB   |
|                         Kinematic A* fallback)                                |
|  - dynamicObstacleAvoidance: Local reactive trajectory detours & replanning   |
+---------------------------------------+---------------------------------------+
                                        |
                                        v
+-------------------------------------------------------------------------------+
|                   VEHICLE CONTROL & DYNAMICS SUBSYSTEM                        |
|  - purePursuitController: Speed-adaptive geometric lookahead steering         |
|  - vehicleController: Longitudinal throttle, braking & emergency stop         |
|  - vehicleModel: 2D Kinematic Bicycle Model (Wheelbase L=2.7m, RK2 update)   |
+---------------------------------------+---------------------------------------+
                                        |
                                        v
+-------------------------------------------------------------------------------+
|                   MONITORING, DASHBOARD & BENCHMARKING                        |
|  - dashboard: Live Tactical GUI (BEV Road Map, Telemetry, Risk State Banner)  |
|  - runSimulation: Closed-loop batch/realtime runner with 8 SIH metrics       |
+-------------------------------------------------------------------------------+
```

---

## 2. Module Interfaces & Data Contracts

| Module | Primary Function | Inputs | Outputs |
| :--- | :--- | :--- | :--- |
| **Config** | `config()` | None | Master parameter struct (`vehicle`, `grid`, `planner`, `risk`, `control`, `sim`, `ui`) |
| **Perception** | `perceptionPipeline(...)` | Scenario, EgoPose, Velocity, Time, Config | `perceptionOut` (`objects`, `drivableArea`, `occupancyGrid`, `nearestObstacle`) |
| **Risk Check** | `collisionChecker(...)` | EgoPose, Velocity, ActivePath, Objects, Config | `collisionReport` (`riskLevel`, `minDistance`, `minTTC`, `needsReplan`, `reason`) |
| **Planner** | `adaptivePathPlanner(...)` | StartPose, GoalPose, OccupancyGrid, Config | `plannedPath` (`waypoints`, `velocities`, `headings`, `cost`, `valid`) |
| **Steering** | `purePursuitController(...)` | EgoPose, Velocity, Waypoints, Config | `steerCmd`, `targetPoint`, `crossTrackError` |
| **Speed/Brake**| `vehicleController(...)` | TargetVel, CurrentVel, RiskLevel, Config | `throttleCmd`, `brakeCmd`, `accelCmd` |
| **Dynamics** | `vehicleModel(...)` | Pose, Velocity, SteerAngle, Accel, dt, Config | `nextPose`, `nextVel`, `yawRate`, `footprintVertices` |

---

## 3. Dual-Mode Toolbox & Fallback Strategy

To ensure 100% demo reliability during live presentations across any computer environment:
- **Navigation Toolbox**: If `plannerHybridAStar` is present, it plans smooth continuous trajectories. If absent, the system seamlessly uses our custom vectorized **Kinematic Grid A\*** with zero performance loss.
- **Automated Driving Toolbox**: Real camera detection interfaces (`yolov4ObjectDetector`, `monoCamera`, `birdsEyeView`) are mapped directly into the data contract, with our synthetic Indian scenario generator providing instant simulation.
- **Pure MATLAB GUI**: Interactive dashboard built entirely with native MATLAB UI components (`uicontrol`, `uipanel`, `axes`) requiring no web servers, node runtimes, or external compilers.
