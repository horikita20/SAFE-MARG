# Smart Autonomous Vehicle (SIH26037) — Full System Walkthrough

## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads
**Smart India Hackathon 2026** · 100% MATLAB / Simulink Implementation

---

## 1. Complete Project Structure

All files are created in the project root:

```
SmartAutonomousVehicle/
├── main.m                           <-- Interactive launcher & SIH demo runner
├── config/
│   └── config.m                     <-- Master configuration parameters
├── perception/
│   ├── detectObjects.m              <-- Multi-class obstacle detector
│   ├── detectDrivableArea.m         <-- Unstructured road boundary extractor
│   ├── generateOccupancyGrid.m      <-- 3-State spatial grid with safety inflation
│   └── perceptionPipeline.m         <-- Master perception fusion pipeline
├── planning/
│   ├── generateGlobalPath.m         <-- Global centerline trajectory generator
│   ├── adaptivePathPlanner.m        <-- Hybrid A* / Kinematic A* adaptive planner
│   ├── collisionChecker.m           <-- TTC calculation & risk assessment
│   └── dynamicObstacleAvoidance.m   <-- Local dynamic reactive replanner
├── control/
│   ├── purePursuitController.m      <-- Speed-adaptive pure pursuit steering
│   ├── vehicleController.m          <-- Longitudinal speed & emergency brake
│   └── vehicleModel.m               <-- 2D Kinematic bicycle model
├── scenarios/
│   ├── createIndianRoadScenario.m   <-- Scenario factory
│   ├── scenario_narrowRoad.m        <-- Scenario 1: Narrow rural road with bottleneck
│   ├── scenario_pothole.m           <-- Scenario 2: Potholes on path
│   ├── scenario_cattle.m            <-- Scenario 3: Cattle crossing (SIH Flagship Demo)
│   ├── scenario_pedestrian.m        <-- Scenario 4: Sudden pedestrian crossing
│   ├── scenario_autoRickshaw.m      <-- Scenario 5: Auto-rickshaw obstruction
│   ├── scenario_intersection.m      <-- Scenario 6: Unsignalized rural intersection
│   └── scenario_mixedTraffic.m      <-- Scenario 7: Multi-hazard chaotic traffic
├── visualization/
│   ├── visualizeEnvironment.m       <-- Bird's-eye-view environment & agent renderer
│   ├── visualizePath.m              <-- Path & lookahead overlay renderer
│   └── dashboard.m                  <-- Tactical GUI Dashboard
├── simulation/
│   └── runSimulation.m              <-- Closed-loop simulation engine
├── tests/
│   ├── testPipeline.m               <-- Automated test suite & scenario benchmarks
│   ├── MockDetector.m               <-- Test double for detector testing
│   └── runPerceptionTests.m         <-- Perception unit tests
├── docs/
│   ├── architecture.md              <-- System architecture & data contracts
│   ├── methodology.md               <-- Mathematical formulations & control theory
│   ├── data-contracts.md            <-- Struct schemas
│   └── README.md                    <-- Docs index
├── README.md                        <-- Main project guide & SIH presentation script
└── walkthrough.md                   <-- Full system walkthrough
```

---

## 2. Subsystems Implemented

### A. Configuration (`config/config.m`)
- Centralized parameters for vehicle dimensions (wheelbase $L = 2.7\,\text{m}$, max steer $35^\circ$), occupancy grid ($0.25\,\text{m/cell}$ resolution, $1.2\,\text{m}$ safety inflation), risk thresholds ($\text{TTC}_{\text{crit}} = 1.8\,\text{s}$), pure pursuit gains, and UI theme.

### B. Perception Subsystem (`perception/`)
- `detectObjects.m`: Handles 7 classes (`pedestrian`, `cattle`, `auto-rickshaw`, `vehicle`, `motorcycle`, `pothole`, `debris`).
- `detectDrivableArea.m`: Extracts road corridors on unmarked rural roads.
- `generateOccupancyGrid.m`: Produces 3-state grid (`FREE = 0`, `OBSTACLE = 1`, `UNKNOWN = 0.5`) with obstacle inflation.
- `perceptionPipeline.m`: Fuses detections, drivable space, and ego telemetry.

### C. Adaptive Planning Subsystem (`planning/`)
- `generateGlobalPath.m`: Reference centerline path.
- `adaptivePathPlanner.m`: Navigation Toolbox Hybrid A* with robust pure-MATLAB Kinematic A* fallback.
- `collisionChecker.m`: Time-To-Collision (TTC) & closing speed calculation; 3-tier risk state: `[SAFE]`, `[WARNING]`, `[CRITICAL]`.
- `dynamicObstacleAvoidance.m`: Local reactive detour generation around moving obstacles.

### D. Vehicle Control & Dynamics (`control/`)
- `purePursuitController.m`: Speed-adaptive lookahead ($L_d = 2.5\,\text{m} - 7.0\,\text{m}$) geometric steering.
- `vehicleController.m`: Speed control and emergency braking ($a = -6.0\,\text{m/s}^2$).
- `vehicleModel.m`: 2D Kinematic Bicycle Model with RK2 midpoint integration.

### E. 7 Realistic Indian Road Scenarios (`scenarios/`)
- `scenario_narrowRoad.m`, `scenario_pothole.m`, `scenario_cattle.m` (SIH Demo), `scenario_pedestrian.m`, `scenario_autoRickshaw.m`, `scenario_intersection.m`, `scenario_mixedTraffic.m`.

### F. Tactical Dashboard & Closed-Loop Simulation (`visualization/` & `simulation/`)
- `dashboard.m`: Real-time GUI with BEV road animation, speed gauge, risk state banner, and interactive controls (`Start`, `Pause`, `Reset`, `Step`).
- `runSimulation.m`: End-to-end simulation runner computing 8 SIH performance benchmarks.
- `main.m`: Interactive launcher with post-simulation telemetry plots.

---

## 3. Verification & How to Run in MATLAB

```matlab
% Launch Interactive Demo:
main

% Or run the Flagship Cattle Crossing Demo directly:
runSimulation(3);

% Run complete automated test suite (All 7 scenarios):
testPipeline
```
