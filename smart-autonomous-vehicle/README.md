# Smart Autonomous Vehicle (SIH26037)
## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

### Project Overview
A 100% free, open-source Autonomous Vehicle MVP built for Smart India Hackathon 2026. The system addresses the challenges of navigating unstructured Indian roads (potholes, stray cattle, jaywalking pedestrians, erratically stopping auto-rickshaws, and lack of lane markings) through a unified perceive-plan-act loop with real-time risk assessment and adaptive replanning.

---

### Tech Stack
- **Backend**: Python 3.12, FastAPI, NumPy, SciPy, Uvicorn, WebSockets
- **Algorithms**: Multi-class FOV perception, 3-state inflated Occupancy Grid, Kinematic Grid A*, Time-To-Collision (TTC) Risk Engine, Geometric Pure Pursuit, 2D Kinematic Bicycle Dynamics (RK2 integration)
- **Frontend**: React 18, Vite, Tailwind CSS, Canvas 2D/3D Tactical Bird's-Eye-View Simulator, Lucide React

---

### Directory Structure
```
smart-autonomous-vehicle/
├── backend/
│   ├── main.py                     # FastAPI entry point
│   ├── config.py                   # Central parameters & thresholds
│   ├── perception/
│   │   ├── object_detector.py      # Multi-class detection & FOV filtering
│   │   ├── drivable_area.py        # Road margin extraction
│   │   ├── occupancy_grid.py       # 3-State grid with safety inflation
│   │   └── perception_pipeline.py  # Unified perception output
│   ├── planning/
│   │   ├── global_path.py          # Reference centerline trajectory
│   │   ├── adaptive_planner.py     # Kinematic Grid A* planner
│   │   ├── collision_checker.py    # TTC & 3-tier risk engine
│   │   └── dynamic_avoidance.py    # Reactive Gaussian detour
│   ├── control/
│   │   ├── pure_pursuit.py         # Speed-adaptive steering controller
│   │   ├── vehicle_controller.py   # Speed PID & emergency brake
│   │   └── kinematic_bicycle.py    # 2D bicycle model (RK2)
│   ├── scenarios/
│   │   └── scenario_definitions.py # 7 Indian road scenarios
│   ├── simulation/
│   │   └── simulation_engine.py    # Closed-loop simulation engine
│   └── api/
│       └── routes.py               # REST & WebSocket endpoints
├── frontend/
│   ├── src/
│   │   ├── App.jsx                 # Tactical dashboard UI
│   │   ├── components/
│   │   │   ├── CanvasView.jsx      # Live BEV map renderer
│   │   │   ├── TelemetryPanel.jsx  # Digital speed & steer gauges
│   │   │   ├── RiskBanner.jsx      # SAFE / WARNING / CRITICAL
│   │   │   ├── Controls.jsx        # Scenario selector & playback
│   │   │   └── MetricsModal.jsx    # SIH performance scorecard
│   │   └── services/api.js         # API & WebSocket client
│   ├── package.json
│   └── vite.config.js
├── tests/
│   └── test_pipeline.py            # Automated test suite
└── requirements.txt
```

---

### How to Run

#### 1. Start the Python Backend:
```bash
cd smart-autonomous-vehicle/backend
python main.py
```
*The REST API and interactive docs will be available at `http://localhost:8000/docs`.*

#### 2. Start the React Frontend:
```bash
cd smart-autonomous-vehicle/frontend
npm run dev
```
*Open `http://localhost:3000` in your browser to interact with the live tactical simulation dashboard.*

#### 3. Run Automated Tests:
```bash
cd smart-autonomous-vehicle
python tests/test_pipeline.py
```
