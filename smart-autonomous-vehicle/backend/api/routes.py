"""
FastAPI REST and WebSocket routes for Smart Autonomous Vehicle.
"""

from typing import Dict, Any, List, Optional
import asyncio
import json
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel

try:
    from ..config import get_config
    from ..scenarios.scenario_definitions import get_all_scenario_metadata, create_indian_road_scenario
    from ..simulation.simulation_engine import SimulationEngine
except (ImportError, ValueError):
    try:
        from backend.config import get_config
        from backend.scenarios.scenario_definitions import get_all_scenario_metadata, create_indian_road_scenario
        from backend.simulation.simulation_engine import SimulationEngine
    except (ImportError, ValueError):
        from config import get_config
        from scenarios.scenario_definitions import get_all_scenario_metadata, create_indian_road_scenario
        from simulation.simulation_engine import SimulationEngine

router = APIRouter()

engine = SimulationEngine(scenario_id=3)


class ScenarioSelectRequest(BaseModel):
    scenario_id: int


@router.get("/scenarios")
def list_scenarios() -> List[Dict[str, Any]]:
    return get_all_scenario_metadata(engine.cfg)


@router.post("/scenarios/select")
def select_scenario(req: ScenarioSelectRequest) -> Dict[str, Any]:
    if req.scenario_id < 1 or req.scenario_id > 7:
        raise HTTPException(status_code=400, detail="Invalid scenario ID. Must be 1 to 7.")
    engine.reset(req.scenario_id)
    state = engine.step()
    return {
        "status": "success",
        "scenario": engine.scenario.to_dict(),
        "state": state.to_dict(),
    }


@router.post("/simulation/reset")
def reset_simulation() -> Dict[str, Any]:
    engine.reset()
    state = engine.step()
    return {"status": "reset", "state": state.to_dict()}


@router.post("/simulation/step")
def step_simulation() -> Dict[str, Any]:
    if engine.goal_reached:
        return {"status": "goal_reached", "state": None}
    state = engine.step()
    return {"status": "active", "state": state.to_dict()}


@router.post("/simulation/run")
def run_full_simulation(req: Optional[ScenarioSelectRequest] = None) -> Dict[str, Any]:
    s_id = req.scenario_id if req and req.scenario_id else engine.scenario_id
    temp_engine = SimulationEngine(scenario_id=s_id)
    metrics, history = temp_engine.run_full()
    return {
        "status": "completed",
        "metrics": metrics.to_dict(),
        "history": history[::2],
    }


@router.websocket("/ws/stream")
async def websocket_stream(websocket: WebSocket):
    await websocket.accept()
    is_running = False
    delay = 0.04

    try:
        while True:
            try:
                msg = await asyncio.wait_for(websocket.receive_text(), timeout=0.01)
                data = json.loads(msg) if msg.startswith("{") else {"action": msg}
                action = data.get("action", "")

                if action == "start":
                    is_running = True
                elif action == "pause":
                    is_running = False
                elif action == "reset":
                    is_running = False
                    engine.reset()
                    await websocket.send_json({"type": "state", "data": engine.step().to_dict()})
                elif action == "step":
                    is_running = False
                    if not engine.goal_reached:
                        await websocket.send_json({"type": "state", "data": engine.step().to_dict()})
                elif action.startswith("select:"):
                    s_id = int(action.split(":")[1])
                    engine.reset(s_id)
                    await websocket.send_json({"type": "state", "data": engine.step().to_dict()})
            except asyncio.TimeoutError:
                pass

            if is_running:
                if not engine.goal_reached and engine.t <= engine.cfg.sim.max_time:
                    state = engine.step()
                    await websocket.send_json({"type": "state", "data": state.to_dict()})
                else:
                    is_running = False
                    await websocket.send_json({"type": "completed", "message": "Goal Reached!"})

            await asyncio.sleep(delay)
    except WebSocketDisconnect:
        pass
