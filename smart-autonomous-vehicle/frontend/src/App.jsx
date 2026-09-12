import React, { useState, useEffect, useRef } from 'react';
import CanvasView from './components/CanvasView';
import TelemetryPanel from './components/TelemetryPanel';
import RiskBanner from './components/RiskBanner';
import Controls from './components/Controls';
import MetricsModal from './components/MetricsModal';
import {
  fetchScenarios,
  selectScenario,
  resetSimulation,
  stepSimulation,
  runFullSimulation,
  createSimulationWebSocket,
} from './services/api';
import { Shield, Sparkles } from 'lucide-react';

export default function App() {
  const [scenarios, setScenarios] = useState([]);
  const [selectedScenarioId, setSelectedScenarioId] = useState(3);
  const [simulationState, setSimulationState] = useState(null);
  const [activeScenario, setActiveScenario] = useState(null);
  const [isRunning, setIsRunning] = useState(false);
  const [isMetricsOpen, setIsMetricsOpen] = useState(false);
  const [metrics, setMetrics] = useState(null);
  const wsRef = useRef(null);

  // Load scenarios on mount
  useEffect(() => {
    async function init() {
      const list = await fetchScenarios();
      setScenarios(list);
      const res = await selectScenario(3);
      if (res.state) {
        setSimulationState(res.state);
        setActiveScenario(res.scenario);
      }
    }
    init();
  }, []);

  // Setup WebSocket connection
  useEffect(() => {
    const ws = createSimulationWebSocket(
      (msg) => {
        if (msg.type === 'state') {
          setSimulationState(msg.data);
        } else if (msg.type === 'completed') {
          setIsRunning(false);
        }
      },
      () => console.log('Simulation WebSocket Connected'),
      () => console.log('Simulation WebSocket Disconnected')
    );
    wsRef.current = ws;

    return () => {
      if (ws) ws.close();
    };
  }, []);

  const handleSelectScenario = async (id) => {
    setSelectedScenarioId(id);
    setIsRunning(false);
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ action: `select:${id}` }));
    } else {
      const res = await selectScenario(id);
      if (res.state) {
        setSimulationState(res.state);
        setActiveScenario(res.scenario);
      }
    }
  };

  const handleStart = () => {
    setIsRunning(true);
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ action: 'start' }));
    }
  };

  const handlePause = () => {
    setIsRunning(false);
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ action: 'pause' }));
    }
  };

  const handleReset = async () => {
    setIsRunning(false);
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ action: 'reset' }));
    } else {
      const res = await resetSimulation();
      if (res.state) setSimulationState(res.state);
    }
  };

  const handleStep = async () => {
    setIsRunning(false);
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ action: 'step' }));
    } else {
      const res = await stepSimulation();
      if (res.state) setSimulationState(res.state);
    }
  };

  const handleOpenMetrics = async () => {
    const res = await runFullSimulation(selectedScenarioId);
    if (res.metrics) {
      setMetrics(res.metrics);
      setIsMetricsOpen(true);
    }
  };

  const collisionReport = simulationState?.collision_report;
  const currentScenarioMeta = scenarios.find((s) => s.id === selectedScenarioId);

  return (
    <div className="h-screen w-screen flex flex-col bg-[#070a12] text-slate-100 overflow-hidden font-sans">
      {/* Top Navigation Header */}
      <header className="h-14 px-6 border-b border-white/10 flex items-center justify-between bg-slate-950/60 backdrop-blur-xl z-20 shrink-0">
        <div className="flex items-center gap-3">
          <div className="p-1.5 rounded-lg bg-gradient-to-tr from-cyan-500 to-blue-600 shadow-md shadow-cyan-500/20 text-white">
            <Shield className="w-5 h-5" />
          </div>
          <div>
            <h1 className="text-sm font-bold tracking-tight text-white flex items-center gap-2">
              <span>SIH26037: Smart Autonomous Vehicle</span>
              <span className="px-2 py-0.5 rounded text-[10px] font-mono bg-cyan-500/20 text-cyan-400 border border-cyan-500/30">
                Open-Source Stack
              </span>
            </h1>
            <p className="text-[10px] font-mono text-slate-400">
              Adaptive Indian Road Navigation & Collision Avoidance
            </p>
          </div>
        </div>

        <div className="flex items-center gap-4 text-xs font-mono">
          <div className="flex items-center gap-2 px-3 py-1 rounded-full bg-slate-900 border border-white/10 text-slate-300">
            <Sparkles className="w-3.5 h-3.5 text-cyan-400" />
            <span>Python + FastAPI + NumPy + React</span>
          </div>
        </div>
      </header>

      {/* Main Workspace Dashboard Grid */}
      <main className="flex-1 p-4 grid grid-cols-1 lg:grid-cols-12 gap-4 min-h-0 overflow-hidden">
        {/* Left / Center: BEV Canvas Simulator (8 cols) */}
        <div className="lg:col-span-8 flex flex-col gap-4 h-full min-h-0">
          <div className="flex-1 min-h-0 relative">
            <CanvasView state={simulationState} scenario={activeScenario || currentScenarioMeta} />
          </div>

          {/* Bottom Risk Banner */}
          <div className="shrink-0">
            <RiskBanner
              riskLevel={collisionReport?.risk_level}
              reason={collisionReport?.reason}
              minDistance={collisionReport?.min_distance}
              minTTC={collisionReport?.min_ttc}
            />
          </div>
        </div>

        {/* Right Side: Telemetry & Controls (4 cols) */}
        <div className="lg:col-span-4 flex flex-col gap-4 h-full overflow-y-auto">
          <TelemetryPanel state={simulationState} activeScenario={currentScenarioMeta} />
          <Controls
            scenarios={scenarios}
            selectedScenarioId={selectedScenarioId}
            onSelectScenario={handleSelectScenario}
            isRunning={isRunning}
            onStart={handleStart}
            onPause={handlePause}
            onReset={handleReset}
            onStep={handleStep}
            onOpenMetrics={handleOpenMetrics}
          />
        </div>
      </main>

      {/* Metrics Modal */}
      <MetricsModal
        isOpen={isMetricsOpen}
        onClose={() => setIsMetricsOpen(false)}
        metrics={metrics}
        scenarioName={currentScenarioMeta?.name || 'Scenario Benchmark'}
      />
    </div>
  );
}
