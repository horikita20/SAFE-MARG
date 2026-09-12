import React from 'react';
import { Play, Pause, RotateCcw, StepForward, BarChart2 } from 'lucide-react';

export default function Controls({
  scenarios,
  selectedScenarioId,
  onSelectScenario,
  isRunning,
  onStart,
  onPause,
  onReset,
  onStep,
  onOpenMetrics,
}) {
  return (
    <div className="glass-panel rounded-xl p-4 flex flex-wrap items-center justify-between gap-4 border border-white/10">
      {/* Scenario Selector Dropdown */}
      <div className="flex items-center gap-3 min-w-[280px]">
        <label className="text-xs font-mono font-semibold text-slate-400 uppercase whitespace-nowrap">
          Scenario:
        </label>
        <select
          value={selectedScenarioId}
          onChange={(e) => onSelectScenario(Number(e.target.value))}
          className="w-full bg-slate-900/90 text-slate-100 text-sm font-medium rounded-lg px-3 py-2 border border-white/10 focus:outline-none focus:border-cyan-400 transition"
        >
          {scenarios.map((sc) => (
            <option key={sc.id} value={sc.id} className="bg-slate-900 text-slate-100">
              {sc.id}: {sc.name}
            </option>
          ))}
        </select>
      </div>

      {/* Playback Action Buttons */}
      <div className="flex items-center gap-2.5">
        {!isRunning ? (
          <button
            onClick={onStart}
            className="flex items-center gap-2 px-5 py-2 rounded-lg bg-gradient-to-r from-emerald-600 to-emerald-500 hover:from-emerald-500 hover:to-emerald-400 text-white font-semibold text-sm shadow-lg shadow-emerald-600/30 transition active:scale-95"
          >
            <Play className="w-4 h-4 fill-current" />
            <span>Start Live Demo</span>
          </button>
        ) : (
          <button
            onClick={onPause}
            className="flex items-center gap-2 px-5 py-2 rounded-lg bg-gradient-to-r from-amber-600 to-amber-500 hover:from-amber-500 hover:to-amber-400 text-white font-semibold text-sm shadow-lg shadow-amber-600/30 transition active:scale-95"
          >
            <Pause className="w-4 h-4 fill-current" />
            <span>Pause</span>
          </button>
        )}

        <button
          onClick={onStep}
          disabled={isRunning}
          className="flex items-center gap-1.5 px-3.5 py-2 rounded-lg bg-slate-800 hover:bg-slate-700 disabled:opacity-50 text-slate-200 font-medium text-sm border border-white/10 transition active:scale-95"
          title="Single Step (0.05s)"
        >
          <StepForward className="w-4 h-4" />
          <span>Step</span>
        </button>

        <button
          onClick={onReset}
          className="flex items-center gap-1.5 px-3.5 py-2 rounded-lg bg-slate-800 hover:bg-rose-900/60 hover:text-rose-200 text-slate-200 font-medium text-sm border border-white/10 transition active:scale-95"
          title="Reset Simulation"
        >
          <RotateCcw className="w-4 h-4" />
          <span>Reset</span>
        </button>

        <button
          onClick={onOpenMetrics}
          className="flex items-center gap-1.5 px-3.5 py-2 rounded-lg bg-cyan-950/80 hover:bg-cyan-900 text-cyan-300 font-medium text-sm border border-cyan-500/40 transition active:scale-95 shadow-sm shadow-cyan-500/20"
          title="View Benchmark Metrics"
        >
          <BarChart2 className="w-4 h-4" />
          <span className="hidden sm:inline">SIH Report</span>
        </button>
      </div>
    </div>
  );
}
