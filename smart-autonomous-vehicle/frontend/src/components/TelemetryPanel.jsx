import React from 'react';
import { Gauge, Navigation, Compass, ShieldAlert, Cpu, Activity, Clock } from 'lucide-react';

export default function TelemetryPanel({ state, activeScenario }) {
  if (!state) {
    return (
      <div className="glass-panel rounded-xl p-5 text-center text-slate-400">
        Loading Telemetry Stream...
      </div>
    );
  }

  const speedKmh = state.speed_kmh || (state.ego_velocity * 3.6) || 0;
  const steerDeg = state.steer_angle_deg || 0;
  const nearestObs = state.perception?.nearest_obstacle;
  const replanCount = state.replan_count || 0;
  const collisionCount = state.collision_count || 0;
  const simTime = state.timestamp || 0;
  const goalDist = state.dist_to_goal || 0;
  const cte = state.cross_track_error || 0;

  return (
    <div className="glass-panel rounded-xl p-5 space-y-4 border border-white/10">
      <div className="flex items-center justify-between border-b border-white/10 pb-3">
        <div className="flex items-center gap-2">
          <Activity className="w-4 h-4 text-cyan-400 animate-pulse" />
          <h2 className="text-xs font-mono font-bold tracking-widest text-slate-300 uppercase">
            Vehicle Telemetry
          </h2>
        </div>
        <span className="text-[11px] font-mono px-2 py-0.5 rounded bg-cyan-950/60 text-cyan-400 border border-cyan-500/30">
          20 Hz Real-Time
        </span>
      </div>

      {/* Speedometer & Steering Gauge */}
      <div className="grid grid-cols-2 gap-3">
        <div className="bg-slate-900/80 rounded-lg p-3 border border-white/5 relative overflow-hidden">
          <div className="flex items-center justify-between text-slate-400 mb-1">
            <span className="text-[10px] font-mono uppercase tracking-wider">Speed</span>
            <Gauge className="w-3.5 h-3.5 text-cyan-400" />
          </div>
          <div className="flex items-baseline gap-1">
            <span className="text-2xl font-black font-mono text-cyan-400">
              {speedKmh.toFixed(1)}
            </span>
            <span className="text-[10px] font-mono text-slate-400">km/h</span>
          </div>
          <div className="text-[10px] font-mono text-slate-500 mt-0.5">
            ({state.ego_velocity?.toFixed(2) || 0} m/s)
          </div>
          <div className="w-full bg-slate-800 h-1 rounded-full mt-2 overflow-hidden">
            <div
              className="bg-cyan-400 h-full transition-all duration-100"
              style={{ width: `${Math.min(100, (speedKmh / 45) * 100)}%` }}
            />
          </div>
        </div>

        <div className="bg-slate-900/80 rounded-lg p-3 border border-white/5 relative overflow-hidden">
          <div className="flex items-center justify-between text-slate-400 mb-1">
            <span className="text-[10px] font-mono uppercase tracking-wider">Steering</span>
            <Compass className="w-3.5 h-3.5 text-amber-400" />
          </div>
          <div className="flex items-baseline gap-1">
            <span className="text-2xl font-black font-mono text-amber-400">
              {steerDeg > 0 ? `+${steerDeg.toFixed(1)}` : steerDeg.toFixed(1)}°
            </span>
          </div>
          <div className="text-[10px] font-mono text-slate-500 mt-0.5">
            {steerDeg > 1 ? 'Steering Left' : steerDeg < -1 ? 'Steering Right' : 'Straight'}
          </div>
          <div className="w-full bg-slate-800 h-1 rounded-full mt-2 overflow-hidden relative">
            <div
              className="bg-amber-400 h-full absolute transition-all duration-100"
              style={{
                left: steerDeg < 0 ? `${50 + (steerDeg / 35) * 50}%` : '50%',
                width: `${(Math.abs(steerDeg) / 35) * 50}%`,
              }}
            />
          </div>
        </div>
      </div>

      {/* Threat Detection & Planner Status */}
      <div className="space-y-2.5">
        <div className="bg-slate-900/60 rounded-lg p-3 border border-white/5 flex items-center justify-between">
          <div className="flex items-center gap-2.5">
            <ShieldAlert className="w-4 h-4 text-rose-400" />
            <div>
              <div className="text-[10px] font-mono uppercase text-slate-400">Hazard Target</div>
              <div className="text-xs font-bold font-mono text-slate-200">
                {nearestObs ? `${nearestObs.class.toUpperCase()} (${nearestObs.distance.toFixed(1)}m)` : 'Clear (>40m)'}
              </div>
            </div>
          </div>
          <div className="text-right">
            <div className="text-[10px] font-mono uppercase text-slate-400">Collisions</div>
            <div className={`text-xs font-bold font-mono ${collisionCount === 0 ? 'text-emerald-400' : 'text-rose-400'}`}>
              {collisionCount === 0 ? '0 (Zero)' : `${collisionCount} Collision!`}
            </div>
          </div>
        </div>

        <div className="bg-slate-900/60 rounded-lg p-3 border border-white/5 flex items-center justify-between">
          <div className="flex items-center gap-2.5">
            <Cpu className="w-4 h-4 text-emerald-400" />
            <div>
              <div className="text-[10px] font-mono uppercase text-slate-400">Planner Status</div>
              <div className="text-xs font-bold font-mono text-slate-200">
                Adaptive A* ({replanCount} Replans)
              </div>
            </div>
          </div>
          <div className="text-right">
            <div className="text-[10px] font-mono uppercase text-slate-400">Cross-Track Error</div>
            <div className="text-xs font-bold font-mono text-cyan-300">
              {cte.toFixed(3)} m
            </div>
          </div>
        </div>
      </div>

      {/* Progress & Time */}
      <div className="pt-2 border-t border-white/10 flex items-center justify-between text-xs font-mono text-slate-400">
        <div className="flex items-center gap-1.5">
          <Clock className="w-3.5 h-3.5 text-slate-500" />
          <span>Elapsed: <strong className="text-slate-200">{simTime.toFixed(2)}s</strong></span>
        </div>
        <div className="flex items-center gap-1.5">
          <Navigation className="w-3.5 h-3.5 text-slate-500" />
          <span>Goal Dist: <strong className="text-slate-200">{goalDist.toFixed(1)}m</strong></span>
        </div>
      </div>
    </div>
  );
}
