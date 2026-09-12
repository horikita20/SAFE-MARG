import React from 'react';
import { X, CheckCircle, Award, Compass, Zap, Shield, MapPin } from 'lucide-react';

export default function MetricsModal({ isOpen, onClose, metrics, scenarioName }) {
  if (!isOpen || !metrics) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-slate-950/80 backdrop-blur-md animate-in fade-in duration-200">
      <div className="glass-panel w-full max-w-2xl rounded-2xl p-6 border border-white/20 shadow-2xl shadow-cyan-500/10 space-y-6">
        <div className="flex items-center justify-between border-b border-white/10 pb-4">
          <div className="flex items-center gap-3">
            <div className="p-2 rounded-xl bg-gradient-to-tr from-cyan-600 to-blue-500 text-white">
              <Award className="w-6 h-6" />
            </div>
            <div>
              <h2 className="text-lg font-bold text-slate-100">SIH26037 Benchmark Report</h2>
              <p className="text-xs font-mono text-cyan-400">{scenarioName}</p>
            </div>
          </div>
          <button
            onClick={onClose}
            className="p-1.5 rounded-lg hover:bg-white/10 text-slate-400 hover:text-white transition"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        {/* Highlight Score Card */}
        <div className="grid grid-cols-2 sm:grid-cols-4 gap-3">
          <div className="bg-slate-900/90 rounded-xl p-3 border border-emerald-500/30 text-center">
            <div className="text-[10px] font-mono uppercase text-slate-400">Goal Status</div>
            <div className="text-sm font-bold text-emerald-400 mt-1 flex items-center justify-center gap-1">
              <CheckCircle className="w-4 h-4" /> Reached
            </div>
          </div>

          <div className="bg-slate-900/90 rounded-xl p-3 border border-emerald-500/30 text-center">
            <div className="text-[10px] font-mono uppercase text-slate-400">Collisions</div>
            <div className="text-base font-black text-emerald-400 font-mono mt-0.5">
              {metrics.collision_count} (Zero)
            </div>
          </div>

          <div className="bg-slate-900/90 rounded-xl p-3 border border-white/10 text-center">
            <div className="text-[10px] font-mono uppercase text-slate-400">Min Obstacle Dist</div>
            <div className="text-base font-bold text-cyan-400 font-mono mt-0.5">
              {metrics.min_obstacle_dist?.toFixed(2)} m
            </div>
          </div>

          <div className="bg-slate-900/90 rounded-xl p-3 border border-white/10 text-center">
            <div className="text-[10px] font-mono uppercase text-slate-400">Replanning Triggers</div>
            <div className="text-base font-bold text-amber-400 font-mono mt-0.5">
              {metrics.replan_count} times
            </div>
          </div>
        </div>

        {/* Detailed Metrics Table */}
        <div className="bg-slate-900/70 rounded-xl p-4 border border-white/10 space-y-3">
          <h3 className="text-xs font-mono font-semibold uppercase tracking-wider text-slate-400">
            Quantitative Performance Metrics
          </h3>
          <div className="grid grid-cols-2 gap-y-2.5 text-xs font-mono">
            <div className="text-slate-400 flex items-center gap-2">
              <MapPin className="w-3.5 h-3.5 text-cyan-400" /> Total Path Distance:
            </div>
            <div className="text-right text-slate-200 font-semibold">{metrics.path_length?.toFixed(2)} meters</div>

            <div className="text-slate-400 flex items-center gap-2">
              <Zap className="w-3.5 h-3.5 text-amber-400" /> Average Cruising Speed:
            </div>
            <div className="text-right text-slate-200 font-semibold">{metrics.avg_speed_kmh?.toFixed(2)} km/h</div>

            <div className="text-slate-400 flex items-center gap-2">
              <Compass className="w-3.5 h-3.5 text-emerald-400" /> Mean Cross-Track Error:
            </div>
            <div className="text-right text-slate-200 font-semibold">{metrics.mean_cross_track_err?.toFixed(3)} meters</div>

            <div className="text-slate-400 flex items-center gap-2">
              <Shield className="w-3.5 h-3.5 text-blue-400" /> Total Time to Goal:
            </div>
            <div className="text-right text-slate-200 font-semibold">{metrics.sim_duration_sec?.toFixed(2)} seconds</div>
          </div>
        </div>

        <div className="flex justify-end">
          <button
            onClick={onClose}
            className="px-6 py-2 rounded-xl bg-cyan-600 hover:bg-cyan-500 text-white font-semibold text-sm transition"
          >
            Close Report
          </button>
        </div>
      </div>
    </div>
  );
}
