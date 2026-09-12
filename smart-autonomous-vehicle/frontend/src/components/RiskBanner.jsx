import React from 'react';
import { ShieldCheck, AlertTriangle, AlertOctagon } from 'lucide-react';

export default function RiskBanner({ riskLevel, reason, minDistance, minTTC }) {
  const level = String(riskLevel || 'SAFE').toUpperCase();

  const getStyle = () => {
    switch (level) {
      case 'CRITICAL':
        return {
          bg: 'bg-gradient-to-r from-rose-950/80 via-rose-900/60 to-rose-950/80 border-rose-500/80',
          text: 'text-rose-200',
          badge: 'bg-rose-500 text-white shadow-lg shadow-rose-500/50',
          icon: AlertOctagon,
          glow: 'animate-pulse-critical',
        };
      case 'WARNING':
        return {
          bg: 'bg-gradient-to-r from-amber-950/80 via-amber-900/60 to-amber-950/80 border-amber-500/80',
          text: 'text-amber-200',
          badge: 'bg-amber-500 text-slate-950 shadow-lg shadow-amber-500/40',
          icon: AlertTriangle,
          glow: 'border-amber-500 shadow-amber-500/20',
        };
      default:
        return {
          bg: 'bg-gradient-to-r from-emerald-950/80 via-slate-900/60 to-emerald-950/80 border-emerald-500/50',
          text: 'text-emerald-300',
          badge: 'bg-emerald-500/20 text-emerald-300 border border-emerald-400/40',
          icon: ShieldCheck,
          glow: 'border-emerald-500/30',
        };
    }
  };

  const style = getStyle();
  const Icon = style.icon;

  return (
    <div
      className={`relative w-full rounded-xl border p-4 transition-all duration-300 ${style.bg} ${style.glow} flex flex-col md:flex-row items-center justify-between gap-4`}
    >
      <div className="flex items-center gap-3 w-full md:w-auto">
        <div className={`p-2.5 rounded-lg ${style.badge} flex items-center justify-center`}>
          <Icon className="w-6 h-6 animate-pulse" />
        </div>
        <div>
          <div className="flex items-center gap-2">
            <span className="text-xs font-mono tracking-wider uppercase text-slate-400">Tactical Risk Assessment</span>
            <span className={`px-2 py-0.5 text-xs font-bold font-mono rounded ${style.badge}`}>
              {level}
            </span>
          </div>
          <p className="text-sm font-medium text-slate-100 mt-0.5 max-w-2xl leading-snug">
            {reason || 'Corridor clear. Nominal path tracking active.'}
          </p>
        </div>
      </div>

      <div className="flex items-center gap-6 self-end md:self-auto border-t md:border-t-0 md:border-l border-white/10 pt-2 md:pt-0 md:pl-6 w-full md:w-auto justify-around">
        <div className="text-center">
          <div className="text-[10px] font-mono uppercase text-slate-400">Nearest Threat</div>
          <div className="text-base font-bold font-mono text-cyan-400">
            {minDistance < 100 ? `${minDistance.toFixed(1)}m` : '>40m'}
          </div>
        </div>
        <div className="text-center">
          <div className="text-[10px] font-mono uppercase text-slate-400">Time-To-Collision</div>
          <div className="text-base font-bold font-mono text-amber-400">
            {minTTC < 50 ? `${minTTC.toFixed(1)}s` : 'Inf'}
          </div>
        </div>
      </div>
    </div>
  );
}
