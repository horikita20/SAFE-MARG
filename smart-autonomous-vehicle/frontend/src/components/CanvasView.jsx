import React, { useRef, useEffect } from 'react';

export default function CanvasView({ state, scenario }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Handle responsive canvas sizing
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    ctx.clearRect(0, 0, width, height);

    const egoPose = state?.ego_pose || [0, 0, 0];
    const egoX = egoPose[0];
    const egoY = egoPose[1];

    // Coordinate mapping: camera follows ego vehicle
    // World coordinates: X right (meters), Y up (meters)
    const scale = Math.min(width / 45, height / 18); // Pixels per meter
    const cameraOffsetX = width * 0.25; // Ego vehicle kept at 25% from left
    const cameraOffsetY = height * 0.5;  // Center vertically

    const worldToScreen = (wx, wy) => {
      const sx = cameraOffsetX + (wx - egoX) * scale;
      const sy = cameraOffsetY - (wy - 0.0) * scale; // Y inverted for screen
      return [sx, sy];
    };

    // 1. Off-road Dirt / Shoulder Background
    ctx.fillStyle = '#14171f';
    ctx.fillRect(0, 0, width, height);

    // 2. Draw Drivable Road Surface
    const drivable = state?.perception?.drivable_area;
    const leftBound = drivable?.left_boundary || [];
    const rightBound = drivable?.right_boundary || [];

    if (leftBound.length > 0 && rightBound.length > 0) {
      ctx.beginPath();
      const [startLx, startLy] = worldToScreen(leftBound[0][0], leftBound[0][1]);
      ctx.moveTo(startLx, startLy);

      for (let i = 1; i < leftBound.length; i++) {
        const [lx, ly] = worldToScreen(leftBound[i][0], leftBound[i][1]);
        ctx.lineTo(lx, ly);
      }

      for (let i = rightBound.length - 1; i >= 0; i--) {
        const [rx, ry] = worldToScreen(rightBound[i][0], rightBound[i][1]);
        ctx.lineTo(rx, ry);
      }
      ctx.closePath();

      // Asphalt Fill
      ctx.fillStyle = '#1e2430';
      ctx.fill();
      ctx.strokeStyle = '#475569';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Dashed Centerline
      const centerline = drivable?.centerline || [];
      if (centerline.length > 1) {
        ctx.beginPath();
        ctx.setLineDash([8, 8]);
        const [cx0, cy0] = worldToScreen(centerline[0][0], centerline[0][1]);
        ctx.moveTo(cx0, cy0);
        for (let i = 1; i < centerline.length; i++) {
          const [cx, cy] = worldToScreen(centerline[i][0], centerline[i][1]);
          ctx.lineTo(cx, cy);
        }
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    // 3. Draw Destination Goal Marker
    const goalPose = scenario?.goal_pose || [70, 0, 0];
    const [goalSx, goalSy] = worldToScreen(goalPose[0], goalPose[1]);
    ctx.beginPath();
    ctx.arc(goalSx, goalSy, 1.8 * scale, 0, 2 * Math.PI);
    ctx.strokeStyle = 'rgba(244, 63, 94, 0.4)';
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.arc(goalSx, goalSy, 0.8 * scale, 0, 2 * Math.PI);
    ctx.fillStyle = '#f43f5e';
    ctx.fill();
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 11px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText('GOAL', goalSx, goalSy - 1.2 * scale);

    // 4. Draw Trajectory History Trail
    const history = state?.trajectory_history || [];
    if (history.length > 1) {
      ctx.beginPath();
      const [hx0, hy0] = worldToScreen(history[0][0], history[0][1]);
      ctx.moveTo(hx0, hy0);
      for (let i = 1; i < history.length; i++) {
        const [hx, hy] = worldToScreen(history[i][0], history[i][1]);
        ctx.lineTo(hx, hy);
      }
      ctx.strokeStyle = 'rgba(0, 180, 216, 0.45)';
      ctx.lineWidth = 2.5;
      ctx.stroke();
    }

    // 5. Draw Active Planned Path
    const activePath = state?.active_path?.waypoints || [];
    if (activePath.length > 1) {
      ctx.beginPath();
      const [px0, py0] = worldToScreen(activePath[0][0], activePath[0][1]);
      ctx.moveTo(px0, py0);
      for (let i = 1; i < activePath.length; i++) {
        const [px, py] = worldToScreen(activePath[i][0], activePath[i][1]);
        ctx.lineTo(px, py);
      }
      ctx.strokeStyle = '#00e5ff';
      ctx.lineWidth = 3.0;
      ctx.shadowColor = '#00e5ff';
      ctx.shadowBlur = 10;
      ctx.stroke();
      ctx.shadowBlur = 0;
    }

    // 6. Draw Lookahead Target Marker
    const targetPt = state?.target_lookahead;
    if (targetPt) {
      const [tx, ty] = worldToScreen(targetPt[0], targetPt[1]);
      ctx.beginPath();
      ctx.arc(tx, ty, 6, 0, 2 * Math.PI);
      ctx.fillStyle = '#f59e0b';
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // 7. Draw Detected Obstacles with Safety Buffer Rings
    const objects = state?.perception?.objects || [];
    objects.forEach((obj) => {
      const [ox, oy] = obj.world_position;
      const [sx, sy] = worldToScreen(ox, oy);
      const rPix = obj.radius * scale;
      const inflPix = (obj.radius + 1.2) * scale;
      const cls = String(obj.class).toLowerCase();

      // Inflation buffer ring
      ctx.beginPath();
      ctx.arc(sx, sy, inflPix, 0, 2 * Math.PI);
      ctx.strokeStyle = cls === 'cattle' ? 'rgba(234, 88, 12, 0.3)' : 'rgba(239, 68, 68, 0.3)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      // Draw obstacle body based on class
      ctx.save();
      ctx.translate(sx, sy);

      if (cls === 'cattle') {
        // Cow representation
        ctx.fillStyle = '#ea580c';
        ctx.fillRect(-1.0 * scale, -0.55 * scale, 2.0 * scale, 1.1 * scale);
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.2;
        ctx.strokeRect(-1.0 * scale, -0.55 * scale, 2.0 * scale, 1.1 * scale);
      } else if (cls === 'pothole') {
        // Pothole dark pit
        ctx.beginPath();
        ctx.arc(0, 0, rPix, 0, 2 * Math.PI);
        ctx.fillStyle = '#1c1917';
        ctx.fill();
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2;
        ctx.stroke();
      } else if (cls === 'auto-rickshaw') {
        // 3-wheeler auto rickshaw (yellow/green)
        ctx.fillStyle = '#eab308';
        ctx.fillRect(-1.2 * scale, -0.7 * scale, 2.4 * scale, 1.4 * scale);
        ctx.fillStyle = '#15803d';
        ctx.fillRect(-0.4 * scale, -0.7 * scale, 1.6 * scale, 1.4 * scale);
        ctx.strokeStyle = '#ffffff';
        ctx.strokeRect(-1.2 * scale, -0.7 * scale, 2.4 * scale, 1.4 * scale);
      } else if (cls === 'pedestrian') {
        // Pedestrian circle
        ctx.beginPath();
        ctx.arc(0, 0, rPix, 0, 2 * Math.PI);
        ctx.fillStyle = '#38bdf8';
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      } else if (cls === 'motorcycle') {
        ctx.fillStyle = '#ec4899';
        ctx.fillRect(-0.9 * scale, -0.35 * scale, 1.8 * scale, 0.7 * scale);
      } else {
        ctx.beginPath();
        ctx.arc(0, 0, rPix, 0, 2 * Math.PI);
        ctx.fillStyle = '#94a3b8';
        ctx.fill();
      }

      // Dynamic velocity vector
      if (obj.world_velocity && (Math.abs(obj.world_velocity[0]) > 0.1 || Math.abs(obj.world_velocity[1]) > 0.1)) {
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(obj.world_velocity[0] * scale * 1.2, -obj.world_velocity[1] * scale * 1.2);
        ctx.strokeStyle = '#facc15';
        ctx.lineWidth = 2.0;
        ctx.stroke();
      }

      ctx.restore();

      // Label text
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 9px JetBrains Mono';
      ctx.textAlign = 'center';
      ctx.fillText(`${cls.toUpperCase()} (${obj.distance.toFixed(1)}m)`, sx, sy - inflPix - 4);
    });

    // 8. Draw Ego Autonomous Vehicle
    const footprint = state?.footprint_vertices || [];
    if (footprint.length === 4) {
      ctx.beginPath();
      const [f0x, f0y] = worldToScreen(footprint[0][0], footprint[0][1]);
      ctx.moveTo(f0x, f0y);
      for (let i = 1; i < 4; i++) {
        const [fx, fy] = worldToScreen(footprint[i][0], footprint[i][1]);
        ctx.lineTo(fx, fy);
      }
      ctx.closePath();
      ctx.fillStyle = '#06b6d4';
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Heading indicator arrow
      const [evx, evy] = worldToScreen(egoX, egoY);
      const eth = egoPose[2];
      ctx.beginPath();
      ctx.moveTo(evx, evy);
      ctx.lineTo(evx + 2.5 * scale * Math.cos(eth), evy - 2.5 * scale * Math.sin(eth));
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2.5;
      ctx.stroke();

      ctx.fillStyle = '#06b6d4';
      ctx.font = 'bold 10px JetBrains Mono';
      ctx.textAlign = 'center';
      ctx.fillText('EGO VEHICLE', evx, evy + 2.2 * scale);
    }
  }, [state, scenario]);

  return (
    <div className="relative w-full h-full rounded-xl overflow-hidden glass-panel border border-white/10 flex flex-col">
      <div className="absolute top-3 left-3 z-10 bg-slate-950/80 backdrop-blur-md px-3 py-1.5 rounded-lg border border-white/10 flex items-center gap-2 text-xs font-mono text-slate-300">
        <span className="w-2 h-2 rounded-full bg-cyan-400 animate-ping" />
        <span>Bird's-Eye-View (BEV) Tactical Simulation</span>
      </div>
      <canvas ref={canvasRef} className="w-full h-full block" />
    </div>
  );
}
