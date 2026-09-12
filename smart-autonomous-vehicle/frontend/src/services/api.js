/**
 * REST API client & WebSocket helper for SIH26037 backend.
 */

const API_BASE = 'http://localhost:8000/api';
const WS_BASE = 'ws://localhost:8000/api/ws/stream';

export async function fetchScenarios() {
  try {
    const res = await fetch(`${API_BASE}/scenarios`);
    if (!res.ok) throw new Error('Failed to fetch scenarios');
    return await res.json();
  } catch (err) {
    console.error(err);
    return [];
  }
}

export async function selectScenario(scenarioId) {
  const res = await fetch(`${API_BASE}/scenarios/select`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ scenario_id: scenarioId }),
  });
  return await res.json();
}

export async function resetSimulation() {
  const res = await fetch(`${API_BASE}/simulation/reset`, { method: 'POST' });
  return await res.json();
}

export async function stepSimulation() {
  const res = await fetch(`${API_BASE}/simulation/step`, { method: 'POST' });
  return await res.json();
}

export async function runFullSimulation(scenarioId) {
  const res = await fetch(`${API_BASE}/simulation/run`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ scenario_id: scenarioId }),
  });
  return await res.json();
}

export function createSimulationWebSocket(onMessage, onOpen, onClose) {
  let ws = null;
  try {
    ws = new WebSocket(WS_BASE);
    ws.onopen = () => onOpen && onOpen();
    ws.onmessage = (evt) => {
      try {
        const payload = JSON.parse(evt.data);
        onMessage && onMessage(payload);
      } catch (e) {
        console.error('WS Parse Error', e);
      }
    };
    ws.onclose = () => onClose && onClose();
    ws.onerror = (err) => console.error('WS Error:', err);
  } catch (e) {
    console.error('Failed to create WebSocket:', e);
  }
  return ws;
}
