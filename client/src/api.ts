export interface Snapshot {
  t: number;
  lat: number;
  lon: number;
  depth_m: number;
  speed_m_s: number;
  heading_deg: number;
  pitch_deg: number;
  roll_deg: number;
  soc: number;
  voltage_V: number;
  current_A: number;
  power_W: number;
  energy_J: number;
  distance_m: number;
  wp: number;
  wp_total: number;
  running: boolean;
  finished: boolean;
  grounded: boolean;
  plan_loaded: boolean;
}

export interface Waypoint {
  lat: number;
  lon: number;
  depth_m: number;
  speed_m_s: number;
}

export interface Plan {
  distance_m: number;
  duration_s: number;
  energy_J: number;
  waypoints: Waypoint[];
  routed_around_land?: boolean;
  error?: string;
}

export interface MissionRequest {
  start_lat: number;
  start_lon: number;
  goal_lat: number;
  goal_lon: number;
  cruise_depth_m: number;
  cruise_speed_m_s: number;
  descent_rate_m_s: number;
}

const base = ""; // proxied through Vite

export async function postMission(req: MissionRequest): Promise<Plan> {
  const r = await fetch(`${base}/api/mission`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(req),
  });
  // The server returns the plan JSON even on 4xx so we can surface a message.
  let body: Plan | null = null;
  try {
    body = (await r.json()) as Plan;
  } catch {
    // fall through
  }
  if (!r.ok) {
    const msg = (body && body.error) || `HTTP ${r.status}`;
    throw new Error(msg);
  }
  if (!body) throw new Error("empty response");
  return body;
}

export type ControlAction = "play" | "pause" | "reset" | "set_speed" | "set_cursor";

export async function postControl(action: ControlAction, value?: number): Promise<void> {
  const body: Record<string, unknown> = { action };
  if (value !== undefined) body.value = value;
  const r = await fetch(`${base}/api/control`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!r.ok) throw new Error(`control: ${r.status}`);
}

export interface HistoryChunk {
  since: number;
  total: number;
  items: Snapshot[];
}

export async function fetchHistory(since: number): Promise<HistoryChunk> {
  const r = await fetch(`${base}/api/history?since=${since}`);
  if (!r.ok) throw new Error(`history: ${r.status}`);
  return r.json();
}

// ---------------------------- Environment ----------------------------------

export interface BboxGrid {
  lat_min: number;
  lon_min: number;
  lat_max: number;
  lon_max: number;
  width: number;
  height: number;
}

export interface BathyGrid extends BboxGrid {
  max_depth_m: number;
  depths: number[]; // row-major, ny rows of nx values, integer meters
}

export interface CurrentsGrid extends BboxGrid {
  depth_m: number;
  u: number[]; // east-positive m/s, row-major
  v: number[]; // north-positive m/s, row-major
}

function bboxQs(b: BboxGrid): string {
  return `lat_min=${b.lat_min}&lon_min=${b.lon_min}` +
    `&lat_max=${b.lat_max}&lon_max=${b.lon_max}` +
    `&width=${b.width}&height=${b.height}`;
}

export async function fetchBathymetry(b: BboxGrid): Promise<BathyGrid> {
  const r = await fetch(`${base}/api/bathymetry?${bboxQs(b)}`);
  if (!r.ok) throw new Error(`bathymetry: ${r.status}`);
  return r.json();
}

export async function fetchCurrents(b: BboxGrid, depth_m: number): Promise<CurrentsGrid> {
  const r = await fetch(`${base}/api/currents?${bboxQs(b)}&depth_m=${depth_m}`);
  if (!r.ok) throw new Error(`currents: ${r.status}`);
  return r.json();
}

// ---------------------------- Vehicle --------------------------------------

export interface VehicleInfo {
  name: string;
  mass_kg: number;
  volume_m3: number;
  length_m?: number;
  radius_m?: number;
}

export async function postVehicle(length_m: number, radius_m: number, mass_kg: number): Promise<VehicleInfo> {
  const r = await fetch(`${base}/api/vehicle`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ length_m, radius_m, mass_kg }),
  });
  if (!r.ok) throw new Error(`vehicle: ${r.status}`);
  return r.json();
}

export function openStream(onSnap: (s: Snapshot) => void, onErr: () => void): EventSource {
  const es = new EventSource(`${base}/api/stream`);
  es.onmessage = (ev) => {
    try {
      onSnap(JSON.parse(ev.data) as Snapshot);
    } catch {
      // ignore parse error
    }
  };
  es.onerror = () => onErr();
  return es;
}
