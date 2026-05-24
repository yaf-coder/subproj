import "maplibre-gl/dist/maplibre-gl.css";
import "uplot/dist/uPlot.min.css";

import { initMap } from "./map";
import { Telemetry } from "./telemetry";
import {
  fetchBathymetry,
  fetchCurrents,
  fetchHistory,
  openStream,
  postControl,
  postMission,
  postVehicle,
  type BboxGrid,
  type Plan,
  type Snapshot,
} from "./api";

interface UiState {
  start: { lng: number; lat: number } | null;
  goal: { lng: number; lat: number } | null;
  plan: Plan | null;
}

const ui: UiState = { start: null, goal: null, plan: null };

// ----------------------------- Playback state ------------------------------

interface Playback {
  history: Snapshot[];
  scrubbing: boolean;       // true when the user is driving the slider
  liveSnap: Snapshot | null; // latest snapshot from SSE
  speed: number;             // current playback speed multiplier
}

const pb: Playback = { history: [], scrubbing: false, liveSnap: null, speed: 1 };

// ----------------------------- Wiring --------------------------------------

const mapEl = document.getElementById("map")!;
const handle = initMap(mapEl, onMapClick);

const tel = new Telemetry(
  document.getElementById("chart-depth")!,
  document.getElementById("chart-power")!,
  document.getElementById("conn-status")!,
);

const slider = document.getElementById("pb-slider") as HTMLInputElement;
const timeLabel = document.getElementById("pb-time")!;
const timeTotalLabel = document.getElementById("pb-time-total")!;
const liveBtn = document.getElementById("pb-live") as HTMLButtonElement;
const speedBtns = Array.from(
  document.querySelectorAll<HTMLButtonElement>(".pb-speed"),
);

// Track-line drawing. The orange polyline represents "where the sub has been"
// up to the playback cursor. We rebuild it (rather than appending) so
// scrubbing backward correctly trims the track to where the cursor now is,
// and scrubbing forward / playing extends it. Throttled to 200 ms wall-time
// so a 30 Hz SSE stream doesn't cause 30 Hz polyline rebuilds.
let lastTrackRefreshMs = 0;
let lastTrackCursorT = -1;

let stream: EventSource | null = null;

function connectStream() {
  if (stream) stream.close();
  tel.setConnected("warn", "connecting…");
  stream = openStream(
    (s) => {
      pb.liveSnap = s;
      tel.setConnected(
        "ok",
        pb.scrubbing
          ? `streaming · cursor t=${s.t.toFixed(1)}s · (scrubbing)`
          : `streaming · cursor t=${s.t.toFixed(1)}s`,
      );
      if (!pb.scrubbing) {
        tel.update(s);
        if (s.plan_loaded) handle.setSub(s.lon, s.lat, s.heading_deg);
        // Snap the slider to the server's cursor position so the user can
        // see where the live playback is. (Was previously snapping to the
        // last history index, which is the END of the precomputed mission.)
        if (pb.history.length > 0) {
          const idx = indexForTime(s.t);
          slider.value = String(idx);
          updateTimeLabels();
        }
        // Extend / update the orange track polyline up to the current
        // cursor. Throttled inside refreshTrack so this is cheap.
        refreshTrack(s.t);
      } else {
        // Still feed the chart with the live point so it keeps growing.
        tel.updateChartOnly(s);
      }
    },
    () => {
      tel.setConnected("bad", "stream lost · retrying…");
      setTimeout(connectStream, 1500);
    },
  );
}
connectStream();

// Binary-search the local history mirror for the index whose `t` is closest
// to the given sim time. Used to map server cursor times to slider indices.
function indexForTime(t: number): number {
  if (pb.history.length === 0) return 0;
  let lo = 0;
  let hi = pb.history.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (pb.history[mid].t < t) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

// ----------------------------- History polling -----------------------------

async function pollHistory() {
  try {
    const chunk = await fetchHistory(pb.history.length);
    if (chunk.items.length > 0) {
      pb.history.push(...chunk.items);
      // The scrub range always reflects the full precomputed extent, so
      // the user can drag past the live playback cursor immediately as
      // precompute fills history in.
      slider.max = String(Math.max(0, pb.history.length - 1));
      updateTimeLabels();
      // Track may need to extend if precompute caught up to where the
      // cursor already is. Cheap no-op when there's nothing new to add.
      refreshTrack(pb.liveSnap?.t ?? 0);
    }
  } catch {
    // ignored
  }
}
setInterval(pollHistory, 400);

// Rebuild the orange track polyline to cover history[0..cursor]. We
// downsample to ~1 sample per sim-second so very long missions don't ship
// hundreds of thousands of vertices to MapLibre.
function refreshTrack(cursorT: number, force = false) {
  if (pb.history.length === 0) {
    handle.clearTrack();
    lastTrackCursorT = -1;
    return;
  }
  const now =
    typeof performance !== "undefined" ? performance.now() : Date.now();
  if (
    !force &&
    now - lastTrackRefreshMs < 200 &&
    Math.abs(cursorT - lastTrackCursorT) < 0.5
  ) {
    return;
  }
  lastTrackRefreshMs = now;
  lastTrackCursorT = cursorT;

  const endIdx = indexForTime(cursorT);
  const coords: [number, number][] = [];
  let lastSampledT = -2;
  for (let i = 0; i <= endIdx && i < pb.history.length; i++) {
    const e = pb.history[i];
    if (!e.plan_loaded) continue;
    if (e.t - lastSampledT >= 1.0 || i === endIdx) {
      coords.push([e.lon, e.lat]);
      lastSampledT = e.t;
    }
  }
  handle.setTrack(coords);
}

// ----------------------------- Slider + LIVE -------------------------------

function updateTimeLabels() {
  const total = pb.history.length > 0 ? pb.history[pb.history.length - 1].t : 0;
  timeTotalLabel.textContent = `/ ${total.toFixed(1)} s`;
  const idx = parseInt(slider.value, 10);
  const t = pb.history[idx]?.t ?? 0;
  timeLabel.textContent = `t = ${t.toFixed(1)} s`;
}

// While dragging the slider (continuous `input` events): update the local
// display only — purely visual feedback at zero latency. We do not hit the
// server on every event because the user can drag the bar dozens of times
// per second.
slider.addEventListener("input", () => {
  if (pb.history.length === 0) return;
  pb.scrubbing = true;
  liveBtn.classList.remove("live-active");
  liveBtn.classList.add("scrubbing");
  liveBtn.textContent = "GO LIVE";
  const idx = Math.min(parseInt(slider.value, 10), pb.history.length - 1);
  const s = pb.history[idx];
  if (s) {
    tel.updateTextOnly(s);
    if (s.plan_loaded) handle.setSub(s.lon, s.lat, s.heading_deg);
    refreshTrack(s.t);
  }
  updateTimeLabels();
});

// On slider release (`change`): move the server's playback cursor to the
// scrub position so hitting Play resumes from where the user dropped the
// thumb (instead of jumping back to where the server cursor was before).
slider.addEventListener("change", () => {
  if (pb.history.length === 0) return;
  const idx = Math.min(parseInt(slider.value, 10), pb.history.length - 1);
  const s = pb.history[idx];
  if (s) postControl("set_cursor", s.t).catch(console.error);
});

// GO LIVE jumps to wherever the server's playback cursor currently is —
// not to the end of precomputed history. The two are different now: the
// precomputed history may extend hundreds of sim-seconds ahead of where
// the live playback has actually progressed.
liveBtn.addEventListener("click", () => {
  pb.scrubbing = false;
  if (pb.liveSnap && pb.history.length > 0) {
    slider.value = String(indexForTime(pb.liveSnap.t));
    tel.update(pb.liveSnap);
    if (pb.liveSnap.plan_loaded)
      handle.setSub(pb.liveSnap.lon, pb.liveSnap.lat, pb.liveSnap.heading_deg);
    refreshTrack(pb.liveSnap.t, true);
  } else {
    slider.value = "0";
  }
  liveBtn.classList.remove("scrubbing");
  liveBtn.classList.add("live-active");
  liveBtn.textContent = "LIVE";
  updateTimeLabels();
});
// Start in live mode visually.
liveBtn.classList.add("live-active");

// ----------------------------- Speed buttons -------------------------------

function setActiveSpeedBtn(speed: number) {
  for (const b of speedBtns) {
    const v = parseFloat(b.dataset.speed!);
    b.classList.toggle("active", Math.abs(v - speed) < 1e-6);
  }
}

for (const b of speedBtns) {
  b.addEventListener("click", async () => {
    const v = parseFloat(b.dataset.speed!);
    pb.speed = v;
    setActiveSpeedBtn(v);
    try {
      await postControl("set_speed", v);
    } catch (e) {
      console.error(e);
    }
  });
}
setActiveSpeedBtn(1);

// ----------------------------- Map click + Plan ----------------------------

function onMapClick(lng: number, lat: number) {
  if (!ui.start) {
    ui.start = { lng, lat };
    handle.setStart([lng, lat]);
  } else if (!ui.goal) {
    ui.goal = { lng, lat };
    handle.setGoal([lng, lat]);
  } else {
    // Third click resets.
    ui.start = { lng, lat };
    ui.goal = null;
    handle.setStart([lng, lat]);
    handle.setGoal(null);
    handle.setRoute(null);
    handle.clearTrack();
    ui.plan = null;
    tel.setPlanSummary(0, 0, 0, 0);
  }
}

function resetClientHistory() {
  pb.history = [];
  pb.scrubbing = false;
  pb.liveSnap = null;
  lastTrackRefreshMs = 0;
  lastTrackCursorT = -1;
  handle.clearTrack();
  slider.max = "0";
  slider.value = "0";
  liveBtn.classList.remove("scrubbing");
  liveBtn.classList.add("live-active");
  liveBtn.textContent = "LIVE";
  updateTimeLabels();
}

document.getElementById("btn-plan")!.addEventListener("click", async () => {
  if (!ui.start || !ui.goal) {
    alert("Click the map to place a start and a goal.");
    return;
  }
  const cruise_depth_m = parseFloat(
    (document.getElementById("cruise-depth") as HTMLInputElement).value,
  );
  const cruise_speed_m_s = parseFloat(
    (document.getElementById("cruise-speed") as HTMLInputElement).value,
  );
  const descent_rate_m_s = parseFloat(
    (document.getElementById("descent-rate") as HTMLInputElement).value,
  );

  try {
    const plan = await postMission({
      start_lat: ui.start.lat,
      start_lon: ui.start.lng,
      goal_lat: ui.goal.lat,
      goal_lon: ui.goal.lng,
      cruise_depth_m,
      cruise_speed_m_s,
      descent_rate_m_s,
    });
    ui.plan = plan;
    handle.setRoute(plan);
    handle.clearTrack();
    tel.reset();
    resetClientHistory();
    tel.setPlanSummary(
      plan.distance_m,
      plan.duration_s,
      plan.energy_J,
      plan.waypoints.length,
    );
  } catch (e) {
    alert(`Plan failed: ${e}`);
  }
});

document.getElementById("btn-play")!.addEventListener("click", () =>
  postControl("play").catch(console.error),
);
document.getElementById("btn-pause")!.addEventListener("click", () =>
  postControl("pause").catch(console.error),
);
document.getElementById("btn-reset")!.addEventListener("click", () => {
  handle.clearTrack();
  tel.reset();
  resetClientHistory();
  postControl("reset").catch(console.error);
});

// ---------------------------- Vehicle config ------------------------------

const vehLengthEl = document.getElementById("veh-length") as HTMLInputElement;
const vehRadiusEl = document.getElementById("veh-radius") as HTMLInputElement;
const vehMassEl = document.getElementById("veh-mass") as HTMLInputElement;
const vehVolumeEl = document.getElementById("veh-volume")!;
const vehBuoyEl = document.getElementById("veh-buoy")!;

function updateVehicleReadout(length_m: number, radius_m: number, mass_kg: number, volume_m3: number) {
  vehVolumeEl.textContent = `${(volume_m3 * 1000).toFixed(2)} L`;
  // Buoyancy ratio: displaced mass / dry mass. > 1 = positive.
  const buoy_pct = (volume_m3 * 1025 / Math.max(0.1, mass_kg) - 1) * 100;
  vehBuoyEl.textContent = `${buoy_pct >= 0 ? "+" : ""}${buoy_pct.toFixed(1)}%`;
  void length_m;
  void radius_m;
}

document.getElementById("btn-vehicle")!.addEventListener("click", async () => {
  const length_m = parseFloat(vehLengthEl.value);
  const radius_m = parseFloat(vehRadiusEl.value);
  const mass_kg = parseFloat(vehMassEl.value);
  try {
    const v = await postVehicle(length_m, radius_m, mass_kg);
    updateVehicleReadout(length_m, radius_m, mass_kg, v.volume_m3);
  } catch (e) {
    alert(`Vehicle update failed: ${e}`);
  }
});

// Initial readout based on default input values.
updateVehicleReadout(
  parseFloat(vehLengthEl.value),
  parseFloat(vehRadiusEl.value),
  parseFloat(vehMassEl.value),
  Math.PI * 0.10 * 0.10 * 1.0 * 1.002, // matches torpedo() formula
);

// ---------------------------- Environment overlays ------------------------

const ovBathEl = document.getElementById("ov-bath") as HTMLInputElement;
const ovCurrEl = document.getElementById("ov-curr") as HTMLInputElement;

ovBathEl.addEventListener("change", () => handle.setBathymetryVisible(ovBathEl.checked));
ovCurrEl.addEventListener("change", () => handle.setCurrentsVisible(ovCurrEl.checked));

function viewportBboxGrid(): BboxGrid {
  const b = handle.map.getBounds();
  return {
    lat_min: b.getSouth(),
    lon_min: b.getWest(),
    lat_max: b.getNorth(),
    lon_max: b.getEast(),
    width: 80,
    height: 56,
  };
}

let refreshTimer: number | null = null;
let lastBboxKey = "";

function scheduleOverlayRefresh(immediate = false) {
  if (refreshTimer !== null) {
    window.clearTimeout(refreshTimer);
    refreshTimer = null;
  }
  const run = async () => {
    refreshTimer = null;
    const bath_bbox = viewportBboxGrid();
    const curr_bbox: BboxGrid = { ...bath_bbox, width: 14, height: 10 };
    const key = [
      bath_bbox.lat_min.toFixed(3),
      bath_bbox.lon_min.toFixed(3),
      bath_bbox.lat_max.toFixed(3),
      bath_bbox.lon_max.toFixed(3),
    ].join("/");
    if (key === lastBboxKey) return;
    lastBboxKey = key;
    // Pick a representative depth for the current field overlay — the cruise
    // depth the user set, so they're seeing what their sub will fly through.
    const depth = parseFloat(
      (document.getElementById("cruise-depth") as HTMLInputElement).value,
    );
    try {
      const [bath, curr] = await Promise.all([
        fetchBathymetry(bath_bbox),
        fetchCurrents(curr_bbox, isFinite(depth) ? depth : 0),
      ]);
      handle.setBathymetry(bath);
      handle.setCurrents(curr);
    } catch (e) {
      console.error("overlay fetch failed", e);
    }
  };
  if (immediate) {
    void run();
  } else {
    refreshTimer = window.setTimeout(run, 400);
  }
}

if (handle.map.loaded()) {
  scheduleOverlayRefresh(true);
} else {
  handle.map.on("load", () => scheduleOverlayRefresh(true));
}
handle.map.on("moveend", () => scheduleOverlayRefresh());
