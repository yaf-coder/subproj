import "maplibre-gl/dist/maplibre-gl.css";
import "uplot/dist/uPlot.min.css";

import { initMap } from "./map";
import { Telemetry } from "./telemetry";
import { openStream, postControl, postMission, type Plan } from "./api";

interface UiState {
  start: { lng: number; lat: number } | null;
  goal: { lng: number; lat: number } | null;
  plan: Plan | null;
}

const ui: UiState = { start: null, goal: null, plan: null };

const mapEl = document.getElementById("map")!;
const handle = initMap(mapEl, onMapClick);

const tel = new Telemetry(
  document.getElementById("chart-depth")!,
  document.getElementById("chart-power")!,
  document.getElementById("conn-status")!,
);

let lastSnapTime = -1;
let stream: EventSource | null = null;

function connectStream() {
  if (stream) stream.close();
  tel.setConnected("warn", "connecting…");
  stream = openStream(
    (s) => {
      tel.setConnected("ok", `streaming · sim t=${s.t.toFixed(1)}s`);
      tel.update(s);
      if (s.plan_loaded) {
        handle.setSub(s.lon, s.lat, s.heading_deg);
        // Throttle track points to ~1 Hz of sim time
        if (s.t - lastSnapTime > 1.0) {
          handle.appendTrack(s.lon, s.lat);
          lastSnapTime = s.t;
        }
      }
    },
    () => {
      tel.setConnected("bad", "stream lost · retrying…");
      setTimeout(connectStream, 1500);
    },
  );
}
connectStream();

function onMapClick(lng: number, lat: number) {
  if (!ui.start) {
    ui.start = { lng, lat };
    handle.setStart([lng, lat]);
  } else if (!ui.goal) {
    ui.goal = { lng, lat };
    handle.setGoal([lng, lat]);
  } else {
    // Third click resets
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

document.getElementById("btn-plan")!.addEventListener("click", async () => {
  if (!ui.start || !ui.goal) {
    alert("Click the map to place a start and a goal.");
    return;
  }
  const cruise_depth_m = parseFloat((document.getElementById("cruise-depth") as HTMLInputElement).value);
  const cruise_speed_m_s = parseFloat((document.getElementById("cruise-speed") as HTMLInputElement).value);
  const descent_rate_m_s = parseFloat((document.getElementById("descent-rate") as HTMLInputElement).value);

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
    tel.setPlanSummary(plan.distance_m, plan.duration_s, plan.energy_J, plan.waypoints.length);
  } catch (e) {
    alert(`Plan failed: ${e}`);
  }
});

document.getElementById("btn-play")!.addEventListener("click", () => postControl("play").catch(console.error));
document.getElementById("btn-pause")!.addEventListener("click", () => postControl("pause").catch(console.error));
document.getElementById("btn-reset")!.addEventListener("click", () => {
  handle.clearTrack();
  tel.reset();
  postControl("reset").catch(console.error);
});
