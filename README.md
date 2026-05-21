# bathyscaphe

A full-stack AUV (autonomous underwater vehicle) mission simulator.

- **Physics engine**: 6-DOF rigid-body marine dynamics (Fossen-style) in C++17 + Eigen,
  integrated with RK4 at 200 Hz. Models include thruster forces, hydrostatic
  buoyancy with restoring moment, linear & quadratic damping, body-frame Coriolis,
  added-mass diagonal, simplified propeller power, brushless motor efficiency curve,
  and a Li-ion pack with internal resistance, Peukert capacity model, and thermal mass.
- **Planner**: dive → great-circle cruise → surface, with energy estimate.
- **Server**: cpp-httplib (vendored single header) exposes JSON REST and SSE streaming.
- **Frontend**: MapLibre GL JS for the map, uPlot for charts, TypeScript.

This is **milestone 1** — an end-to-end vertical slice. See *Roadmap* below for what's
deliberately simplified and what comes in M2/M3.

## Repo layout

```
server/
  CMakeLists.txt            # FetchContent pulls Eigen on first configure
  include/, src/            # C++ sources
  third_party/httplib.h     # vendored single-header HTTP/SSE library
client/
  index.html, src/          # TS + MapLibre + uPlot
  package.json
```

## Prerequisites

- CMake >= 3.16, a C++17 compiler (Apple clang / gcc / clang)
- Node 18+ and npm
- Internet (CMake fetches Eigen on first build)

## Build & run

### Server

```sh
cd server
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/bathyscaphe                  # listens on 0.0.0.0:8080
./build/bathyscaphe --port 8089      # custom port
```

### Client

```sh
cd client
npm install
npm run dev                          # opens http://localhost:5173
```

The Vite dev server proxies `/api/*` to `http://localhost:8080`. If the server runs
on a different port, edit `client/vite.config.ts`.

## How to use

1. Open the dev UI in a browser.
2. Click anywhere on the map → places the **start** (blue).
3. Click again → places the **goal** (orange).
4. Adjust cruise depth/speed in the left panel and click **Plan**. The planner draws
   the dashed route polyline and shows distance/duration/energy estimates.
5. Click **Play**. The C++ physics engine starts simulating; the right panel streams
   live telemetry at ~30 Hz, the blue sub icon follows the route, and an orange
   track polyline trails it.
6. **Pause** / **Reset** are wired through the same `/api/control` endpoint.

A third map click resets start/goal so you can replan.

## API

| Method | Path | Body | Purpose |
|---|---|---|---|
| `POST` | `/api/mission` | `{start_lat, start_lon, goal_lat, goal_lon, cruise_depth_m, cruise_speed_m_s, descent_rate_m_s}` | Plan a mission, returns waypoints + estimates. |
| `POST` | `/api/control` | `{action: "play" \| "pause" \| "reset"}` | Sim control. |
| `GET` | `/api/snapshot` | — | One-shot current state. |
| `GET` | `/api/plan` | — | Current loaded plan. |
| `GET` | `/api/stream` | — | Server-Sent Events, ~30 Hz state stream. |
| `GET` | `/api/health` | — | Liveness. |

## Physics in one paragraph

State is a 6-DOF rigid body in body coordinates (linear + angular velocity), with
world-frame position and a unit quaternion for attitude. The equation of motion is
`M_eff · dv = F_thrusters + F_buoyancy + F_gravity + F_drag − m·(ω × v)`, with the
angular analogue using the body-frame inertia tensor plus added inertia. Buoyancy
acts at the center of buoyancy (offset above CG, which is what gives the sub its
righting moment in roll and pitch). Damping is split into linear and quadratic
diagonal terms in body axes. Each thruster applies a body-frame force at a body-frame
point; mechanical power is `T · (|V_axial| + V_ref_induced)`; electrical power runs
through a brushless motor efficiency curve and then the Li-ion pack (with internal
resistance giving voltage sag and Peukert reducing capacity at high currents).

## Roadmap

**M1 (this commit) — end-to-end skeleton.** Hardcoded reference vehicle, simplified
propeller power, no bathymetry (flat 200 m floor), no currents, planner is just
great-circle interpolation.

**M2 — real physics & planning.**
- TOML vehicle configs; swap between torpedo/glider/hover-class.
- Full Glauert induced-power propeller model with per-thruster disk area.
- 2D motor efficiency map from a CSV.
- A* energy-optimal planner over a lat/lon grid.
- GEBCO bathymetry as a raster source for the map and as a hard constraint
  for the planner.

**M3 — environment & smarts.**
- Ocean current vector fields (synthetic, then HYCOM/Copernicus).
- Replan-on-deviation (D* Lite).
- Failure injection: thruster faults, current forecast errors, battery aging.
- 3D viewer (tilted MapLibre + glTF sub model).

## License

MIT.
