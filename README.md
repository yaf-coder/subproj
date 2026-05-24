# swordfish

A full-stack autonomous underwater vehicle (AUV) mission simulator with a deterministic
six-degree-of-freedom physics engine, an energy-aware land-avoiding mission planner,
**real-world bathymetry** (GMT earth_relief, bundled global 15-arcmin) and
**real-world ocean currents** (HYCOM GLBy0.08, bundled 1° × 7 depth levels), a real-time
PID flight controller, and a browser front-end that renders a globally-georeferenced
map with bathymetry and current-vector overlays, live telemetry streaming, mission
history, and variable-speed playback with timeline scrubbing. Procedural and
synthetic environment models are still in the build as fallbacks for missing files
and out-of-coverage regions.

This document is intentionally verbose. It describes both *what* the system does and
*how* it does it, with reference to the relevant files, sub-systems, and the
mathematical models behind each piece.

---

## Table of contents

1. [Architecture overview](#1-architecture-overview)
2. [Repository layout](#2-repository-layout)
3. [Build & run](#3-build--run)
4. [Operational walkthrough](#4-operational-walkthrough)
5. [Server: physics core](#5-server-physics-core)
   - 5.1 [State and integration](#51-state-and-integration)
   - 5.2 [Six-DOF dynamics (Fossen)](#52-six-dof-dynamics-fossen)
   - 5.3 [Vehicle parameterization](#53-vehicle-parameterization)
   - 5.4 [Powertrain: motor and battery](#54-powertrain-motor-and-battery)
6. [Server: environment models](#6-server-environment-models)
   - 6.1 [Land mask](#61-land-mask)
   - 6.2 [Procedural bathymetry](#62-procedural-bathymetry)
   - 6.3 [Synthetic current field](#63-synthetic-current-field)
7. [Server: controller and mission management](#7-server-controller-and-mission-management)
   - 7.1 [PID flight controller](#71-pid-flight-controller)
   - 7.2 [Simulation manager](#72-simulation-manager)
   - 7.3 [Mission planner with A* land avoidance](#73-mission-planner-with-a-land-avoidance)
8. [Server: HTTP / SSE API](#8-server-http--sse-api)
9. [Frontend](#9-frontend)
   - 9.1 [Map and overlays](#91-map-and-overlays)
   - 9.2 [Vehicle and mission configuration](#92-vehicle-and-mission-configuration)
   - 9.3 [Telemetry panel and charts](#93-telemetry-panel-and-charts)
   - 9.4 [Playback bar: scrubbing and speed control](#94-playback-bar-scrubbing-and-speed-control)
10. [Coordinate conventions](#10-coordinate-conventions)
11. [Performance notes and limitations](#11-performance-notes-and-limitations)
12. [Future work](#12-future-work)

---

## 1. Architecture overview

The system is a two-tier client/server application. The server is a native C++17
binary that owns all simulation state and runs the physics, controllers, planner,
and environment models. The client is a TypeScript single-page application built
with Vite that renders the map, accepts user input, and visualizes telemetry. The
two communicate over HTTP plus Server-Sent Events (SSE) for live state streaming.

```
┌─────────────────────────────────────────────────────────────────────────┐
│ Browser (TypeScript + Vite)                                             │
│   ┌─────────────┐  ┌──────────────┐  ┌────────────────────────────────┐│
│   │ MapLibre GL │  │ Vehicle &    │  │ Telemetry panel + uPlot charts ││
│   │   OSM tiles │  │  mission cfg │  │                                ││
│   │   route     │  │ Overlays UI  │  │ depth(t), power(t)+SoC(t)      ││
│   │   track     │  └──────────────┘  └────────────────────────────────┘│
│   │   bathy raster overlay (canvas → image source)                     │
│   │   currents arrow markers                                            │
│   │   playback bar — scrub spans full precomputed range                │
│   └─────────────┘                                                       │
└──────────────────────────────▲──────────────────────────────────────────┘
                  REST + SSE   │   /api/{mission, control, stream, ...}
                               │   set_cursor for scrub-the-server-cursor
┌──────────────────────────────▼──────────────────────────────────────────┐
│ Server (C++17, single binary)                                           │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ SimulationManager (precompute-then-replay)                       │   │
│  │                                                                  │   │
│  │   precompute_thread_  (background, ~125× real-time)              │   │
│  │   ────────────────► fills history_ via RK4 + PID controller      │   │
│  │                                                                  │   │
│  │   loop_ (~30 Hz wall, animates cursor through history_)          │   │
│  │   ────────────────► cursor_t_s_ += dt_wall * speed_              │   │
│  │                                                                  │   │
│  │   snapshot() = lerp(history_[i], history_[i+1]) at cursor_t_s_   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│           ▲                                ▲                            │
│           │ uses                           │ uses                       │
│  ┌────────┴──────────┐         ┌──────────┴──────────────────────────┐ │
│  │ Planner            │         │ HTTP/SSE (cpp-httplib)             │ │
│  │  - great-circle    │         │  - /api/{mission, control,         │ │
│  │  - A* land-avoid   │         │     snapshot, plan, stream,        │ │
│  └────────────────────┘         │     history, bathymetry, currents, │ │
│                                 │     vehicle, health}               │ │
│                                 └────────────────────────────────────┘ │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │ Physics: dynamics, integrator, powertrain                       │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │ Environment: LandMask, Raster+Procedural Bathymetry,            │    │
│  │              Raster+Synthetic CurrentField                      │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

The server is single-process. All simulation state is owned by the `SimulationManager`
singleton, guarded by a single `std::mutex`. The physics loop runs on a dedicated
thread; the HTTP handlers are invoked on cpp-httplib's worker pool. The history
log, snapshot reads, plan loads, vehicle updates, and control commands all
serialize through the same mutex. The mutex is uncontended in steady state because
the physics loop holds it only for the duration of its inner block (one or more
physics ticks plus housekeeping) and the HTTP handlers' work is brief.

---

## 2. Repository layout

```
.
├── README.md                                — this file
├── server/                                  — C++17 simulation server
│   ├── CMakeLists.txt                       — CMake configuration; FetchContent → Eigen 3.4.0
│   ├── data/
│   │   ├── ne_10m_land.geojson              — Natural Earth 1:10m land polygons (≈10 MB)
│   │   ├── ne_50m_land.geojson              — fallback 1:50m polygons (≈1.6 MB)
│   │   ├── earth_relief_15m.bath            — GMT global bathymetry (15-arcmin, ≈4 MB)
│   │   └── hycom_1deg_7depths.curr          — HYCOM ocean currents (1°, 7 depths, ≈7 MB)
│   ├── include/                             — public headers
│   │   ├── geo/
│   │   │   ├── coords.hpp                   — LatLon, LocalFrame (ENU), great_circle_m
│   │   │   └── land_mask.hpp                — LandMask, polygon storage, point-in-poly
│   │   ├── net/
│   │   │   └── server.hpp                   — run_server(), ServerConfig
│   │   ├── physics/
│   │   │   ├── bathymetry.hpp               — Bathymetry interface, ProceduralBathymetry
│   │   │   ├── currents.hpp                 — CurrentField interface, SyntheticCurrentField
│   │   │   ├── dynamics.hpp                 — compute_derivatives(), DynamicsTelemetry
│   │   │   ├── environment.hpp              — Environment (rho, g, seafloor, currents)
│   │   │   ├── integrator.hpp               — rk4_step()
│   │   │   ├── powertrain.hpp               — motor_step(), battery_step(), readings
│   │   │   ├── raster_bathymetry.hpp        — RasterBathymetry (.bath loader)
│   │   │   ├── raster_current_field.hpp     — RasterCurrentField (.curr loader)
│   │   │   ├── raster_grid.hpp              — binary file-format constants & headers
│   │   │   ├── state.hpp                    — State, Derivative
│   │   │   └── vehicle.hpp                  — VehicleParams, HullParams, MotorParams,
│   │   │                                       BatteryParams, Thruster,
│   │   │                                       VehicleParams::torpedo(L, R, m)
│   │   ├── planner/
│   │   │   └── planner.hpp                  — MissionRequest, Plan, plan_mission()
│   │   └── sim/
│   │       └── sim_manager.hpp              — SimulationManager class, StateSnapshot
│   ├── src/                                 — implementations
│   │   ├── geo/{coords,land_mask}.cpp
│   │   ├── net/server.cpp                   — all HTTP/SSE handlers + JSON marshalling
│   │   ├── physics/{bathymetry,currents,
│   │   │             dynamics,integrator,
│   │   │             powertrain,
│   │   │             raster_bathymetry,
│   │   │             raster_current_field}.cpp
│   │   ├── planner/planner.cpp              — great-circle + A* over local ENU grid
│   │   ├── sim/sim_manager.cpp              — control loop, PID, history, speed pacing
│   │   └── main.cpp                         — entry point, CLI parsing, wiring
│   └── third_party/
│       └── httplib.h                        — vendored cpp-httplib single header
├── tools/
│   └── fetch_env_data.py                    — downloads GMT bathymetry + HYCOM currents
│                                              and packs to the bundled binary format
└── client/                                  — TypeScript / Vite frontend
    ├── index.html                           — single-page layout, CSS, control widgets
    ├── package.json, tsconfig.json, vite.config.ts
    └── src/
        ├── api.ts                           — typed REST/SSE client
        ├── main.ts                          — wiring, scrub state, overlay scheduling
        ├── map.ts                           — MapLibre setup, all map layers + markers
        └── telemetry.ts                     — uPlot charts, telemetry-row updates
```

---

## 3. Build & run

### Prerequisites

- **C++17** compiler (Apple Clang 14+, GCC 9+, or Clang 9+).
- **CMake ≥ 3.16.** First configure downloads **Eigen 3.4.0** via `FetchContent` from
  `gitlab.com/libeigen/eigen`. Subsequent builds are offline.
- **Node ≥ 18** and `npm` for the frontend toolchain (TypeScript, Vite, MapLibre GL JS,
  uPlot).

### Server

```sh
cd server
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/swordfish                       # default: bind 0.0.0.0:8080
./build/swordfish --port 8089             # custom TCP port
./build/swordfish --host 127.0.0.1        # custom bind address
./build/swordfish --land path/to.geojson  # custom land polygons
./build/swordfish --no-land               # disable LandMask
./build/swordfish --bathymetry x.bath     # custom .bath file
./build/swordfish --no-bathymetry         # procedural bathymetry only
./build/swordfish --currents x.curr       # custom .curr file
./build/swordfish --no-currents           # synthetic currents only
```

On first start the server:

1. Loads the Natural Earth 1:10m land polygons (≈6800 features) into the
   `LandMask` spatial index. Falls back to 1:50m if 1:10m is unavailable.
2. Builds the `ProceduralBathymetry` distance-to-coast grid (720 × 360 cells,
   half-degree resolution) via a chamfer distance transform seeded by the
   land mask. This one-time cost is roughly 7–9 s on a modern laptop; the
   bottleneck is the 259 200 point-in-polygon classifications, not the
   distance transform itself. This becomes the fallback for the raster
   bathymetry.
3. Loads the bundled raster bathymetry (`data/earth_relief_15m.bath`,
   global 15-arc-minute GMT earth_relief) and validates the header.
4. Loads the bundled raster currents (`data/hycom_1deg_7depths.curr`, a
   1° resampled snapshot of HYCOM `GLBy0.08/expt_93.0` at 7 depth levels)
   and validates the header.
5. Constructs the `SyntheticCurrentField` as a closed-form fallback.
6. Starts the physics thread at 200 Hz with a paused, no-plan state.
7. Begins listening for HTTP/SSE.

A typical startup log:

```
LandMask: loaded 6837 polygons from data/ne_10m_land.geojson
ProceduralBathymetry: built 720x360 distance-to-coast grid in 7489 ms
RasterBathymetry: loaded 1440x720 cells from data/earth_relief_15m.bath
  (bbox lat [-89.875,89.875] lon [-179.875,179.875]; land=353505 ocean=683295
   nodata=0; elev range [-18484, 11971] m)
RasterCurrentField: loaded 375x355x7 from data/hycom_1deg_7depths.curr
  (bbox lat [-80,89.92] lon [0,359.04]; depth [0,500] m; surface valid=92354
   masked=40771; |c|_max=2.537 |c|_mean=0.247 m/s)
swordfish server listening on 0.0.0.0:8080
```

If either binary data file is missing, the server logs a notice and degrades
gracefully to the procedural / synthetic fallback. The data files can be
re-generated at any time with:

```sh
python3 -m venv .venv && source .venv/bin/activate && pip install h5py numpy
python3 tools/fetch_env_data.py bundle
```

### Client

```sh
cd client
npm install
npm run dev      # opens http://localhost:5173
```

The Vite dev server proxies `/api/*` to `http://localhost:8080`. If you change
the server port edit `client/vite.config.ts`.

---

## 4. Operational walkthrough

1. Open `http://localhost:5173`.
2. The left panel contains four sections: **Vehicle & Mission** instructions,
   cruise depth/speed/descent-rate inputs, **Vehicle (torpedo)** geometry inputs,
   and **Map overlays** toggles. The right panel is the live telemetry readout.
3. Click anywhere on the map to place the **start** marker (cyan). Click again to
   place the **goal** marker (orange). A third click resets and re-starts the
   selection.
4. Adjust **Cruise depth (m)**, **Cruise speed (m/s)**, **Descent rate (m/s)**.
5. If you want a different vehicle, change **Length / Radius / Mass** and click
   **Apply vehicle**. The volume and buoyancy readout updates immediately.
6. Click **Plan**. If the great-circle line from start to goal crosses any land
   polygon, the planner falls back to A* on a local ENU grid; otherwise it uses
   great-circle interpolation. The planned route is drawn as a cyan dashed
   polyline with low-opacity dots marking each individual waypoint. The plan
   estimate (distance, duration, energy, waypoint count) is shown in the panel.
7. Click **Play**. The C++ physics runs forward; live state streams back over
   SSE at ~30 Hz. The sub icon follows the route. An orange "actual track"
   polyline trails the sub.
8. Use the **playback bar** at the bottom of the map to scrub the timeline or
   change speed (0.5×, 1×, 2×, 4×, 8×, 16×). Scrubbing temporarily decouples
   the displayed sub position from live state; the **LIVE** button restores it.
9. **Pause** / **Reset** stop the sim. Reset also clears the orange track, the
   history, and the playback bar.

---

## 5. Server: physics core

### 5.1 State and integration

`server/include/physics/state.hpp` defines the integrated state vector and its
time derivative.

```cpp
struct State {
    Eigen::Vector3d p_w;          // position, world frame (ENU), meters
    Eigen::Quaterniond q;         // attitude, body -> world unit quaternion
    Eigen::Vector3d v_b;          // linear velocity, body frame, m/s
    Eigen::Vector3d w_b;          // angular velocity, body frame, rad/s
    double soc;                   // battery state of charge, [0, 1]
    double T_batt;                // battery temperature, K
    double energy_used_J;
    double distance_traveled_m;
    double mission_time_s;
};
```

Integration is **classic 4th-order Runge-Kutta** in
`server/src/physics/integrator.cpp`, with a fixed step of **5 ms (200 Hz)**.
The quaternion derivative `dq = 0.5 · q ⊗ ω` is integrated additively and
re-normalized at the end of each step to keep `|q| = 1`. RK4 was chosen over
explicit Euler because the buoyancy-restoring moment together with the quadratic
damping forms a lightly-damped second-order system in pitch and roll, which
Euler integrates unstably at any reasonable step size.

The environment values (`sea_floor_depth_m`, `current_w`) are sampled once per
physics step from the bathymetry and current field at the sub's *current*
position and held constant across the four RK4 substeps. This is exact for the
sub's translation rate (< 3 m per step at any realistic speed) compared to the
spatial scales of both fields (≥ tens of meters for bathymetry noise, ≥ ~3 km
for the smallest current eddies).

### 5.2 Six-DOF dynamics (Fossen)

`server/src/physics/dynamics.cpp` implements a Fossen-style decomposition:

```
M_eff · dν/dt + C(ν)·ν + D(ν)·ν + g(η) = τ_thrusters
```

where `ν = [v_b; ω_b]` is the body-frame twist, `M_eff = M_rigid + M_added`
(rigid body plus added mass of entrained water), `C(ν)` collects the Coriolis
and centripetal cross-coupling terms (we keep only `m·(ω × v)` and `ω × (I·ω)`),
`D(ν)·ν = D_lin·ν + D_quad·|ν|·ν` is the diagonal hydrodynamic damping in
surge/sway/heave/roll/pitch/yaw, and `g(η)` is the restoring force due to
gravity and buoyancy acting through CG and CB respectively.

The implementation:

- **Thruster contribution.** Each thruster is a body-frame force at a body-frame
  point. Total `F_thr_b = Σ c_i · F_max_i · axis_i` and torque
  `M_thr_b = Σ position_i × F_i`. Mechanical shaft power per thruster is the
  simplified propeller model `P_mech = |T| · (|V_a| + V_ref_induced)`, where
  `V_a` is the axial inflow speed taken from the body-frame relative-to-water
  velocity and `V_ref_induced = 1.0 m/s` is a fixed characteristic induced
  velocity.

- **Gravity & buoyancy.** Both act in world frame and are rotated into body
  frame for force balance; the buoyancy force is applied at `CB` so it produces
  a restoring moment `M_buoy_b = CB × F_buoy_b`. With `CB` above `CG` (the
  reference torpedo has `CB_z = +0.025 m`), the vehicle is pendulously stable
  in pitch and roll.

- **Damping.** Computed against the relative velocity `v_rel_b = v_b − R^T · u_w`
  so currents enter through the drag, not as a position-driven force.

- **Newton-Euler.** With diagonal `M_eff_lin = m·1 + M_a_diag` and
  `I_eff = I + I_a_diag`:
  ```
  dv_b/dt = (F_total_b − m·(ω × v_b)) / M_eff_lin
  dω_b/dt = (M_total_b − ω × (I·ω))   / I_eff
  ```

The output telemetry (pitch/roll/yaw, depth, total thrust, total drag, motor
power, battery current/voltage, grounding flag) is filled into a
`DynamicsTelemetry` struct that the simulation manager copies into each
`StateSnapshot`.

### 5.3 Vehicle parameterization

`server/include/physics/vehicle.hpp` defines a fully parameterized torpedo:

```cpp
VehicleParams::torpedo(length_m, radius_m, mass_kg);
```

scales every relevant quantity from the (1.0 m, 0.10 m, 25 kg) reference:

| Quantity | Formula |
|---|---|
| Hull volume | `max(π·R²·L, m/ρ_water) · 1.002` (≥ 0.2 % positive buoyancy) |
| Inertia, longitudinal | `½ m R²` (solid cylinder) |
| Inertia, transverse | `(1/12) m L² + ¼ m R²` (thin rod + radial term) |
| Added mass, surge | `0.10 · m_disp` (slender body) |
| Added mass, sway/heave | `1.00 · m_disp` |
| Added inertia, roll | `0.02 · m_disp · R²` |
| Added inertia, pitch/yaw | `0.20 · m_disp · L²` |
| Linear damping surge | `k_lin · A_front`, `k_lin = 800` |
| Linear damping sway/heave | `k_lin · A_side`, `A_side = 2·R·L` |
| Quadratic damping | analogous with `k_quad = 600` |
| Thruster positions | `(−0.50, 0.35, −0.40, −0.45) · L` for aft / bow / stern / lat |
| Thruster authority | `35 / 15 / 15 / 12 N · (A_front / A_ref)` |
| Center of buoyancy `z` | `0.025 m · (R / 0.10 m)` (scales with radius) |

Inputs are clamped (`0.2 ≤ L ≤ 6.0`, `0.03 ≤ R ≤ 0.5`, `2 ≤ m ≤ 800`) so silly
values can't NaN the dynamics. The motor and battery parameters are not yet
exposed to the UI; they default to:

- Motor: `η_peak = 0.85`, `load_peak = 0.75`, asymmetric quadratic falloff
  (`α_lo = 1.2`, `α_hi = 0.6`), `idle_power = 2.0 W`.
- Battery: 7s Li-ion (`V_nom = 25.2`, `V_full = 29.4`, `V_empty = 21.0`), 40 Ah,
  Peukert exponent `k = 1.08`, internal resistance 0.04 Ω, thermal mass
  8000 J/K and conductance to seawater 6 W/K.

### 5.4 Powertrain: motor and battery

`server/src/physics/powertrain.cpp` exposes two pure functions:

- `motor_step(MotorParams, P_mech_W, |load|) → MotorOutput`. The efficiency
  model is a hand-crafted asymmetric parabola:
  ```
  η(load) = η_peak · (1 − α · (1 − load/load_peak)²)
  ```
  with `α = α_lo` for `load < load_peak` and `α = α_hi` above. Then
  `P_elec = idle_power + P_mech / max(η, 0.05)`.

- `battery_step(BatteryParams, P_elec_W, SoC, T_K) → BatteryReading`. Open-circuit
  voltage is a smooth piecewise function of SoC: `V_oc = V_empty + (V_full − V_empty) · soc²·(3 − 2·soc)`
  (a Hermite cubic with zero slope at the endpoints). Terminal current solves
  `P_elec = V_term · I, V_term = V_oc − I · R_int` exactly via the quadratic
  formula. Capacity is reduced by **Peukert's law** at high currents:
  `dSoC/dt = − I · (I/I_ref)^(k−1) / (capacity · 3600)`. Resistive heating
  `I²·R_int` feeds the thermal lumped-capacitance model with conductance to
  seawater at the prevailing water temperature.

These models together mean the simulator captures three behaviors that flat-rate
efficiency models miss: **voltage sag** under high current, **capacity loss**
during high-power maneuvers (Peukert), and **operating-point sensitivity** of
motor efficiency (a thruster at 5 % command draws ≈ 10 W of electrical for
≈ 0 W mechanical due to the `idle_power` and the low-load efficiency cliff).

---

## 6. Server: environment models

### 6.1 Land mask

`server/{include,src}/geo/land_mask.{hpp,cpp}` provides a `LandMask` class
backed by **Natural Earth 1:10m land polygons** (public domain). At construction
the GeoJSON is parsed (a minimal hand-rolled scanner that handles only
`Polygon` and `MultiPolygon` geometry types — sufficient for Natural Earth and
avoids pulling in a JSON library), and a 360 × 180 spatial index of
**1-degree cells** is built. Each cell stores the indices of polygons whose
bounding box overlaps the cell.

Query semantics:

- `is_land(lat, lon)` — O(candidate polygons per cell) ray-casting
  point-in-polygon, with a hole test against subsequent rings.
- `segment_crosses_land(a, b, n)` — samples a great-circle path between `a` and
  `b` at `n` intermediate points and short-circuits on the first land hit.

The mask is used in three places:

1. The **planner**, to reject missions whose start or goal is on land, and to
   trigger A* fallback when the straight line crosses land.
2. The **simulation manager**, as a per-step safety check (sets `grounded = true`
   and stops the sim if the sub ever enters a land cell — should be a no-op if
   the planner is doing its job).
3. The **bathymetry constructor**, to seed the distance-to-coast grid.

### 6.2 Bathymetry

The system ships with **two** bathymetry implementations that share a common
virtual interface:

```cpp
class Bathymetry {
public:
    virtual double depth_at(geo::LatLon ll) const = 0;
};
class ProceduralBathymetry : public Bathymetry { ... };
class RasterBathymetry   : public Bathymetry { ... };
```

At startup the server prefers `RasterBathymetry` (loaded from a packed binary
`.bath` file) and falls back to `ProceduralBathymetry` if no raster file is
found *or* if a particular query lands outside the raster's coverage. The
fallback wiring is set up in `main.cpp`:

```cpp
auto proc_bath   = std::make_unique<ProceduralBathymetry>(land);
auto raster_bath = std::make_unique<RasterBathymetry>(proc_bath.get());
raster_bath->load_file("data/earth_relief_15m.bath");
const Bathymetry* effective_bath = raster_bath.get();
```

#### 6.2.1 RasterBathymetry (real GMT earth_relief)

`RasterBathymetry` consumes a flat binary format defined in
`physics/raster_grid.hpp` (`BathyHeader` — 64-byte header + `width*height`
little-endian `float32` cells of elevation in meters, negative below sea level,
row-major with `j=0` at `lat_min`). The default bundled dataset
(`server/data/earth_relief_15m.bath`, ~4 MB) is:

- **Source:** GMT's `earth_relief_15m_p.grd`, a public-domain global
  topography/bathymetry product distributed by the Generic Mapping Tools
  project.
- **Resolution:** 15-arc-minute (≈ 28 km at the equator), pixel-centered.
- **Coverage:** Global, -89.875° to 89.875° latitude, -179.875° to 179.875°
  longitude.
- **Range:** −18 484 m (Mariana Trench-ish) to +11 971 m (Himalayan ice).

At query time, `depth_at(LatLon)`:
1. Wraps longitude into the dataset's range (the bundled grid is global so
   any input longitude works).
2. Computes fractional cell coordinates `(fx, fy)` in cell-center space.
3. Bilinearly interpolates the four corner elevations. If any corner is
   nodata (sentinel `-1e30`), defers to the fallback rather than averaging,
   to avoid producing spurious shallow shelves at the land/water cliff.
4. Returns `max(0, -elevation_m)` so positive output means meters below sea
   level (land returns 0).

Higher-resolution variants (10 m ≈ 9 MB binary, 6 m ≈ 25 MB) can be downloaded
by running `tools/fetch_env_data.py bathymetry --resolution 10m`.

#### 6.2.2 ProceduralBathymetry (fallback)

The procedural implementation produces a believable shape — deep offshore,
shallow on the shelf, canyons-and-ridges noise — without any data dependency.
It serves two purposes: a fallback when the raster file is missing, and a
spatial back-stop for raster queries that land outside the raster's bbox
(e.g., a high-resolution regional `.bath` file with only Pacific coverage
would defer to the procedural model in the Atlantic). It:

- **Builds a global 720 × 360 distance-to-coast grid** (half-degree resolution)
  at construction by classifying each cell center as land or water through the
  `LandMask`, then running a **two-pass Borgefors chamfer distance transform**
  (3-4 chamfer, scaled to `(1, √2)` straight/diagonal weights). The transform
  is O(N) over the grid and runs in tens of milliseconds; the dominant cost is
  the 259 200 `LandMask::is_land` classifications (~7 s on a modern laptop).
- **Bilinearly interpolates** the distance grid at query time and passes the
  result through a **continental-shelf curve**:

  | Distance from coast | Depth |
  |---|---|
  | 0 to 50 km | linear 0 → 200 m (shelf) |
  | 50 to 150 km | linear 200 → 3000 m (slope) |
  | 150 to 600 km | linear 3000 → 5000 m (abyssal plain) |

- **Adds three octaves of sinusoidal procedural noise** in (lat, lon), tapered
  down to zero within 20 km of shore so the noise can't accidentally produce
  above-water features. Peak amplitude offshore is ±200 m, which gives the
  seafloor visible canyons and ridges without being unrealistic.

The simulation manager queries `depth_at()` once per physics step and stores the
result in `Environment::sea_floor_depth_m`; the dynamics flag `grounded = true`
whenever the sub's depth meets or exceeds it.

### 6.3 Ocean currents

As with bathymetry, the system ships with **two** implementations of a common
virtual interface, with `RasterCurrentField` preferred and
`SyntheticCurrentField` as a fallback for missing files or queries outside
coverage.

#### 6.3.1 RasterCurrentField (real HYCOM)

The bundled dataset (`server/data/hycom_1deg_7depths.curr`, ~7 MB) is a single
snapshot from **HYCOM GLBy0.08 expt_93.0**, the U.S. Navy's operational
1/12-degree global ocean reanalysis. We subset it via OPeNDAP to a coarser
1° resolution and **seven standard depth levels** (0, 10, 30, 50, 100, 200,
500 m), keeping `water_u` (east-positive m/s) and `water_v` (north-positive
m/s). The binary format (`CurrentsHeader` — 96-byte header + depth axis +
two `float32` blocks `[n_levels][height][width]`) is described in
`physics/raster_grid.hpp`.

`velocity_at(LatLon, depth_m)` performs **trilinear interpolation** in
(lat, lon, depth):
1. Wraps longitude if the dataset is global (HYCOM is 0°–360°).
2. Brackets the query depth between two adjacent depth levels; clamps to the
   surface or to the deepest level on out-of-range queries (no extrapolation).
3. Bilinearly interpolates `u` and `v` on each bracketing depth plane.
   Corner-by-corner nodata handling replaces masked corners with the mean of
   the valid corners so a single missing cell doesn't take out the whole
   interpolation.
4. Linearly blends the two planes by the bracketed depth fraction.
5. Returns `(u, v, 0)` — the underlying product has no vertical component.

Real-data validation against well-known currents:

| Location | Bearing | Mean magnitude | Comment |
|---|---|---|---|
| Kuroshio Extension (35°N, 145°E) | 60° | 0.68 m/s | Eastward at the Kuroshio extension axis ✓ |
| Gulf Stream (39°N, 67°W) | 349° | 0.38 m/s | NE meander typical of mid-Atlantic ✓ |
| Agulhas Return (38°S, 38°E) | 54° | 0.64 m/s | Northeast retroflection direction ✓ |
| Pacific Equator (0°, 160°W) | 282° | 0.85 m/s | Westward — the South Equatorial Current ✓ |
| ACC near Drake Passage (57°S, 65°W) | 32° | 0.45 m/s | Eastward circumpolar flow component ✓ |

Depth profile through the Gulf Stream (39°N, 67°W), correctly weakening with
depth:

| Depth (m) | mean &#124;c&#124; (m/s) | max &#124;c&#124; (m/s) |
|---|---|---|
| 0 | 0.657 | 1.123 |
| 50 | 0.472 | 1.081 |
| 100 | 0.453 | 0.981 |
| 200 | 0.380 | 0.907 |
| 500 | 0.251 | 0.477 |

The preprocessing tool `tools/fetch_env_data.py currents` can re-fetch a
fresh snapshot or pull a finer grid (`--stride 6` for ½° resolution, etc.).

#### 6.3.2 SyntheticCurrentField (fallback)

When the raster file is missing or a query lands outside the raster's bbox,
the synthetic field takes over. It returns a **world-frame** velocity vector
`(u_east, v_north, 0)` in m/s at any (lat, lon, depth). The field is the
sum of six layers spanning roughly five orders of magnitude in spatial scale,
so variation is visible whether you're looking at a continent or a 2 km square
of ocean:

| Layer | Amplitude | Spatial scale | Purpose |
|---|---|---|---|
| 1 Continental zonal | ±0.04 m/s | hemisphere | weak westward-at-equator background |
| 1b Planetary gyres | ±0.07 m/s | ≈10 000 km | makes the background depend on **lon** too |
| 2 Regional eddies | ±0.10 m/s | ≈500 km | visible at zoom ≤ 6 |
| 3 Mesoscale | ±0.12 m/s | ≈80 km | the realistic Rossby-radius scale; dominates mission-scale variation |
| 4 Submesoscale | ±0.07 m/s | ≈14 km | visible at zoom 11–12 |
| 5 Fine filaments | ±0.04 m/s | ≈3 km | visible at zoom 13–14, prevents close-up overlays from looking constant |

Each layer is a separable product of `sin`/`cos` of `lon_deg` and `lat_deg` with
hand-picked angular frequencies and phase offsets; this is computationally
trivial and gives a smoothly varying, isotropic-looking field everywhere on
Earth. Total worst-case constructive surface magnitude is ≈ 0.45 m/s, with
typical magnitudes 0.05–0.25 m/s — realistic numbers for non-jet ocean currents.

**Depth decay** is applied as a single multiplicative factor
`exp(−max(0, depth_m) / 200 m)`, an Ekman-like e-folding scale. At 200 m depth
the field is at 37 % of surface strength; at 600 m, ~5 %.

The simulation manager queries `velocity_at(ll, depth)` once per physics step
and writes the result into `Environment::current_w`. The dynamics use
`v_rel_b = v_b − R^T · current_w` for *both* damping and the simplified
propeller-power term, so currents affect drag, attitude, energy consumption,
and apparent thruster efficiency in physically consistent ways.

### 6.4 Bundled binary grid format

To avoid dragging a NetCDF or HDF5 dependency into the C++ server, the
preprocessed raster data files are stored in a **versioned, byte-stable
custom format** with two flavors. Both are documented in
`server/include/physics/raster_grid.hpp`.

#### .bath (BathyHeader + elev grid)

```
offset  type    field           notes
─────────────────────────────────────────────────────────────────
  0     u32     magic           = 0x59485442  ("BTHY", little-endian)
  4     u32     version         = 1
  8     u32     width           longitudinal cell count
 12     u32     height          latitudinal cell count
 16     f64     lat_min         southern edge (cell-center conv.)
 24     f64     lat_max         northern edge
 32     f64     lon_min         western edge
 40     f64     lon_max         eastern edge
 48     f32     nodata_value    sentinel (typically -1e30 or NaN)
 52     u8[12]  pad             reserved
 64     f32[w*h] elev_m          row-major; j=0 = lat_min row
                                negative = below MSL (depth)
                                positive = land elevation
```

#### .curr (CurrentsHeader + depth axis + u, v blocks)

```
offset   type        field           notes
─────────────────────────────────────────────────────────────────
   0     u32         magic           = 0x52525543  ("CURR", LE)
   4     u32         version         = 1
   8     u32         width
  12     u32         height
  16     u32         n_levels
  20     u32         reserved        = 0
  24     f64         lat_min
  32     f64         lat_max
  40     f64         lon_min
  48     f64         lon_max
  56     f32         nodata_value
  60     u8[36]      pad             reserved
  96     f32[n_levels]   depths_m    monotonic, ≥ 0
        f32[n_levels*h*w] u_east_m_s  level-major, row-major
        f32[n_levels*h*w] v_north_m_s level-major, row-major
```

Both formats use little-endian throughout; both use a cell-center sample
convention; both expose `nodata_value` so source mask flags survive the
roundtrip. The C++ readers validate magic, version, dimension sanity, and bbox
sanity before mapping any data, so a corrupted or wrong-version file errors
out cleanly with a logged message rather than producing garbage.

### 6.5 Preprocessing tool (`tools/fetch_env_data.py`)

A self-contained Python script that downloads source data and writes our
binary format. Requires `pip install h5py numpy` (or `netCDF4` instead of
`h5py`) — a venv setup is in the repo root (`.venv/`).

```sh
# Fetch both default datasets (≈11 MB total):
python3 tools/fetch_env_data.py bundle

# Or fetch them individually:
python3 tools/fetch_env_data.py bathymetry --resolution 15m
python3 tools/fetch_env_data.py currents --stride 12 --time-idx 100
```

Implementation notes:

- **Bathymetry path:** downloads `earth_relief_{15m,10m,06m}_p.grd` from the
  Generic Mapping Tools server, reads with `h5py` (or `netCDF4` if installed),
  normalizes axis ordering so the output's `j=0` row is at the minimum latitude,
  replaces NaN/inf with the nodata sentinel, packs to `.bath`.
- **Currents path:** issues per-(variable, depth-level) ASCII subset requests
  to HYCOM's THREDDS OPeNDAP server at `tds.hycom.org`, parses the
  multi-block response into NumPy arrays, decodes HYCOM's
  `Int16 scale_factor=0.001 fill=-30000` packing into float m/s, normalizes
  axis ordering, packs to `.curr`. Retries each request up to 3 times.
- Both subcommands emit a one-line summary on success with the cell count,
  bbox, and value range so you can sanity-check before launching the server.

The tool is the **only** consumer of NetCDF/HDF5 in the project; the runtime
server has no such dependency.

---

## 7. Server: controller and mission management

### 7.1 PID flight controller

`SimulationManager::compute_commands_locked()` in `server/src/sim/sim_manager.cpp`
implements three independent control loops that consume the current waypoint
and the sub's current state and produce four normalized thruster commands.

All three loops use **derivative-on-measurement** with a one-pole low-pass on the
derivative term (`τ_d = 80 ms`). Derivative-on-measurement avoids "derivative
kick" at waypoint transitions, where the setpoint can change discontinuously
but the measurement does not. The low-pass filter coefficient is
`α_d = sim_dt / (sim_dt + τ_d) ≈ 0.059` at the default 200 Hz, so high-frequency
measurement noise is heavily attenuated.

| Loop | Form | Gains | Notes |
|---|---|---|---|
| **Yaw** | PD | `kP = 1.5, kD = 0.5` | Setpoint is the bearing to the current waypoint in world frame. `yaw_err` is forced to zero within 1 m of a waypoint to avoid spurious targets. |
| **Depth rate** | PID | `kP = 0.7, kI = 0.4, kD = 0.20` | Setpoint clamps to `± wp.cruise_speed_m_s` so even a 100 m depth error doesn't demand a 100 m/s dive. Integral wind-up clamped to ±3. |
| **Surge speed** | PID + feedforward | `kP = 0.8, kI = 0.25, kD = 0.15` | Setpoint is `wp.cruise_speed_m_s` reduced by `max(0, cos(yaw_err))` during turns and ramped down on approach to the final waypoint. Feedforward is `0.12 · v_target²` (rough drag estimate). Integral clamped to `[−2, +5]` (asymmetric because thrust is `[0, 1]`, not `[−1, +1]`). |

Integrators are reset on **waypoint advance**, on **`load_plan()`**, and on
**`reset_to_start()`**. Previous-measurement state for the derivative term is
similarly reset on plan-load and reset, but **not** on waypoint advance — the
measurement is physically continuous across segment transitions and resetting
it would create a one-step kick.

The four thruster channels of the reference torpedo are mapped:
- channel 0 (`main_aft`, surge): `surge_cmd` ∈ [0, 1]
- channels 1 & 2 (`bow_vert`, `stern_vert`, heave + pitch): `vert_cmd` ∈ [−1, 1]
- channel 3 (`stern_lat`, yaw + sway): `yaw_cmd` ∈ [−1, 1]

The stern lateral thruster has a sign-flipped relationship between commanded
thrust and yaw moment (because its position is aft and its axis is `+y`), which
is handled inside the controller.

### 7.2 Simulation manager (precompute + replay model)

`server/{include,src}/sim/sim_manager.{hpp,cpp}` is the orchestrator. The
architecture is **precompute-then-replay**, not live-physics: a background
thread runs the entire mission's physics as fast as the CPU allows, filling a
history buffer; the playback loop then just walks a cursor through that
history at the user's chosen wall-clock speed. There is no live physics
during playback — `Play` is animation, not simulation.

The class owns:

- the integrated `physics::State` and the latest `DynamicsTelemetry` (used
  only inside the precompute thread),
- the active `VehicleParams`,
- the current `Plan` and the **local ENU `LocalFrame`** anchored at mission
  start,
- pointers (non-owning) to the `LandMask`, `Bathymetry`, and `CurrentField`,
- PID controller state (used only during precompute),
- a recorded **history** of `StateSnapshot` entries at 10 Hz of sim time,
- a **playback cursor** (`cursor_t_s_`), playback **speed**, and pause/play
  flag, all guarded by `std::mutex mu_`,
- a background `precompute_thread_` and an atomic `precompute_should_stop_`
  flag for cancellation.

#### 7.2.1 The precompute thread

When `load_plan()` (or `set_vehicle()` while a plan is loaded) is called:

1. Any in-flight precompute thread is signaled to stop and joined.
2. Under `mu_`, the new plan/vehicle is installed and physics state
   (integrator, PID terms, history) is reset.
3. A new precompute thread is spawned and `load_plan()` returns
   **immediately** (≈ 40 ms total for the HTTP handler).

The precompute thread runs in batches:

```
while not should_stop and total_steps < cap:
    lock mu_
        run kBatchSteps (500) RK4 physics steps via advance_physics_step_locked
        append StateSnapshot to history every 100 ms of sim time
        if finished_:
            push final snapshot
            break
    unlock mu_
    sleep 50 µs   // yield to other handlers
```

500 steps × ~40 µs per step = ~20 ms of lock-holding per batch, then a brief
release. Other operations (snapshot reads, control commands, history fetches)
get periodic windows so the server stays responsive while precompute runs.

On a typical mission of a few hundred sim-seconds, the entire mission is
precomputed in 2–6 wall-seconds. For very long or non-terminating missions,
precompute is hard-capped at 7200 sim-seconds (`kMaxPrecomputeSimSeconds`) so
it always finishes within ~60 wall-seconds.

`advance_physics_step_locked()` is the per-step work and does, in order:

1. Refresh `Environment::sea_floor_depth_m` and `Environment::current_w` from
   the bathymetry and current field at the sub's current lat/lon and depth.
2. Compute thruster commands via the PID controller.
3. Integrate one RK4 step.
4. Check the **land safety condition** — if the sub's lat/lon has moved into
   a land cell, set `grounded`, mark `finished`, return.
5. Check **waypoint advancement** — within 10 m horizontal and 4 m vertical
   of the current waypoint (6 m / 3 m for the final waypoint), increment
   `wp_idx_` and reset PID integrators. The generous tolerance accommodates
   real currents pushing the sub off-station faster than the PID can fully
   compensate.
6. Check **battery brownout** — if `SoC ≤ 0.02`, finish.
7. Append a `StateSnapshot` every 100 ms of sim time. History is capped at
   200 000 entries.

#### 7.2.2 The playback loop

The main loop, started by `start_loop()` at server startup, is now a
wall-clock animator that never runs physics:

```
while not stop_requested:
    lock mu_
    wait on cv_ until (stop or running)
    if running and history not empty:
        max_t = history.back().t_sim_s
        if cursor < max_t:
            cursor += 33ms * speed
            if cursor > max_t: cursor = max_t
        elif finished_ and not precompute_busy_:
            // mission complete and we've watched it all: stop
            running = false
        // else: precompute is still filling history ahead of us;
        // hold the cursor and wait for more.
    unlock mu_
    sleep until next 33ms wall tick
```

`speed_` is the multiplier on cursor advance vs wall-clock. At 1× the cursor
advances at real-time pace; at 16× the cursor advances 16 sim-seconds per
wall-second. The SSE stream samples the cursor's interpolated snapshot at
~30 Hz, so the sub appears to fly through the map smoothly even at high
playback speeds.

#### 7.2.3 Interpolated snapshots

The /api/snapshot endpoint and the SSE stream both call
`interpolated_snapshot_locked()`, which binary-searches `history_` for the
two entries bracketing `cursor_t_s_` and linearly interpolates between them.
All numeric fields (lat, lon, depth, speed, attitude, SoC, voltage, power,
energy, distance) are lerped; **heading uses an angle-aware lerp** that
takes the short way around the ±180° seam; integer fields (current waypoint)
use nearest-neighbor.

The `finished` flag in the returned snapshot is **derived**: true only when
the precompute completed AND the cursor is at the very end of history.
Scrubbing back to the middle of a completed mission correctly reports
`finished = false` because the mission "had not yet finished at that point
in time."

#### 7.2.4 Scrubbing the entire timeline

The `/api/control` endpoint accepts a `set_cursor` action with a sim-time
value, which jumps `cursor_t_s_` to the requested time (clamped to
`[0, history.back().t]`). The frontend's playback bar:

- Updates the slider's *max* to `history.length − 1` as precompute fills
  history (so the scrub range grows in front of the user's eyes).
- During live playback, snaps the slider's *value* to the index closest to
  the server cursor on every SSE message.
- On slider drag (`input` event), updates the local display at zero
  latency — does not touch the server.
- On slider release (`change` event), POSTs `set_cursor` so the server's
  playback cursor jumps to the dropped position. Hitting Play after a
  scrub resumes playback from there.
- `GO LIVE` snaps the slider back to the server's current cursor position
  (which may be anywhere in the precomputed range, not necessarily at the
  end).

Because precompute runs ~100× faster than 1× real-time playback, the user
can scrub anywhere in the mission within a few seconds of hitting Plan —
including parts they have never "watched."

### 7.3 Mission planner with A* land avoidance

`server/{include,src}/planner/planner.cpp` is invoked by the `/api/mission`
handler. Inputs:

```cpp
struct MissionRequest {
    geo::LatLon start, goal;
    double cruise_depth_m = 50.0;
    double cruise_speed_m_s = 1.2;
    double descent_rate_m_s = 0.4;
    double sample_spacing_m = 30.0;
};
```

Algorithm:

1. **Land guards.** If start or goal is on land, return a plan with
   `error = "start position is on land"` (or `"goal..."`) and no waypoints.

2. **Decide cruise polyline.** Call
   `land.segment_crosses_land(start, goal, 96)`. If `false`, generate the cruise
   leg by **great-circle interpolation** at `sample_spacing_m`. If `true`, fall
   back to A* on a local ENU grid:

   - Build a `LocalFrame` at the midpoint of `start` and `goal`.
   - Compute the ENU bounding box of `start` and `goal`, padded by
     `max(dx, dy, 1500m) · 0.6 + 1500 m`.
   - Choose **cell size** = clamp(`max_extent / 250`, 75 m, 750 m). This keeps
     the grid at most 250 × 250 cells (~62 500 nodes) regardless of mission
     length.
   - Mark cells whose center is `is_land` as blocked.
   - **Spiral-snap** start and goal cells to the nearest unblocked cell within
     50 cells.
   - Run **8-connected A*** with the Euclidean (in meters) heuristic and edge
     weights of `cell_m` for cardinal moves and `cell_m · √2` for diagonals,
     forbidding diagonal corner-cuts through land.
   - Recover the path of cell centers, prepend the exact start, append the
     exact goal.

3. **Line-of-sight smoothing.** Greedy shortcut: starting from waypoint *i*,
   advance *j* as far as possible while `los_clear(p[i], p[j], land)` remains
   true (the LOS check samples the great-circle at ≥ 8 points / 75 m
   resolution). This produces a polyline of typically 5–20 waypoints from a
   raw A* path of hundreds of cells.

4. **Resample at `sample_spacing_m`.** Place a waypoint every ~30 m along the
   smoothed polyline so the PID controller has dense steering targets.

5. **Assemble the full plan.** Prepend two dive waypoints (surface at start
   then cruise depth at start, both at `descent_rate_m_s`) and append a
   surface-at-goal waypoint. Each cruise waypoint carries `cruise_speed_m_s`;
   the dive/surface waypoints carry `descent_rate_m_s`.

6. **Estimate distance, duration, energy** using a steady-state model
   `P ~ k · v³ + idle` (with `k = 25, idle = 8 W, η = 0.7`) that's intentionally
   independent of the physics. The estimate is informative; the actual sim
   energy comes from the integrated `state_.energy_used_J`.

The planner is purely a function of the mission request and the land mask; it
takes no state from the simulator, so the user can re-plan without disturbing
an active sim until they call `load_plan`.

---

## 8. Server: HTTP / SSE API

All endpoints emit `application/json` except `/api/stream`, which uses
`text/event-stream`. CORS is enabled for any origin via the `Access-Control-*`
headers. A blanket `OPTIONS` handler returns 204 for preflight.

| Method | Path | Body / Query | Purpose |
|---|---|---|---|
| `GET` | `/api/health` | — | Liveness check (`{"ok": true}`). |
| `POST` | `/api/mission` | `{start_lat, start_lon, goal_lat, goal_lon, cruise_depth_m, cruise_speed_m_s, descent_rate_m_s, sample_spacing_m?}` | Plans a mission, loads it into the sim, returns the plan. On planner error (e.g., goal on land) returns `HTTP 422` with the plan JSON containing an `error` string and empty waypoints. |
| `POST` | `/api/control` | `{action: "play" \| "pause" \| "reset" \| "set_speed" \| "set_cursor", value?}` | Sim control. `set_speed` requires `value` (playback multiplier, clamped to `[0.1, 32.0]`). `set_cursor` requires `value` (sim seconds; clamped to `[0, history.back().t]`). `play` from the very end of a finished mission rewinds to t=0; `play` while precompute is still running holds at the end of available history until more arrives. |
| `GET` | `/api/snapshot` | — | One-shot current state, same shape as the SSE messages. |
| `GET` | `/api/plan` | — | The currently loaded plan. |
| `GET` | `/api/stream` | — | **SSE** stream of `StateSnapshot` JSON at ~30 Hz of wall-clock time. Each event is a single `data: {...}\n\n`. |
| `GET` | `/api/history?since=N` | — | Returns recorded snapshots from index `N` onward as `{since, total, items: [...]}`. Enables incremental client-side polling. |
| `GET` | `/api/bathymetry` | `lat_min, lon_min, lat_max, lon_max, width, height` (4–192 inclusive) | Returns a row-major grid of integer-meter depths sampled at cell centers over the bbox, plus `max_depth_m`. |
| `GET` | `/api/currents` | `lat_min, lon_min, lat_max, lon_max, width, height, depth_m?` | Returns row-major `u` (east) and `v` (north) arrays in m/s at the given depth (default surface). |
| `GET` | `/api/vehicle` | — | Current vehicle name, mass, displacement, thruster count. |
| `POST` | `/api/vehicle` | `{length_m, radius_m, mass_kg}` | Replaces the active vehicle with a fresh `torpedo()` derived from the inputs. |

`StateSnapshot` JSON fields:

| Field | Type | Description |
|---|---|---|
| `t` | number | mission time, seconds |
| `lat`, `lon` | number | sub world position, decimal degrees |
| `depth_m` | number | positive meters below sea level |
| `speed_m_s` | number | body-frame speed magnitude |
| `heading_deg` | number | yaw measured from east, CCW positive (∈ ℝ; the UI normalizes to compass bearing) |
| `pitch_deg`, `roll_deg` | number | Euler angles (ZYX) |
| `soc` | number | battery state of charge, [0, 1] |
| `voltage_V`, `current_A`, `power_W` | number | terminal battery readings |
| `energy_J`, `distance_m` | number | mission-level accumulators |
| `wp`, `wp_total` | int | waypoint progress |
| `running`, `finished`, `grounded`, `plan_loaded` | bool | sim state flags |

---

## 9. Frontend

The frontend is intentionally minimal: vanilla TypeScript, no framework,
two third-party runtime libraries (MapLibre GL JS, uPlot), Vite for build and
dev server.

### 9.1 Map and overlays

`client/src/map.ts` initializes **two** MapLibre `Map` instances in a stacked
top/bottom split inside the main panel:

- **Top: 2D top-down view.** OSM raster base tiles, no pitch. Bathymetry
  colormap overlay, current-arrow markers, start/goal pins, planned route,
  orange track, sub icon.
- **Bottom: 3D tilted terrain view.** Same OSM base tiles plus a
  `raster-dem` source from the AWS Open Data **Mapzen Terrain Tiles**
  (public, no-auth, terrarium-encoded global elevation, **including
  bathymetry**), enabled via `map.setTerrain({ source: "terrain",
  exaggeration: 4 })`. Initial camera at pitch 62°, bearing 20°. A
  hillshade layer on top of OSM gives the terrain visible relief even in
  shallow-perspective views. The same route, track, sub, start/goal pins
  are mirrored from the 2D map.

Clicks on **either** map place start/goal — both maps share an
`onMapClick(lng, lat)` handler. The maps pan and zoom independently
(no synchronized viewport) so the user can keep a fixed top-down view
while orbiting the 3D terrain underneath.

A small composite helper, `combineHandles([handle2d, handle3d])` in
`map.ts`, wraps both maps into one `MapHandle` whose shared visualization
methods (`setStart`, `setGoal`, `setRoute`, `setSub`, `setTrack`,
`clearTrack`) broadcast to both. Bathymetry colormap and current-arrow
overlays only apply to the 2D map — the 3D map renders bathymetry through
real terrain, so a flat overlay on top would be redundant. The `MapHandle` returned by `initMap()` exposes setters for
every overlay artifact (start/goal pins, planned route line, route waypoint
dots, sub marker, actual track, bathymetry raster, current arrows).

Layer ordering, from bottom to top:

1. `osm` (raster) — OpenStreetMap tiles.
2. `bathymetry-layer` (raster) — added below `route-line` when present. Source
   is an `image` source whose data URL is generated from an offscreen canvas
   in `bathyCanvas(grid)`. Color map is **cyan → deep blue → near-black** in
   `sqrt(depth / max_depth)` space so shallow detail isn't crushed. Land
   pixels are fully transparent so the OSM coastline stays visible. Opacity
   is 0.7 when enabled, 0 when disabled (toggle by `setBathymetryVisible`).
3. `route-line` (line) — the planner's dashed cyan polyline.
4. `route-waypoints-dots` (circle) — small (`r = 3`), 45 % opacity dots at
   each interior waypoint; the start and goal pins overlap the endpoints.
5. `track-line` (line) — solid orange polyline of the sub's actual path,
   sampled at ≤ 1 Hz of sim time and **driven by the history buffer** rather
   than the SSE stream so it extends smoothly at high playback speeds.
6. HTML markers (not in the style stack):
   - **Start** pin (cyan circle), **goal** pin (orange circle).
   - **Sub** marker — CSS triangle, rotated to compass bearing (map bearing =
     90° − `heading_deg`, since `heading_deg` is measured from east CCW).
   - **Current arrow markers** — one HTML `div` per grid cell in the active
     `CurrentsGrid`. The div is a horizontal gradient line with a CSS-triangle
     arrowhead at the right end and a small dot at the left, then rotated by
     `atan2(−v, u) ` (degrees). The marker anchor is `"left"`, so the lat/lon
     sample sits at the arrow's tail.

Overlay refresh is driven by **viewport changes**. `main.ts` listens for the
map's `moveend` event, debounces by 400 ms, computes a key from the bbox
(rounded to 0.001°), and refetches `/api/bathymetry?width=80&height=56` and
`/api/currents?width=14&height=10&depth_m=<cruise_depth>` only if the key
differs from the previous fetch. Bathymetry is re-uploaded to the same
MapLibre image source via `updateImage`; current arrows are torn down and
re-created (cheap at 14 × 10 = 140 markers).

The bathymetry endpoint requests grid dimensions chosen as a balance between
resolution and JSON size; 80 × 56 = 4480 integers ≈ 25 KB and renders smoothly.
The currents endpoint uses a coarser 14 × 10 grid because arrow density
beyond that becomes visually noisy.

### 9.2 Vehicle and mission configuration

The **Vehicle (torpedo)** section in the left panel exposes three inputs:
length (m), radius (m), mass (kg). On **Apply vehicle**, the client POSTs to
`/api/vehicle` and displays the returned **volume** (in liters) and
**buoyancy ratio** `(ρ·V / m − 1) × 100%`. The reference torpedo (1.0 m,
0.10 m, 25 kg) produces volume 31.5 L and buoyancy +29.4 %, because the
hull is large enough that the cylindrical volume exceeds the
neutral-buoyancy volume (25 kg / 1025 kg/m³ ≈ 24 L) and the factory uses
the larger of the two.

The **Cruise depth / speed / Descent rate** inputs are read at `Plan` time and
sent to `/api/mission`. The cruise speed input also drives the *current*
overlay refresh depth, so changing it and replanning brings the displayed
current field to the new operating depth on the next viewport refresh.

### 9.3 Telemetry panel and charts

`client/src/telemetry.ts` exposes a `Telemetry` class that owns two **uPlot**
instances:

- `chart-depth` — single trace, depth (m) vs mission time (s).
- `chart-power` — two traces, electrical power (W) and SoC (%, scaled).

Both charts are append-only ring buffers of `MAX_POINTS = 600` samples. The
panel also updates a column of numeric rows: mission time, depth, speed,
heading, pitch/roll, waypoint progress, state label (`no plan / paused /
running / complete / brownout / GROUNDED`), battery SoC, voltage, current,
power, and total energy used.

`update(s)` is split into `updateTextOnly(s)` and `updateChartOnly(s)` so the
playback bar can scrub the numeric readout to historical values while the
charts keep accumulating live data.

### 9.4 Playback bar: scrubbing and speed control

The playback bar (in `client/src/main.ts` plus its DOM in `index.html`) is a
floating panel pinned at the bottom of the map. It contains:

- A **LIVE / GO LIVE** button that toggles scrub mode.
- A **timeline slider** ranging over `[0, history.length − 1]`.
- Mission-time labels showing the scrub position and the latest time.
- Six **speed buttons** (0.5×, 1×, 2×, 4×, 8×, 16×) that POST `set_speed` to
  `/api/control`.

The client maintains a local `pb.history: Snapshot[]` mirror of the server's
history log, populated by polling `/api/history?since=<len>` every 400 ms
and appending new entries. When `pb.scrubbing == false`, the slider follows
the latest entry and incoming SSE updates the sub marker, telemetry rows,
and charts as usual. When the user moves the slider, `pb.scrubbing` is set
to `true`; SSE still arrives but only the **charts** consume it
(`updateChartOnly`), while the sub marker and telemetry rows are driven by
`history[slider.value]`. Pressing **GO LIVE** restores the live view.

History is recorded at **10 Hz of sim time** (not wall time) so the scrub
resolution is independent of playback speed; a 16× run has the same number
of samples per simulated second as a 1× run.

---

## 10. Coordinate conventions

- **World frame.** ENU at the mission origin (set on each `load_plan`):
  `x = east`, `y = north`, `z = up`. Depth is `−p_w.z`.
- **Body frame.** `x = forward`, `y = port`, `z = up` (right-handed). The
  sub's surge axis is body-`+x`.
- **Quaternion.** Body → world rotation, stored as Eigen `Quaterniond` and
  renormalized at the end of each RK4 step.
- **Euler angles.** ZYX Tait-Bryan (yaw-pitch-roll) extracted from the
  rotation matrix on telemetry output only.
- **Heading.** In telemetry, `heading_deg` is the yaw angle measured from
  east (+x) in the CCW direction (mathematical convention). The UI converts to
  compass bearing by `map_bearing = 90 − heading_deg`.
- **Currents.** World-frame `(u_east, v_north, 0)` in m/s. The dynamics use
  body-frame relative-to-water velocity by rotating: `current_b = R^T · current_w`.

---

## 11. Performance notes and limitations

- **Server startup ≈ 7–9 s** dominated by the 259 k `LandMask::is_land`
  classifications during bathymetry construction. The chamfer transform
  itself is negligible.
- **Per-step cost.** The precompute thread runs one RK4 step (4 calls to
  `compute_derivatives`), one `Bathymetry::depth_at` query, one
  `CurrentField::velocity_at` query, one PID evaluation, one waypoint check,
  and occasionally one snapshot push per ~40 µs on a modern Apple Silicon
  laptop, giving roughly 25 k physics steps per wall-second or ~125× real-time.
  The playback loop is essentially free (cursor arithmetic + one
  interpolation per SSE tick).
- **Precompute blocks one core fully.** When the mission is being
  precomputed, one CPU core saturates for a few wall-seconds. During that
  window the server is still responsive (the mutex is released between
  500-step batches) but other queries see ~20 ms of added latency.
- **Precompute cap.** Hard-limited to 7200 sim-seconds. Non-terminating
  missions (e.g., the sub can never reach the goal because of currents)
  precompute the full 7200 s then stop with `finished_ = true`.
- **The land mask is geographic, not bathymetric.** A point in open water is
  classified as "not land" even if the seafloor there is technically above
  current sea level (it never is in practice). The grounding check uses the
  raster bathymetry's actual sea-floor depth, not the land mask.
- **Bathymetry resolution.** The bundled raster is 15-arc-minute (≈ 28 km
  cells). Coastal canyons smaller than ~30 km will be blurred. Run
  `tools/fetch_env_data.py bathymetry --resolution 06m` for a 6-arc-minute
  variant (≈ 11 km cells, ~25 MB).
- **Currents are a single HYCOM snapshot, not time-resolved.** The bundled
  file represents one instant; there is no diurnal cycle, no tides, no
  inertial response, and no time interpolation. The trilinear interpolator
  handles space and depth but holds time constant.
- **Vertical currents are zero.** Neither HYCOM `uv3z` nor the synthetic
  fallback provides a `w` component.
- **The planner energy estimate uses a `k·v³ + idle` model independent of the
  vehicle parameters.** Don't expect tight agreement with the actual energy
  the dynamics report; the dynamics are the source of truth.
- **The browser holds a copy of the full history.** At 10 Hz over a 1-hour
  mission that's 36 000 entries (~5 MB of JS objects). Acceptable; not free.

---

## 12. Future work

- **Time-varying currents.** The current `RasterCurrentField` holds a single
  HYCOM snapshot in memory and ignores the time axis. A time-varying variant
  would memory-map a stack of timestamped snapshots and add a 4th
  interpolation dimension. The OPeNDAP query path in `fetch_env_data.py`
  already supports time indexing.
- **Multi-resolution raster pyramid.** When a regional 1-arc-minute coastal
  bathymetry is loaded, the global 15-arc-minute should still serve as the
  outer-bbox fallback. This is already supported by the fallback wiring
  pattern; the missing piece is composing multiple `RasterBathymetry` objects
  in priority order rather than a single raster + procedural.
- **Vehicle classes beyond torpedo.** Glider (no thrusters except a tail
  rudder, depth driven by buoyancy engine) and hover-class (downward
  thrusters dominant, low forward drag) sharing the same `VehicleParams`
  shape.
- **Motor efficiency map** from a 2-D CSV or table rather than the
  asymmetric-parabola model.
- **Glauert induced-power propeller model** with per-thruster disk area.
- **Replan-on-deviation** (D* Lite) so the sim can recover if the actual
  trajectory deviates from the plan (which currently doesn't happen because
  there's no disturbance large enough to defeat the PID, but will once
  currents become unsteady or thrusters can fault).
- **Failure injection.** Thruster faults, battery cell faults, IMU drift.
- **3D viewer.** Three.js diorama showing the sub in the water column with
  the route, bathymetry mesh, and current vectors visualized in 3D.

---

## License

MIT.
