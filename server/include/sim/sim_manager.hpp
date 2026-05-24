#pragma once

#include "geo/coords.hpp"
#include "geo/land_mask.hpp"
#include "physics/bathymetry.hpp"
#include "physics/currents.hpp"
#include "physics/dynamics.hpp"
#include "physics/environment.hpp"
#include "physics/state.hpp"
#include "physics/vehicle.hpp"
#include "planner/planner.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace bathy::sim {

// Plain-old data snapshot of the live simulation state, safe to copy out.
struct StateSnapshot {
    double t_sim_s = 0.0;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double depth_m = 0.0;
    double speed_m_s = 0.0;
    double heading_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    double soc = 1.0;
    double battery_voltage_V = 0.0;
    double battery_current_A = 0.0;
    double power_elec_W = 0.0;
    double energy_used_J = 0.0;
    double distance_traveled_m = 0.0;
    int current_waypoint = 0;
    int total_waypoints = 0;
    bool running = false;
    bool finished = false;
    bool grounded = false;
    bool plan_loaded = false;
};

class SimulationManager {
public:
    SimulationManager();
    ~SimulationManager();

    void set_vehicle(physics::VehicleParams v);
    physics::VehicleParams vehicle() const;
    void set_land_mask(const geo::LandMask* land); // not owning; may be null
    void set_bathymetry(const physics::Bathymetry* b);
    void set_currents(const physics::CurrentField* c);

    // load_plan() synchronously runs the entire mission's physics to
    // completion (or to a generous time cap), populates the full history,
    // then leaves the playback cursor at t=0. After load_plan() returns,
    // the entire timeline is available for scrubbing.
    void load_plan(planner::Plan plan, geo::LatLon origin);

    // Playback controls — these manipulate a cursor that walks through the
    // precomputed history at wall-clock speed * the playback speed. They do
    // not re-run physics.
    void play();
    void pause();
    void reset_to_start();
    void set_speed(double s);    // playback multiplier; clamped to [0.1, 32.0]
    double speed() const;
    void set_cursor(double t_sim_s); // jump cursor to sim time t
    double cursor_t() const;
    double total_duration_s() const; // sim-time covered by history (0 if no plan)

    void start_loop();
    void stop_loop();

    StateSnapshot snapshot() const;
    planner::Plan current_plan() const;

    // Recorded snapshots at fixed sim-time intervals. Lock-free snapshot copy.
    // `since_index` skips records before that index (returns the rest).
    std::vector<StateSnapshot> history(std::size_t since_index = 0) const;
    std::size_t history_size() const;

private:
    void loop();
    // Runs one physics RK4 step plus all associated bookkeeping (land safety,
    // waypoint advancement, brownout check, history record). Used only by
    // the precompute phase — the runtime loop() never calls this.
    void advance_physics_step_locked(double dt);
    physics::ThrusterCommands compute_commands_locked();
    StateSnapshot make_snapshot_locked() const;
    // Build a snapshot at the current playback cursor by linearly
    // interpolating between adjacent history entries.
    StateSnapshot interpolated_snapshot_locked() const;
    // Stops any precompute thread and waits for it to exit.
    void cancel_precompute();
    // Spawns a background thread that runs advance_physics_step_locked
    // repeatedly until the mission ends or precompute_should_stop_ is set.
    // The thread releases the mutex periodically so other handlers can run.
    void spawn_precompute();
    void precompute_main();
    // Reset all per-mission physics state in preparation for a new precompute.
    // Caller must hold mu_.
    void reset_physics_for_precompute_locked();

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::thread thread_;
    std::atomic<bool> stop_thread_{false};

    // Background precompute thread + cancellation flag.
    std::thread precompute_thread_;
    std::atomic<bool> precompute_should_stop_{false};
    std::atomic<bool> precompute_busy_{false};

    physics::State state_;
    physics::VehicleParams vehicle_;
    physics::Environment env_;
    physics::DynamicsTelemetry tel_;

    planner::Plan plan_;
    std::unique_ptr<geo::LocalFrame> frame_;
    int wp_idx_ = 0;

    bool running_ = false;
    bool finished_ = false;
    bool plan_loaded_ = false;

    // PID controller state.
    //   *_int_   : integrator (anti-windup-clamped)
    //   prev_*_  : previous measurement, for derivative-on-measurement
    //   *_d_filt_: low-pass-filtered derivative term (tau = 80 ms)
    double speed_int_ = 0.0;
    double depth_rate_int_ = 0.0;
    double prev_u_ = 0.0;
    double prev_depth_rate_ = 0.0;
    double prev_yaw_ = 0.0;
    double surge_d_filt_ = 0.0;
    double depth_d_filt_ = 0.0;
    double yaw_d_filt_ = 0.0;

    const geo::LandMask* land_ = nullptr;
    const physics::Bathymetry* bath_ = nullptr;
    const physics::CurrentField* currents_ = nullptr;

    // Physics step size for precompute. The runtime loop has a different
    // (wall-clock-driven) cadence.
    double sim_dt_ = 0.005; // 200 Hz physics

    // Playback state.
    double speed_ = 1.0;       // multiplier on cursor advance vs wall-clock
    double cursor_t_s_ = 0.0;  // current playback time within the precomputed history

    // History recording: written by advance_physics_step_locked during
    // precompute; read by the playback loop and the /api/history endpoint.
    static constexpr double kHistoryDt = 0.1; // 10 Hz of sim-time
    double history_accum_ = 0.0;
    std::vector<StateSnapshot> history_;
};

} // namespace bathy::sim
