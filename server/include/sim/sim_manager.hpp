#pragma once

#include "geo/coords.hpp"
#include "geo/land_mask.hpp"
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
    void set_land_mask(const geo::LandMask* land); // not owning; may be null
    void load_plan(planner::Plan plan, geo::LatLon origin);
    void play();
    void pause();
    void reset_to_start();
    void set_speed(double s); // playback multiplier; clamped to [0.1, 32.0]
    double speed() const;
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
    void step_locked(double dt);
    physics::ThrusterCommands compute_commands_locked();
    StateSnapshot make_snapshot_locked() const;

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::thread thread_;
    std::atomic<bool> stop_thread_{false};

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

    // Playback control.
    double speed_ = 1.0;
    double sim_dt_ = 0.005; // 200 Hz physics

    // History recording.
    static constexpr double kHistoryDt = 0.1; // 10 Hz of sim-time
    double history_accum_ = 0.0;
    std::vector<StateSnapshot> history_;
};

} // namespace bathy::sim
