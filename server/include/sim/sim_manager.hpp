#pragma once

#include "geo/coords.hpp"
#include "physics/dynamics.hpp"
#include "physics/environment.hpp"
#include "physics/state.hpp"
#include "physics/vehicle.hpp"
#include "planner/planner.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

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
    void load_plan(planner::Plan plan, geo::LatLon origin);
    void play();
    void pause();
    void reset_to_start();
    void start_loop();
    void stop_loop();

    StateSnapshot snapshot() const;
    planner::Plan current_plan() const;

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

    double sim_dt_ = 0.005; // 200 Hz physics
};

} // namespace bathy::sim
