#include "sim/sim_manager.hpp"

#include "physics/integrator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace bathy::sim {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double wrap_pi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
} // namespace

SimulationManager::SimulationManager() {
    vehicle_ = physics::VehicleParams::reference_torpedo();
}

SimulationManager::~SimulationManager() {
    stop_loop();
}

void SimulationManager::set_vehicle(physics::VehicleParams v) {
    std::lock_guard<std::mutex> lk(mu_);
    vehicle_ = std::move(v);
}

void SimulationManager::load_plan(planner::Plan plan, geo::LatLon origin) {
    std::lock_guard<std::mutex> lk(mu_);
    plan_ = std::move(plan);
    frame_ = std::make_unique<geo::LocalFrame>(origin);
    wp_idx_ = 0;
    finished_ = false;
    plan_loaded_ = true;
    state_ = physics::State{};
    // start at origin, on surface, level, full charge
}

void SimulationManager::play() {
    std::lock_guard<std::mutex> lk(mu_);
    if (plan_loaded_ && !finished_) running_ = true;
    cv_.notify_all();
}

void SimulationManager::pause() {
    std::lock_guard<std::mutex> lk(mu_);
    running_ = false;
}

void SimulationManager::reset_to_start() {
    std::lock_guard<std::mutex> lk(mu_);
    state_ = physics::State{};
    wp_idx_ = 0;
    finished_ = false;
    running_ = false;
}

void SimulationManager::start_loop() {
    stop_thread_.store(false);
    thread_ = std::thread(&SimulationManager::loop, this);
}

void SimulationManager::stop_loop() {
    stop_thread_.store(true);
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void SimulationManager::loop() {
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    const auto dt_ns = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(sim_dt_));

    while (!stop_thread_.load()) {
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(50),
                [this] { return stop_thread_.load() || running_; });
            if (stop_thread_.load()) return;
            if (running_) step_locked(sim_dt_);
        }
        next_tick += dt_ns;
        const auto now = clock::now();
        if (next_tick > now) {
            std::this_thread::sleep_until(next_tick);
        } else {
            // We're behind; resync to avoid runaway catch-up.
            next_tick = now;
        }
    }
}

void SimulationManager::step_locked(double dt) {
    if (finished_) {
        running_ = false;
        return;
    }
    auto cmd = compute_commands_locked();
    state_ = physics::rk4_step(state_, dt, vehicle_, cmd, env_, tel_);

    // Waypoint advancement: if within acceptance radius (5 m horiz + 2 m depth), advance.
    if (frame_ && wp_idx_ < static_cast<int>(plan_.waypoints.size())) {
        const auto& wp = plan_.waypoints[wp_idx_];
        const Eigen::Vector2d wp_enu = frame_->to_enu(wp.ll);
        const Eigen::Vector2d cur_enu(state_.p_w.x(), state_.p_w.y());
        const double horiz = (wp_enu - cur_enu).norm();
        const double depth_err = std::fabs(-state_.p_w.z() - wp.depth_m);
        if (horiz < 5.0 && depth_err < 2.0) {
            ++wp_idx_;
            if (wp_idx_ >= static_cast<int>(plan_.waypoints.size())) {
                finished_ = true;
                running_ = false;
            }
        }
    }

    // Battery brownout
    if (state_.soc <= 0.02) {
        finished_ = true;
        running_ = false;
    }
}

physics::ThrusterCommands SimulationManager::compute_commands_locked() {
    physics::ThrusterCommands cmd(vehicle_.thrusters.size(), 0.0);
    if (!frame_ || wp_idx_ >= static_cast<int>(plan_.waypoints.size())) return cmd;

    const auto& wp = plan_.waypoints[wp_idx_];
    const Eigen::Vector2d wp_enu = frame_->to_enu(wp.ll);
    const Eigen::Vector2d cur_enu(state_.p_w.x(), state_.p_w.y());
    const Eigen::Vector2d delta = wp_enu - cur_enu;

    // Bearing in world ENU frame: east=x, north=y. Heading measured from +x (east),
    // counter-clockwise (so +x = 0 yaw, +y = +pi/2 yaw).
    const double target_yaw = std::atan2(delta.y(), delta.x());

    // Current yaw from quaternion (ZYX convention)
    const Eigen::Matrix3d R = state_.q.normalized().toRotationMatrix();
    const double cur_yaw = std::atan2(R(1, 0), R(0, 0));
    const double yaw_err = wrap_pi(target_yaw - cur_yaw);

    // Depth (positive going down). target depth from waypoint; current from state.p_w.z (z up).
    const double cur_depth = -state_.p_w.z();
    const double depth_err = wp.depth_m - cur_depth; // + means must go deeper

    // ----- Thruster assignment (matches reference_torpedo layout) -----
    //  0: main aft  (surge)
    //  1: bow vert  (heave + pitch)
    //  2: stern vert (heave + pitch)
    //  3: stern lat (yaw + sway)

    // P-controller for yaw: stern_lat command. Positive lat thrust => +y body force =>
    // yaw moment z = (-0.45,0,0) x (0,F,0) = (0,0,-0.45F)  (negative yaw).
    // So to increase yaw, command negative lat thrust.
    const double k_yaw = 1.5;
    const double yaw_cmd = std::clamp(-k_yaw * yaw_err, -1.0, 1.0);

    // Depth P controller: positive depth_err means deeper; world +z is up,
    // so we need a downward (-z world) force => -z body (when level) => negative vertical thrust.
    const double k_depth = 0.25;
    double vert_cmd = std::clamp(-k_depth * depth_err, -1.0, 1.0);

    // Add a tiny pitch-keeping term: drive pitch toward depth-error-proportional setpoint.
    // We don't fully use this in M1 — keep simple.

    // Surge: P controller toward target distance, but throttle down if yaw error is large.
    const double horiz_err = delta.norm();
    const double yaw_gain = std::cos(std::clamp(yaw_err, -kPi / 2.0, kPi / 2.0));
    const double k_fwd = 0.04; // per meter
    double surge_cmd = std::clamp(k_fwd * horiz_err * yaw_gain, 0.0, 1.0);

    // If we're on a near-vertical leg (mostly need to change depth, almost no horiz),
    // damp surge.
    if (horiz_err < 3.0 && std::fabs(depth_err) > 1.0) {
        surge_cmd *= 0.2;
    }

    cmd[0] = surge_cmd;
    cmd[1] = vert_cmd;
    cmd[2] = vert_cmd;
    cmd[3] = yaw_cmd;
    return cmd;
}

StateSnapshot SimulationManager::make_snapshot_locked() const {
    StateSnapshot s;
    s.t_sim_s = state_.mission_time_s;
    if (frame_) {
        Eigen::Vector2d enu(state_.p_w.x(), state_.p_w.y());
        const geo::LatLon ll = frame_->to_latlon(enu);
        s.lat_deg = ll.lat_deg;
        s.lon_deg = ll.lon_deg;
    } else {
        s.lat_deg = 0.0;
        s.lon_deg = 0.0;
    }
    s.depth_m = -state_.p_w.z();
    s.speed_m_s = state_.v_b.norm();

    const Eigen::Matrix3d R = state_.q.normalized().toRotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    s.heading_deg = yaw * 180.0 / kPi;
    s.pitch_deg = tel_.pitch_deg;
    s.roll_deg = tel_.roll_deg;
    s.soc = state_.soc;
    s.battery_voltage_V = tel_.battery_voltage_V;
    s.battery_current_A = tel_.battery_current_A;
    s.power_elec_W = tel_.power_elec_W;
    s.energy_used_J = state_.energy_used_J;
    s.distance_traveled_m = state_.distance_traveled_m;
    s.current_waypoint = wp_idx_;
    s.total_waypoints = static_cast<int>(plan_.waypoints.size());
    s.running = running_;
    s.finished = finished_;
    s.grounded = tel_.grounded;
    s.plan_loaded = plan_loaded_;
    return s;
}

StateSnapshot SimulationManager::snapshot() const {
    std::lock_guard<std::mutex> lk(mu_);
    return make_snapshot_locked();
}

planner::Plan SimulationManager::current_plan() const {
    std::lock_guard<std::mutex> lk(mu_);
    return plan_;
}

} // namespace bathy::sim
