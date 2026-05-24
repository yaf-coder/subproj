#include "sim/sim_manager.hpp"

#include "physics/integrator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace bathy::sim {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double wrap_pi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

// Lerp two angles in degrees taking the short way around the ±180 seam.
inline double lerp_angle_deg(double a, double b, double t) {
    const double diff = std::fmod(b - a + 540.0, 360.0) - 180.0;
    return a + diff * t;
}

// Hard cap on precompute work so a runaway long mission can't block forever.
// 7200 sim-seconds at 200 Hz = 1.44 million steps; on a modern laptop the
// inner RK4 + powertrain + environment step is fast enough that the full
// 1.44 M is still under ~15 wall-seconds.
constexpr double kMaxPrecomputeSimSeconds = 7200.0;
} // namespace

SimulationManager::SimulationManager() {
    vehicle_ = physics::VehicleParams::reference_torpedo();
}

SimulationManager::~SimulationManager() {
    cancel_precompute();
    stop_loop();
}

void SimulationManager::set_vehicle(physics::VehicleParams v) {
    // Cancel any in-flight precompute (it's using the old vehicle).
    cancel_precompute();
    bool need_precompute = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        vehicle_ = std::move(v);
        if (plan_loaded_) {
            reset_physics_for_precompute_locked();
            cursor_t_s_ = 0.0;
            running_ = false;
            need_precompute = true;
        }
    }
    if (need_precompute) spawn_precompute();
}

physics::VehicleParams SimulationManager::vehicle() const {
    std::lock_guard<std::mutex> lk(mu_);
    return vehicle_;
}

void SimulationManager::set_land_mask(const geo::LandMask* land) {
    std::lock_guard<std::mutex> lk(mu_);
    land_ = land;
}

void SimulationManager::set_bathymetry(const physics::Bathymetry* b) {
    std::lock_guard<std::mutex> lk(mu_);
    bath_ = b;
}

void SimulationManager::set_currents(const physics::CurrentField* c) {
    std::lock_guard<std::mutex> lk(mu_);
    currents_ = c;
}

void SimulationManager::set_speed(double s) {
    std::lock_guard<std::mutex> lk(mu_);
    speed_ = std::clamp(s, 0.1, 32.0);
    cv_.notify_all();
}

double SimulationManager::speed() const {
    std::lock_guard<std::mutex> lk(mu_);
    return speed_;
}

void SimulationManager::set_cursor(double t_sim_s) {
    std::lock_guard<std::mutex> lk(mu_);
    if (history_.empty()) {
        cursor_t_s_ = 0.0;
        return;
    }
    const double max_t = history_.back().t_sim_s;
    cursor_t_s_ = std::clamp(t_sim_s, 0.0, max_t);
    // Clear finished if user scrubbed back into the timeline so they can
    // resume play from this point.
    finished_ = (cursor_t_s_ >= max_t - 1e-6);
    cv_.notify_all();
}

double SimulationManager::cursor_t() const {
    std::lock_guard<std::mutex> lk(mu_);
    return cursor_t_s_;
}

double SimulationManager::total_duration_s() const {
    std::lock_guard<std::mutex> lk(mu_);
    return history_.empty() ? 0.0 : history_.back().t_sim_s;
}

std::size_t SimulationManager::history_size() const {
    std::lock_guard<std::mutex> lk(mu_);
    return history_.size();
}

std::vector<StateSnapshot> SimulationManager::history(std::size_t since_index) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (since_index >= history_.size()) return {};
    return std::vector<StateSnapshot>(history_.begin() + since_index, history_.end());
}

void SimulationManager::load_plan(planner::Plan plan, geo::LatLon origin) {
    // Stop any in-flight precompute first; it's running on the old plan.
    cancel_precompute();
    {
        std::lock_guard<std::mutex> lk(mu_);
        plan_ = std::move(plan);
        frame_ = std::make_unique<geo::LocalFrame>(origin);
        plan_loaded_ = true;
        running_ = false;
        cursor_t_s_ = 0.0;
        reset_physics_for_precompute_locked();
    }
    // Spawn precompute *after* releasing mu_ so the new thread can acquire it.
    spawn_precompute();
}

void SimulationManager::play() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!plan_loaded_ || history_.empty()) return;
    // If cursor is at the very end AND the mission's physics is done,
    // rewind to start so Play does something. If precompute is still
    // running, leave the cursor where it is — more history will arrive
    // shortly and the loop will resume advancing automatically.
    const double max_t = history_.back().t_sim_s;
    if (cursor_t_s_ >= max_t - 1e-6 && finished_ && !precompute_busy_.load()) {
        cursor_t_s_ = 0.0;
    }
    running_ = true;
    cv_.notify_all();
}

void SimulationManager::pause() {
    std::lock_guard<std::mutex> lk(mu_);
    running_ = false;
}

void SimulationManager::reset_to_start() {
    std::lock_guard<std::mutex> lk(mu_);
    cursor_t_s_ = 0.0;
    running_ = false;
    if (!history_.empty()) {
        finished_ = false;
    }
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

// -----------------------------------------------------------------------
// Playback loop: walks `cursor_t_s_` forward through `history_` at
// wall-clock pace * playback speed. Does NOT run physics — all the physics
// is done up-front in precompute_full_locked() inside load_plan().
// -----------------------------------------------------------------------
void SimulationManager::loop() {
    using clock = std::chrono::steady_clock;
    constexpr auto kTickWall = std::chrono::milliseconds(33); // ~30 wall-Hz
    constexpr double kTickWallSec = 0.033;

    auto next_tick = clock::now();

    while (!stop_thread_.load()) {
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(50),
                [this] { return stop_thread_.load() || running_; });
            if (stop_thread_.load()) return;

            if (running_ && !history_.empty()) {
                const double max_t = history_.back().t_sim_s;
                if (cursor_t_s_ < max_t) {
                    cursor_t_s_ += kTickWallSec * speed_;
                    if (cursor_t_s_ > max_t) cursor_t_s_ = max_t;
                } else if (finished_ && !precompute_busy_.load()) {
                    // We've reached the actual end of a completed mission.
                    // Stop playback. (If precompute is still adding history,
                    // we just hold the cursor and wait for more.)
                    cursor_t_s_ = max_t;
                    running_ = false;
                }
            }
        }

        next_tick += kTickWall;
        const auto now = clock::now();
        if (next_tick > now) {
            std::this_thread::sleep_until(next_tick);
        } else {
            next_tick = now;
        }
    }
}

// -----------------------------------------------------------------------
// Async precompute infrastructure.
//
// Precompute runs in its own thread, doing physics in batches of ~500
// steps under the mutex and yielding briefly between batches so the HTTP
// handlers, playback loop, and snapshot reads can interleave. This means
// /api/mission and /api/vehicle return immediately, the scrub bar's
// range grows as the precompute thread fills history, and the user can
// watch the bar fill in real time.
// -----------------------------------------------------------------------
void SimulationManager::reset_physics_for_precompute_locked() {
    history_.clear();
    history_accum_ = 0.0;
    state_ = physics::State{};
    wp_idx_ = 0;
    finished_ = false;
    speed_int_ = 0.0;
    depth_rate_int_ = 0.0;
    prev_u_ = 0.0;
    prev_depth_rate_ = 0.0;
    prev_yaw_ = 0.0;
    surge_d_filt_ = 0.0;
    depth_d_filt_ = 0.0;
    yaw_d_filt_ = 0.0;
    tel_ = physics::DynamicsTelemetry{};
}

void SimulationManager::cancel_precompute() {
    precompute_should_stop_.store(true);
    if (precompute_thread_.joinable()) {
        precompute_thread_.join();
    }
    precompute_should_stop_.store(false);
    precompute_busy_.store(false);
}

void SimulationManager::spawn_precompute() {
    // We assume cancel_precompute was already called by the caller.
    precompute_should_stop_.store(false);
    precompute_busy_.store(true);
    precompute_thread_ = std::thread(&SimulationManager::precompute_main, this);
}

void SimulationManager::precompute_main() {
    const auto t0 = std::chrono::steady_clock::now();
    const int max_steps = static_cast<int>(kMaxPrecomputeSimSeconds / sim_dt_);
    int total_steps = 0;

    // Initial seed snapshot at t=0 so cursor_t=0 has something to lerp
    // against — otherwise the first 100 ms of mission time would be empty.
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (precompute_should_stop_.load() || !plan_loaded_) {
            precompute_busy_.store(false);
            return;
        }
        history_.push_back(make_snapshot_locked());
    }

    // Run physics in batches of N steps per locked window so other
    // handlers can interleave. With N=500 and ~40 μs/step, each batch
    // holds the mutex for ~20 ms.
    constexpr int kBatchSteps = 500;
    bool done = false;

    while (!precompute_should_stop_.load() && total_steps < max_steps && !done) {
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (precompute_should_stop_.load()) break;
            int batch = 0;
            while (batch < kBatchSteps && total_steps < max_steps && !finished_) {
                advance_physics_step_locked(sim_dt_);
                ++batch;
                ++total_steps;
            }
            if (finished_) {
                // Append a final snapshot so the cursor can reach the very end.
                history_.push_back(make_snapshot_locked());
                done = true;
            }
        }
        // Yield to other handlers between batches.
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    if (!done && total_steps >= max_steps) {
        std::lock_guard<std::mutex> lk(mu_);
        finished_ = true; // soft finish at the cap
        history_.push_back(make_snapshot_locked());
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    double final_t = 0.0;
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (!history_.empty()) final_t = history_.back().t_sim_s;
    }
    std::cerr << "SimulationManager: precompute "
              << (precompute_should_stop_.load() ? "cancelled" : "complete")
              << " — " << total_steps << " steps (" << final_t
              << " sim-s) in " << ms << " ms\n";
    precompute_busy_.store(false);
}

void SimulationManager::advance_physics_step_locked(double dt) {
    if (finished_) {
        return;
    }

    // Refresh local environment from bathymetry + currents at the sub's
    // current position. We hold these constant across the RK4 substeps —
    // both fields vary slowly enough in space that the small position
    // change inside one 5 ms step doesn't matter.
    if (frame_) {
        const Eigen::Vector2d enu(state_.p_w.x(), state_.p_w.y());
        const geo::LatLon ll = frame_->to_latlon(enu);
        const double cur_depth_m = std::max(0.0, -state_.p_w.z());
        if (bath_) env_.sea_floor_depth_m = bath_->depth_at(ll);
        if (currents_) env_.current_w = currents_->velocity_at(ll, cur_depth_m);
    }

    auto cmd = compute_commands_locked();
    state_ = physics::rk4_step(state_, dt, vehicle_, cmd, env_, tel_);

    // Land safety: if our (lat, lon) is on land, mark grounded and stop.
    if (land_ && land_->loaded() && frame_) {
        const Eigen::Vector2d enu(state_.p_w.x(), state_.p_w.y());
        const geo::LatLon ll = frame_->to_latlon(enu);
        if (land_->is_land(ll)) {
            tel_.grounded = true;
            finished_ = true;
            return;
        }
    }

    // Waypoint advancement. Acceptance radius is generous (10 m horizontal,
    // 4 m vertical) so the sub doesn't get stuck oscillating around a
    // waypoint when real currents push it off course faster than the PID
    // can compensate. The last waypoint uses a tighter criterion so we don't
    // declare "complete" too far from the goal.
    if (frame_ && wp_idx_ < static_cast<int>(plan_.waypoints.size())) {
        const auto& wp = plan_.waypoints[wp_idx_];
        const int wp_count = static_cast<int>(plan_.waypoints.size());
        const bool is_last = (wp_idx_ == wp_count - 1);
        const Eigen::Vector2d wp_enu = frame_->to_enu(wp.ll);
        const Eigen::Vector2d cur_enu(state_.p_w.x(), state_.p_w.y());
        const double horiz = (wp_enu - cur_enu).norm();
        const double depth_err = std::fabs(-state_.p_w.z() - wp.depth_m);
        const double horiz_tol = is_last ? 6.0 : 10.0;
        const double depth_tol = is_last ? 3.0 : 4.0;
        if (horiz < horiz_tol && depth_err < depth_tol) {
            ++wp_idx_;
            // Reset integrators on segment transition to avoid carry-over windup.
            speed_int_ = 0.0;
            depth_rate_int_ = 0.0;
            if (wp_idx_ >= wp_count) {
                finished_ = true;
            }
        }
    }

    // Battery brownout
    if (state_.soc <= 0.02) {
        finished_ = true;
    }

    // Record history at fixed sim-time intervals so scrub resolution is
    // independent of playback speed. Cap total entries so a long mission
    // doesn't grow without bound.
    history_accum_ += dt;
    if (history_accum_ >= kHistoryDt) {
        history_accum_ -= kHistoryDt;
        if (history_.size() < 200000) {
            history_.push_back(make_snapshot_locked());
        }
    }
}

physics::ThrusterCommands SimulationManager::compute_commands_locked() {
    physics::ThrusterCommands cmd(vehicle_.thrusters.size(), 0.0);
    if (!frame_ || wp_idx_ >= static_cast<int>(plan_.waypoints.size())) return cmd;

    const auto& wp = plan_.waypoints[wp_idx_];
    const Eigen::Vector2d wp_enu = frame_->to_enu(wp.ll);
    const Eigen::Vector2d cur_enu(state_.p_w.x(), state_.p_w.y());
    const Eigen::Vector2d delta = wp_enu - cur_enu;
    const double horiz_err = delta.norm();

    const double target_yaw = std::atan2(delta.y(), delta.x());
    const Eigen::Matrix3d R = state_.q.normalized().toRotationMatrix();
    const double cur_yaw = std::atan2(R(1, 0), R(0, 0));
    const double yaw_err = horiz_err < 1.0 ? 0.0 : wrap_pi(target_yaw - cur_yaw);

    const double cur_depth = -state_.p_w.z();
    const double depth_err = wp.depth_m - cur_depth;

    const Eigen::Vector3d v_world = R * state_.v_b;
    const double cur_depth_rate = -v_world.z();
    const double cur_u = state_.v_b.x();

    const double tau_d = 0.08;
    const double alpha_d = sim_dt_ / (sim_dt_ + tau_d);

    const double d_u_raw = (cur_u - prev_u_) / sim_dt_;
    surge_d_filt_ = alpha_d * d_u_raw + (1.0 - alpha_d) * surge_d_filt_;
    prev_u_ = cur_u;

    const double d_dr_raw = (cur_depth_rate - prev_depth_rate_) / sim_dt_;
    depth_d_filt_ = alpha_d * d_dr_raw + (1.0 - alpha_d) * depth_d_filt_;
    prev_depth_rate_ = cur_depth_rate;

    const double d_yaw_raw = wrap_pi(cur_yaw - prev_yaw_) / sim_dt_;
    yaw_d_filt_ = alpha_d * d_yaw_raw + (1.0 - alpha_d) * yaw_d_filt_;
    prev_yaw_ = cur_yaw;

    const double k_yaw_p = 1.5, k_yaw_d = 0.5;
    const double yaw_cmd = std::clamp(-k_yaw_p * yaw_err + k_yaw_d * yaw_d_filt_, -1.0, 1.0);

    const double depth_rate_target =
        std::clamp(depth_err, -wp.cruise_speed_m_s, wp.cruise_speed_m_s);
    const double depth_rate_err = depth_rate_target - cur_depth_rate;
    depth_rate_int_ = std::clamp(depth_rate_int_ + depth_rate_err * sim_dt_, -3.0, 3.0);
    const double k_dr_p = 0.7, k_dr_i = 0.4, k_dr_d = 0.20;
    double vert_cmd = std::clamp(
        -(k_dr_p * depth_rate_err + k_dr_i * depth_rate_int_) + k_dr_d * depth_d_filt_,
        -1.0, 1.0);

    double target_speed = wp.cruise_speed_m_s;
    const double yaw_gain = std::max(0.0, std::cos(std::clamp(yaw_err, -kPi, kPi)));
    target_speed *= yaw_gain;

    const int wp_count = static_cast<int>(plan_.waypoints.size());
    if (wp_idx_ == wp_count - 1) {
        const double brake = std::clamp(horiz_err / 10.0, 0.0, 1.0);
        target_speed *= brake;
    }
    if (horiz_err < 2.0) target_speed = 0.0;

    const double speed_err = target_speed - cur_u;
    speed_int_ = std::clamp(speed_int_ + speed_err * sim_dt_, -2.0, 5.0);

    const double k_v_p = 0.8;
    const double k_v_i = 0.25;
    const double k_v_d = 0.15;
    const double ff = std::clamp(0.12 * target_speed * target_speed, 0.0, 0.6);
    double surge_cmd = std::clamp(
        k_v_p * speed_err + k_v_i * speed_int_ - k_v_d * surge_d_filt_ + ff,
        0.0, 1.0);

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

StateSnapshot SimulationManager::interpolated_snapshot_locked() const {
    // The `finished` flag in the returned snapshot reflects "this is the
    // end of a finished mission" — true only if the precompute completed
    // (finished_) AND the cursor is at the last history entry. That way a
    // scrub back to the middle correctly reports finished=false even
    // though the mission as a whole did finish.
    const bool at_end = !history_.empty() &&
                        cursor_t_s_ >= history_.back().t_sim_s - 1e-6;
    const bool snap_finished = finished_ && at_end;

    StateSnapshot out{};
    out.plan_loaded = plan_loaded_;
    out.total_waypoints = static_cast<int>(plan_.waypoints.size());
    out.running = running_;
    out.finished = snap_finished;

    if (history_.empty()) return out;

    const double t = std::clamp(cursor_t_s_, 0.0, history_.back().t_sim_s);
    auto it = std::lower_bound(
        history_.begin(), history_.end(), t,
        [](const StateSnapshot& s, double tt) { return s.t_sim_s < tt; });

    if (it == history_.begin()) {
        out = history_.front();
    } else if (it == history_.end()) {
        out = history_.back();
    } else {
        const StateSnapshot& b = *it;
        const StateSnapshot& a = *(it - 1);
        const double span = b.t_sim_s - a.t_sim_s;
        const double f = span > 1e-9 ? (t - a.t_sim_s) / span : 0.0;

        out.t_sim_s = t;
        out.lat_deg = lerp(a.lat_deg, b.lat_deg, f);
        out.lon_deg = lerp(a.lon_deg, b.lon_deg, f);
        out.depth_m = lerp(a.depth_m, b.depth_m, f);
        out.speed_m_s = lerp(a.speed_m_s, b.speed_m_s, f);
        out.heading_deg = lerp_angle_deg(a.heading_deg, b.heading_deg, f);
        out.pitch_deg = lerp(a.pitch_deg, b.pitch_deg, f);
        out.roll_deg = lerp(a.roll_deg, b.roll_deg, f);
        out.soc = lerp(a.soc, b.soc, f);
        out.battery_voltage_V = lerp(a.battery_voltage_V, b.battery_voltage_V, f);
        out.battery_current_A = lerp(a.battery_current_A, b.battery_current_A, f);
        out.power_elec_W = lerp(a.power_elec_W, b.power_elec_W, f);
        out.energy_used_J = lerp(a.energy_used_J, b.energy_used_J, f);
        out.distance_traveled_m = lerp(a.distance_traveled_m, b.distance_traveled_m, f);
        out.current_waypoint = (f < 0.5) ? a.current_waypoint : b.current_waypoint;
        out.grounded = a.grounded || b.grounded;
    }

    // Re-apply live state-bits *after* the copy from history (history was
    // recorded with whatever running/finished was at record time, but those
    // are time-of-day values not state-at-cursor values).
    out.plan_loaded = plan_loaded_;
    out.total_waypoints = static_cast<int>(plan_.waypoints.size());
    out.running = running_;
    out.finished = snap_finished;
    return out;
}

StateSnapshot SimulationManager::snapshot() const {
    std::lock_guard<std::mutex> lk(mu_);
    return interpolated_snapshot_locked();
}

planner::Plan SimulationManager::current_plan() const {
    std::lock_guard<std::mutex> lk(mu_);
    return plan_;
}

} // namespace bathy::sim
