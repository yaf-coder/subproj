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

void SimulationManager::set_land_mask(const geo::LandMask* land) {
    std::lock_guard<std::mutex> lk(mu_);
    land_ = land;
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
    std::lock_guard<std::mutex> lk(mu_);
    plan_ = std::move(plan);
    frame_ = std::make_unique<geo::LocalFrame>(origin);
    wp_idx_ = 0;
    finished_ = false;
    plan_loaded_ = true;
    state_ = physics::State{};
    speed_int_ = 0.0;
    depth_rate_int_ = 0.0;
    prev_u_ = 0.0;
    prev_depth_rate_ = 0.0;
    prev_yaw_ = 0.0;
    surge_d_filt_ = 0.0;
    depth_d_filt_ = 0.0;
    yaw_d_filt_ = 0.0;
    tel_.grounded = false;
    history_.clear();
    history_accum_ = 0.0;
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
    speed_int_ = 0.0;
    depth_rate_int_ = 0.0;
    prev_u_ = 0.0;
    prev_depth_rate_ = 0.0;
    prev_yaw_ = 0.0;
    surge_d_filt_ = 0.0;
    depth_d_filt_ = 0.0;
    yaw_d_filt_ = 0.0;
    tel_.grounded = false;
    history_.clear();
    history_accum_ = 0.0;
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

    while (!stop_thread_.load()) {
        // Snapshot speed under lock; do the physics steps under the same lock.
        int steps = 1;
        double wall_dt_s = sim_dt_;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(50),
                [this] { return stop_thread_.load() || running_; });
            if (stop_thread_.load()) return;
            if (running_) {
                // For speed >= 1: run N physics steps per wall tick, wall_dt = sim_dt_.
                // For speed <  1: 1 physics step per wall tick, wall_dt = sim_dt_ / speed.
                if (speed_ >= 1.0) {
                    steps = std::max(1, static_cast<int>(std::round(speed_)));
                    wall_dt_s = sim_dt_;
                } else {
                    steps = 1;
                    wall_dt_s = sim_dt_ / std::max(speed_, 0.05);
                }
                // Cap CPU at ~5000 physics steps per outer iteration so a runaway
                // speed setting can't lock up the worker.
                steps = std::min(steps, 5000);
                for (int i = 0; i < steps && running_ && !finished_; ++i) {
                    step_locked(sim_dt_);
                }
            }
        }

        const auto dt_ns = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(wall_dt_s));
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

    // Land safety: if our (lat, lon) is on land, mark grounded and stop.
    if (land_ && land_->loaded() && frame_) {
        const Eigen::Vector2d enu(state_.p_w.x(), state_.p_w.y());
        const geo::LatLon ll = frame_->to_latlon(enu);
        if (land_->is_land(ll)) {
            tel_.grounded = true;
            finished_ = true;
            running_ = false;
            return;
        }
    }

    // Waypoint advancement: if within acceptance radius (5 m horiz + 2 m depth), advance.
    if (frame_ && wp_idx_ < static_cast<int>(plan_.waypoints.size())) {
        const auto& wp = plan_.waypoints[wp_idx_];
        const Eigen::Vector2d wp_enu = frame_->to_enu(wp.ll);
        const Eigen::Vector2d cur_enu(state_.p_w.x(), state_.p_w.y());
        const double horiz = (wp_enu - cur_enu).norm();
        const double depth_err = std::fabs(-state_.p_w.z() - wp.depth_m);
        if (horiz < 5.0 && depth_err < 2.0) {
            ++wp_idx_;
            // Reset integrators on segment transition to avoid carry-over windup.
            speed_int_ = 0.0;
            depth_rate_int_ = 0.0;
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

    // Record history at fixed sim-time intervals so scrub resolution is
    // independent of playback speed. Cap total entries so a long mission
    // doesn't grow without bound (~24 hr at 10 Hz = 864k entries; we cap at
    // 200k = ~5.5 hr).
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

    // Bearing in world ENU frame: east=x, north=y. Heading measured from +x (east),
    // counter-clockwise (so +x = 0 yaw, +y = +pi/2 yaw).
    const double target_yaw = std::atan2(delta.y(), delta.x());

    // Current yaw from quaternion (ZYX convention)
    const Eigen::Matrix3d R = state_.q.normalized().toRotationMatrix();
    const double cur_yaw = std::atan2(R(1, 0), R(0, 0));
    const double yaw_err = horiz_err < 1.0 ? 0.0 : wrap_pi(target_yaw - cur_yaw);

    // Depth (positive going down). target depth from waypoint; current from state.p_w.z (z up).
    const double cur_depth = -state_.p_w.z();
    const double depth_err = wp.depth_m - cur_depth; // + means must go deeper

    const Eigen::Vector3d v_world = R * state_.v_b;
    const double cur_depth_rate = -v_world.z(); // world z is up; descending => +rate
    const double cur_u = state_.v_b.x();

    // ----- Filtered measurement derivatives (derivative-on-measurement) -----
    //
    // We compute d/dt of the *measurement*, not the error, so a discontinuous
    // setpoint change (e.g., waypoint advance flipping target_yaw) doesn't
    // cause a derivative "kick". A one-pole low-pass with tau=80 ms keeps
    // numerical noise (and dynamics faster than the loop bandwidth) out of D.
    const double tau_d = 0.08;
    const double alpha_d = sim_dt_ / (sim_dt_ + tau_d);

    const double d_u_raw = (cur_u - prev_u_) / sim_dt_;
    surge_d_filt_ = alpha_d * d_u_raw + (1.0 - alpha_d) * surge_d_filt_;
    prev_u_ = cur_u;

    const double d_dr_raw = (cur_depth_rate - prev_depth_rate_) / sim_dt_;
    depth_d_filt_ = alpha_d * d_dr_raw + (1.0 - alpha_d) * depth_d_filt_;
    prev_depth_rate_ = cur_depth_rate;

    // wrap_pi on the delta so we don't blow up across the ±pi seam.
    const double d_yaw_raw = wrap_pi(cur_yaw - prev_yaw_) / sim_dt_;
    yaw_d_filt_ = alpha_d * d_yaw_raw + (1.0 - alpha_d) * yaw_d_filt_;
    prev_yaw_ = cur_yaw;

    // ----- Thruster assignment (matches reference_torpedo layout) -----
    //  0: main aft  (surge)
    //  1: bow vert  (heave + pitch)
    //  2: stern vert (heave + pitch)
    //  3: stern lat (yaw + sway)

    // ----- Yaw controller (PD) -----
    //
    // Standard form is u = kP*err - kD*d(meas)/dt. The stern-lat actuator's
    // sign convention is flipped (positive lat thrust => negative yaw moment),
    // so we apply -u: yaw_cmd = -kP*err + kD*d(meas)/dt. Positive yaw rate
    // therefore contributes positive yaw_cmd, which produces a negative yaw
    // moment that damps the rotation.
    const double k_yaw_p = 1.5, k_yaw_d = 0.5;
    const double yaw_cmd = std::clamp(-k_yaw_p * yaw_err + k_yaw_d * yaw_d_filt_, -1.0, 1.0);

    // ----- Depth controller (PID): track wp.cruise_speed_m_s as descent rate -----
    //
    // The waypoint's cruise_speed_m_s is the requested *travel rate to this
    // waypoint*. On descent/ascent waypoints this acts as a depth-rate target;
    // on horizontal cruise waypoints depth_err is small and the rate target
    // collapses to ~0, which is what we want.
    //
    // Vertical thrusters: positive vert_cmd = +z body = upward thrust, so to
    // descend (positive descent rate) we need vert_cmd < 0. Output is therefore
    // negated. With D-on-measurement: a rising descent rate damps itself by
    // pushing vert_cmd up (less downward thrust).
    const double depth_rate_target = std::clamp(depth_err, -wp.cruise_speed_m_s, wp.cruise_speed_m_s);
    const double depth_rate_err = depth_rate_target - cur_depth_rate;
    depth_rate_int_ = std::clamp(depth_rate_int_ + depth_rate_err * sim_dt_, -3.0, 3.0);
    const double k_dr_p = 0.7, k_dr_i = 0.4, k_dr_d = 0.20;
    double vert_cmd = std::clamp(
        -(k_dr_p * depth_rate_err + k_dr_i * depth_rate_int_) + k_dr_d * depth_d_filt_,
        -1.0, 1.0);

    // ----- Surge controller (PID + FF): track wp.cruise_speed_m_s as forward speed -----
    //
    // Forward speed = body-frame x velocity. Slow down when:
    //   * yaw error is large (don't barrel forward while turning)
    //   * we're close to the very last waypoint (final braking)
    double target_speed = wp.cruise_speed_m_s;

    // Reduce target when yaw is way off (use cos, clipped to >= 0).
    const double yaw_gain = std::max(0.0, std::cos(std::clamp(yaw_err, -kPi, kPi)));
    target_speed *= yaw_gain;

    // Decelerate on approach to the final waypoint.
    const int wp_count = static_cast<int>(plan_.waypoints.size());
    if (wp_idx_ == wp_count - 1) {
        // Linear ramp from 10 m down to 0 within ~5 m of the goal.
        const double brake = std::clamp(horiz_err / 10.0, 0.0, 1.0);
        target_speed *= brake;
    }

    // If this leg is mostly vertical (start dive / final ascent at same lat/lon),
    // surge is irrelevant; keep it idle so the depth controller isn't fought.
    if (horiz_err < 2.0) target_speed = 0.0;

    const double speed_err = target_speed - cur_u;
    speed_int_ = std::clamp(speed_int_ + speed_err * sim_dt_, -2.0, 5.0);

    const double k_v_p = 0.8;
    const double k_v_i = 0.25;
    const double k_v_d = 0.15;
    // Feedforward: rough fraction of max thrust needed to sustain target_speed.
    // At target_speed ~ 1 m/s the reference torpedo needs ~12% of max thrust;
    // assume ~v^2 scaling.
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
