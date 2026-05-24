#include "physics/dynamics.hpp"

#include "physics/powertrain.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace swordfish::physics {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// Simplified propeller power model:
//   P_mech = |T| * (|V_a| + V_ref)
// where V_a is axial inflow speed and V_ref is a characteristic induced velocity.
// Captures: zero thrust => zero shaft power; power grows with both thrust and
// advance speed. Real model would use Glauert's induced power formula + disk area.
constexpr double kVRefInduced = 1.0;
} // namespace

Derivative compute_derivatives(
    const State& s,
    const VehicleParams& v,
    const ThrusterCommands& cmd,
    const Environment& env,
    DynamicsTelemetry& out_tel) {

    Derivative d;
    const auto& h = v.hull;

    // Body->world rotation; ENU world frame, z up.
    const Eigen::Matrix3d R = s.q.normalized().toRotationMatrix();

    // Velocity relative to surrounding water, in body frame.
    const Eigen::Vector3d current_b = R.transpose() * env.current_w;
    const Eigen::Vector3d v_rel_b = s.v_b - current_b;

    // ----- Thrusters: body-frame force, torque, and mechanical power -----
    Eigen::Vector3d F_th_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d M_th_b = Eigen::Vector3d::Zero();
    double P_mech_total = 0.0;
    double P_elec_total = 0.0;

    const size_t n = v.thrusters.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& t = v.thrusters[i];
        const double c = (i < cmd.size()) ? std::clamp(cmd[i], -1.0, 1.0) : 0.0;
        const Eigen::Vector3d axis = t.axis_b.normalized();
        const Eigen::Vector3d F_i = c * t.max_thrust_N * axis;
        F_th_b += F_i;
        M_th_b += t.position_b.cross(F_i);

        const double V_a = v_rel_b.dot(axis); // signed axial inflow
        const double T_abs = std::fabs(c) * t.max_thrust_N;
        const double P_mech_i = T_abs * (std::fabs(V_a) + kVRefInduced);

        const MotorOutput mo = motor_step(v.motor, P_mech_i, std::fabs(c));
        P_mech_total += P_mech_i;
        P_elec_total += mo.power_elec_W;
    }

    // ----- Gravity & buoyancy (restoring) -----
    // Body frame origin == CG by convention.
    const double Fg = h.mass_kg * env.g;
    const double Fb_mag = env.rho_water * h.volume_m3 * env.g;
    const Eigen::Vector3d Fg_w(0.0, 0.0, -Fg);
    const Eigen::Vector3d Fb_w(0.0, 0.0, Fb_mag);
    const Eigen::Vector3d Fg_b = R.transpose() * Fg_w;
    const Eigen::Vector3d Fb_b = R.transpose() * Fb_w;
    const Eigen::Vector3d M_buoy_b = h.cob_b.cross(Fb_b);

    // ----- Hydrodynamic damping (linear + quadratic) -----
    Eigen::Matrix<double, 6, 1> nu;
    nu << v_rel_b, s.w_b;
    Eigen::Matrix<double, 6, 1> drag6;
    for (int i = 0; i < 6; ++i) {
        drag6(i) = -h.D_lin(i) * nu(i) - h.D_quad(i) * std::fabs(nu(i)) * nu(i);
    }
    const Eigen::Vector3d F_drag_b = drag6.head<3>();
    const Eigen::Vector3d M_drag_b = drag6.tail<3>();

    // ----- Newton-Euler in body frame -----
    const Eigen::Vector3d M_eff_lin =
        h.mass_kg * Eigen::Vector3d::Ones() + h.added_mass_diag;
    const Eigen::Vector3d I_eff = h.inertia_diag + h.added_inertia_diag;

    const Eigen::Vector3d F_total_b = F_th_b + Fg_b + Fb_b + F_drag_b;
    const Eigen::Vector3d M_total_b = M_th_b + M_buoy_b + M_drag_b;

    // m*(dv_b + w x v_b) = F  =>  dv_b = (F - m*(w x v_b)) / m_eff
    const Eigen::Vector3d cor_lin = h.mass_kg * s.w_b.cross(s.v_b);
    Eigen::Vector3d dv_b;
    for (int i = 0; i < 3; ++i) {
        dv_b(i) = (F_total_b(i) - cor_lin(i)) / M_eff_lin(i);
    }

    // I*dw + w x (I*w) = M
    const Eigen::Vector3d Iw = h.inertia_diag.asDiagonal() * s.w_b;
    const Eigen::Vector3d cor_ang = s.w_b.cross(Iw);
    Eigen::Vector3d dw_b;
    for (int i = 0; i < 3; ++i) {
        dw_b(i) = (M_total_b(i) - cor_ang(i)) / I_eff(i);
    }

    // ----- Kinematics -----
    d.dp_w = R * s.v_b;
    const Eigen::Quaterniond omega_q(0.0, s.w_b.x(), s.w_b.y(), s.w_b.z());
    const Eigen::Quaterniond dq_full = s.q * omega_q;
    d.dq = Eigen::Quaterniond(0.5 * dq_full.w(),
                              0.5 * dq_full.x(),
                              0.5 * dq_full.y(),
                              0.5 * dq_full.z());
    d.dv_b = dv_b;
    d.dw_b = dw_b;

    // ----- Battery -----
    const BatteryReading br = battery_step(v.battery, P_elec_total, s.soc, s.T_batt);
    d.dsoc = br.dsoc_per_s;
    d.dT_batt = br.dT_per_s;
    d.denergy = P_elec_total;
    d.ddistance = s.v_b.norm();

    // ----- Telemetry -----
    out_tel.speed_m_s = s.v_b.norm();
    out_tel.thrust_total_N = F_th_b.norm();
    out_tel.drag_total_N = F_drag_b.norm();
    out_tel.power_mech_W = P_mech_total;
    out_tel.power_elec_W = P_elec_total;
    out_tel.battery_current_A = br.current_A;
    out_tel.battery_voltage_V = br.voltage_V;

    const double pitch = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
    const double roll = std::atan2(R(2, 1), R(2, 2));
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    out_tel.roll_deg = rad2deg(roll);
    out_tel.pitch_deg = rad2deg(pitch);
    out_tel.yaw_deg = rad2deg(yaw);
    out_tel.depth_m = -s.p_w.z();
    out_tel.grounded = (out_tel.depth_m >= env.sea_floor_depth_m);

    return d;
}

} // namespace swordfish::physics
