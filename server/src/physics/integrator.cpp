#include "physics/integrator.hpp"

namespace bathy::physics {

namespace {

State apply(const State& s, const Derivative& d, double dt) {
    State out = s;
    out.p_w = s.p_w + d.dp_w * dt;
    out.q.coeffs() = s.q.coeffs() + d.dq.coeffs() * dt;
    out.q.normalize();
    out.v_b = s.v_b + d.dv_b * dt;
    out.w_b = s.w_b + d.dw_b * dt;
    out.soc = s.soc + d.dsoc * dt;
    out.T_batt = s.T_batt + d.dT_batt * dt;
    out.energy_used_J = s.energy_used_J + d.denergy * dt;
    out.distance_traveled_m = s.distance_traveled_m + d.ddistance * dt;
    out.mission_time_s = s.mission_time_s + dt;
    return out;
}

Derivative weighted(const Derivative& k1, const Derivative& k2,
                    const Derivative& k3, const Derivative& k4) {
    Derivative k;
    k.dp_w = (k1.dp_w + 2.0 * k2.dp_w + 2.0 * k3.dp_w + k4.dp_w) / 6.0;
    k.dq.coeffs() = (k1.dq.coeffs() + 2.0 * k2.dq.coeffs() +
                     2.0 * k3.dq.coeffs() + k4.dq.coeffs()) / 6.0;
    k.dv_b = (k1.dv_b + 2.0 * k2.dv_b + 2.0 * k3.dv_b + k4.dv_b) / 6.0;
    k.dw_b = (k1.dw_b + 2.0 * k2.dw_b + 2.0 * k3.dw_b + k4.dw_b) / 6.0;
    k.dsoc = (k1.dsoc + 2.0 * k2.dsoc + 2.0 * k3.dsoc + k4.dsoc) / 6.0;
    k.dT_batt = (k1.dT_batt + 2.0 * k2.dT_batt + 2.0 * k3.dT_batt + k4.dT_batt) / 6.0;
    k.denergy = (k1.denergy + 2.0 * k2.denergy + 2.0 * k3.denergy + k4.denergy) / 6.0;
    k.ddistance = (k1.ddistance + 2.0 * k2.ddistance + 2.0 * k3.ddistance + k4.ddistance) / 6.0;
    return k;
}

} // namespace

State rk4_step(const State& s, double dt, const VehicleParams& v,
               const ThrusterCommands& cmd, const Environment& env,
               DynamicsTelemetry& out_tel) {
    DynamicsTelemetry t_dummy;
    const Derivative k1 = compute_derivatives(s, v, cmd, env, out_tel);
    const Derivative k2 = compute_derivatives(apply(s, k1, dt * 0.5), v, cmd, env, t_dummy);
    const Derivative k3 = compute_derivatives(apply(s, k2, dt * 0.5), v, cmd, env, t_dummy);
    const Derivative k4 = compute_derivatives(apply(s, k3, dt),       v, cmd, env, t_dummy);
    return apply(s, weighted(k1, k2, k3, k4), dt);
}

} // namespace bathy::physics
