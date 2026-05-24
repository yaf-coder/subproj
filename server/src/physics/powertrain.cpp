#include "physics/powertrain.hpp"

#include <algorithm>
#include <cmath>

namespace swordfish::physics {

MotorOutput motor_step(const MotorParams& m, double power_mech_W, double load) {
    load = std::clamp(std::fabs(load), 0.0, 1.0);
    const double lp = m.load_peak;
    const double dev = load - lp;
    const double alpha = (dev < 0.0) ? m.alpha_lo : m.alpha_hi;
    const double denom = (dev < 0.0) ? (lp * lp) : ((1.0 - lp) * (1.0 - lp));

    double eta = m.eta_peak * (1.0 - alpha * (dev * dev) / std::max(denom, 1e-9));
    eta = std::clamp(eta, 0.05, m.eta_peak);

    const double p_elec_motor = (load > 1e-4) ? (power_mech_W / eta) : 0.0;
    return MotorOutput{power_mech_W, p_elec_motor + m.idle_power_W, eta};
}

BatteryReading battery_step(const BatteryParams& b, double power_demand_W, double soc, double T) {
    soc = std::clamp(soc, 0.0, 1.0);

    // Piecewise-linear open-circuit voltage curve.
    const double v_oc = b.empty_voltage_V + (b.full_voltage_V - b.empty_voltage_V) * soc;

    // Solve P = V*I, V = v_oc - I*R  =>  I*R - I*v_oc + P = 0  (quadratic in I)
    //   -R*I^2 + v_oc*I - P = 0
    //   I = (v_oc - sqrt(v_oc^2 - 4*R*P)) / (2*R)
    const double R = std::max(b.internal_resistance_ohm, 1e-6);
    const double disc = v_oc * v_oc - 4.0 * R * power_demand_W;
    double I = 0.0;
    if (power_demand_W <= 0.0) {
        I = 0.0;
    } else if (disc <= 0.0) {
        // Demand exceeds maximum deliverable power; clamp at short-circuit-ish.
        I = v_oc / (2.0 * R);
    } else {
        I = (v_oc - std::sqrt(disc)) / (2.0 * R);
    }
    const double v_term = std::max(0.0, v_oc - I * R);

    // Peukert: effective capacity decreases with current.
    // Q_eff(I) = Q_nom * (I_ref / max(I, I_ref))^(k - 1)
    // dSoC/dt = -I / (3600 * Q_eff)
    const double ratio = b.I_ref_A / std::max(I, b.I_ref_A);
    const double q_eff_Ah = b.capacity_Ah * std::pow(ratio, b.peukert_k - 1.0);
    const double dsoc = (I > 0.0) ? -I / (3600.0 * std::max(q_eff_Ah, 1e-3)) : 0.0;

    // Thermal: resistive heating - conduction to surrounding water.
    const double q_gen = I * I * R;
    const double q_loss = b.thermal_conductance_W_per_K * (T - b.T_water_K);
    const double dT = (q_gen - q_loss) / std::max(b.thermal_mass_J_per_K, 1.0);

    return BatteryReading{v_term, I, dsoc, dT};
}

} // namespace swordfish::physics
