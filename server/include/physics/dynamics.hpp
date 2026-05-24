#pragma once

#include "physics/environment.hpp"
#include "physics/state.hpp"
#include "physics/vehicle.hpp"

#include <vector>

namespace swordfish::physics {

// Per-thruster commands in [-1, 1]. Size must match vehicle.thrusters.size().
using ThrusterCommands = std::vector<double>;

// Telemetry computed during dynamics evaluation; surfaced for logging/UI.
struct DynamicsTelemetry {
    double speed_m_s = 0.0;
    double thrust_total_N = 0.0;
    double drag_total_N = 0.0;
    double power_mech_W = 0.0;
    double power_elec_W = 0.0;
    double battery_current_A = 0.0;
    double battery_voltage_V = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    double yaw_deg = 0.0;
    double depth_m = 0.0;
    bool grounded = false;
};

// Compute the time derivative of the state and the instantaneous telemetry.
// The function is pure: no global state, no allocations beyond the input vectors.
Derivative compute_derivatives(
    const State& s,
    const VehicleParams& v,
    const ThrusterCommands& cmd,
    const Environment& env,
    DynamicsTelemetry& out_tel);

} // namespace swordfish::physics
