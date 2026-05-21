#pragma once

#include "physics/dynamics.hpp"
#include "physics/state.hpp"

namespace bathy::physics {

// Classic RK4 step. Re-evaluates dynamics 4 times with the same control input
// and environment (zero-order-hold over the step).
State rk4_step(
    const State& s,
    double dt,
    const VehicleParams& v,
    const ThrusterCommands& cmd,
    const Environment& env,
    DynamicsTelemetry& out_tel);

} // namespace bathy::physics
