#pragma once

#include "physics/vehicle.hpp"

namespace bathy::physics {

struct MotorOutput {
    double power_mech_W;
    double power_elec_W;
    double efficiency;
};

// Compute electrical power required from mechanical shaft power.
// load is |command| in [0,1]; sign of command does not affect efficiency.
MotorOutput motor_step(const MotorParams& m, double power_mech_W, double load);

struct BatteryReading {
    double voltage_V;       // sagged terminal voltage
    double current_A;
    double dsoc_per_s;      // negative when discharging
    double dT_per_s;        // K/s
};

// Given electrical power demand, compute the current the pack must source,
// the voltage sag, and the rate of change of SoC and temperature.
BatteryReading battery_step(const BatteryParams& b, double power_demand_W, double soc, double T);

} // namespace bathy::physics
