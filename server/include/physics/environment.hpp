#pragma once

#include <Eigen/Dense>

namespace swordfish::physics {

// Environment is intentionally minimal for M1. M2 adds bathymetry sampling and currents.
struct Environment {
    double rho_water = 1025.0;            // seawater density kg/m^3
    double g = 9.81;                       // gravity m/s^2
    double sea_floor_depth_m = 200.0;     // flat for M1
    double surface_depth_m = 0.0;
    Eigen::Vector3d current_w = Eigen::Vector3d::Zero(); // world-frame current m/s
};

} // namespace swordfish::physics
