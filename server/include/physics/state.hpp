#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace swordfish::physics {

// World frame: ENU (East-North-Up) local tangent plane at the mission origin.
// Body frame: x-forward, y-port (left), z-up. (Standard right-handed.)
//
// State variables:
//   position p_w  : world frame position (m)
//   orientation q : unit quaternion, body->world rotation
//   linear velocity v_b  : body frame (m/s)
//   angular velocity w_b : body frame (rad/s)
//
// Auxiliary (not integrated):
//   battery state of charge soc in [0, 1]
//   battery temperature T_batt (K)

struct State {
    Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d v_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_b = Eigen::Vector3d::Zero();

    double soc = 1.0;
    double T_batt = 288.15; // 15 C, typical seawater

    // Mission-level accumulators (useful for telemetry)
    double energy_used_J = 0.0;
    double distance_traveled_m = 0.0;
    double mission_time_s = 0.0;
};

// Time derivative of the integrated portion of the state.
struct Derivative {
    Eigen::Vector3d dp_w = Eigen::Vector3d::Zero();
    Eigen::Quaterniond dq = Eigen::Quaterniond(0, 0, 0, 0); // not a unit quat - derivative
    Eigen::Vector3d dv_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d dw_b = Eigen::Vector3d::Zero();

    double dsoc = 0.0;
    double dT_batt = 0.0;
    double denergy = 0.0;
    double ddistance = 0.0;
};

} // namespace swordfish::physics
