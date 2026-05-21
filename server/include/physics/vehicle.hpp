#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace bathy::physics {

// Thruster: a force applied at a body-frame position along a body-frame axis.
// Magnitude is determined by a commanded normalized thrust in [-1, 1] times max_thrust_N.
struct Thruster {
    Eigen::Vector3d position_b;     // body frame application point (m)
    Eigen::Vector3d axis_b;         // unit vector, body frame direction of thrust
    double max_thrust_N = 50.0;     // peak thrust at full command
    double design_rpm = 2000.0;     // RPM at max thrust (for motor curve)
};

// Brushless DC motor with a quadratic efficiency model:
//   efficiency = eta_peak * (1 - alpha * (1 - load/load_peak)^2)
// load is |normalized command| in [0, 1]. This gives a smooth peak around the design point
// and falls off both at low load (mostly no-load losses) and high load (I^2R losses).
struct MotorParams {
    double eta_peak = 0.85;
    double load_peak = 0.75;       // command level at peak efficiency
    double alpha_lo = 1.2;         // shape low side
    double alpha_hi = 0.6;         // shape high side
    double idle_power_W = 2.0;     // electronics + no-load draw
};

// Li-ion battery pack.
// Open-circuit voltage modeled as a simple piecewise function of SoC.
// Effective capacity uses Peukert: usable_Ah(I) = nominal_Ah * (I_ref / I)^(k-1).
// Internal resistance gives a voltage sag and resistive heating.
struct BatteryParams {
    double nominal_voltage_V = 25.2;   // 7s Li-ion nominal
    double full_voltage_V = 29.4;
    double empty_voltage_V = 21.0;
    double capacity_Ah = 40.0;
    double peukert_k = 1.08;
    double internal_resistance_ohm = 0.04;
    double thermal_mass_J_per_K = 8000.0;
    double thermal_conductance_W_per_K = 6.0; // to seawater @ ~T_water
    double T_water_K = 283.15;                // 10 C
    double I_ref_A = 10.0;                    // reference for Peukert
};

// Hull/hydro params. We use a Fossen-style decomposition:
//   tau_hydro = -(M_a * dv/dt) - D_linear * v - D_quad * |v| .* v
// where M_a is the added mass (diagonal in body axes for a fore-aft symmetric body).
struct HullParams {
    double mass_kg = 25.0;                            // dry mass
    double volume_m3 = 0.025;                         // displaced volume (sets buoyancy)
    Eigen::Vector3d cog_b = Eigen::Vector3d::Zero();  // center of gravity, body frame
    Eigen::Vector3d cob_b = {0.0, 0.0, 0.02};         // center of buoyancy, body frame
                                                      // (slightly above CG => self-righting)

    // Diagonal inertia tensor in body frame (kg m^2).
    Eigen::Vector3d inertia_diag = {0.15, 1.5, 1.5};

    // Added mass (diagonal: surge, sway, heave).
    Eigen::Vector3d added_mass_diag = {2.0, 25.0, 25.0};
    // Added inertia (roll, pitch, yaw).
    Eigen::Vector3d added_inertia_diag = {0.05, 1.0, 1.0};

    // Linear damping (surge, sway, heave, roll, pitch, yaw).
    Eigen::Matrix<double, 6, 1> D_lin =
        (Eigen::Matrix<double, 6, 1>() << 4.0, 30.0, 30.0, 1.0, 8.0, 8.0).finished();

    // Quadratic damping coefficients (same ordering).
    Eigen::Matrix<double, 6, 1> D_quad =
        (Eigen::Matrix<double, 6, 1>() << 8.0, 100.0, 100.0, 2.0, 30.0, 30.0).finished();
};

struct VehicleParams {
    std::string name = "ref-torpedo";
    HullParams hull;
    MotorParams motor;
    BatteryParams battery;
    std::vector<Thruster> thrusters;

    // A canonical ~25 kg torpedo-shape AUV with 4 thrusters:
    //   1. Aft surge (main propulsion)
    //   2. Bow vertical (heave + pitch)
    //   3. Stern vertical (heave + pitch)
    //   4. Stern lateral (yaw + sway)
    static inline VehicleParams reference_torpedo() {
        return torpedo(1.0, 0.10, 25.0);
    }

    // Parameterized torpedo factory.
    //
    //   length_m : nose-to-tail length (default 1.0 m)
    //   radius_m : maximum hull radius (default 0.10 m)
    //   mass_kg  : dry mass; volume is derived so the vehicle ends up
    //              ~0.2 % positively buoyant in seawater (default 25 kg)
    //
    // Derived quantities scale with hull geometry so the dynamics stay
    // self-consistent:
    //   * Volume   ~ pi*r^2*L (cylinder approximation; capsule head/tail
    //                          tapers cancel at this fidelity)
    //   * Inertia  ~ solid cylinder about the longitudinal axis,
    //                thin-rod approximation about transverse axes
    //   * Added mass / inertia scale with displaced volume
    //   * Damping coefficients scale with frontal area (longitudinal) and
    //     lateral profile area (transverse)
    //   * Thruster positions and authority scale with length / frontal area
    static inline VehicleParams torpedo(double length_m, double radius_m, double mass_kg) {
        VehicleParams v;
        v.name = "torpedo";

        // Clamp inputs so silly values can't NaN the dynamics.
        const double L = std::max(0.2, std::min(length_m, 6.0));
        const double R = std::max(0.03, std::min(radius_m, 0.5));
        const double m = std::max(2.0, std::min(mass_kg, 800.0));

        constexpr double kPiV = 3.14159265358979323846;
        const double vol_hull    = kPiV * R * R * L;
        const double vol_neutral = m / 1025.0;
        // Pick whichever volume is larger so we never sit below neutral,
        // then add 0.2 % positive buoyancy on top.
        const double vol = std::max(vol_hull, vol_neutral) * 1.002;

        v.hull.mass_kg    = m;
        v.hull.volume_m3  = vol;
        v.hull.cog_b      = {0.0, 0.0, 0.0};
        v.hull.cob_b      = {0.0, 0.0, 0.025 * R / 0.10}; // CB above CG, scales with R

        // Inertia: solid cylinder about longitudinal axis (Ix),
        // thin-rod approximation about transverse axes (Iy, Iz).
        const double Ix  = 0.5 * m * R * R;
        const double Iyz = (1.0 / 12.0) * m * L * L + 0.25 * m * R * R;
        v.hull.inertia_diag = {Ix, Iyz, Iyz};

        // Added mass for a slender cigar-shape body:
        //   surge ~ 0.1 m_disp (low because the body is fore-aft slender)
        //   sway/heave ~ m_disp (transverse motion drags lots of water)
        const double m_disp = 1025.0 * vol;
        v.hull.added_mass_diag    = {0.10 * m_disp, m_disp, m_disp};
        v.hull.added_inertia_diag = {0.02 * m_disp * R * R,
                                     0.20 * m_disp * L * L,
                                     0.20 * m_disp * L * L};

        // Damping coefficients. Linear ~ wetted area, quadratic ~ projected
        // area. Tuned so the reference (1 m, 0.10 m, 25 kg) torpedo matches
        // the original hand-tuned defaults.
        const double A_front = kPiV * R * R;
        const double A_side  = 2.0 * R * L;
        const double k_lin   = 800.0;
        const double k_quad  = 600.0;
        v.hull.D_lin  <<  k_lin  * A_front,
                          k_lin  * A_side,
                          k_lin  * A_side,
                          0.1 * k_lin  * A_front * R * R,
                          0.4 * k_lin  * A_side  * L * L,
                          0.4 * k_lin  * A_side  * L * L;
        v.hull.D_quad <<  k_quad * A_front,
                          k_quad * A_side,
                          k_quad * A_side,
                          0.1 * k_quad * A_front * R * R,
                          0.4 * k_quad * A_side  * L * L,
                          0.4 * k_quad * A_side  * L * L;

        // Thruster positions stay at the same fraction of L as the reference.
        const double aft_x   = -0.50 * L;
        const double bow_x   =  0.35 * L;
        const double stern_x = -0.40 * L;
        const double lat_x   = -0.45 * L;

        // Thrust authority scales with frontal area. The reference vehicle
        // (R = 0.10 m) sets the unit scale.
        const double thrust_scale = std::max(0.4, A_front / (kPiV * 0.10 * 0.10));

        Thruster main_aft;
        main_aft.position_b = {aft_x, 0.0, 0.0};
        main_aft.axis_b = {1.0, 0.0, 0.0};
        main_aft.max_thrust_N = 35.0 * thrust_scale;
        v.thrusters.push_back(main_aft);

        Thruster bow_vert;
        bow_vert.position_b = {bow_x, 0.0, 0.0};
        bow_vert.axis_b = {0.0, 0.0, 1.0};
        bow_vert.max_thrust_N = 15.0 * thrust_scale;
        v.thrusters.push_back(bow_vert);

        Thruster stern_vert;
        stern_vert.position_b = {stern_x, 0.0, 0.0};
        stern_vert.axis_b = {0.0, 0.0, 1.0};
        stern_vert.max_thrust_N = 15.0 * thrust_scale;
        v.thrusters.push_back(stern_vert);

        Thruster stern_lat;
        stern_lat.position_b = {lat_x, 0.0, 0.0};
        stern_lat.axis_b = {0.0, 1.0, 0.0};
        stern_lat.max_thrust_N = 12.0 * thrust_scale;
        v.thrusters.push_back(stern_lat);

        return v;
    }
};

} // namespace bathy::physics
