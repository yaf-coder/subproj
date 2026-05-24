#include "physics/currents.hpp"

#include <cmath>

namespace swordfish::physics {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double d2r(double d) { return d * kPi / 180.0; }
} // namespace

Eigen::Vector3d SyntheticCurrentField::velocity_at(geo::LatLon ll, double depth_m) const {
    const double lat_r = d2r(ll.lat_deg);
    const double lat_d = ll.lat_deg;
    const double lon_d = ll.lon_deg;

    // The field is the sum of five layers spanning ~5 orders of magnitude in
    // spatial scale, so that variation is visible whether you're looking at
    // a continent or a 2 km square of ocean.

    // ---- Layer 1: continental zonal background (lat-driven only) ----
    // Kept small so it doesn't dominate the variable layers below — it
    // contributes a faint preferred direction (westward at low lat, weak
    // eastward at high lat) without making the whole field look uniform.
    double u = -0.04 * std::cos(2.0 * lat_r);
    double v =  0.02 * std::sin(2.0 * lat_r);

    // ---- Layer 1b: planetary gyres (lon AND lat dependent, ~10000 km) ----
    // Two cells around the globe so panning east-west visibly changes the
    // background, not just up-down.
    u += 0.07 * std::sin(0.035 * lon_d + 1.0) * std::cos(2.0 * lat_r);
    v += 0.06 * std::cos(0.035 * lon_d - 0.5) * std::cos(2.0 * lat_r);

    // ---- Layer 2: regional eddies (~500 km wavelength) ----
    u += 0.08 * std::sin(1.2 * lon_d + 1.1) * std::cos(1.5 * lat_d - 0.4);
    v += 0.07 * std::cos(1.4 * lon_d - 0.7) * std::sin(1.3 * lat_d + 1.9);

    // ---- Layer 3: mesoscale eddies (~80 km) ----
    // Realistic Rossby-radius-ish features. This is what dominates variation
    // along a 10 km AUV mission.
    u += 0.12 * std::sin(8.7 * lon_d + 0.3) * std::cos(7.9 * lat_d - 1.2);
    v += 0.10 * std::cos(9.1 * lon_d - 1.4) * std::sin(8.3 * lat_d + 0.6);

    // ---- Layer 4: submesoscale (~14 km) ----
    u += 0.07 * std::sin(48.0 * lon_d + 0.9) * std::cos(42.0 * lat_d - 0.5);
    v += 0.06 * std::cos(53.0 * lon_d - 0.2) * std::sin(45.0 * lat_d + 1.3);

    // ---- Layer 5: fine filaments (~3 km) ----
    // These give visible variation in close-up overlay views where the other
    // layers would look constant.
    u += 0.04 * std::sin(230.0 * lon_d + 0.6) * std::cos(210.0 * lat_d - 0.7);
    v += 0.04 * std::cos(240.0 * lon_d - 1.1) * std::sin(220.0 * lat_d + 0.4);

    // ---- Ekman-like depth decay (e-folding scale ~200 m) ----
    const double decay = std::exp(-std::max(0.0, depth_m) / 200.0);
    u *= decay;
    v *= decay;

    return {u, v, 0.0};
}

} // namespace swordfish::physics
