#include "planner/planner.hpp"

#include <algorithm>
#include <cmath>

namespace bathy::planner {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double d2r(double d) { return d * kPi / 180.0; }
inline double r2d(double r) { return r * 180.0 / kPi; }

// Spherical-earth great-circle interpolation. fraction in [0, 1].
geo::LatLon slerp_latlon(geo::LatLon a, geo::LatLon b, double f) {
    const double phi1 = d2r(a.lat_deg), lam1 = d2r(a.lon_deg);
    const double phi2 = d2r(b.lat_deg), lam2 = d2r(b.lon_deg);
    const double d = 2.0 * std::asin(std::sqrt(
        std::pow(std::sin((phi2 - phi1) / 2.0), 2.0) +
        std::cos(phi1) * std::cos(phi2) *
        std::pow(std::sin((lam2 - lam1) / 2.0), 2.0)));
    if (d < 1e-9) return a;
    const double A = std::sin((1.0 - f) * d) / std::sin(d);
    const double B = std::sin(f * d) / std::sin(d);
    const double x = A * std::cos(phi1) * std::cos(lam1) + B * std::cos(phi2) * std::cos(lam2);
    const double y = A * std::cos(phi1) * std::sin(lam1) + B * std::cos(phi2) * std::sin(lam2);
    const double z = A * std::sin(phi1) + B * std::sin(phi2);
    const double phi = std::atan2(z, std::sqrt(x * x + y * y));
    const double lam = std::atan2(y, x);
    return {r2d(phi), r2d(lam)};
}
} // namespace

Plan plan_mission(const MissionRequest& req) {
    Plan p;
    const double horiz_m = geo::great_circle_m(req.start, req.goal);

    // 1) Dive at start position.
    p.waypoints.push_back({req.start, 0.0, req.descent_rate_m_s});
    p.waypoints.push_back({req.start, req.cruise_depth_m, req.descent_rate_m_s});

    // 2) Cruise: discrete samples along the great-circle.
    const int n = std::max(1, static_cast<int>(std::ceil(horiz_m / req.sample_spacing_m)));
    for (int i = 1; i <= n; ++i) {
        const double f = static_cast<double>(i) / n;
        p.waypoints.push_back({slerp_latlon(req.start, req.goal, f),
                               req.cruise_depth_m, req.cruise_speed_m_s});
    }

    // 3) Surface at goal.
    p.waypoints.push_back({req.goal, 0.0, req.descent_rate_m_s});

    // Distance: vertical legs + horizontal.
    const double dist = horiz_m + 2.0 * req.cruise_depth_m;
    p.estimated_distance_m = dist;

    // Duration: cruise time + dive/surface time.
    const double t_cruise = horiz_m / std::max(req.cruise_speed_m_s, 1e-3);
    const double t_vert = 2.0 * req.cruise_depth_m / std::max(req.descent_rate_m_s, 1e-3);
    p.estimated_duration_s = t_cruise + t_vert;

    // Energy: rough steady-state P ~ k*v^3 + idle. k chosen so that ~1 m/s ~ 25 W shaft.
    const double k = 25.0;       // W / (m/s)^3
    const double idle = 8.0;     // W
    const double eta = 0.7;      // overall electrical efficiency
    const double P_cruise = (k * std::pow(req.cruise_speed_m_s, 3.0) + idle) / eta;
    const double P_vert = (k * std::pow(req.descent_rate_m_s, 3.0) + idle) / eta;
    p.estimated_energy_J = P_cruise * t_cruise + P_vert * t_vert;

    return p;
}

} // namespace bathy::planner
