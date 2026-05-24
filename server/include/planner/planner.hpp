#pragma once

#include "geo/coords.hpp"
#include "geo/land_mask.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace swordfish::planner {

struct Waypoint {
    geo::LatLon ll;
    double depth_m;        // positive depth, 0 = surface
    double cruise_speed_m_s;
};

struct MissionRequest {
    geo::LatLon start;
    geo::LatLon goal;
    double cruise_depth_m = 50.0;
    double cruise_speed_m_s = 1.2;
    double descent_rate_m_s = 0.4;   // vertical
    double sample_spacing_m = 30.0;  // along-track sampling for the cruise leg
};

struct Plan {
    std::vector<Waypoint> waypoints;
    double estimated_distance_m = 0.0;
    double estimated_duration_s = 0.0;
    double estimated_energy_J = 0.0;
    // Non-empty on planner failure (e.g. start/goal on land, no path found).
    std::string error;
    // True if A* routing around land was used.
    bool routed_around_land = false;
};

// Plan a dive -> cruise -> surface mission.
//
// If `land` is non-null and loaded:
//   * Returns an error plan if start or goal lies on land.
//   * If the great-circle between start and goal crosses land, an A* search
//     over a local ENU grid is run with land cells blocked, then line-of-sight
//     smoothed and re-sampled at `sample_spacing_m`.
//   * Otherwise falls back to plain great-circle interpolation.
//
// Energy estimate uses a simple steady-state power model:
//   P_cruise ~ k_drag * v^3 + idle
Plan plan_mission(const MissionRequest& req, const geo::LandMask* land = nullptr);

} // namespace swordfish::planner
