#pragma once

#include "geo/coords.hpp"

#include <Eigen/Dense>
#include <vector>

namespace bathy::planner {

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
};

// M1 planner: dive, great-circle interpolation at cruise depth, surface.
// Energy estimate uses a simple steady-state power model:
//   P_cruise ~ k_drag * v^3 + idle
Plan plan_mission(const MissionRequest& req);

} // namespace bathy::planner
