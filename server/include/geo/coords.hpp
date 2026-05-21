#pragma once

#include <Eigen/Dense>

namespace bathy::geo {

struct LatLon {
    double lat_deg;
    double lon_deg;
};

// Local tangent plane (ENU) projection at a chosen origin.
// All operations use a spherical-earth approximation (R = 6371 km).
// Accuracy is more than adequate for mission scales of < ~50 km.
class LocalFrame {
public:
    explicit LocalFrame(LatLon origin);

    // Convert a (lat, lon) point to ENU offset from origin (meters).
    // ENU: x = East, y = North, z = Up (callers set z separately, typically 0 at surface).
    Eigen::Vector2d to_enu(LatLon p) const;

    // Convert ENU offset back to (lat, lon).
    LatLon to_latlon(Eigen::Vector2d enu) const;

    LatLon origin() const { return origin_; }

private:
    LatLon origin_;
    double m_per_deg_lat_;
    double m_per_deg_lon_;
};

// Great-circle distance in meters (haversine).
double great_circle_m(LatLon a, LatLon b);

} // namespace bathy::geo
