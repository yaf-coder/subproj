#include "geo/coords.hpp"

#include <cmath>

namespace swordfish::geo {

namespace {
constexpr double kEarthR = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
inline double d2r(double d) { return d * kPi / 180.0; }
} // namespace

LocalFrame::LocalFrame(LatLon origin) : origin_(origin) {
    const double lat_r = d2r(origin.lat_deg);
    m_per_deg_lat_ = kPi * kEarthR / 180.0;            // ~111195 m
    m_per_deg_lon_ = m_per_deg_lat_ * std::cos(lat_r); // shrinks with latitude
}

Eigen::Vector2d LocalFrame::to_enu(LatLon p) const {
    const double east_m = (p.lon_deg - origin_.lon_deg) * m_per_deg_lon_;
    const double north_m = (p.lat_deg - origin_.lat_deg) * m_per_deg_lat_;
    return {east_m, north_m};
}

LatLon LocalFrame::to_latlon(Eigen::Vector2d enu) const {
    LatLon p;
    p.lon_deg = origin_.lon_deg + enu.x() / m_per_deg_lon_;
    p.lat_deg = origin_.lat_deg + enu.y() / m_per_deg_lat_;
    return p;
}

double great_circle_m(LatLon a, LatLon b) {
    const double phi1 = d2r(a.lat_deg);
    const double phi2 = d2r(b.lat_deg);
    const double dphi = d2r(b.lat_deg - a.lat_deg);
    const double dlam = d2r(b.lon_deg - a.lon_deg);
    const double s = std::sin(dphi * 0.5);
    const double c = std::sin(dlam * 0.5);
    const double h = s * s + std::cos(phi1) * std::cos(phi2) * c * c;
    return 2.0 * kEarthR * std::asin(std::sqrt(std::clamp(h, 0.0, 1.0)));
}

} // namespace swordfish::geo
