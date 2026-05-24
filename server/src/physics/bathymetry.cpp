#include "physics/bathymetry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace swordfish::physics {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double d2r(double d) { return d * kPi / 180.0; }
inline int clampi(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }
} // namespace

ProceduralBathymetry::ProceduralBathymetry(const geo::LandMask& land)
    : dist_m_(static_cast<std::size_t>(kIxLon) * static_cast<std::size_t>(kIxLat), 0.0f) {
    const auto t0 = std::chrono::steady_clock::now();

    // Step 1: classify each cell center as land (dist = 0) or water (dist = +inf).
    const float kInf = std::numeric_limits<float>::max();
    for (int j = 0; j < kIxLat; ++j) {
        const double lat = -90.0 + (j + 0.5) * kCellDeg;
        for (int i = 0; i < kIxLon; ++i) {
            const double lon = -180.0 + (i + 0.5) * kCellDeg;
            dist_m_[idx(i, j)] = land.is_land(lat, lon) ? 0.0f : kInf;
        }
    }

    // Step 2: chamfer distance transform (Borgefors 3-4 chamfer, scaled),
    // two passes. We work in *cells* and convert to meters at query time.
    // Distance approximations:
    //   straight neighbor:   1.0
    //   diagonal neighbor:   sqrt(2)  (~1.4142)
    // For better accuracy near the poles we'd want a geodesic transform,
    // but the half-degree cell is coarse enough that this is fine.
    const float kStraight = 1.0f;
    const float kDiag = 1.41421356f;

    // Forward pass: visit cells in raster order, propagate from upper / left.
    for (int j = 0; j < kIxLat; ++j) {
        for (int i = 0; i < kIxLon; ++i) {
            float d = dist_m_[idx(i, j)];
            if (d == 0.0f) continue;
            // Longitude wraps; latitude clamps.
            const int wL = (i - 1 + kIxLon) % kIxLon;
            const int wR = (i + 1) % kIxLon;
            if (j > 0) {
                d = std::min(d, dist_m_[idx(wL, j - 1)] + kDiag);
                d = std::min(d, dist_m_[idx(i,  j - 1)] + kStraight);
                d = std::min(d, dist_m_[idx(wR, j - 1)] + kDiag);
            }
            d = std::min(d, dist_m_[idx(wL, j)] + kStraight);
            dist_m_[idx(i, j)] = d;
        }
    }
    // Backward pass: reverse raster order.
    for (int j = kIxLat - 1; j >= 0; --j) {
        for (int i = kIxLon - 1; i >= 0; --i) {
            float d = dist_m_[idx(i, j)];
            if (d == 0.0f) continue;
            const int wL = (i - 1 + kIxLon) % kIxLon;
            const int wR = (i + 1) % kIxLon;
            d = std::min(d, dist_m_[idx(wR, j)] + kStraight);
            if (j + 1 < kIxLat) {
                d = std::min(d, dist_m_[idx(wL, j + 1)] + kDiag);
                d = std::min(d, dist_m_[idx(i,  j + 1)] + kStraight);
                d = std::min(d, dist_m_[idx(wR, j + 1)] + kDiag);
            }
            dist_m_[idx(i, j)] = d;
        }
    }

    // Step 3: convert cell counts to meters. At latitude phi the cell is
    // kCellDeg degrees, but along longitude it shrinks by cos(phi). We use
    // the local cell-width in meters as the conversion factor — biased
    // slightly long for diagonal motion but well within the noise of a
    // half-degree procedural model.
    const double m_per_cell_lat = kCellDeg * kPi * kEarthR / 180.0; // ~55.6 km
    for (int j = 0; j < kIxLat; ++j) {
        const double lat = -90.0 + (j + 0.5) * kCellDeg;
        const double m_per_cell_lon = m_per_cell_lat * std::cos(d2r(lat));
        const double m_per_cell = std::min(m_per_cell_lat, m_per_cell_lon);
        for (int i = 0; i < kIxLon; ++i) {
            float& d = dist_m_[idx(i, j)];
            if (d != std::numeric_limits<float>::max()) {
                d = static_cast<float>(d * m_per_cell);
            }
        }
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    std::cerr << "ProceduralBathymetry: built "
              << kIxLon << "x" << kIxLat
              << " distance-to-coast grid in " << ms << " ms\n";
}

double ProceduralBathymetry::dist_at(double lat_deg, double lon_deg) const {
    // Bilinear interpolation in cell-center space.
    double lon = lon_deg;
    while (lon < -180.0) lon += 360.0;
    while (lon >= 180.0) lon -= 360.0;

    const double fx = (lon + 180.0) / kCellDeg - 0.5;
    const double fy = std::clamp((lat_deg + 90.0) / kCellDeg - 0.5,
                                 0.0, static_cast<double>(kIxLat - 1));
    const int i0 = ((static_cast<int>(std::floor(fx)) % kIxLon) + kIxLon) % kIxLon;
    const int j0 = clampi(static_cast<int>(std::floor(fy)), 0, kIxLat - 1);
    const int i1 = (i0 + 1) % kIxLon;
    const int j1 = std::min(j0 + 1, kIxLat - 1);
    const double tx = fx - std::floor(fx);
    const double ty = fy - std::floor(fy);

    const double d00 = dist_m_[idx(i0, j0)];
    const double d10 = dist_m_[idx(i1, j0)];
    const double d01 = dist_m_[idx(i0, j1)];
    const double d11 = dist_m_[idx(i1, j1)];
    return (d00 * (1 - tx) + d10 * tx) * (1 - ty) + (d01 * (1 - tx) + d11 * tx) * ty;
}

double ProceduralBathymetry::shelf_curve_m(double dist_m) {
    // Three regimes, smooth piecewise:
    //   0..50 km   -> 0..200 m
    //   50..150 km -> 200..3000 m (steep)
    //   150+ km    -> 3000..5000 m (asymptote)
    if (dist_m <= 0.0) return 0.0;
    if (dist_m < 50000.0) {
        return 200.0 * (dist_m / 50000.0);
    }
    if (dist_m < 150000.0) {
        const double t = (dist_m - 50000.0) / 100000.0;
        return 200.0 + (3000.0 - 200.0) * t;
    }
    // Asymptote to 5000m beyond ~600km offshore.
    const double t = std::min(1.0, (dist_m - 150000.0) / 450000.0);
    return 3000.0 + (5000.0 - 3000.0) * t;
}

double ProceduralBathymetry::depth_at(geo::LatLon ll) const {
    const double dist_m = dist_at(ll.lat_deg, ll.lon_deg);
    double depth = shelf_curve_m(dist_m);
    if (depth <= 0.0) return 0.0;

    // Procedural variation so the seafloor isn't a smooth slope.
    // Three octaves of sinusoidal noise in lat/lon, amplitude tapered down
    // near shore so we don't accidentally produce above-water features.
    const double lat_r = d2r(ll.lat_deg);
    const double lon_r = d2r(ll.lon_deg);
    double n = 0.0;
    n += std::sin(8.0 * lat_r) * std::cos(8.0 * lon_r);
    n += 0.5 * std::sin(20.0 * lat_r + 1.1) * std::cos(17.0 * lon_r - 0.4);
    n += 0.25 * std::sin(53.0 * lat_r - 0.7) * std::cos(47.0 * lon_r + 2.3);

    const double taper = std::min(1.0, dist_m / 20000.0); // 0 near shore, 1 by 20 km offshore
    const double amp_m = 200.0 * taper;                   // ±200 m offshore
    depth += amp_m * n;
    if (depth < 1.0) depth = 1.0;
    return depth;
}

} // namespace swordfish::physics
