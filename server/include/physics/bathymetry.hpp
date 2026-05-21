#pragma once

#include "geo/coords.hpp"
#include "geo/land_mask.hpp"

#include <memory>
#include <vector>

namespace bathy::physics {

// Bathymetry: depth of the seafloor at a given (lat, lon).
//
// All depths are positive numbers in meters below mean sea level. Land
// returns 0.0.
//
// The class is virtual so a future RasterBathymetry can replace the
// procedural one with bundled GEBCO/ETOPO data without touching the
// physics or planner.
class Bathymetry {
public:
    virtual ~Bathymetry() = default;
    virtual double depth_at(geo::LatLon ll) const = 0;
};

// Procedural bathymetry that uses a LandMask to derive plausible depths:
//   * Pre-computes a global distance-to-coast grid (0.5-degree resolution)
//     once at construction by doing a BFS outward from land cells.
//   * At query time, bilinearly interpolates distance-to-coast and maps it
//     through a continental-shelf curve:
//       0      m offshore -> 0      m depth (land)
//       0..50  km          -> 0..200 m  (shelf)
//       50..150 km         -> 200..3000 m  (continental slope)
//       150+   km          -> 3000..5000 m  (abyssal plain)
//   * Adds low-frequency procedural variation so the seafloor isn't a
//     featureless slope.
class ProceduralBathymetry : public Bathymetry {
public:
    // Builds the distance-to-coast grid from the provided LandMask. The mask
    // must remain alive for the lifetime of this object (we keep a pointer).
    explicit ProceduralBathymetry(const geo::LandMask& land);

    double depth_at(geo::LatLon ll) const override;

private:
    // Grid layout: kIxLon x kIxLat cells at half-degree resolution.
    static constexpr int kIxLon = 720;
    static constexpr int kIxLat = 360;
    static constexpr double kCellDeg = 0.5;
    static constexpr double kEarthR = 6371000.0;

    // Distance from each cell center to nearest land cell (meters).
    // Land cells store 0.0.
    std::vector<float> dist_m_;

    static int idx(int i, int j) { return j * kIxLon + i; }
    double dist_at(double lat_deg, double lon_deg) const;
    static double shelf_curve_m(double dist_m);
};

} // namespace bathy::physics
