#pragma once

#include "geo/coords.hpp"
#include "physics/currents.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace bathy::physics {

// RasterCurrentField samples ocean currents from a preprocessed binary grid
// (.curr file, see physics/raster_grid.hpp). At query time it trilinearly
// interpolates the u/v fields in (lat, lon, depth) and returns a
// world-frame velocity vector (east, north, vertical=0) in m/s.
//
// Cells masked as nodata cause the trilinear corner contribution to be
// skipped; if all corners are nodata, or the query is outside the grid
// extent, the optional `fallback` CurrentField is consulted. The vertical
// component is always zero in this implementation; the underlying data
// products (HYCOM, OSCAR) provide only horizontal components.
class RasterCurrentField : public CurrentField {
public:
    explicit RasterCurrentField(const CurrentField* fallback = nullptr);

    bool load_file(const std::string& path);

    bool loaded() const { return loaded_; }
    std::size_t width() const { return width_; }
    std::size_t height() const { return height_; }
    std::size_t levels() const { return depths_.size(); }
    double lat_min() const { return lat_min_; }
    double lat_max() const { return lat_max_; }
    double lon_min() const { return lon_min_; }
    double lon_max() const { return lon_max_; }
    double min_depth() const { return depths_.empty() ? 0.0 : depths_.front(); }
    double max_depth() const { return depths_.empty() ? 0.0 : depths_.back(); }

    // CurrentField interface.
    Eigen::Vector3d velocity_at(geo::LatLon ll, double depth_m) const override;

private:
    // Bracket depth `d` between depths_[k] and depths_[k+1]. Sets `t` to the
    // interpolation fraction in [0, 1]. Returns false if depths_ is empty or
    // d is below depths_.front()/above depths_.back() with no extrapolation
    // allowed (we clamp instead: depths shallower than the shallowest level
    // use the surface; depths deeper than the deepest level use the bottom
    // and the caller is told via `clamp_out`).
    bool bracket_depth(double d, std::size_t& k_out, double& t_out,
                       bool& clamped_out) const;

    const CurrentField* fallback_ = nullptr;
    bool loaded_ = false;
    std::size_t width_ = 0;
    std::size_t height_ = 0;
    double lat_min_ = 0.0, lat_max_ = 0.0;
    double lon_min_ = 0.0, lon_max_ = 0.0;
    float nodata_ = 0.0f;
    std::vector<float> depths_;   // n_levels, monotonically increasing
    std::vector<float> u_;        // size = n_levels * width * height (level-major)
    std::vector<float> v_;
};

} // namespace bathy::physics
