#pragma once

#include "geo/coords.hpp"
#include "physics/bathymetry.hpp"

#include <memory>
#include <string>
#include <vector>

namespace bathy::physics {

// RasterBathymetry samples bathymetry from a preprocessed binary grid
// (.bath file, see physics/raster_grid.hpp). At query time it bilinearly
// interpolates the underlying elevation field and returns positive depth
// in meters below mean sea level. Cells masked as nodata, or queries
// outside the grid extent, defer to an optional `fallback` Bathymetry.
//
// The class is thread-safe after load_file() returns; the underlying
// elevation array is never mutated.
class RasterBathymetry : public Bathymetry {
public:
    // `fallback` may be null. If non-null, it is queried for points outside
    // the grid bounding box or where the four interpolation corners are all
    // nodata. The fallback is *not* owned.
    explicit RasterBathymetry(const Bathymetry* fallback = nullptr);

    // Loads a .bath file from disk. Returns false on missing file, bad magic,
    // version mismatch, or any other validation failure. On failure the
    // object remains in an unloaded state and queries delegate entirely to
    // the fallback (or return 0 if no fallback).
    bool load_file(const std::string& path);

    bool loaded() const { return loaded_; }
    std::size_t width() const { return width_; }
    std::size_t height() const { return height_; }
    double lat_min() const { return lat_min_; }
    double lat_max() const { return lat_max_; }
    double lon_min() const { return lon_min_; }
    double lon_max() const { return lon_max_; }

    // Bathymetry interface: returns positive meters below sea level at the
    // query point. Land (positive elevation in the raster) returns 0.
    double depth_at(geo::LatLon ll) const override;

private:
    // Returns true if (lat, lon) maps to a valid interpolation cell within
    // the raster extent. Writes the interpolated *elevation* (negative =
    // below sea level) into `elev_out`.
    bool sample_elevation(double lat, double lon, double& elev_out) const;

    const Bathymetry* fallback_ = nullptr;
    bool loaded_ = false;
    std::size_t width_ = 0;
    std::size_t height_ = 0;
    double lat_min_ = 0.0, lat_max_ = 0.0;
    double lon_min_ = 0.0, lon_max_ = 0.0;
    float nodata_ = 0.0f;
    // elev_[j * width_ + i] = elevation at cell (i, j). Stored as float so a
    // global 1440 x 720 grid is ~4 MB.
    std::vector<float> elev_;
};

} // namespace bathy::physics
