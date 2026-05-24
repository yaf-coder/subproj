#include "physics/raster_bathymetry.hpp"

#include "physics/raster_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

namespace bathy::physics {

namespace {
inline bool is_nodata(float v, float sentinel) {
    return std::isnan(v) || v == sentinel;
}
} // namespace

RasterBathymetry::RasterBathymetry(const Bathymetry* fallback) : fallback_(fallback) {}

bool RasterBathymetry::load_file(const std::string& path) {
    loaded_ = false;
    elev_.clear();

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::cerr << "RasterBathymetry: could not open " << path << "\n";
        return false;
    }

    BathyHeader h{};
    in.read(reinterpret_cast<char*>(&h), sizeof(h));
    if (!in || in.gcount() != static_cast<std::streamsize>(sizeof(h))) {
        std::cerr << "RasterBathymetry: short read on header from " << path << "\n";
        return false;
    }
    if (h.magic != kBathyMagic) {
        std::cerr << "RasterBathymetry: bad magic 0x" << std::hex << h.magic
                  << " in " << path << " (expected 0x" << kBathyMagic
                  << ")" << std::dec << "\n";
        return false;
    }
    if (h.version != kFormatVersion) {
        std::cerr << "RasterBathymetry: unsupported version " << h.version
                  << " in " << path << " (expected " << kFormatVersion << ")\n";
        return false;
    }
    if (h.width < 2 || h.height < 2 || h.width > 100000 || h.height > 100000) {
        std::cerr << "RasterBathymetry: implausible dimensions "
                  << h.width << "x" << h.height << "\n";
        return false;
    }
    if (!(h.lat_min < h.lat_max) || !(h.lon_min < h.lon_max)) {
        std::cerr << "RasterBathymetry: degenerate bbox in " << path << "\n";
        return false;
    }
    if (!(h.lat_min >= -90.0 && h.lat_max <= 90.0)) {
        std::cerr << "RasterBathymetry: latitude out of range in " << path << "\n";
        return false;
    }

    const std::size_t n = static_cast<std::size_t>(h.width) * static_cast<std::size_t>(h.height);
    elev_.resize(n);
    in.read(reinterpret_cast<char*>(elev_.data()),
            static_cast<std::streamsize>(n * sizeof(float)));
    if (!in) {
        std::cerr << "RasterBathymetry: short read on body ("
                  << in.gcount() << " of " << n * sizeof(float) << " bytes)\n";
        elev_.clear();
        return false;
    }

    width_ = h.width;
    height_ = h.height;
    lat_min_ = h.lat_min;
    lat_max_ = h.lat_max;
    lon_min_ = h.lon_min;
    lon_max_ = h.lon_max;
    nodata_ = h.nodata_value;
    loaded_ = true;

    // Diagnostic stats: how much of the grid is land vs ocean?
    std::size_t land_cells = 0, ocean_cells = 0, nodata_cells = 0;
    double min_elev = 1e30, max_elev = -1e30;
    for (float v : elev_) {
        if (is_nodata(v, nodata_)) { ++nodata_cells; continue; }
        if (v > 0) ++land_cells; else ++ocean_cells;
        if (v < min_elev) min_elev = v;
        if (v > max_elev) max_elev = v;
    }
    std::cerr << "RasterBathymetry: loaded " << width_ << "x" << height_
              << " cells from " << path
              << " (bbox lat [" << lat_min_ << "," << lat_max_
              << "] lon [" << lon_min_ << "," << lon_max_ << "]; "
              << "land=" << land_cells << " ocean=" << ocean_cells
              << " nodata=" << nodata_cells
              << "; elev range [" << min_elev << ", " << max_elev << "] m)\n";
    return true;
}

bool RasterBathymetry::sample_elevation(double lat, double lon, double& elev_out) const {
    if (!loaded_) return false;

    // Reject queries outside the grid extent so the fallback runs there.
    if (lat < lat_min_ || lat > lat_max_) return false;
    // Allow longitude wrap if the raster spans the whole globe (or close to it):
    // bring lon into [lon_min_, lon_min_ + 360).
    double lonq = lon;
    const double lon_span = lon_max_ - lon_min_;
    if (lon_span >= 359.0) {
        while (lonq < lon_min_) lonq += 360.0;
        while (lonq >= lon_min_ + 360.0) lonq -= 360.0;
    } else if (lonq < lon_min_ || lonq > lon_max_) {
        return false;
    }

    // Fractional cell coordinates (cell-center convention).
    const double fx = (lonq - lon_min_) / (lon_max_ - lon_min_) * (width_ - 1);
    const double fy = (lat - lat_min_)  / (lat_max_ - lat_min_) * (height_ - 1);
    int i0 = static_cast<int>(std::floor(fx));
    int j0 = static_cast<int>(std::floor(fy));
    int i1 = i0 + 1;
    int j1 = j0 + 1;
    // Clamp to valid corners (handles fx == width_-1 exactly).
    if (i0 < 0) i0 = 0;
    if (j0 < 0) j0 = 0;
    if (i1 > static_cast<int>(width_) - 1) i1 = width_ - 1;
    if (j1 > static_cast<int>(height_) - 1) j1 = height_ - 1;
    if (i0 > i1) i0 = i1;
    if (j0 > j1) j0 = j1;
    const double tx = std::clamp(fx - i0, 0.0, 1.0);
    const double ty = std::clamp(fy - j0, 0.0, 1.0);

    const auto sample = [&](int i, int j) {
        return elev_[static_cast<std::size_t>(j) * width_ + static_cast<std::size_t>(i)];
    };
    const float v00 = sample(i0, j0);
    const float v10 = sample(i1, j0);
    const float v01 = sample(i0, j1);
    const float v11 = sample(i1, j1);

    // If any corner is nodata, defer to the fallback. We could be cleverer
    // (mean-of-valid-corners) but at typical grid resolutions the cliff
    // between coast and ocean is exactly where mixed corners happen, and
    // averaging them produces visibly wrong shallow shelves.
    if (is_nodata(v00, nodata_) || is_nodata(v10, nodata_) ||
        is_nodata(v01, nodata_) || is_nodata(v11, nodata_)) {
        return false;
    }

    const double e0 = static_cast<double>(v00) * (1.0 - tx) + static_cast<double>(v10) * tx;
    const double e1 = static_cast<double>(v01) * (1.0 - tx) + static_cast<double>(v11) * tx;
    elev_out = e0 * (1.0 - ty) + e1 * ty;
    return true;
}

double RasterBathymetry::depth_at(geo::LatLon ll) const {
    double elev = 0.0;
    if (sample_elevation(ll.lat_deg, ll.lon_deg, elev)) {
        // Convention: stored elevation is positive above sea level, negative
        // below. depth_at returns positive depth below sea level.
        return elev >= 0.0 ? 0.0 : -elev;
    }
    if (fallback_) return fallback_->depth_at(ll);
    return 0.0;
}

} // namespace bathy::physics
