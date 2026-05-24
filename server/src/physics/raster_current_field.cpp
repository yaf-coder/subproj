#include "physics/raster_current_field.hpp"

#include "physics/raster_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

namespace swordfish::physics {

namespace {
inline bool is_nodata(float v, float sentinel) {
    return std::isnan(v) || v == sentinel;
}
} // namespace

RasterCurrentField::RasterCurrentField(const CurrentField* fallback)
    : fallback_(fallback) {}

bool RasterCurrentField::load_file(const std::string& path) {
    loaded_ = false;
    depths_.clear();
    u_.clear();
    v_.clear();

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::cerr << "RasterCurrentField: could not open " << path << "\n";
        return false;
    }

    CurrentsHeader h{};
    in.read(reinterpret_cast<char*>(&h), sizeof(h));
    if (!in || in.gcount() != static_cast<std::streamsize>(sizeof(h))) {
        std::cerr << "RasterCurrentField: short read on header from " << path << "\n";
        return false;
    }
    if (h.magic != kCurrentMagic) {
        std::cerr << "RasterCurrentField: bad magic 0x" << std::hex << h.magic
                  << " (expected 0x" << kCurrentMagic << ")" << std::dec << "\n";
        return false;
    }
    if (h.version != kFormatVersion) {
        std::cerr << "RasterCurrentField: unsupported version " << h.version << "\n";
        return false;
    }
    if (h.width < 2 || h.height < 2 || h.n_levels < 1 ||
        h.width > 100000 || h.height > 100000 || h.n_levels > 200) {
        std::cerr << "RasterCurrentField: implausible dims "
                  << h.width << "x" << h.height << "x" << h.n_levels << "\n";
        return false;
    }
    if (!(h.lat_min < h.lat_max) || !(h.lon_min < h.lon_max)) {
        std::cerr << "RasterCurrentField: degenerate bbox\n";
        return false;
    }
    if (!(h.lat_min >= -90.0 && h.lat_max <= 90.0)) {
        std::cerr << "RasterCurrentField: latitude out of range\n";
        return false;
    }

    depths_.resize(h.n_levels);
    in.read(reinterpret_cast<char*>(depths_.data()),
            static_cast<std::streamsize>(h.n_levels * sizeof(float)));
    if (!in) {
        std::cerr << "RasterCurrentField: short read on depth axis\n";
        return false;
    }
    for (std::size_t k = 1; k < depths_.size(); ++k) {
        if (!(depths_[k] >= depths_[k - 1])) {
            std::cerr << "RasterCurrentField: depth axis not monotonically non-decreasing "
                      << "(depths[" << k - 1 << "]=" << depths_[k - 1]
                      << " depths[" << k << "]=" << depths_[k] << ")\n";
            return false;
        }
    }

    const std::size_t cells = static_cast<std::size_t>(h.width)
                            * static_cast<std::size_t>(h.height);
    const std::size_t total = cells * h.n_levels;
    u_.resize(total);
    v_.resize(total);
    in.read(reinterpret_cast<char*>(u_.data()),
            static_cast<std::streamsize>(total * sizeof(float)));
    if (!in) {
        std::cerr << "RasterCurrentField: short read on u block\n";
        return false;
    }
    in.read(reinterpret_cast<char*>(v_.data()),
            static_cast<std::streamsize>(total * sizeof(float)));
    if (!in) {
        std::cerr << "RasterCurrentField: short read on v block\n";
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

    // Diagnostic stats: how many cells are valid at the surface? Range of |c|?
    std::size_t valid = 0, masked = 0;
    double max_mag = 0.0, sum_mag = 0.0;
    for (std::size_t idx = 0; idx < cells; ++idx) {
        const float ui = u_[idx];
        const float vi = v_[idx];
        if (is_nodata(ui, nodata_) || is_nodata(vi, nodata_)) {
            ++masked;
        } else {
            const double m = std::hypot(static_cast<double>(ui), static_cast<double>(vi));
            sum_mag += m;
            if (m > max_mag) max_mag = m;
            ++valid;
        }
    }
    const double mean_mag = valid ? sum_mag / valid : 0.0;
    std::cerr << "RasterCurrentField: loaded " << width_ << "x" << height_
              << "x" << depths_.size() << " from " << path
              << " (bbox lat [" << lat_min_ << "," << lat_max_
              << "] lon [" << lon_min_ << "," << lon_max_ << "]; "
              << "depth [" << depths_.front() << "," << depths_.back() << "] m; "
              << "surface valid=" << valid << " masked=" << masked
              << "; |c|_max=" << max_mag << " |c|_mean=" << mean_mag << " m/s)\n";
    return true;
}

bool RasterCurrentField::bracket_depth(double d, std::size_t& k_out,
                                       double& t_out, bool& clamped_out) const {
    clamped_out = false;
    if (depths_.empty()) return false;
    if (depths_.size() == 1) {
        k_out = 0;
        t_out = 0.0;
        clamped_out = (d > depths_[0] + 1e-3) || (d < depths_[0] - 1e-3);
        return true;
    }
    if (d <= depths_.front()) {
        k_out = 0;
        t_out = 0.0;
        clamped_out = (d < depths_.front() - 1e-3);
        return true;
    }
    if (d >= depths_.back()) {
        k_out = depths_.size() - 2;
        t_out = 1.0;
        clamped_out = (d > depths_.back() + 1e-3);
        return true;
    }
    // Linear scan; the axis is small (≤ ~40 levels in HYCOM).
    for (std::size_t k = 0; k + 1 < depths_.size(); ++k) {
        if (depths_[k] <= d && d <= depths_[k + 1]) {
            const double span = depths_[k + 1] - depths_[k];
            t_out = span > 0 ? (d - depths_[k]) / span : 0.0;
            k_out = k;
            return true;
        }
    }
    return false;
}

Eigen::Vector3d RasterCurrentField::velocity_at(geo::LatLon ll, double depth_m) const {
    if (!loaded_) {
        if (fallback_) return fallback_->velocity_at(ll, depth_m);
        return Eigen::Vector3d::Zero();
    }

    const double lat = ll.lat_deg;
    double lon = ll.lon_deg;

    if (lat < lat_min_ || lat > lat_max_) {
        if (fallback_) return fallback_->velocity_at(ll, depth_m);
        return Eigen::Vector3d::Zero();
    }
    const double lon_span = lon_max_ - lon_min_;
    if (lon_span >= 359.0) {
        while (lon < lon_min_) lon += 360.0;
        while (lon >= lon_min_ + 360.0) lon -= 360.0;
    } else if (lon < lon_min_ || lon > lon_max_) {
        if (fallback_) return fallback_->velocity_at(ll, depth_m);
        return Eigen::Vector3d::Zero();
    }

    const double fx = (lon - lon_min_) / (lon_max_ - lon_min_) * (width_ - 1);
    const double fy = (lat - lat_min_) / (lat_max_ - lat_min_) * (height_ - 1);
    int i0 = static_cast<int>(std::floor(fx));
    int j0 = static_cast<int>(std::floor(fy));
    int i1 = i0 + 1;
    int j1 = j0 + 1;
    if (i0 < 0) i0 = 0;
    if (j0 < 0) j0 = 0;
    if (i1 > static_cast<int>(width_) - 1) i1 = width_ - 1;
    if (j1 > static_cast<int>(height_) - 1) j1 = height_ - 1;
    if (i0 > i1) i0 = i1;
    if (j0 > j1) j0 = j1;
    const double tx = std::clamp(fx - i0, 0.0, 1.0);
    const double ty = std::clamp(fy - j0, 0.0, 1.0);

    std::size_t k = 0;
    double tz = 0.0;
    bool clamped = false;
    if (!bracket_depth(depth_m, k, tz, clamped)) {
        if (fallback_) return fallback_->velocity_at(ll, depth_m);
        return Eigen::Vector3d::Zero();
    }

    const std::size_t plane = width_ * height_;
    auto sample_at = [&](const std::vector<float>& field,
                         int i, int j, std::size_t lvl) -> double {
        const std::size_t idx =
            lvl * plane + static_cast<std::size_t>(j) * width_ + static_cast<std::size_t>(i);
        return static_cast<double>(field[idx]);
    };

    auto interp_plane = [&](const std::vector<float>& field, std::size_t lvl,
                            double& out) -> bool {
        const std::size_t idx00 =
            lvl * plane + static_cast<std::size_t>(j0) * width_ + static_cast<std::size_t>(i0);
        const std::size_t idx10 =
            lvl * plane + static_cast<std::size_t>(j0) * width_ + static_cast<std::size_t>(i1);
        const std::size_t idx01 =
            lvl * plane + static_cast<std::size_t>(j1) * width_ + static_cast<std::size_t>(i0);
        const std::size_t idx11 =
            lvl * plane + static_cast<std::size_t>(j1) * width_ + static_cast<std::size_t>(i1);
        const float a00 = field[idx00];
        const float a10 = field[idx10];
        const float a01 = field[idx01];
        const float a11 = field[idx11];
        const bool n00 = is_nodata(a00, nodata_);
        const bool n10 = is_nodata(a10, nodata_);
        const bool n01 = is_nodata(a01, nodata_);
        const bool n11 = is_nodata(a11, nodata_);
        if (n00 && n10 && n01 && n11) return false;
        // Replace nodata corners with the average of valid neighbors so a
        // single bad cell doesn't take out the whole interpolation cell.
        double v00 = n00 ? 0.0 : static_cast<double>(a00);
        double v10 = n10 ? 0.0 : static_cast<double>(a10);
        double v01 = n01 ? 0.0 : static_cast<double>(a01);
        double v11 = n11 ? 0.0 : static_cast<double>(a11);
        int valid_n = (!n00) + (!n10) + (!n01) + (!n11);
        if (valid_n < 4) {
            const double mean = (v00 + v10 + v01 + v11) / valid_n;
            if (n00) v00 = mean;
            if (n10) v10 = mean;
            if (n01) v01 = mean;
            if (n11) v11 = mean;
        }
        const double e0 = v00 * (1.0 - tx) + v10 * tx;
        const double e1 = v01 * (1.0 - tx) + v11 * tx;
        out = e0 * (1.0 - ty) + e1 * ty;
        (void)sample_at; // silence unused warning if nothing else uses it
        return true;
    };

    double u_lo, u_hi, v_lo, v_hi;
    bool got_lo = interp_plane(u_, k, u_lo) && interp_plane(v_, k, v_lo);
    bool got_hi;
    if (k + 1 < depths_.size()) {
        got_hi = interp_plane(u_, k + 1, u_hi) && interp_plane(v_, k + 1, v_hi);
    } else {
        got_hi = false;
    }

    if (!got_lo && !got_hi) {
        if (fallback_) return fallback_->velocity_at(ll, depth_m);
        return Eigen::Vector3d::Zero();
    }

    double u_out = 0.0, v_out = 0.0;
    if (got_lo && got_hi) {
        u_out = u_lo * (1.0 - tz) + u_hi * tz;
        v_out = v_lo * (1.0 - tz) + v_hi * tz;
    } else if (got_lo) {
        u_out = u_lo;
        v_out = v_lo;
    } else {
        u_out = u_hi;
        v_out = v_hi;
    }
    return {u_out, v_out, 0.0};
}

} // namespace swordfish::physics
