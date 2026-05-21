#include "geo/land_mask.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

namespace bathy::geo {

namespace {

inline int clampi(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
}

// Skip plain whitespace only.
inline void skip_ws(const char*& p, const char* end) {
    while (p < end && std::isspace(static_cast<unsigned char>(*p))) ++p;
}

// Check whether s[p..] starts with the given literal.
inline bool starts_with(const char* p, const char* end, const char* lit) {
    while (*lit) {
        if (p >= end || *p != *lit) return false;
        ++p;
        ++lit;
    }
    return true;
}

// Skip the next ':' from p onward.
inline bool skip_to_colon(const char*& p, const char* end) {
    while (p < end && *p != ':') ++p;
    if (p >= end) return false;
    ++p;
    return true;
}
} // namespace

void LandMask::Ring::finalize_bbox() {
    if (pts.empty()) return;
    min_lon = max_lon = pts[0];
    min_lat = max_lat = pts[1];
    for (std::size_t i = 0; i + 1 < pts.size(); i += 2) {
        const double lo = pts[i];
        const double la = pts[i + 1];
        min_lon = std::min(min_lon, lo);
        max_lon = std::max(max_lon, lo);
        min_lat = std::min(min_lat, la);
        max_lat = std::max(max_lat, la);
    }
}

void LandMask::Polygon::finalize_bbox() {
    if (rings.empty()) return;
    rings[0].finalize_bbox();
    min_lon = rings[0].min_lon;
    max_lon = rings[0].max_lon;
    min_lat = rings[0].min_lat;
    max_lat = rings[0].max_lat;
    for (std::size_t k = 1; k < rings.size(); ++k) rings[k].finalize_bbox();
}

LandMask::LandMask() = default;

int LandMask::cell_index(double lon_deg, double lat_deg) const {
    // Wrap longitude into [-180, 180).
    double lon = lon_deg;
    while (lon < -180.0) lon += 360.0;
    while (lon >= 180.0) lon -= 360.0;
    const double lat = std::clamp(lat_deg, -90.0, 89.999);
    const int i = clampi(static_cast<int>(std::floor(lon + 180.0)), 0, kIxLon - 1);
    const int j = clampi(static_cast<int>(std::floor(lat + 90.0)), 0, kIxLat - 1);
    return j * kIxLon + i;
}

bool LandMask::point_in_ring(double lon, double lat, const Ring& r) {
    // Even-odd ray casting.
    const auto& p = r.pts;
    const std::size_t n = p.size() / 2;
    if (n < 3) return false;
    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = p[2 * i], yi = p[2 * i + 1];
        const double xj = p[2 * j], yj = p[2 * j + 1];
        const bool crosses = ((yi > lat) != (yj > lat)) &&
            (lon < (xj - xi) * (lat - yi) / ((yj - yi) == 0.0 ? 1e-30 : (yj - yi)) + xi);
        if (crosses) inside = !inside;
    }
    return inside;
}

bool LandMask::is_land(double lat_deg, double lon_deg) const {
    if (!loaded_) return false;
    const int cell = cell_index(lon_deg, lat_deg);
    const auto& candidates = index_[cell];
    for (int poly_idx : candidates) {
        const Polygon& poly = polygons_[poly_idx];
        if (lon_deg < poly.min_lon || lon_deg > poly.max_lon ||
            lat_deg < poly.min_lat || lat_deg > poly.max_lat) continue;
        if (!point_in_ring(lon_deg, lat_deg, poly.rings[0])) continue;
        bool in_hole = false;
        for (std::size_t k = 1; k < poly.rings.size(); ++k) {
            if (point_in_ring(lon_deg, lat_deg, poly.rings[k])) { in_hole = true; break; }
        }
        if (!in_hole) return true;
    }
    return false;
}

bool LandMask::segment_crosses_land(LatLon a, LatLon b, int samples) const {
    if (!loaded_) return false;
    // Linear interpolation in lat/lon is fine for the scales we care about
    // (< ~100 km). For longer segments, this could underestimate, but the
    // sample density covers it.
    if (samples < 2) samples = 2;
    for (int k = 0; k <= samples; ++k) {
        const double f = static_cast<double>(k) / samples;
        const double lat = a.lat_deg + (b.lat_deg - a.lat_deg) * f;
        const double lon = a.lon_deg + (b.lon_deg - a.lon_deg) * f;
        if (is_land(lat, lon)) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// GeoJSON parser
// ---------------------------------------------------------------------------
//
// This is a minimal scanner for GeoJSON FeatureCollections containing
// (Multi)Polygon geometries. It doesn't try to be a general JSON parser —
// it scans for `"type":"Polygon"` / `"MultiPolygon"` strings and the matching
// `"coordinates":[…]` blocks, then walks the bracketed coordinate tree by
// tracking depth.
//
// Polygon       coords: [ [ [lon,lat], ... ], ... ]              -> point depth = 3
// MultiPolygon  coords: [ [ [ [lon,lat], ... ], ... ], ... ]     -> point depth = 4
//
namespace {
struct ParseSink {
    std::vector<LandMask::Polygon>* out;
};
ParseSink* g_sink = nullptr;

void parse_coords(const char*& p, const char* end, int point_depth) {
    int depth = 0;
    LandMask::Polygon cur_poly;
    LandMask::Ring cur_ring;
    double cur_lon = 0.0, cur_lat = 0.0;
    int point_num = 0;

    auto finalize_polygon = [&]() {
        if (!cur_poly.rings.empty()) {
            cur_poly.finalize_bbox();
            g_sink->out->push_back(std::move(cur_poly));
            cur_poly = LandMask::Polygon{};
        }
    };

    while (p < end) {
        const char c = *p;
        if (c == '[') {
            ++depth;
            ++p;
            if (depth == point_depth) point_num = 0;
        } else if (c == ']') {
            if (depth == point_depth) {
                if (point_num >= 2) {
                    cur_ring.pts.push_back(cur_lon);
                    cur_ring.pts.push_back(cur_lat);
                }
            } else if (depth == point_depth - 1) {
                if (!cur_ring.pts.empty()) {
                    cur_poly.rings.push_back(std::move(cur_ring));
                    cur_ring = LandMask::Ring{};
                }
            } else if (depth == point_depth - 2) {
                // MultiPolygon: one polygon ends.
                finalize_polygon();
            }
            --depth;
            ++p;
            if (depth == 0) {
                // Polygon (point_depth == 3) closes its single polygon here.
                if (point_depth == 3) finalize_polygon();
                return;
            }
        } else if (std::isspace(static_cast<unsigned char>(c)) || c == ',') {
            ++p;
        } else {
            // Attempt to parse a number.
            if (depth == point_depth && point_num < 2) {
                char* endp = nullptr;
                const double v = std::strtod(p, &endp);
                if (endp == p) { ++p; continue; }
                if (point_num == 0) cur_lon = v;
                else cur_lat = v;
                ++point_num;
                p = endp;
            } else {
                // Skip number-like characters (or unknown token).
                while (p < end && (std::isdigit(static_cast<unsigned char>(*p)) ||
                                   *p == '.' || *p == '-' || *p == '+' ||
                                   *p == 'e' || *p == 'E')) ++p;
                if (p < end && !std::isspace(static_cast<unsigned char>(*p)) &&
                    *p != ',' && *p != '[' && *p != ']') ++p;
            }
        }
    }
}

} // namespace

bool LandMask::load_geojson(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "LandMask: could not open " << path << "\n";
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    const std::string data = ss.str();
    if (data.empty()) {
        std::cerr << "LandMask: empty file " << path << "\n";
        return false;
    }

    polygons_.clear();
    for (auto& cell : index_) cell.clear();

    ParseSink sink{&polygons_};
    g_sink = &sink;

    const char* p = data.data();
    const char* end = p + data.size();

    int cur_point_depth = 0; // 3 for Polygon, 4 for MultiPolygon, 0 = ignore

    // Forward scan: look for "type" -> Polygon/MultiPolygon, then "coordinates":
    while (p < end) {
        if (*p == '"') {
            // Detect "type" or "coordinates" tokens.
            if (starts_with(p, end, "\"type\"")) {
                p += 6;
                if (!skip_to_colon(p, end)) break;
                skip_ws(p, end);
                if (p < end && *p == '"') {
                    ++p;
                    const char* s = p;
                    while (p < end && *p != '"') ++p;
                    std::string ty(s, p - s);
                    if (p < end) ++p;
                    if (ty == "Polygon") cur_point_depth = 3;
                    else if (ty == "MultiPolygon") cur_point_depth = 4;
                    else cur_point_depth = 0;
                }
                continue;
            }
            if (starts_with(p, end, "\"coordinates\"")) {
                p += 13;
                if (!skip_to_colon(p, end)) break;
                skip_ws(p, end);
                if (p < end && *p == '[' && cur_point_depth > 0) {
                    parse_coords(p, end, cur_point_depth);
                    cur_point_depth = 0;
                }
                continue;
            }
            // Some other string — skip past its closing quote (handle backslash).
            ++p;
            while (p < end && *p != '"') {
                if (*p == '\\' && p + 1 < end) p += 2;
                else ++p;
            }
            if (p < end) ++p;
        } else {
            ++p;
        }
    }
    g_sink = nullptr;

    // Build the spatial index: each polygon is registered to every 1° cell
    // its bbox overlaps.
    for (int idx = 0; idx < static_cast<int>(polygons_.size()); ++idx) {
        const Polygon& poly = polygons_[idx];
        const int i_lo = clampi(static_cast<int>(std::floor(poly.min_lon + 180.0)), 0, kIxLon - 1);
        const int i_hi = clampi(static_cast<int>(std::floor(poly.max_lon + 180.0)), 0, kIxLon - 1);
        const int j_lo = clampi(static_cast<int>(std::floor(poly.min_lat + 90.0)), 0, kIxLat - 1);
        const int j_hi = clampi(static_cast<int>(std::floor(poly.max_lat + 90.0)), 0, kIxLat - 1);
        for (int j = j_lo; j <= j_hi; ++j) {
            for (int i = i_lo; i <= i_hi; ++i) {
                index_[j * kIxLon + i].push_back(idx);
            }
        }
    }

    loaded_ = !polygons_.empty();
    std::cerr << "LandMask: loaded " << polygons_.size() << " polygons from "
              << path << "\n";
    return loaded_;
}

} // namespace bathy::geo
