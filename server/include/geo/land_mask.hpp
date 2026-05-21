#pragma once

#include "geo/coords.hpp"

#include <array>
#include <string>
#include <vector>

namespace bathy::geo {

// Land mask backed by Natural Earth land polygons (or any GeoJSON
// FeatureCollection of Polygon / MultiPolygon features in WGS84 lon/lat).
//
// Spatial queries:
//   - is_land(lat, lon): true if the point lies inside any polygon's outer
//                       ring and not inside a hole.
//   - segment_crosses_land(a, b, n): samples the great-circle segment and
//                                    returns true on any land hit.
//
// Loading is one-time at startup. The mask owns a 360x180 (1-degree) spatial
// index that maps each cell to the IDs of polygons whose bounding box
// intersects the cell, so queries are O(small).
class LandMask {
public:
    LandMask();

    // Load from a GeoJSON file. Returns false on parse / IO error.
    bool load_geojson(const std::string& path);

    // Manually mark the mask as "empty" (everything is water). Useful for
    // tests or when the data file is missing — planner/sim degrade gracefully.
    bool loaded() const { return loaded_; }

    // True if the (lat, lon) point is on land.
    bool is_land(double lat_deg, double lon_deg) const;
    bool is_land(LatLon p) const { return is_land(p.lat_deg, p.lon_deg); }

    // True if the great-circle-interpolated segment a->b passes over any
    // land cell. `samples` is the number of intermediate points tested.
    bool segment_crosses_land(LatLon a, LatLon b, int samples = 64) const;

    // Stats (for logging).
    std::size_t polygon_count() const { return polygons_.size(); }

    // Storage types are exposed (rather than private) so the GeoJSON parser
    // in the .cpp can construct them directly. They are otherwise an
    // implementation detail.
    struct Ring {
        // Flat (lon, lat) pairs, lon at even indices.
        std::vector<double> pts;
        double min_lon = 0, max_lon = 0, min_lat = 0, max_lat = 0;
        void finalize_bbox();
    };
    struct Polygon {
        // First ring is the outer boundary; subsequent rings are holes.
        std::vector<Ring> rings;
        double min_lon = 0, max_lon = 0, min_lat = 0, max_lat = 0;
        void finalize_bbox();
    };

private:
    static bool point_in_ring(double lon, double lat, const Ring& r);

    // Index dims: 360 x 180 of 1-degree cells. Cell (i, j) covers
    //   lon in [i-180, i-180+1),  lat in [j-90, j-90+1).
    static constexpr int kIxLon = 360;
    static constexpr int kIxLat = 180;
    int cell_index(double lon_deg, double lat_deg) const;

    bool loaded_ = false;
    std::vector<Polygon> polygons_;
    std::array<std::vector<int>, kIxLon * kIxLat> index_;
};

} // namespace bathy::geo
