#include "planner/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace bathy::planner {

namespace {
constexpr double kPi = 3.14159265358979323846;
inline double d2r(double d) { return d * kPi / 180.0; }
inline double r2d(double r) { return r * 180.0 / kPi; }

// Spherical-earth great-circle interpolation. fraction in [0, 1].
geo::LatLon slerp_latlon(geo::LatLon a, geo::LatLon b, double f) {
    const double phi1 = d2r(a.lat_deg), lam1 = d2r(a.lon_deg);
    const double phi2 = d2r(b.lat_deg), lam2 = d2r(b.lon_deg);
    const double d = 2.0 * std::asin(std::sqrt(
        std::pow(std::sin((phi2 - phi1) / 2.0), 2.0) +
        std::cos(phi1) * std::cos(phi2) *
        std::pow(std::sin((lam2 - lam1) / 2.0), 2.0)));
    if (d < 1e-9) return a;
    const double A = std::sin((1.0 - f) * d) / std::sin(d);
    const double B = std::sin(f * d) / std::sin(d);
    const double x = A * std::cos(phi1) * std::cos(lam1) + B * std::cos(phi2) * std::cos(lam2);
    const double y = A * std::cos(phi1) * std::sin(lam1) + B * std::cos(phi2) * std::sin(lam2);
    const double z = A * std::sin(phi1) + B * std::sin(phi2);
    const double phi = std::atan2(z, std::sqrt(x * x + y * y));
    const double lam = std::atan2(y, x);
    return {r2d(phi), r2d(lam)};
}

// ---------------------------------------------------------------------------
// A* on a local ENU grid that avoids land cells.
// ---------------------------------------------------------------------------

struct Grid {
    geo::LocalFrame frame;
    double cell_m;
    int nx, ny;
    double origin_e, origin_n; // ENU coords of cell (0,0) center
    std::vector<uint8_t> blocked; // size nx*ny, 1 = land

    Grid(geo::LatLon center, double cs, int W, int H, double oe, double on)
        : frame(center), cell_m(cs), nx(W), ny(H),
          origin_e(oe), origin_n(on),
          blocked(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), 0) {}

    int idx(int i, int j) const { return j * nx + i; }
    Eigen::Vector2d enu_of(int i, int j) const {
        return {origin_e + i * cell_m, origin_n + j * cell_m};
    }
    geo::LatLon latlon_of(int i, int j) const {
        return frame.to_latlon(enu_of(i, j));
    }
    bool in_bounds(int i, int j) const {
        return i >= 0 && j >= 0 && i < nx && j < ny;
    }
};

void mark_land_cells(Grid& g, const geo::LandMask& land) {
    for (int j = 0; j < g.ny; ++j) {
        for (int i = 0; i < g.nx; ++i) {
            const geo::LatLon ll = g.latlon_of(i, j);
            if (land.is_land(ll)) g.blocked[g.idx(i, j)] = 1;
        }
    }
}

// Snap (i, j) to the nearest unblocked cell by spiral search. Returns false
// if none found within `max_radius` cells.
bool snap_to_open(const Grid& g, int& i, int& j, int max_radius = 50) {
    if (g.in_bounds(i, j) && !g.blocked[g.idx(i, j)]) return true;
    for (int r = 1; r <= max_radius; ++r) {
        for (int dj = -r; dj <= r; ++dj) {
            for (int di = -r; di <= r; ++di) {
                if (std::max(std::abs(di), std::abs(dj)) != r) continue;
                const int ni = i + di, nj = j + dj;
                if (!g.in_bounds(ni, nj)) continue;
                if (!g.blocked[g.idx(ni, nj)]) {
                    i = ni; j = nj;
                    return true;
                }
            }
        }
    }
    return false;
}

// 8-connected A*. Returns ordered cell indices from start to goal, empty on fail.
std::vector<int> astar(const Grid& g, int si, int sj, int gi, int gj) {
    const int N = g.nx * g.ny;
    std::vector<double> gscore(N, std::numeric_limits<double>::infinity());
    std::vector<int> parent(N, -1);
    std::vector<uint8_t> closed(N, 0);

    auto h = [&](int i, int j) {
        const double dx = (i - gi) * g.cell_m, dy = (j - gj) * g.cell_m;
        return std::sqrt(dx * dx + dy * dy);
    };

    using QNode = std::pair<double, int>; // (f, idx)
    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;

    const int start_idx = g.idx(si, sj);
    gscore[start_idx] = 0.0;
    open.push({h(si, sj), start_idx});

    const int goal_idx = g.idx(gi, gj);
    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool reached = (start_idx == goal_idx);
    while (!open.empty()) {
        const auto top = open.top();
        open.pop();
        const int cur = top.second;
        if (closed[cur]) continue;
        if (cur == goal_idx) { reached = true; break; }
        closed[cur] = 1;
        const int ci = cur % g.nx, cj = cur / g.nx;
        for (int k = 0; k < 8; ++k) {
            const int ni = ci + dx8[k], nj = cj + dy8[k];
            if (!g.in_bounds(ni, nj)) continue;
            const int nidx = g.idx(ni, nj);
            if (g.blocked[nidx] || closed[nidx]) continue;
            // Block diagonal corner-cutting through land.
            if (dx8[k] != 0 && dy8[k] != 0) {
                if (g.blocked[g.idx(ci + dx8[k], cj)] ||
                    g.blocked[g.idx(ci, cj + dy8[k])]) continue;
            }
            const double step = (dx8[k] != 0 && dy8[k] != 0)
                                    ? g.cell_m * std::sqrt(2.0)
                                    : g.cell_m;
            const double tentative = gscore[cur] + step;
            if (tentative < gscore[nidx]) {
                gscore[nidx] = tentative;
                parent[nidx] = cur;
                open.push({tentative + h(ni, nj), nidx});
            }
        }
    }

    if (!reached) return {};
    std::vector<int> path;
    for (int c = goal_idx; c != -1; c = parent[c]) {
        path.push_back(c);
        if (c == start_idx) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Line-of-sight: are all sample points along the great-circle from a to b
// off land?  Used to shortcut adjacent A* waypoints.
bool los_clear(geo::LatLon a, geo::LatLon b, const geo::LandMask& land) {
    const double dist_m = geo::great_circle_m(a, b);
    const int n = std::max(8, static_cast<int>(std::ceil(dist_m / 75.0)));
    return !land.segment_crosses_land(a, b, n);
}

// Greedy line-of-sight smoothing: repeatedly drop intermediate waypoints
// whose neighbors have a clear LOS between them.
std::vector<geo::LatLon> smooth_los(const std::vector<geo::LatLon>& pts,
                                    const geo::LandMask& land) {
    if (pts.size() <= 2) return pts;
    std::vector<geo::LatLon> out;
    out.push_back(pts.front());
    std::size_t i = 0;
    while (i + 1 < pts.size()) {
        std::size_t j = pts.size() - 1;
        // Find the furthest index reachable by LOS from i.
        while (j > i + 1 && !los_clear(pts[i], pts[j], land)) --j;
        out.push_back(pts[j]);
        i = j;
    }
    return out;
}

// Re-sample a polyline at roughly `spacing_m` meters (great-circle).
std::vector<geo::LatLon> resample_polyline(const std::vector<geo::LatLon>& pts,
                                           double spacing_m) {
    std::vector<geo::LatLon> out;
    if (pts.empty()) return out;
    out.push_back(pts.front());
    for (std::size_t k = 1; k < pts.size(); ++k) {
        const geo::LatLon a = pts[k - 1];
        const geo::LatLon b = pts[k];
        const double seg = geo::great_circle_m(a, b);
        const int n = std::max(1, static_cast<int>(std::ceil(seg / std::max(spacing_m, 1.0))));
        for (int s = 1; s <= n; ++s) {
            const double f = static_cast<double>(s) / n;
            out.push_back(slerp_latlon(a, b, f));
        }
    }
    return out;
}

// Try to find a land-avoiding path; return empty vector on failure.
std::vector<geo::LatLon> route_around_land(geo::LatLon start, geo::LatLon goal,
                                           const geo::LandMask& land) {
    const geo::LatLon mid{0.5 * (start.lat_deg + goal.lat_deg),
                          0.5 * (start.lon_deg + goal.lon_deg)};
    geo::LocalFrame frame(mid);
    const Eigen::Vector2d s_enu = frame.to_enu(start);
    const Eigen::Vector2d g_enu = frame.to_enu(goal);

    const double dx = std::abs(g_enu.x() - s_enu.x());
    const double dy = std::abs(g_enu.y() - s_enu.y());
    const double pad = std::max({dx, dy, 1500.0}) * 0.6 + 1500.0;

    const double e_min = std::min(s_enu.x(), g_enu.x()) - pad;
    const double e_max = std::max(s_enu.x(), g_enu.x()) + pad;
    const double n_min = std::min(s_enu.y(), g_enu.y()) - pad;
    const double n_max = std::max(s_enu.y(), g_enu.y()) + pad;

    const double max_extent = std::max(e_max - e_min, n_max - n_min);
    // Cell size: keep grid <= 250x250 cells, clamp between 75m and 750m.
    double cell_m = max_extent / 250.0;
    cell_m = std::clamp(cell_m, 75.0, 750.0);
    const int nx = std::max(2, static_cast<int>(std::ceil((e_max - e_min) / cell_m)) + 1);
    const int ny = std::max(2, static_cast<int>(std::ceil((n_max - n_min) / cell_m)) + 1);

    Grid g(mid, cell_m, nx, ny, e_min, n_min);
    mark_land_cells(g, land);

    int si = static_cast<int>(std::round((s_enu.x() - e_min) / cell_m));
    int sj = static_cast<int>(std::round((s_enu.y() - n_min) / cell_m));
    int gi = static_cast<int>(std::round((g_enu.x() - e_min) / cell_m));
    int gj = static_cast<int>(std::round((g_enu.y() - n_min) / cell_m));
    si = std::clamp(si, 0, nx - 1);
    sj = std::clamp(sj, 0, ny - 1);
    gi = std::clamp(gi, 0, nx - 1);
    gj = std::clamp(gj, 0, ny - 1);

    if (!snap_to_open(g, si, sj) || !snap_to_open(g, gi, gj)) return {};
    const std::vector<int> path = astar(g, si, sj, gi, gj);
    if (path.empty()) return {};

    std::vector<geo::LatLon> route;
    route.reserve(path.size() + 2);
    route.push_back(start);
    for (int c : path) {
        const int i = c % g.nx;
        const int j = c / g.nx;
        route.push_back(g.latlon_of(i, j));
    }
    route.push_back(goal);
    return smooth_los(route, land);
}

double polyline_length_m(const std::vector<geo::LatLon>& pts) {
    double total = 0.0;
    for (std::size_t k = 1; k < pts.size(); ++k) {
        total += geo::great_circle_m(pts[k - 1], pts[k]);
    }
    return total;
}

} // namespace

Plan plan_mission(const MissionRequest& req, const geo::LandMask* land) {
    Plan p;

    if (land && land->loaded()) {
        if (land->is_land(req.start)) {
            p.error = "start position is on land";
            return p;
        }
        if (land->is_land(req.goal)) {
            p.error = "goal position is on land";
            return p;
        }
    }

    // Decide cruise polyline.
    std::vector<geo::LatLon> cruise_pts;
    if (land && land->loaded() && land->segment_crosses_land(req.start, req.goal, 96)) {
        cruise_pts = route_around_land(req.start, req.goal, *land);
        if (cruise_pts.empty()) {
            p.error = "no land-free route found between start and goal";
            return p;
        }
        p.routed_around_land = true;
    } else {
        const double horiz_m = geo::great_circle_m(req.start, req.goal);
        const int n = std::max(1, static_cast<int>(std::ceil(horiz_m / req.sample_spacing_m)));
        cruise_pts.reserve(static_cast<std::size_t>(n) + 1);
        cruise_pts.push_back(req.start);
        for (int i = 1; i <= n; ++i) {
            const double f = static_cast<double>(i) / n;
            cruise_pts.push_back(slerp_latlon(req.start, req.goal, f));
        }
    }

    // Re-sample cruise polyline at sample_spacing_m for control granularity.
    const auto cruise_samples = resample_polyline(cruise_pts, req.sample_spacing_m);
    const double horiz_m = polyline_length_m(cruise_samples);

    // 1) Dive at start position.
    p.waypoints.push_back({req.start, 0.0, req.descent_rate_m_s});
    p.waypoints.push_back({req.start, req.cruise_depth_m, req.descent_rate_m_s});

    // 2) Cruise: skip the duplicate first sample (== start).
    for (std::size_t i = 1; i < cruise_samples.size(); ++i) {
        p.waypoints.push_back({cruise_samples[i], req.cruise_depth_m, req.cruise_speed_m_s});
    }

    // 3) Surface at goal.
    p.waypoints.push_back({req.goal, 0.0, req.descent_rate_m_s});

    // Distance: vertical legs + horizontal.
    const double dist = horiz_m + 2.0 * req.cruise_depth_m;
    p.estimated_distance_m = dist;

    // Duration: cruise time + dive/surface time.
    const double t_cruise = horiz_m / std::max(req.cruise_speed_m_s, 1e-3);
    const double t_vert = 2.0 * req.cruise_depth_m / std::max(req.descent_rate_m_s, 1e-3);
    p.estimated_duration_s = t_cruise + t_vert;

    // Energy: rough steady-state P ~ k*v^3 + idle.
    const double k = 25.0;
    const double idle = 8.0;
    const double eta = 0.7;
    const double P_cruise = (k * std::pow(req.cruise_speed_m_s, 3.0) + idle) / eta;
    const double P_vert = (k * std::pow(req.descent_rate_m_s, 3.0) + idle) / eta;
    p.estimated_energy_J = P_cruise * t_cruise + P_vert * t_vert;

    return p;
}

} // namespace bathy::planner
