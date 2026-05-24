#include "net/server.hpp"

#include "httplib.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace bathy::net {

namespace {

// ----- Tiny JSON helpers (no deps). M1-grade: enough for our payloads. -----

// Find the first JSON numeric value following the given key in `s`.
// Returns true and writes to `out` on success.
bool find_number(const std::string& s, const std::string& key, double& out) {
    const std::string needle = "\"" + key + "\"";
    size_t p = s.find(needle);
    if (p == std::string::npos) return false;
    p = s.find(':', p);
    if (p == std::string::npos) return false;
    ++p;
    while (p < s.size() && (s[p] == ' ' || s[p] == '\t' || s[p] == '\n')) ++p;
    char* endp = nullptr;
    const char* start = s.c_str() + p;
    double v = std::strtod(start, &endp);
    if (endp == start) return false;
    out = v;
    return true;
}

bool find_string(const std::string& s, const std::string& key, std::string& out) {
    const std::string needle = "\"" + key + "\"";
    size_t p = s.find(needle);
    if (p == std::string::npos) return false;
    p = s.find(':', p);
    if (p == std::string::npos) return false;
    p = s.find('"', p + 1);
    if (p == std::string::npos) return false;
    size_t end = s.find('"', p + 1);
    if (end == std::string::npos) return false;
    out = s.substr(p + 1, end - p - 1);
    return true;
}

std::string snapshot_to_json(const sim::StateSnapshot& s) {
    std::ostringstream os;
    os.precision(9);
    os << "{";
    os << "\"t\":" << s.t_sim_s;
    os << ",\"lat\":" << s.lat_deg;
    os << ",\"lon\":" << s.lon_deg;
    os << ",\"depth_m\":" << s.depth_m;
    os << ",\"speed_m_s\":" << s.speed_m_s;
    os << ",\"heading_deg\":" << s.heading_deg;
    os << ",\"pitch_deg\":" << s.pitch_deg;
    os << ",\"roll_deg\":" << s.roll_deg;
    os << ",\"soc\":" << s.soc;
    os << ",\"voltage_V\":" << s.battery_voltage_V;
    os << ",\"current_A\":" << s.battery_current_A;
    os << ",\"power_W\":" << s.power_elec_W;
    os << ",\"energy_J\":" << s.energy_used_J;
    os << ",\"distance_m\":" << s.distance_traveled_m;
    os << ",\"wp\":" << s.current_waypoint;
    os << ",\"wp_total\":" << s.total_waypoints;
    os << ",\"running\":" << (s.running ? "true" : "false");
    os << ",\"finished\":" << (s.finished ? "true" : "false");
    os << ",\"grounded\":" << (s.grounded ? "true" : "false");
    os << ",\"plan_loaded\":" << (s.plan_loaded ? "true" : "false");
    os << "}";
    return os.str();
}

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::string plan_to_json(const planner::Plan& p) {
    std::ostringstream os;
    os.precision(9);
    os << "{";
    os << "\"distance_m\":" << p.estimated_distance_m;
    os << ",\"duration_s\":" << p.estimated_duration_s;
    os << ",\"energy_J\":" << p.estimated_energy_J;
    os << ",\"routed_around_land\":" << (p.routed_around_land ? "true" : "false");
    if (!p.error.empty()) {
        os << ",\"error\":\"" << json_escape(p.error) << "\"";
    }
    os << ",\"waypoints\":[";
    for (size_t i = 0; i < p.waypoints.size(); ++i) {
        const auto& w = p.waypoints[i];
        if (i) os << ",";
        os << "{\"lat\":" << w.ll.lat_deg
           << ",\"lon\":" << w.ll.lon_deg
           << ",\"depth_m\":" << w.depth_m
           << ",\"speed_m_s\":" << w.cruise_speed_m_s << "}";
    }
    os << "]}";
    return os.str();
}

void add_cors(httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type");
}

} // namespace

void run_server(sim::SimulationManager& mgr,
                const ServerConfig& cfg,
                const geo::LandMask* land,
                const physics::Bathymetry* bath,
                const physics::CurrentField* currents) {
    httplib::Server svr;

    // Global CORS preflight.
    svr.Options(".*", [](const httplib::Request&, httplib::Response& res) {
        add_cors(res);
        res.status = 204;
    });

    svr.Get("/api/health", [](const httplib::Request&, httplib::Response& res) {
        add_cors(res);
        res.set_content("{\"ok\":true}", "application/json");
    });

    svr.Post("/api/mission", [&mgr, land](const httplib::Request& req, httplib::Response& res) {
        add_cors(res);
        const std::string& body = req.body;
        double s_lat, s_lon, g_lat, g_lon;
        if (!find_number(body, "start_lat", s_lat) ||
            !find_number(body, "start_lon", s_lon) ||
            !find_number(body, "goal_lat", g_lat) ||
            !find_number(body, "goal_lon", g_lon)) {
            res.status = 400;
            res.set_content("{\"error\":\"missing start_lat/start_lon/goal_lat/goal_lon\"}",
                            "application/json");
            return;
        }
        planner::MissionRequest mr;
        mr.start = {s_lat, s_lon};
        mr.goal = {g_lat, g_lon};
        find_number(body, "cruise_depth_m", mr.cruise_depth_m);
        find_number(body, "cruise_speed_m_s", mr.cruise_speed_m_s);
        find_number(body, "descent_rate_m_s", mr.descent_rate_m_s);
        find_number(body, "sample_spacing_m", mr.sample_spacing_m);

        planner::Plan plan = planner::plan_mission(mr, land);
        if (plan.error.empty()) {
            mgr.load_plan(plan, mr.start);
        } else {
            res.status = 422;
        }
        res.set_content(plan_to_json(plan), "application/json");
    });

    svr.Post("/api/control", [&mgr](const httplib::Request& req, httplib::Response& res) {
        add_cors(res);
        std::string action;
        if (!find_string(req.body, "action", action)) {
            res.status = 400;
            res.set_content("{\"error\":\"missing action\"}", "application/json");
            return;
        }
        if (action == "play") mgr.play();
        else if (action == "pause") mgr.pause();
        else if (action == "reset") mgr.reset_to_start();
        else if (action == "set_speed") {
            double v = 1.0;
            if (!find_number(req.body, "value", v)) {
                res.status = 400;
                res.set_content("{\"error\":\"set_speed needs numeric value\"}", "application/json");
                return;
            }
            mgr.set_speed(v);
        }
        else if (action == "set_cursor") {
            double t = 0.0;
            if (!find_number(req.body, "value", t)) {
                res.status = 400;
                res.set_content("{\"error\":\"set_cursor needs numeric value (sim seconds)\"}",
                                "application/json");
                return;
            }
            mgr.set_cursor(t);
        }
        else { res.status = 400; res.set_content("{\"error\":\"bad action\"}", "application/json"); return; }
        res.set_content("{\"ok\":true}", "application/json");
    });

    svr.Get("/api/snapshot", [&mgr](const httplib::Request&, httplib::Response& res) {
        add_cors(res);
        res.set_content(snapshot_to_json(mgr.snapshot()), "application/json");
    });

    // -------------------------------------------------------------------
    // Environment sampling: bathymetry & currents over a lat/lon bbox.
    // -------------------------------------------------------------------
    auto parse_bbox = [](const httplib::Request& req,
                         double& lat_min, double& lon_min,
                         double& lat_max, double& lon_max,
                         int& nx, int& ny) -> bool {
        try {
            lat_min = std::stod(req.get_param_value("lat_min"));
            lon_min = std::stod(req.get_param_value("lon_min"));
            lat_max = std::stod(req.get_param_value("lat_max"));
            lon_max = std::stod(req.get_param_value("lon_max"));
        } catch (...) {
            return false;
        }
        nx = req.has_param("width") ? std::atoi(req.get_param_value("width").c_str()) : 48;
        ny = req.has_param("height") ? std::atoi(req.get_param_value("height").c_str()) : 32;
        nx = std::clamp(nx, 4, 192);
        ny = std::clamp(ny, 4, 192);
        return true;
    };

    svr.Get("/api/bathymetry", [&mgr, bath, land, parse_bbox]
            (const httplib::Request& req, httplib::Response& res) {
        (void)mgr;
        add_cors(res);
        double lat_min, lon_min, lat_max, lon_max;
        int nx, ny;
        if (!parse_bbox(req, lat_min, lon_min, lat_max, lon_max, nx, ny)) {
            res.status = 400;
            res.set_content("{\"error\":\"missing lat_min/lon_min/lat_max/lon_max\"}",
                            "application/json");
            return;
        }
        std::ostringstream os;
        os.precision(7);
        os << "{\"lat_min\":" << lat_min
           << ",\"lon_min\":" << lon_min
           << ",\"lat_max\":" << lat_max
           << ",\"lon_max\":" << lon_max
           << ",\"width\":" << nx
           << ",\"height\":" << ny
           << ",\"max_depth_m\":0";
        double max_depth = 0.0;
        std::vector<double> depths(static_cast<std::size_t>(nx) * ny, 0.0);
        for (int j = 0; j < ny; ++j) {
            const double lat = lat_min + (lat_max - lat_min) * (j + 0.5) / ny;
            for (int i = 0; i < nx; ++i) {
                const double lon = lon_min + (lon_max - lon_min) * (i + 0.5) / nx;
                double d = 0.0;
                const bool on_land = land && land->loaded() && land->is_land(lat, lon);
                if (!on_land && bath) d = bath->depth_at({lat, lon});
                depths[static_cast<std::size_t>(j) * nx + i] = d;
                if (d > max_depth) max_depth = d;
            }
        }
        os << ",\"depths\":[";
        for (std::size_t k = 0; k < depths.size(); ++k) {
            if (k) os << ",";
            // 1 m resolution is plenty for visualization; emit as integer to
            // keep JSON small.
            os << static_cast<long long>(std::lround(depths[k]));
        }
        os << "]}";
        std::string body = os.str();
        // Backfill max_depth (cheap string fixup so we don't double-scan).
        const std::string key = "\"max_depth_m\":0";
        const auto pos = body.find(key);
        if (pos != std::string::npos) {
            std::ostringstream rep;
            rep << "\"max_depth_m\":" << static_cast<long long>(std::lround(max_depth));
            body.replace(pos, key.size(), rep.str());
        }
        res.set_content(body, "application/json");
    });

    svr.Get("/api/currents", [&mgr, currents, land, parse_bbox]
            (const httplib::Request& req, httplib::Response& res) {
        (void)mgr;
        add_cors(res);
        double lat_min, lon_min, lat_max, lon_max;
        int nx, ny;
        if (!parse_bbox(req, lat_min, lon_min, lat_max, lon_max, nx, ny)) {
            res.status = 400;
            res.set_content("{\"error\":\"missing lat_min/lon_min/lat_max/lon_max\"}",
                            "application/json");
            return;
        }
        double depth_m = 0.0;
        if (req.has_param("depth_m")) {
            try { depth_m = std::stod(req.get_param_value("depth_m")); } catch (...) {}
        }
        std::ostringstream os;
        os.precision(6);
        os << "{\"lat_min\":" << lat_min
           << ",\"lon_min\":" << lon_min
           << ",\"lat_max\":" << lat_max
           << ",\"lon_max\":" << lon_max
           << ",\"width\":" << nx
           << ",\"height\":" << ny
           << ",\"depth_m\":" << depth_m
           << ",\"u\":[";
        // u = east-positive, v = north-positive, both m/s.
        std::vector<double> us, vs;
        us.reserve(static_cast<std::size_t>(nx) * ny);
        vs.reserve(static_cast<std::size_t>(nx) * ny);
        for (int j = 0; j < ny; ++j) {
            const double lat = lat_min + (lat_max - lat_min) * (j + 0.5) / ny;
            for (int i = 0; i < nx; ++i) {
                const double lon = lon_min + (lon_max - lon_min) * (i + 0.5) / nx;
                Eigen::Vector3d c = Eigen::Vector3d::Zero();
                const bool on_land = land && land->loaded() && land->is_land(lat, lon);
                if (!on_land && currents) c = currents->velocity_at({lat, lon}, depth_m);
                us.push_back(c.x());
                vs.push_back(c.y());
            }
        }
        for (std::size_t k = 0; k < us.size(); ++k) {
            if (k) os << ",";
            os << us[k];
        }
        os << "],\"v\":[";
        for (std::size_t k = 0; k < vs.size(); ++k) {
            if (k) os << ",";
            os << vs[k];
        }
        os << "]}";
        res.set_content(os.str(), "application/json");
    });

    // -------------------------------------------------------------------
    // Vehicle configuration: torpedo with (length, radius, mass).
    // -------------------------------------------------------------------
    svr.Get("/api/vehicle", [&mgr](const httplib::Request&, httplib::Response& res) {
        add_cors(res);
        const auto v = mgr.vehicle();
        std::ostringstream os;
        os.precision(6);
        os << "{\"name\":\"" << v.name << "\""
           << ",\"mass_kg\":" << v.hull.mass_kg
           << ",\"volume_m3\":" << v.hull.volume_m3
           << ",\"thruster_count\":" << v.thrusters.size()
           << "}";
        res.set_content(os.str(), "application/json");
    });

    svr.Post("/api/vehicle", [&mgr](const httplib::Request& req, httplib::Response& res) {
        add_cors(res);
        double length_m = 1.0, radius_m = 0.10, mass_kg = 25.0;
        find_number(req.body, "length_m", length_m);
        find_number(req.body, "radius_m", radius_m);
        find_number(req.body, "mass_kg", mass_kg);
        auto v = physics::VehicleParams::torpedo(length_m, radius_m, mass_kg);
        mgr.set_vehicle(v);
        std::ostringstream os;
        os.precision(6);
        os << "{\"name\":\"" << v.name << "\""
           << ",\"mass_kg\":" << v.hull.mass_kg
           << ",\"volume_m3\":" << v.hull.volume_m3
           << ",\"length_m\":" << length_m
           << ",\"radius_m\":" << radius_m
           << "}";
        res.set_content(os.str(), "application/json");
    });

    svr.Get("/api/history", [&mgr](const httplib::Request& req, httplib::Response& res) {
        add_cors(res);
        std::size_t since = 0;
        if (req.has_param("since")) {
            try { since = std::stoull(req.get_param_value("since")); }
            catch (...) { since = 0; }
        }
        const auto items = mgr.history(since);
        const std::size_t total = mgr.history_size();
        std::ostringstream os;
        os << "{\"since\":" << since
           << ",\"total\":" << total
           << ",\"items\":[";
        for (std::size_t i = 0; i < items.size(); ++i) {
            if (i) os << ",";
            os << snapshot_to_json(items[i]);
        }
        os << "]}";
        res.set_content(os.str(), "application/json");
    });

    svr.Get("/api/plan", [&mgr](const httplib::Request&, httplib::Response& res) {
        add_cors(res);
        res.set_content(plan_to_json(mgr.current_plan()), "application/json");
    });

    // Server-Sent Events: stream state at ~30 Hz.
    svr.Get("/api/stream", [&mgr](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Cache-Control", "no-cache");
        res.set_chunked_content_provider(
            "text/event-stream",
            [&mgr](size_t /*offset*/, httplib::DataSink& sink) {
                while (sink.is_writable()) {
                    const auto snap = mgr.snapshot();
                    const std::string msg = "data: " + snapshot_to_json(snap) + "\n\n";
                    if (!sink.write(msg.data(), msg.size())) return false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }
                return true;
            });
    });

    std::cerr << "bathyscaphe server listening on " << cfg.host << ":" << cfg.port << "\n";
    svr.listen(cfg.host, cfg.port);
}

} // namespace bathy::net
