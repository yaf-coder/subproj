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

std::string plan_to_json(const planner::Plan& p) {
    std::ostringstream os;
    os.precision(9);
    os << "{";
    os << "\"distance_m\":" << p.estimated_distance_m;
    os << ",\"duration_s\":" << p.estimated_duration_s;
    os << ",\"energy_J\":" << p.estimated_energy_J;
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

void run_server(sim::SimulationManager& mgr, const ServerConfig& cfg) {
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

    svr.Post("/api/mission", [&mgr](const httplib::Request& req, httplib::Response& res) {
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

        planner::Plan plan = planner::plan_mission(mr);
        mgr.load_plan(plan, mr.start);
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
        else { res.status = 400; res.set_content("{\"error\":\"bad action\"}", "application/json"); return; }
        res.set_content("{\"ok\":true}", "application/json");
    });

    svr.Get("/api/snapshot", [&mgr](const httplib::Request&, httplib::Response& res) {
        add_cors(res);
        res.set_content(snapshot_to_json(mgr.snapshot()), "application/json");
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
