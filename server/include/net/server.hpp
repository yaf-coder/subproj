#pragma once

#include "geo/land_mask.hpp"
#include "sim/sim_manager.hpp"

#include <string>

namespace bathy::net {

struct ServerConfig {
    std::string host = "0.0.0.0";
    int port = 8080;
};

// Blocking; runs the server until SIGINT or stop().
// `land` may be null; when present, it is forwarded to the planner so the
// route avoids land.
void run_server(sim::SimulationManager& mgr,
                const ServerConfig& cfg,
                const geo::LandMask* land = nullptr);

} // namespace bathy::net
