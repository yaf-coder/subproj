#pragma once

#include "sim/sim_manager.hpp"

#include <string>

namespace bathy::net {

struct ServerConfig {
    std::string host = "0.0.0.0";
    int port = 8080;
};

// Blocking; runs the server until SIGINT or stop().
void run_server(sim::SimulationManager& mgr, const ServerConfig& cfg);

} // namespace bathy::net
