#pragma once

#include "geo/land_mask.hpp"
#include "physics/bathymetry.hpp"
#include "physics/currents.hpp"
#include "sim/sim_manager.hpp"

#include <string>

namespace swordfish::net {

struct ServerConfig {
    std::string host = "0.0.0.0";
    int port = 8080;
};

// Blocking; runs the server until SIGINT or stop().
// All pointer parameters may be null; the corresponding feature degrades
// gracefully when missing (planner will not avoid land, environment endpoints
// return empty grids).
void run_server(sim::SimulationManager& mgr,
                const ServerConfig& cfg,
                const geo::LandMask* land = nullptr,
                const physics::Bathymetry* bath = nullptr,
                const physics::CurrentField* currents = nullptr);

} // namespace swordfish::net
