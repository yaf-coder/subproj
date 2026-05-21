#include "geo/land_mask.hpp"
#include "net/server.hpp"
#include "sim/sim_manager.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    bathy::net::ServerConfig cfg;
    // Default to the higher-resolution 1:10m land polygons for accurate coastal
    // demos; the 1:50m file is kept as a fallback if 10m is missing.
    std::string land_path = "data/ne_10m_land.geojson";
    const std::string land_fallback = "data/ne_50m_land.geojson";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if ((a == "--port" || a == "-p") && i + 1 < argc) {
            cfg.port = std::atoi(argv[++i]);
        } else if (a == "--host" && i + 1 < argc) {
            cfg.host = argv[++i];
        } else if (a == "--land" && i + 1 < argc) {
            land_path = argv[++i];
        } else if (a == "--no-land") {
            land_path.clear();
        } else if (a == "--help" || a == "-h") {
            std::cout <<
                "usage: bathyscaphe [--host H] [--port P] [--land PATH | --no-land]\n";
            return 0;
        }
    }

    bathy::geo::LandMask land;
    if (!land_path.empty()) {
        // Try the requested path (and a couple of cwd-relative fallbacks),
        // then the 1:50m fallback dataset. Allows running the binary from
        // either `server/` or `server/build/`.
        const std::vector<std::string> candidates = {
            land_path,
            "../" + land_path,
            "../server/" + land_path,
            land_fallback,
            "../" + land_fallback,
            "../server/" + land_fallback,
        };
        bool ok = false;
        for (const auto& p : candidates) {
            if (land.load_geojson(p)) { ok = true; break; }
        }
        if (!ok) {
            std::cerr << "(warning) continuing without land mask; "
                         "planner will not avoid land and sim will not detect grounding\n";
        }
    }

    bathy::sim::SimulationManager mgr;
    mgr.set_land_mask(land.loaded() ? &land : nullptr);
    mgr.start_loop();

    bathy::net::run_server(mgr, cfg, land.loaded() ? &land : nullptr);

    mgr.stop_loop();
    return 0;
}
