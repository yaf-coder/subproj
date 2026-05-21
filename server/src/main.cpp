#include "net/server.hpp"
#include "sim/sim_manager.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    bathy::net::ServerConfig cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if ((a == "--port" || a == "-p") && i + 1 < argc) {
            cfg.port = std::atoi(argv[++i]);
        } else if (a == "--host" && i + 1 < argc) {
            cfg.host = argv[++i];
        } else if (a == "--help" || a == "-h") {
            std::cout << "usage: bathyscaphe [--host H] [--port P]\n";
            return 0;
        }
    }

    bathy::sim::SimulationManager mgr;
    mgr.start_loop();

    bathy::net::run_server(mgr, cfg);

    mgr.stop_loop();
    return 0;
}
