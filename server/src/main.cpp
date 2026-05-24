#include "geo/land_mask.hpp"
#include "net/server.hpp"
#include "physics/bathymetry.hpp"
#include "physics/currents.hpp"
#include "physics/raster_bathymetry.hpp"
#include "physics/raster_current_field.hpp"
#include "sim/sim_manager.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

// Try multiple cwd-relative locations for a bundled data file so the binary
// works whether it's run from `server/` or `server/build/`.
std::vector<std::string> path_candidates(const std::string& base) {
    if (base.empty()) return {};
    return {
        base,
        "../" + base,
        "../server/" + base,
    };
}

} // namespace

int main(int argc, char** argv) {
    bathy::net::ServerConfig cfg;
    // Default to the higher-resolution 1:10m land polygons for accurate coastal
    // demos; the 1:50m file is kept as a fallback if 10m is missing.
    std::string land_path = "data/ne_10m_land.geojson";
    const std::string land_fallback = "data/ne_50m_land.geojson";
    std::string bath_path = "data/earth_relief_15m.bath";
    std::string curr_path = "data/hycom_1deg_7depths.curr";
    bool want_raster_bath = true;
    bool want_raster_curr = true;

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
        } else if (a == "--bathymetry" && i + 1 < argc) {
            bath_path = argv[++i];
        } else if (a == "--no-bathymetry") {
            want_raster_bath = false;
        } else if (a == "--currents" && i + 1 < argc) {
            curr_path = argv[++i];
        } else if (a == "--no-currents") {
            want_raster_curr = false;
        } else if (a == "--help" || a == "-h") {
            std::cout <<
                "usage: bathyscaphe [options]\n"
                "  --host H              bind address (default 0.0.0.0)\n"
                "  --port P              port (default 8080)\n"
                "  --land PATH           land polygon GeoJSON (default ne_10m)\n"
                "  --no-land             disable land detection entirely\n"
                "  --bathymetry PATH     packed binary bathymetry (.bath)\n"
                "  --no-bathymetry       force procedural bathymetry only\n"
                "  --currents PATH       packed binary currents (.curr)\n"
                "  --no-currents         force synthetic currents only\n";
            return 0;
        }
    }

    // ---- Land mask ------------------------------------------------------
    bathy::geo::LandMask land;
    if (!land_path.empty()) {
        std::vector<std::string> candidates = path_candidates(land_path);
        for (const auto& p : path_candidates(land_fallback)) candidates.push_back(p);
        bool ok = false;
        for (const auto& p : candidates) {
            if (land.load_geojson(p)) { ok = true; break; }
        }
        if (!ok) {
            std::cerr << "(warning) continuing without land mask; "
                         "planner will not avoid land and sim will not detect grounding\n";
        }
    }

    // ---- Bathymetry: procedural is the always-available fallback;
    //                  raster is a higher-fidelity overlay when available.
    std::unique_ptr<bathy::physics::ProceduralBathymetry> proc_bath;
    if (land.loaded()) {
        proc_bath = std::make_unique<bathy::physics::ProceduralBathymetry>(land);
    }
    std::unique_ptr<bathy::physics::RasterBathymetry> raster_bath;
    const bathy::physics::Bathymetry* effective_bath = proc_bath.get();
    if (want_raster_bath) {
        raster_bath = std::make_unique<bathy::physics::RasterBathymetry>(proc_bath.get());
        bool ok = false;
        for (const auto& p : path_candidates(bath_path)) {
            if (raster_bath->load_file(p)) { ok = true; break; }
        }
        if (!ok) {
            std::cerr << "(notice) no raster bathymetry loaded; "
                         "falling back to procedural model. "
                         "Run tools/fetch_env_data.py bathymetry to download real data.\n";
            raster_bath.reset();
        } else {
            effective_bath = raster_bath.get();
        }
    }

    // ---- Currents: synthetic field is the fallback; HYCOM raster
    //                overrides where available.
    bathy::physics::SyntheticCurrentField synth_curr;
    std::unique_ptr<bathy::physics::RasterCurrentField> raster_curr;
    const bathy::physics::CurrentField* effective_curr = &synth_curr;
    if (want_raster_curr) {
        raster_curr = std::make_unique<bathy::physics::RasterCurrentField>(&synth_curr);
        bool ok = false;
        for (const auto& p : path_candidates(curr_path)) {
            if (raster_curr->load_file(p)) { ok = true; break; }
        }
        if (!ok) {
            std::cerr << "(notice) no raster currents loaded; "
                         "falling back to synthetic field. "
                         "Run tools/fetch_env_data.py currents to download real data.\n";
            raster_curr.reset();
        } else {
            effective_curr = raster_curr.get();
        }
    }

    // ---- Sim + server wiring -------------------------------------------
    bathy::sim::SimulationManager mgr;
    mgr.set_land_mask(land.loaded() ? &land : nullptr);
    mgr.set_bathymetry(effective_bath);
    mgr.set_currents(effective_curr);
    mgr.start_loop();

    bathy::net::run_server(mgr, cfg,
                           land.loaded() ? &land : nullptr,
                           effective_bath,
                           effective_curr);

    mgr.stop_loop();
    return 0;
}
