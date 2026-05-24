#pragma once

#include <cstdint>

namespace swordfish::physics {

// ---------------------------------------------------------------------------
// Bundled binary grid formats (.bath, .curr).
//
// These are stand-alone, no-dependency formats used to ship preprocessed
// real-world bathymetry and ocean-current data alongside the binary, so that
// the C++ server doesn't need a NetCDF or HDF5 dependency at runtime.
//
// All multi-byte integers and floats are little-endian. The preprocessing
// tool (tools/fetch_env_data.py) writes these files; RasterBathymetry and
// RasterCurrentField read them.
// ---------------------------------------------------------------------------

// "BTHY" little-endian
constexpr std::uint32_t kBathyMagic = 0x59485442u;
// "CURR" little-endian
constexpr std::uint32_t kCurrentMagic = 0x52525543u;
constexpr std::uint32_t kFormatVersion = 1u;

#pragma pack(push, 1)

// 64-byte header for .bath files. Followed immediately by
//   float32 elevation_m[height * width]   // row-major, j=0 is lat_min row
//                                         // negative => below mean sea level
//                                         // (i.e. positive bathymetric depth);
//                                         // positive => land elevation.
// Cell-center convention: sample (i, j) sits at
//   lat = lat_min + (lat_max - lat_min) * j / (height - 1)
//   lon = lon_min + (lon_max - lon_min) * i / (width - 1)
struct BathyHeader {
    std::uint32_t magic;          // == kBathyMagic                  (offset 0)
    std::uint32_t version;        // == kFormatVersion               (offset 4)
    std::uint32_t width;          // longitudinal cell count         (offset 8)
    std::uint32_t height;         // latitudinal cell count          (offset 12)
    double        lat_min;        // bbox, in degrees                (offset 16)
    double        lat_max;        //                                 (offset 24)
    double        lon_min;        //                                 (offset 32)
    double        lon_max;        //                                 (offset 40)
    float         nodata_value;   // sentinel; cells equal to this   (offset 48)
                                  // are masked
    std::uint8_t  pad[12];        // zeroed; reserved                (offset 52)
};
static_assert(sizeof(BathyHeader) == 64, "BathyHeader must be exactly 64 bytes");

// 96-byte header for .curr files. Followed by:
//   float32 depths_m[n_levels]                     // monotonically increasing (>= 0)
//   float32 u_east_m_s[n_levels * height * width]  // level-major, row-major
//   float32 v_north_m_s[n_levels * height * width] // level-major, row-major
// Same cell-center convention as the bathymetry format.
struct CurrentsHeader {
    std::uint32_t magic;          // == kCurrentMagic                (offset 0)
    std::uint32_t version;        // == kFormatVersion               (offset 4)
    std::uint32_t width;          // longitudinal cell count         (offset 8)
    std::uint32_t height;         // latitudinal cell count          (offset 12)
    std::uint32_t n_levels;       // depth-axis cell count           (offset 16)
    std::uint32_t reserved;       // zero                            (offset 20)
    double        lat_min;        // bbox, in degrees                (offset 24)
    double        lat_max;        //                                 (offset 32)
    double        lon_min;        //                                 (offset 40)
    double        lon_max;        //                                 (offset 48)
    float         nodata_value;   //                                 (offset 56)
    std::uint8_t  pad[36];        // zeroed; reserved                (offset 60)
};
static_assert(sizeof(CurrentsHeader) == 96, "CurrentsHeader must be exactly 96 bytes");

#pragma pack(pop)

} // namespace swordfish::physics
