#pragma once

#include "geo/coords.hpp"

#include <Eigen/Dense>

namespace swordfish::physics {

// World-frame ocean current at a given position and depth, in m/s.
//   x = east, y = north, z = vertical (up positive)
// All implementations should produce zero vertical component for now;
// vertical mixing isn't modeled.
class CurrentField {
public:
    virtual ~CurrentField() = default;
    virtual Eigen::Vector3d velocity_at(geo::LatLon ll, double depth_m) const = 0;
};

// Procedural current field with structure useful for demos:
//
//   * A latitude-dependent zonal flow that loosely echoes real ocean
//     circulation (westward equatorial / eastward mid-latitude).
//   * A large-scale rotational pattern at planetary scales.
//   * Multi-frequency sinusoidal "eddies" so adjacent mission areas
//     experience different currents.
//   * Ekman-like exponential decay with depth (e-folding ~200 m).
//
// All amplitudes are tuned to be in the 0.05 - 0.40 m/s range at the
// surface, which is realistic for coastal currents and big enough to
// visibly bend planned routes.
class SyntheticCurrentField : public CurrentField {
public:
    Eigen::Vector3d velocity_at(geo::LatLon ll, double depth_m) const override;
};

} // namespace swordfish::physics
