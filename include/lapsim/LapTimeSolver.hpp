#pragma once

#include "lapsim/PhysicsConfig.hpp"
#include "lapsim/Telemetry.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/Vehicle.hpp"

#include <string>

namespace lapsim {

/// Unified lap-time solver that branches on PhysicsConfig flags
/// to reproduce all four legacy solver behaviours.
class LapTimeSolver {
public:
    /// Solve the lap and return full telemetry.
    /// Internally selects basic kinematic or QSS path based on cfg flags.
    [[nodiscard]] auto solve(const Track& track, const Vehicle& veh,
                             const PhysicsConfig& cfg) const -> Telemetry;

    [[nodiscard]] auto name() const -> std::string { return "LapTimeSolver"; }
};

} // namespace lapsim
