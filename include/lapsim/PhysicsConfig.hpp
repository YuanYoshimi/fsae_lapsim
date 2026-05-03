#pragma once

#include <string>

namespace lapsim {

/// Toggleable physics flags for the unified LapTimeSolver.
/// Named presets reproduce the four legacy solver behaviours exactly.
struct PhysicsConfig {
    bool   continuous_profile = true;   ///< QSS forward/backward pass (vs basic kinematic)
    bool   friction_circle    = true;   ///< Elliptical g-g constraint
    bool   aero               = true;   ///< Drag + downforce effects
    double ds                 = 0.5;    ///< Spatial discretization step [m]

    /// @name Named presets (factory methods)
    /// @{
    [[nodiscard]] static auto basic() -> PhysicsConfig;
    [[nodiscard]] static auto qss()   -> PhysicsConfig;
    [[nodiscard]] static auto fc()    -> PhysicsConfig;
    [[nodiscard]] static auto aero_preset() -> PhysicsConfig;
    /// @}

    /// Parse a preset name ("basic", "qss", "fc", "aero") to a PhysicsConfig.
    /// @throws std::invalid_argument on unrecognised name.
    [[nodiscard]] static auto from_preset(const std::string& name) -> PhysicsConfig;

    /// Returns "basic"/"qss"/"fc"/"aero" if flags match a preset, else "custom".
    [[nodiscard]] auto preset_name() const -> std::string;

    /// Human-readable summary of enabled physics features.
    [[nodiscard]] auto description() const -> std::string;
};

} // namespace lapsim
