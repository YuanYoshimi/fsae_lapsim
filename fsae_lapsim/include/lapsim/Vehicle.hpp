#pragma once

#include "lapsim/Aero.hpp"

#include <string>

namespace lapsim {

/// Complete vehicle parameter set for an FSAE car.
struct Vehicle {
    double mass_kg         = 0.0;  ///< Total mass including driver [kg]
    double wheelbase_m     = 0.0;  ///< Wheelbase [m]
    double cda_m2          = 0.0;  ///< Drag area Cd*A [m^2]
    double cla_m2          = 0.0;  ///< Lift (downforce) area Cl*A [m^2]
    double mu              = 0.0;  ///< Peak tire-road friction coefficient
    double max_accel_mps2  = 0.0;  ///< Max longitudinal accel/decel [m/s^2]
    double max_power_w     = 0.0;  ///< Peak engine/motor power [W]
    double h_cg_m          = 0.0;  ///< Centre-of-gravity height [m]
    double g_mps2          = 9.81; ///< Gravitational acceleration [m/s^2]

    Aero aero;                     ///< Aerodynamic model (drag + downforce)

    /// Load vehicle parameters from a YAML file.
    /// Falls back to sensible defaults if the file cannot be read.
    /// @param path Filesystem path to the YAML config.
    /// @return Vehicle populated with values from the file.
    [[nodiscard]] static auto from_yaml(const std::string& path) -> Vehicle;
};

} // namespace lapsim
