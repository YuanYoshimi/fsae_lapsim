#pragma once

namespace lapsim {

/// Aerodynamic force calculator for a vehicle at a given speed.
class Aero {
public:
    /// @param rho Air density [kg/m^3].
    /// @param CdA Drag area Cd*A [m^2].
    /// @param ClA Downforce area Cl*A [m^2].
    explicit Aero(double rho = 1.225, double CdA = 0.0, double ClA = 0.0);

    /// Aerodynamic drag force: F_d = 0.5 * rho * v^2 * CdA.
    /// @param v Vehicle speed [m/s].
    /// @return Drag force [N], always >= 0.
    [[nodiscard]] auto drag_force(double v) const -> double;

    /// Aerodynamic downforce: F_l = 0.5 * rho * v^2 * ClA.
    /// @param v Vehicle speed [m/s].
    /// @return Downforce [N], always >= 0.
    [[nodiscard]] auto downforce(double v) const -> double;

    [[nodiscard]] auto rho() const -> double { return rho_; }
    [[nodiscard]] auto cda() const -> double { return CdA_; }
    [[nodiscard]] auto cla() const -> double { return ClA_; }

private:
    double rho_;
    double CdA_;
    double ClA_;
};

} // namespace lapsim
