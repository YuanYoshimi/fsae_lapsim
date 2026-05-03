#pragma once

#include "lapsim/Telemetry.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/Vehicle.hpp"

#include <string>
#include <vector>

namespace lapsim {

// ── Helper structs for testable sub-computations ──────────────────────

struct StraightTimeResult {
    double v_exit  = 0.0;
    double v_peak  = 0.0;
    double time    = 0.0;
};

[[nodiscard]] auto compute_corner_speed(double mu, double g, double radius) -> double;

[[nodiscard]] auto compute_straight_time(double v_i, double v_f,
                                         double length, double accel) -> StraightTimeResult;

// ── Abstract solver interface ─────────────────────────────────────────

class Solver {
public:
    virtual ~Solver() = default;

    [[nodiscard]] virtual auto solve(const Track& track,
                                     const Vehicle& veh) const -> Telemetry = 0;

    [[nodiscard]] virtual auto name() const -> std::string = 0;
};

// ── BasicSolver (point-mass, per-segment) ─────────────────────────────

class BasicSolver final : public Solver {
public:
    [[nodiscard]] auto solve(const Track& track,
                             const Vehicle& veh) const -> Telemetry override;
    [[nodiscard]] auto name() const -> std::string override;
};

// ── QssSolver (forward/backward continuous profile) ───────────────────

class QssSolver final : public Solver {
public:
    /// @param ds Spatial discretization step [m].
    explicit QssSolver(double ds = 0.5);

    [[nodiscard]] auto solve(const Track& track,
                             const Vehicle& veh) const -> Telemetry override;
    [[nodiscard]] auto name() const -> std::string override;

    [[nodiscard]] auto ds() const -> double { return ds_; }

private:
    double ds_;
};

// ── FrictionCircleSolver (combined lateral + longitudinal grip) ────────

/// QSS solver with elliptical g-g constraint:
///   (a_lat/(mu*g))^2 + (a_long/a_max)^2 <= 1.
/// mu*g is the lateral grip limit; a_max is the longitudinal limit.
/// The circular form is the special case where a_max == mu*g.
class FrictionCircleSolver final : public Solver {
public:
    /// @param ds Spatial discretization step [m].
    explicit FrictionCircleSolver(double ds = 0.5);

    [[nodiscard]] auto solve(const Track& track,
                             const Vehicle& veh) const -> Telemetry override;
    [[nodiscard]] auto name() const -> std::string override;

    [[nodiscard]] auto ds() const -> double { return ds_; }

    /// Available longitudinal acceleration under the elliptical g-g constraint:
    ///   a_long_max = a_max * sqrt(max(0, 1 - (v^2*|kappa|/(mu*g))^2)).
    /// @param v         Current speed [m/s].
    /// @param abs_kappa Absolute curvature [1/m].
    /// @param mu_g      mu * g  [m/s^2].
    /// @param a_max     Powertrain/brake limit [m/s^2].
    [[nodiscard]] static auto a_long_max(double v, double abs_kappa,
                                         double mu_g, double a_max) -> double;

private:
    double ds_;
};

// ── AeroSolver (friction circle + drag + downforce) ────────────────────

/// QSS solver with elliptical g-g constraint, aerodynamic drag, and
/// speed-dependent lateral grip from downforce.
class AeroSolver final : public Solver {
public:
    /// @param ds Spatial discretization step [m].
    explicit AeroSolver(double ds = 0.5);

    [[nodiscard]] auto solve(const Track& track,
                             const Vehicle& veh) const -> Telemetry override;
    [[nodiscard]] auto name() const -> std::string override;

    [[nodiscard]] auto ds() const -> double { return ds_; }

    /// Drag deceleration magnitude: rho*CdA*v^2 / (2*mass).
    [[nodiscard]] static auto drag_decel(double v, double rho,
                                         double CdA, double mass) -> double;

    /// Downforce-induced additional "g" per unit mass: 0.5*rho*ClA*v^2 / mass.
    [[nodiscard]] static auto downforce_per_mass(double v, double rho,
                                                 double ClA, double mass) -> double;

    /// Corner speed cap with speed-dependent grip (closed-form).
    /// v_cap^2 = mu*g / (|kappa| - mu*rho*ClA/(2m)).
    /// Returns v_max_safety=100 m/s if denominator <= 0 (downforce dominates).
    [[nodiscard]] static auto v_cap_aero(double abs_kappa, const Vehicle& veh) -> double;

    /// Speed-dependent lateral grip cap: mu*g + (mu*rho*ClA/(2m))*v^2.
    [[nodiscard]] static auto a_lat_max_aero(double v, const Vehicle& veh) -> double;

    /// Available tire longitudinal acceleration under the elliptical
    /// g-g constraint with speed-dependent lateral cap (excludes drag).
    [[nodiscard]] static auto a_drive_max_aero(double v, double abs_kappa,
                                               const Vehicle& veh) -> double;

private:
    double ds_;
};

} // namespace lapsim
