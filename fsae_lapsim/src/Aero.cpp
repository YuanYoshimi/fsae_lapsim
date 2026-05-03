#include "lapsim/Aero.hpp"

namespace lapsim {

Aero::Aero(double rho, double CdA, double ClA)
    : rho_{rho}, CdA_{CdA}, ClA_{ClA} {}

auto Aero::drag_force(double v) const -> double {
    return 0.5 * rho_ * v * v * CdA_;
}

auto Aero::downforce(double v) const -> double {
    return 0.5 * rho_ * v * v * ClA_;
}

} // namespace lapsim
