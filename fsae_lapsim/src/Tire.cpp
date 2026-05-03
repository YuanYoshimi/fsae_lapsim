#include "lapsim/Tire.hpp"

namespace lapsim {

ConstantMuTire::ConstantMuTire(double mu) : mu_{mu} {}

auto ConstantMuTire::max_lateral_force(
    [[maybe_unused]] double normal_load_n) const -> double {
    return 0.0; // TODO: mu_ * normal_load_n
}

auto ConstantMuTire::max_longitudinal_force(
    [[maybe_unused]] double normal_load_n) const -> double {
    return 0.0; // TODO: mu_ * normal_load_n
}

} // namespace lapsim
