#pragma once

#include <memory>

namespace lapsim {

/// Abstract interface for a tire friction model.
class TireModel {
public:
    virtual ~TireModel() = default;

    /// Compute the maximum lateral force available at a given normal load.
    /// @param normal_load_n Vertical load on the tire [N].
    /// @return Maximum lateral force [N].
    [[nodiscard]] virtual auto max_lateral_force(double normal_load_n) const -> double = 0;

    /// Compute the maximum longitudinal force available at a given normal load.
    /// @param normal_load_n Vertical load on the tire [N].
    /// @return Maximum longitudinal force [N].
    [[nodiscard]] virtual auto max_longitudinal_force(double normal_load_n) const -> double = 0;
};

/// Simplest tire model: constant friction coefficient mu.
class ConstantMuTire final : public TireModel {
public:
    /// @param mu Peak friction coefficient.
    explicit ConstantMuTire(double mu = 1.5);

    [[nodiscard]] auto max_lateral_force(double normal_load_n) const -> double override;
    [[nodiscard]] auto max_longitudinal_force(double normal_load_n) const -> double override;

private:
    double mu_;
};

} // namespace lapsim
