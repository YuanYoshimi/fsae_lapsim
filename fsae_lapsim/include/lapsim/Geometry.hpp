#pragma once

#include <cmath>
#include <numbers>

namespace lapsim {

/// Minimal 2-D vector for track geometry.
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    [[nodiscard]] auto operator+(const Vec2& rhs) const -> Vec2 {
        return {x + rhs.x, y + rhs.y};
    }

    [[nodiscard]] auto operator-(const Vec2& rhs) const -> Vec2 {
        return {x - rhs.x, y - rhs.y};
    }

    [[nodiscard]] auto operator*(double s) const -> Vec2 {
        return {x * s, y * s};
    }

    [[nodiscard]] auto dot(const Vec2& rhs) const -> double {
        return x * rhs.x + y * rhs.y;
    }

    [[nodiscard]] auto norm() const -> double {
        return std::sqrt(x * x + y * y);
    }

    [[nodiscard]] auto distance_to(const Vec2& other) const -> double {
        return (*this - other).norm();
    }

    /// Angle of vector from +x axis [rad].
    [[nodiscard]] static auto angle_of(const Vec2& v) -> double {
        return std::atan2(v.y, v.x);
    }
};

[[nodiscard]] inline auto operator*(double s, const Vec2& v) -> Vec2 {
    return {s * v.x, s * v.y};
}

/// Normalize angle to (-pi, pi].
[[nodiscard]] inline auto normalize_angle(double a) -> double {
    a = std::fmod(a, 2.0 * std::numbers::pi);
    if (a > std::numbers::pi) a -= 2.0 * std::numbers::pi;
    if (a <= -std::numbers::pi) a += 2.0 * std::numbers::pi;
    return a;
}

} // namespace lapsim
