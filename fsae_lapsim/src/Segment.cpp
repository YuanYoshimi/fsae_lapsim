#include "lapsim/Segment.hpp"

#include <cmath>
#include <numbers>

namespace lapsim {

// ── Straight ────────────────────────────────────────────────────────────────

Straight::Straight(const std::string& id, Vec2 start, double heading, double length)
    : length_m_{length} {
    id_ = id;
    start_point_ = start;
    start_heading_ = heading;
}

auto Straight::length() const -> double { return length_m_; }

auto Straight::curvature([[maybe_unused]] double s) const -> double { return 0.0; }

auto Straight::position(double s) const -> Vec2 {
    // p(s) = start + s * unit_direction
    return start_point_ + Vec2{std::cos(start_heading_), std::sin(start_heading_)} * s;
}

auto Straight::heading([[maybe_unused]] double s) const -> double {
    return start_heading_;
}

auto Straight::type_name() const -> std::string { return "straight"; }

// ── Arc ─────────────────────────────────────────────────────────────────────

Arc::Arc(const std::string& id, Vec2 start, double heading,
         double radius, double swept_angle)
    : radius_{radius}
    , swept_angle_{swept_angle}
    , sign_{(swept_angle >= 0.0) ? 1.0 : -1.0} {
    id_ = id;
    start_point_ = start;
    start_heading_ = heading;

    // Center lies perpendicular to entry tangent at distance R.
    // Left turn (sign +1): center is to the LEFT  (heading + pi/2).
    // Right turn (sign -1): center is to the RIGHT (heading - pi/2).
    double center_dir = heading + sign_ * std::numbers::pi / 2.0;
    center_ = start + Vec2{std::cos(center_dir), std::sin(center_dir)} * radius_;

    // Angle from center back to start point.
    // start - center points opposite to center_dir, so start_angle = center_dir + pi.
    start_angle_ = center_dir + std::numbers::pi;
}

auto Arc::length() const -> double {
    return radius_ * std::abs(swept_angle_);
}

auto Arc::curvature([[maybe_unused]] double s) const -> double {
    // Signed curvature: positive for left turns, negative for right turns.
    return sign_ / radius_;
}

auto Arc::position(double s) const -> Vec2 {
    // Angular progress along the arc: sign * s / R
    double angle = start_angle_ + sign_ * s / radius_;
    return center_ + Vec2{std::cos(angle), std::sin(angle)} * radius_;
}

auto Arc::heading(double s) const -> double {
    // Tangent is perpendicular to the radius, rotated by sign * pi/2
    double angle = start_angle_ + sign_ * s / radius_;
    return normalize_angle(angle + sign_ * std::numbers::pi / 2.0);
}

auto Arc::type_name() const -> std::string { return "arc"; }

} // namespace lapsim
