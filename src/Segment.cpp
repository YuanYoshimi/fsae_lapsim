#include "lapsim/Segment.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

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

auto Straight::left_boundary_point(double s, double width) const -> Vec2 {
    // Left is +90° from heading: perpendicular = (-sin(h), cos(h))
    Vec2 perp{-std::sin(start_heading_), std::cos(start_heading_)};
    return position(s) + perp * (width / 2.0);
}

auto Straight::right_boundary_point(double s, double width) const -> Vec2 {
    Vec2 perp{-std::sin(start_heading_), std::cos(start_heading_)};
    return position(s) - perp * (width / 2.0);
}

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

auto Arc::left_boundary_point(double s, double width) const -> Vec2 {
    double angle = start_angle_ + sign_ * s / radius_;
    Vec2 outward{std::cos(angle), std::sin(angle)};

    // For a LEFT turn (sign_ > 0): center is to the left of travel,
    // outward points right. Left boundary is INSIDE (R - w/2).
    // For a RIGHT turn (sign_ < 0): center is to the right,
    // outward points left. Left boundary is OUTSIDE (R + w/2).
    double r_left = (sign_ > 0.0) ? (radius_ - width / 2.0)
                                   : (radius_ + width / 2.0);

    if (r_left < 0.1) {
        throw std::runtime_error(
            "Arc '" + id_ + "': inside boundary radius " +
            std::to_string(r_left) + " m is too small for width " +
            std::to_string(width) + " m");
    }

    return center_ + outward * r_left;
}

auto Arc::right_boundary_point(double s, double width) const -> Vec2 {
    double angle = start_angle_ + sign_ * s / radius_;
    Vec2 outward{std::cos(angle), std::sin(angle)};

    // Mirror of left: right is OUTSIDE for left turns, INSIDE for right turns.
    double r_right = (sign_ > 0.0) ? (radius_ + width / 2.0)
                                    : (radius_ - width / 2.0);

    if (r_right < 0.1) {
        throw std::runtime_error(
            "Arc '" + id_ + "': inside boundary radius " +
            std::to_string(r_right) + " m is too small for width " +
            std::to_string(width) + " m");
    }

    return center_ + outward * r_right;
}

} // namespace lapsim
