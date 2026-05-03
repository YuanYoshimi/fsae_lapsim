#pragma once

#include "lapsim/Geometry.hpp"

#include <memory>
#include <optional>
#include <string>

namespace lapsim {

/// Abstract base for a single track segment (straight or arc).
class Segment {
public:
    virtual ~Segment() = default;

    [[nodiscard]] auto id() const -> const std::string& { return id_; }
    [[nodiscard]] auto start_point() const -> Vec2 { return start_point_; }
    [[nodiscard]] auto start_heading() const -> double { return start_heading_; }
    [[nodiscard]] auto end_point() const -> Vec2 { return position(length()); }
    [[nodiscard]] auto end_heading() const -> double { return heading(length()); }

    /// Arc length of this segment [m].
    [[nodiscard]] virtual auto length() const -> double = 0;

    /// Signed curvature at arc-length s [1/m]. Positive = left, zero for straights.
    [[nodiscard]] virtual auto curvature(double s) const -> double = 0;

    /// World-space position at arc-length s from segment start.
    [[nodiscard]] virtual auto position(double s) const -> Vec2 = 0;

    /// Heading angle [rad] at arc-length s from segment start.
    [[nodiscard]] virtual auto heading(double s) const -> double = 0;

    /// Human-readable segment type ("straight" or "arc").
    [[nodiscard]] virtual auto type_name() const -> std::string = 0;

    /// Left boundary point at arc-length s for given track width.
    [[nodiscard]] virtual auto left_boundary_point(double s, double width) const -> Vec2 = 0;

    /// Right boundary point at arc-length s for given track width.
    [[nodiscard]] virtual auto right_boundary_point(double s, double width) const -> Vec2 = 0;

protected:
    std::string id_;
    Vec2 start_point_;
    double start_heading_ = 0.0;
};

/// A straight segment of given length.
class Straight final : public Segment {
public:
    /// @param id   Segment identifier (e.g. "S1").
    /// @param start  World-space start position.
    /// @param heading Entry heading [rad].
    /// @param length  Segment length [m].
    Straight(const std::string& id, Vec2 start, double heading, double length);

    [[nodiscard]] auto length() const -> double override;
    [[nodiscard]] auto curvature(double s) const -> double override;
    [[nodiscard]] auto position(double s) const -> Vec2 override;
    [[nodiscard]] auto heading(double s) const -> double override;
    [[nodiscard]] auto type_name() const -> std::string override;
    [[nodiscard]] auto left_boundary_point(double s, double width) const -> Vec2 override;
    [[nodiscard]] auto right_boundary_point(double s, double width) const -> Vec2 override;

    /// YAML-configured constant racing-line offset for this straight.
    /// Sign convention: + = LEFT of travel direction. nullopt = let the
    /// RacingLine algorithm interpolate between adjacent segments' offsets.
    [[nodiscard]] auto racing_offset_m() const noexcept -> std::optional<double> {
        return racing_offset_m_;
    }
    void set_racing_offset_m(double v) { racing_offset_m_ = v; }

private:
    double length_m_;
    std::optional<double> racing_offset_m_;
};

/// A constant-radius arc segment defined by radius and signed swept angle.
class Arc final : public Segment {
public:
    /// @param id     Segment identifier (e.g. "C1").
    /// @param start  World-space start position.
    /// @param heading Entry heading [rad].
    /// @param radius Radius of curvature [m], always positive.
    /// @param swept_angle Signed swept angle [rad]. Positive = left (CCW), negative = right (CW).
    Arc(const std::string& id, Vec2 start, double heading,
        double radius, double swept_angle);

    [[nodiscard]] auto length() const -> double override;
    [[nodiscard]] auto curvature(double s) const -> double override;
    [[nodiscard]] auto position(double s) const -> Vec2 override;
    [[nodiscard]] auto heading(double s) const -> double override;
    [[nodiscard]] auto type_name() const -> std::string override;
    [[nodiscard]] auto left_boundary_point(double s, double width) const -> Vec2 override;
    [[nodiscard]] auto right_boundary_point(double s, double width) const -> Vec2 override;

private:
    double radius_;
    double swept_angle_;
    double sign_;
    Vec2 center_;
    double start_angle_; ///< Angle from center to start point [rad].
};

} // namespace lapsim
