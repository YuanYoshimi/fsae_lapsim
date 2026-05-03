#pragma once

#include "lapsim/Geometry.hpp"
#include "lapsim/Track.hpp"

#include <optional>
#include <vector>

namespace lapsim {

/// Hand-coded out-in-out racing line parameters.
///
/// Offsets are expressed as fractions of (track_width / 2). The sign of
/// each offset is determined by turn direction at build time so callers
/// only specify magnitudes here.
struct RacingLineParams {
    /// Fraction of (track_width/2) the line offsets toward inside at
    /// corner apex. 0 = stay on centerline, 1 = touch inside boundary.
    /// Default 0.85 leaves 15% margin from boundary.
    double apex_offset_frac = 0.85;

    /// Fraction of (track_width/2) the line offsets toward outside at
    /// corner entry.
    double entry_offset_frac = 0.85;

    /// Fraction of (track_width/2) the line offsets toward outside at
    /// corner exit.
    double exit_offset_frac = 0.85;

    /// Distance over which to blend offset on the straight leading INTO
    /// a corner [m]. Cubic Hermite blend; first derivative is matched
    /// at both ends so heading is continuous.
    double entry_blend_m = 8.0;

    /// Distance over which to blend offset on the straight leading OUT
    /// of a corner [m].
    double exit_blend_m = 8.0;

    /// Cap on offset magnitude as a fraction of corner radius. The applied
    /// offset is `min(<frac>·half_width, apex_radius_fraction·R)`. Prevents
    /// tight short corners (e.g. R=5 m) from getting an offset so large
    /// that the racing line zigzags across the centerline twice inside
    /// the arc and ends up slower than the centerline. Default 0.18 keeps
    /// the offset-curve denominator above ~0.82 even at apex.
    double apex_radius_fraction = 0.18;
};

/// A path that stays inside the track corridor, parameterized by
/// centerline arc-length.
///
/// Sign convention: + offset = LEFT of travel direction, - = RIGHT.
/// Built by `build_offset_profile` from segment context (out-in-out for
/// arcs, blend-then-plateau for long straights).
class RacingLine {
public:
    /// Build the racing line for a given track. Track must outlive this object.
    RacingLine(const Track& track, const RacingLineParams& params = {});

    /// Signed lateral offset from centerline at centerline-arc-length s.
    [[nodiscard]] double offset(double s) const;

    /// World position of the racing line at centerline-arc-length s.
    /// Equals track.position(s) + offset(s) * left_normal(s).
    [[nodiscard]] Vec2 position(double s) const;

    /// Tangent direction of the racing line (NOT the centerline) at s,
    /// computed via centered finite difference of position.
    [[nodiscard]] double heading(double s) const;

    /// Curvature of the racing line (NOT the centerline) at s, computed
    /// via centered finite difference of heading.
    [[nodiscard]] double curvature(double s) const;

    /// Total CENTERLINE arc-length [m]. The line is parameterized by
    /// centerline s, not by line-arc-length, for simplicity.
    [[nodiscard]] double track_length() const;

    [[nodiscard]] const RacingLineParams& params() const noexcept;

private:
    /// Per-segment cached context: cumulative arc-length, length, type,
    /// and the entry/apex/exit offsets that define the smoothstep blends.
    struct SegContext {
        double start_s;
        double length;
        bool is_arc;
        double turn_sign; ///< +1 left, -1 right, 0 for straight
        double entry_off;
        double exit_off;
        double apex_off;
        /// YAML-configured constant offset for straights (nullopt = blend).
        std::optional<double> yaml_offset;
    };

    const Track& track_;
    RacingLineParams params_;
    std::vector<SegContext> segs_;

    void build_offset_profile();
    [[nodiscard]] std::size_t find_segment(double s) const;
    [[nodiscard]] double offset_in_segment(const SegContext& sc, double s_local) const;
    [[nodiscard]] Vec2 left_normal(double s) const;
};

} // namespace lapsim
