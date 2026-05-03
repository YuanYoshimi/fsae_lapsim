#pragma once

#include "lapsim/Segment.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lapsim {

/// Result of track geometric validation.
struct ValidationResult {
    bool is_valid = true;
    double max_position_gap = 0.0;   ///< Worst position discontinuity [m].
    double max_heading_gap  = 0.0;   ///< Worst heading discontinuity [rad], in [0, pi].
    bool closes_loop = false;        ///< True if last segment end matches first segment start.
    std::vector<std::string> errors; ///< Human-readable error descriptions.
};

/// An ordered sequence of track segments forming a complete circuit.
class Track {
public:
    Track() = default;

    void set_name(const std::string& name);
    void set_closed(bool closed);
    void set_width(double width_m);

    /// Append a segment to the track.
    void add_segment(std::unique_ptr<Segment> seg);

    [[nodiscard]] auto name() const -> const std::string&;
    [[nodiscard]] auto is_closed() const -> bool;

    /// Constant track width [m].
    [[nodiscard]] auto width() const noexcept -> double;

    /// Number of segments in the track.
    [[nodiscard]] auto segment_count() const -> std::size_t;

    /// Read-only access to segment at index i.
    [[nodiscard]] auto segment(std::size_t i) const -> const Segment&;

    /// Find a segment by its string id. Returns nullptr if not found.
    [[nodiscard]] auto find_segment(const std::string& id) const -> const Segment*;

    /// Total track length [m] (sum of all segment lengths).
    [[nodiscard]] auto total_length() const -> double;

    /// World-space position at global arc-length s from track start.
    [[nodiscard]] auto position(double s) const -> Vec2;

    /// Heading [rad] at global arc-length s from track start.
    [[nodiscard]] auto heading(double s) const -> double;

    /// Left boundary point at global arc-length s, offset by track width.
    [[nodiscard]] auto left_boundary_point(double s) const -> Vec2;

    /// Right boundary point at global arc-length s, offset by track width.
    [[nodiscard]] auto right_boundary_point(double s) const -> Vec2;

    /// Check geometric continuity between segments and closure.
    [[nodiscard]] auto validate() const -> ValidationResult;

private:
    std::string name_;
    bool closed_ = false;
    double width_m_ = 4.0;
    std::vector<std::unique_ptr<Segment>> segments_;
};

} // namespace lapsim
