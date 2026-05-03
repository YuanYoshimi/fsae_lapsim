#include "lapsim/Track.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace lapsim {

void Track::set_name(const std::string& name) { name_ = name; }
void Track::set_closed(bool closed) { closed_ = closed; }

void Track::add_segment(std::unique_ptr<Segment> seg) {
    segments_.push_back(std::move(seg));
}

auto Track::name() const -> const std::string& { return name_; }
auto Track::is_closed() const -> bool { return closed_; }

auto Track::segment_count() const -> std::size_t {
    return segments_.size();
}

auto Track::segment(std::size_t i) const -> const Segment& {
    if (i >= segments_.size()) {
        throw std::out_of_range("Segment index out of range");
    }
    return *segments_[i];
}

auto Track::find_segment(const std::string& id) const -> const Segment* {
    for (const auto& seg : segments_) {
        if (seg->id() == id) return seg.get();
    }
    return nullptr;
}

auto Track::total_length() const -> double {
    double total = 0.0;
    for (const auto& seg : segments_) {
        total += seg->length();
    }
    return total;
}

auto Track::position(double s) const -> Vec2 {
    if (segments_.empty()) return {0.0, 0.0};

    s = std::clamp(s, 0.0, total_length());
    double accumulated = 0.0;
    for (const auto& seg : segments_) {
        double seg_len = seg->length();
        if (s <= accumulated + seg_len) {
            return seg->position(s - accumulated);
        }
        accumulated += seg_len;
    }
    return segments_.back()->end_point();
}

auto Track::heading(double s) const -> double {
    if (segments_.empty()) return 0.0;

    s = std::clamp(s, 0.0, total_length());
    double accumulated = 0.0;
    for (const auto& seg : segments_) {
        double seg_len = seg->length();
        if (s <= accumulated + seg_len) {
            return seg->heading(s - accumulated);
        }
        accumulated += seg_len;
    }
    return segments_.back()->end_heading();
}

auto Track::validate() const -> ValidationResult {
    constexpr double kPosTol = 1e-4;   // meters
    constexpr double kHdgTol = 1e-4;   // radians

    ValidationResult result;

    if (segments_.empty()) {
        result.is_valid = false;
        result.errors.emplace_back("Track has no segments");
        return result;
    }

    auto check_gap = [&](const std::string& label,
                         Vec2 end_pt, double end_hdg,
                         Vec2 start_pt, double start_hdg) {
        double pos_gap = end_pt.distance_to(start_pt);
        double hdg_gap = std::abs(normalize_angle(end_hdg - start_hdg));

        result.max_position_gap = std::max(result.max_position_gap, pos_gap);
        result.max_heading_gap  = std::max(result.max_heading_gap, hdg_gap);

        if (pos_gap > kPosTol) {
            std::ostringstream oss;
            oss << label << ": position gap " << pos_gap
                << " m (tolerance: " << kPosTol << " m)";
            result.errors.push_back(oss.str());
            result.is_valid = false;
        }
        if (hdg_gap > kHdgTol) {
            std::ostringstream oss;
            oss << label << ": heading gap " << hdg_gap
                << " rad (tolerance: " << kHdgTol << " rad)";
            result.errors.push_back(oss.str());
            result.is_valid = false;
        }
    };

    // Check continuity between consecutive segments.
    for (std::size_t i = 0; i + 1 < segments_.size(); ++i) {
        std::string label = "Between " + segments_[i]->id()
                          + " and " + segments_[i + 1]->id();
        check_gap(label,
                  segments_[i]->end_point(), segments_[i]->end_heading(),
                  segments_[i + 1]->start_point(), segments_[i + 1]->start_heading());
    }

    // Check closure (last segment end → first segment start).
    if (closed_) {
        const auto& first = segments_.front();
        const auto& last  = segments_.back();

        double close_pos = last->end_point().distance_to(first->start_point());
        double close_hdg = std::abs(normalize_angle(
            last->end_heading() - first->start_heading()));

        result.max_position_gap = std::max(result.max_position_gap, close_pos);
        result.max_heading_gap  = std::max(result.max_heading_gap, close_hdg);

        result.closes_loop = (close_pos <= kPosTol && close_hdg <= kHdgTol);

        if (close_pos > kPosTol) {
            std::ostringstream oss;
            oss << "Closure (" << last->id() << " end -> " << first->id()
                << " start): position gap " << close_pos
                << " m (tolerance: " << kPosTol << " m)";
            result.errors.push_back(oss.str());
            result.is_valid = false;
        }
        if (close_hdg > kHdgTol) {
            std::ostringstream oss;
            oss << "Closure (" << last->id() << " end -> " << first->id()
                << " start): heading gap " << close_hdg
                << " rad (tolerance: " << kHdgTol << " rad)";
            result.errors.push_back(oss.str());
            result.is_valid = false;
        }
    } else {
        result.closes_loop = false;
    }

    return result;
}

} // namespace lapsim
