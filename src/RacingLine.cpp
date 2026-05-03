#include "lapsim/RacingLine.hpp"

#include "lapsim/Segment.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>

namespace lapsim {

namespace {

/// Cubic Hermite (smoothstep) from a at u=0 to b at u=1, with zero
/// derivative at both endpoints. Picking zero slope at every knot
/// guarantees no kinks where adjacent blends meet.
double hermite01(double a, double b, double u) {
    u = std::clamp(u, 0.0, 1.0);
    double s = u * u * (3.0 - 2.0 * u);
    return a + (b - a) * s;
}

} // namespace

RacingLine::RacingLine(const Track& track, const RacingLineParams& params)
    : track_{track}, params_{params} {
    build_offset_profile();
}

void RacingLine::build_offset_profile() {
    const std::size_t N = track_.segment_count();
    if (N == 0) return;

    const double half_w = track_.width() / 2.0;

    segs_.assign(N, SegContext{});
    {
        double cum = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            const Segment& seg = track_.segment(i);
            segs_[i].start_s = cum;
            segs_[i].length = seg.length();
            cum += segs_[i].length;

            // Sample curvature mid-segment to classify and get sign.
            // Straights return exactly 0; arcs return ±1/R.
            double k = seg.curvature(seg.length() / 2.0);
            if (std::abs(k) > 1e-9) {
                segs_[i].is_arc = true;
                segs_[i].turn_sign = (k > 0.0) ? 1.0 : -1.0;
            } else {
                segs_[i].is_arc = false;
                segs_[i].turn_sign = 0.0;
                // Pull YAML-configured constant offset off the Straight, if any.
                if (const auto* st = dynamic_cast<const Straight*>(&seg)) {
                    segs_[i].yaml_offset = st->racing_offset_m();
                }
            }
        }
    }

    // Pass 1: arcs get nominal entry/apex/exit offsets from params and turn sign,
    // each capped by apex_radius_fraction·R so tight corners don't overshoot.
    // + offset = LEFT of travel. Inside of a LEFT turn = LEFT = positive.
    // Outside of a LEFT turn = RIGHT = negative. Right turn flips both.
    for (std::size_t i = 0; i < N; ++i) {
        if (!segs_[i].is_arc) continue;

        const double R = 1.0 / std::abs(track_.segment(i).curvature(segs_[i].length * 0.5));
        const double radius_cap = params_.apex_radius_fraction * R;
        const double apex_mag  = std::min(params_.apex_offset_frac  * half_w, radius_cap);
        const double entry_mag = std::min(params_.entry_offset_frac * half_w, radius_cap);
        const double exit_mag  = std::min(params_.exit_offset_frac  * half_w, radius_cap);

        segs_[i].apex_off  =  segs_[i].turn_sign * apex_mag;
        segs_[i].entry_off = -segs_[i].turn_sign * entry_mag;
        segs_[i].exit_off  = -segs_[i].turn_sign * exit_mag;

        // "No point trying" guard: if the offset-curve formula at apex would
        // give curvature significantly above the centerline (>1.5×), the
        // racing line is doing more harm than good locally. Fall back to
        // the centerline through the entire arc. A small increase (~1.1× at
        // R=22.5) is fine since the line wins on overall corner shape; only
        // genuinely pathological cases trigger this.
        const double k_track = segs_[i].turn_sign / R;
        const double denom_apex = 1.0 - segs_[i].apex_off * k_track;
        if (denom_apex > 0.0) {
            const double k_line_apex = k_track / denom_apex;
            if (std::abs(k_line_apex) > 1.5 * std::abs(k_track)) {
                std::cerr << "[RacingLine] Warning: corner "
                          << track_.segment(i).id() << " (R=" << R
                          << " m) racing line would exceed centerline curvature ("
                          << "k_line=" << k_line_apex << ", k_track=" << k_track
                          << "); using centerline through this corner\n";
                segs_[i].apex_off = 0.0;
                segs_[i].entry_off = 0.0;
                segs_[i].exit_off = 0.0;
            }
        }
    }

    // Pass 1.5: where two arcs meet directly (no straight between), average
    // prev.exit and curr.entry so both sides agree at the seam. Without this
    // the racing line jumps laterally by up to 2*apex_offset_frac*track_width
    // at opposite-handed arc joints (e.g. left→right chicanes), producing a
    // heading kink the FD can't smooth out. Apex magnitudes are untouched.
    for (std::size_t i = 1; i < N; ++i) {
        if (!segs_[i - 1].is_arc || !segs_[i].is_arc) continue;
        double avg = 0.5 * (segs_[i - 1].exit_off + segs_[i].entry_off);
        segs_[i - 1].exit_off = avg;
        segs_[i].entry_off = avg;
    }

    // Pass 2: straights inherit endpoint offsets from neighbors. Lap start
    // and lap end default to centerline (no closed-loop seam continuity).
    for (std::size_t i = 0; i < N; ++i) {
        if (segs_[i].is_arc) continue;
        segs_[i].entry_off = (i > 0 && segs_[i - 1].is_arc) ? segs_[i - 1].exit_off : 0.0;
        segs_[i].exit_off  = (i + 1 < N && segs_[i + 1].is_arc) ? segs_[i + 1].entry_off : 0.0;
        segs_[i].apex_off  = 0.0;
    }
}

std::size_t RacingLine::find_segment(double s) const {
    // Match Track::position's convention: pick the first segment whose
    // cumulative end is >= s. Binary search would also work; N is small.
    for (std::size_t i = 0; i < segs_.size(); ++i) {
        if (s <= segs_[i].start_s + segs_[i].length) return i;
    }
    return segs_.empty() ? 0 : segs_.size() - 1;
}

double RacingLine::offset_in_segment(const SegContext& sc, double s_local) const {
    if (sc.is_arc) {
        // Piecewise smoothstep: entry → apex over [0, L/2], apex → exit over [L/2, L].
        // Slope is zero at each knot (entry, apex, exit) so heading at segment
        // boundaries equals track heading there.
        double t = (sc.length > 0.0) ? s_local / sc.length : 0.0;
        if (t <= 0.5) return hermite01(sc.entry_off, sc.apex_off, 2.0 * t);
        return hermite01(sc.apex_off, sc.exit_off, 2.0 * t - 1.0);
    }

    const double L_seg = sc.length;

    // YAML-configured constant straight: hold the configured offset across
    // the middle of the segment with a hermite blend at each end so the line
    // meets adjacent segments smoothly. Blend distance scales with the
    // straight's length: real racing lines ease into the next corner's setup
    // gradually rather than snapping in over a fixed distance, and a short
    // hard transition (e.g. 5 m) on a 60 m straight produces a curvature spike
    // that makes a path-aware solver brake for nothing. Cap at L/2 - 1 m so
    // the two blends never overlap and at least 2 m of plateau survives even
    // on very short straights.
    if (sc.yaml_offset.has_value()) {
        const double y = *sc.yaml_offset;
        const double blend = std::min(0.30 * L_seg, L_seg * 0.5 - 1.0);
        if (s_local <= blend) {
            return hermite01(sc.entry_off, y, s_local / blend);
        }
        if (s_local >= L_seg - blend) {
            return hermite01(y, sc.exit_off, (s_local - (L_seg - blend)) / blend);
        }
        return y;
    }

    // No YAML offset → algorithmic blend with centerline plateau in the middle.
    const double be = std::min(params_.entry_blend_m, L_seg / 2.0);
    const double bx = std::min(params_.exit_blend_m, L_seg / 2.0);
    if (be + bx >= L_seg - 1e-9) {
        return hermite01(sc.entry_off, sc.exit_off, s_local / L_seg);
    }
    if (s_local <= be) return hermite01(sc.entry_off, 0.0, s_local / be);
    if (s_local >= L_seg - bx) {
        return hermite01(0.0, sc.exit_off, (s_local - (L_seg - bx)) / bx);
    }
    return 0.0;
}

double RacingLine::offset(double s) const {
    if (segs_.empty()) return 0.0;
    s = std::clamp(s, 0.0, track_.total_length());
    const auto& sc = segs_[find_segment(s)];
    return offset_in_segment(sc, s - sc.start_s);
}

Vec2 RacingLine::left_normal(double s) const {
    double h = track_.heading(s);
    return Vec2{-std::sin(h), std::cos(h)};
}

Vec2 RacingLine::position(double s) const {
    double off = offset(s);
    return track_.position(s) + left_normal(s) * off;
}

double RacingLine::heading(double s) const {
    constexpr double h = 0.1;
    const double L = track_.total_length();
    double s_lo = std::max(0.0, s - h);
    double s_hi = std::min(L, s + h);
    Vec2 dp = position(s_hi) - position(s_lo);
    return std::atan2(dp.y, dp.x);
}

double RacingLine::curvature(double s) const {
    // Standard differential geometry of offset curves: a curve r_off = r + d·n
    // built by offsetting r by signed normal distance d has curvature
    //
    //     κ_off = κ / (1 - d·κ)
    //
    // when d is locally constant (do Carmo, "Differential Geometry of Curves
    // and Surfaces", §1-7). This is exact across every apex plateau where
    // d'(s) = 0, and is closed-form everywhere — no FD across the segment
    // boundaries (where κ_track is discontinuous and FD would spike). The
    // straight-to-arc jump in κ is real physics and the QSS solver smooths
    // it via discretization.
    s = std::clamp(s, 0.0, track_.total_length());
    const double k_track = track_.curvature(s);
    const double off = offset(s);
    const double denom = 1.0 - off * k_track;

    // Sanity guard: denom -> 0 means the offset has crossed the arc center,
    // which is geometrically impossible while |offset| stays inside the
    // track (smallest inside radius is R - width/2 = 5 - 2 = 3 m on this
    // track, and apex_offset_frac · half_width = 0.85 · 2 = 1.7 m, leaving
    // a comfortable denominator floor of 1 - 1.7/3 = 0.43). Tightest real
    // case is C4/C5 (R=5): denom = 1 - 1.7·0.2 = 0.66.
    if (denom <= 0.1) {
        std::cerr << "[RacingLine::curvature] WARN: denominator " << denom
                  << " at s=" << s << " (offset=" << off
                  << ", k_track=" << k_track << ") — offset has approached"
                  << " arc center; returning 0\n";
        return 0.0;
    }
    return k_track / denom;
}

double RacingLine::track_length() const {
    return track_.total_length();
}

const RacingLineParams& RacingLine::params() const noexcept {
    return params_;
}

} // namespace lapsim
