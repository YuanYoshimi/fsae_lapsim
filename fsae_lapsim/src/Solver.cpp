#include "lapsim/Solver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace lapsim {

// ── Free helper functions ─────────────────────────────────────────────

auto compute_corner_speed(double mu, double g, double radius) -> double {
    // v = sqrt(mu * g * R)  — centripetal limit on a flat surface
    return std::sqrt(mu * g * radius);
}

auto compute_straight_time(double v_i, double v_f,
                           double length, double accel) -> StraightTimeResult {
    if (length <= 0.0) return {v_i, v_i, 0.0};

    double v_max = std::sqrt(v_i * v_i + 2.0 * accel * length);

    if (v_f >= v_max) {
        double time = (v_max - v_i) / accel;
        return {v_max, v_max, time};
    }

    double v_peak = std::sqrt((v_i * v_i + v_f * v_f) / 2.0 + accel * length);
    double t_accel = (v_peak - v_i) / accel;
    double t_brake = (v_peak - v_f) / accel;
    return {v_f, v_peak, t_accel + t_brake};
}

// ═══════════════════════════════════════════════════════════════════════
// BasicSolver
// ═══════════════════════════════════════════════════════════════════════

auto BasicSolver::name() const -> std::string { return "BasicSolver"; }

auto BasicSolver::solve(const Track& track, const Vehicle& veh) const -> Telemetry {
    const std::size_t n = track.segment_count();
    const double mu = veh.mu;
    const double g  = veh.g_mps2;
    const double a  = veh.max_accel_mps2;

    std::vector<double> corner_speed(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        if (track.segment(i).type_name() == "arc") {
            double R = 1.0 / std::abs(track.segment(i).curvature(0.0));
            corner_speed[i] = compute_corner_speed(mu, g, R);
        }
    }

    for (std::size_t i = 0; i + 1 < n; ++i) {
        if (track.segment(i).type_name() == "arc" &&
            track.segment(i + 1).type_name() == "arc") {
            double diff = std::abs(corner_speed[i] - corner_speed[i + 1]);
            if (diff > 1e-6) {
                std::cerr << "BasicSolver: adjacent corners "
                          << track.segment(i).id() << " (v="
                          << corner_speed[i] << ") and "
                          << track.segment(i + 1).id() << " (v="
                          << corner_speed[i + 1]
                          << ") have mismatched speeds (delta="
                          << diff << " m/s)\n";
            }
        }
    }

    Telemetry telem;
    double v_prev_exit = 0.0;
    double total_time  = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        const auto& seg = track.segment(i);
        SegmentResult sr;
        sr.id     = seg.id();
        sr.length = seg.length();

        if (seg.type_name() == "arc") {
            double v = corner_speed[i];
            sr.type    = SegmentType::arc;
            sr.v_entry = v;
            sr.v_exit  = v;
            sr.v_peak  = v;
            sr.time    = seg.length() / v;
            v_prev_exit = v;
        } else {
            sr.type = SegmentType::straight;
            double v_i = v_prev_exit;
            double v_f = v_i;
            for (std::size_t j = i + 1; j < n; ++j) {
                if (track.segment(j).type_name() == "arc") {
                    v_f = corner_speed[j];
                    break;
                }
            }
            auto result = compute_straight_time(v_i, v_f, seg.length(), a);
            sr.v_entry = v_i;
            sr.v_exit  = result.v_exit;
            sr.v_peak  = result.v_peak;
            sr.time    = result.time;
            v_prev_exit = result.v_exit;
        }

        total_time += sr.time;
        telem.add_segment_result(sr);
    }

    telem.set_total_lap_time(total_time);
    return telem;
}

// ═══════════════════════════════════════════════════════════════════════
// QssSolver
// ═══════════════════════════════════════════════════════════════════════

QssSolver::QssSolver(double ds) : ds_{ds} {}

auto QssSolver::name() const -> std::string {
    std::ostringstream oss;
    oss << "QssSolver (ds=" << ds_ << " m)";
    return oss.str();
}

auto QssSolver::solve(const Track& track, const Vehicle& veh) const -> Telemetry {
    const double total_length = track.total_length();
    if (total_length <= 0.0) return Telemetry{};

    const double mu    = veh.mu;
    const double g     = veh.g_mps2;
    const double a_max = veh.max_accel_mps2;

    // ── Discretize ────────────────────────────────────────────────────
    const auto N_steps = static_cast<std::size_t>(std::ceil(total_length / ds_));
    const double ds = total_length / static_cast<double>(N_steps);
    const std::size_t num_pts = N_steps + 1;

    // Precompute cumulative segment end arc-lengths for fast lookup.
    const std::size_t num_segs = track.segment_count();
    std::vector<double> seg_end(num_segs);
    {
        double acc = 0.0;
        for (std::size_t j = 0; j < num_segs; ++j) {
            acc += track.segment(j).length();
            seg_end[j] = acc;
        }
    }

    auto find_seg = [&](double s) -> std::size_t {
        for (std::size_t j = 0; j < num_segs; ++j) {
            if (s <= seg_end[j] + 1e-12) return j;
        }
        return num_segs - 1;
    };

    auto curvature_at = [&](double s, std::size_t seg_j) -> double {
        double seg_start = (seg_j == 0) ? 0.0 : seg_end[seg_j - 1];
        double local_s = std::clamp(s - seg_start, 0.0, track.segment(seg_j).length());
        return track.segment(seg_j).curvature(local_s);
    };

    // Step 1: station properties — curvature, v_cap, segment index
    std::vector<double>      v_cap(num_pts);
    std::vector<std::size_t> seg_idx(num_pts);
    for (std::size_t i = 0; i < num_pts; ++i) {
        double s = std::min(static_cast<double>(i) * ds, total_length);
        seg_idx[i] = find_seg(s);
        double kappa = std::abs(curvature_at(s, seg_idx[i]));
        // v_cap = sqrt(mu * g / |kappa|) for corners; effectively infinite for straights
        v_cap[i] = (kappa > 1e-12) ? std::sqrt(mu * g / kappa) : 1e6;
    }

    // ── Forward pass (acceleration-limited) ───────────────────────────
    std::vector<double> v_fwd(num_pts);
    v_fwd[0] = 0.0;  // standing start
    for (std::size_t i = 1; i < num_pts; ++i) {
        // v_next <= sqrt(v_prev^2 + 2 * a_max * ds), capped by curvature limit
        v_fwd[i] = std::min(v_cap[i],
                            std::sqrt(v_fwd[i - 1] * v_fwd[i - 1] + 2.0 * a_max * ds));
    }

    // ── Backward pass (braking-limited) ───────────────────────────────
    std::vector<double> v_bwd(num_pts);
    v_bwd[num_pts - 1] = v_cap[num_pts - 1]; // no constraint from after lap end
    for (std::size_t i = num_pts - 1; i > 0; --i) {
        v_bwd[i - 1] = std::min(v_cap[i - 1],
                                std::sqrt(v_bwd[i] * v_bwd[i] + 2.0 * a_max * ds));
    }

    // ── Combined profile ──────────────────────────────────────────────
    std::vector<double> v(num_pts);
    for (std::size_t i = 0; i < num_pts; ++i) {
        v[i] = std::min(v_fwd[i], v_bwd[i]);
    }

    // ── Time integration (trapezoidal) ────────────────────────────────
    // dt[i] = ds / v_avg  where v_avg = (v[i] + v[i+1]) / 2.
    // Epsilon floor avoids division by zero when car is near-stationary.
    constexpr double kVEps = 1e-6;
    std::vector<double> dt_vec(N_steps);
    std::vector<double> t(num_pts, 0.0);
    for (std::size_t i = 0; i < N_steps; ++i) {
        double v_avg = std::max((v[i] + v[i + 1]) / 2.0, kVEps);
        dt_vec[i] = ds / v_avg;
        t[i + 1] = t[i] + dt_vec[i];
    }
    const double total_time = t[num_pts - 1];

    // ── Per-segment summaries (boundary-interpolated) ─────────────────

    // Interpolate velocity at an arbitrary arc-length in the profile.
    auto interp_v = [&](double s_target) -> double {
        if (s_target <= 0.0) return v[0];
        if (s_target >= total_length) return v[N_steps];
        auto i = static_cast<std::size_t>(s_target / ds);
        if (i >= N_steps) i = N_steps - 1;
        double s_i = static_cast<double>(i) * ds;
        double alpha = (s_target - s_i) / ds;
        return v[i] + alpha * (v[i + 1] - v[i]);
    };

    // Segment boundary arc-lengths.
    std::vector<double> seg_start_s(num_segs, 0.0);
    for (std::size_t j = 1; j < num_segs; ++j)
        seg_start_s[j] = seg_end[j - 1];

    // v_entry / v_exit by interpolation at exact segment boundaries.
    std::vector<double> seg_v_entry(num_segs);
    std::vector<double> seg_v_exit(num_segs);
    seg_v_entry[0] = v[0];
    for (std::size_t j = 1; j < num_segs; ++j)
        seg_v_entry[j] = interp_v(seg_start_s[j]);
    for (std::size_t j = 0; j + 1 < num_segs; ++j)
        seg_v_exit[j] = interp_v(seg_end[j]);
    seg_v_exit[num_segs - 1] = v[N_steps];

    // v_peak: max over all discrete samples within the segment.
    // Boundary interpolations are excluded to avoid reporting above-cap
    // speeds caused by the velocity transition spanning the boundary.
    // Fallback to max(v_entry, v_exit) for segments with no samples.
    std::vector<double> seg_v_max(num_segs, 0.0);
    std::vector<bool>   seg_has_sample(num_segs, false);
    for (std::size_t i = 0; i < num_pts; ++i) {
        std::size_t j = seg_idx[i];
        seg_v_max[j] = std::max(seg_v_max[j], v[i]);
        seg_has_sample[j] = true;
    }
    for (std::size_t j = 0; j < num_segs; ++j) {
        if (!seg_has_sample[j])
            seg_v_max[j] = std::max(seg_v_entry[j], seg_v_exit[j]);
    }

    // Per-segment time: split intervals that straddle a segment boundary.
    std::vector<double> seg_time(num_segs, 0.0);
    for (std::size_t i = 0; i < N_steps; ++i) {
        std::size_t k_lo = seg_idx[i];
        std::size_t k_hi = seg_idx[i + 1];

        if (k_lo == k_hi) {
            seg_time[k_lo] += dt_vec[i];
        } else {
            double s_lo = static_cast<double>(i) * ds;
            double s_b  = seg_end[k_lo];
            double alpha = (s_b - s_lo) / ds;
            double v_b   = v[i] + alpha * (v[i + 1] - v[i]);

            double v_avg_l = std::max((v[i] + v_b) / 2.0, kVEps);
            double v_avg_r = std::max((v_b + v[i + 1]) / 2.0, kVEps);
            double dt_left  = (alpha * ds) / v_avg_l;
            double dt_right = ((1.0 - alpha) * ds) / v_avg_r;

            // Normalize so dt_left + dt_right == dt_vec[i] exactly,
            // preserving total time conservation across the split.
            double scale = dt_vec[i] / (dt_left + dt_right);
            seg_time[k_lo] += dt_left  * scale;
            seg_time[k_hi] += dt_right * scale;
        }
    }

    // Assert time conservation: sum(seg_time) must equal total_time.
    {
        double seg_sum = 0.0;
        for (std::size_t j = 0; j < num_segs; ++j) seg_sum += seg_time[j];
        double leak = std::abs(seg_sum - total_time);
        if (leak > 1e-6) {
            std::cerr << "QssSolver: time conservation violation, leakage = "
                      << leak << " s\n";
        }
    }

    // ── Build Telemetry ───────────────────────────────────────────────
    Telemetry telem;

    for (std::size_t j = 0; j < num_segs; ++j) {
        const auto& seg = track.segment(j);
        SegmentResult sr;
        sr.id      = seg.id();
        sr.length  = seg.length();
        sr.type    = (seg.type_name() == "arc") ? SegmentType::arc : SegmentType::straight;
        sr.v_entry = seg_v_entry[j];
        sr.v_exit  = seg_v_exit[j];
        sr.v_peak  = seg_v_max[j];
        sr.time    = seg_time[j];
        telem.add_segment_result(sr);
    }

    // Populate per-station telemetry samples.
    for (std::size_t i = 0; i < num_pts; ++i) {
        double s_i = std::min(static_cast<double>(i) * ds, total_length);
        auto pos   = track.position(s_i);
        double hdg = track.heading(s_i);
        std::size_t j = seg_idx[i];
        double kappa  = curvature_at(s_i, j);

        // Longitudinal acceleration: a = (v[i+1]^2 - v[i]^2) / (2*ds)
        double a_long_mps2 = 0.0;
        if (i < N_steps) {
            a_long_mps2 = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * ds);
        } else if (N_steps > 0) {
            a_long_mps2 = (v[N_steps] * v[N_steps] - v[N_steps - 1] * v[N_steps - 1]) / (2.0 * ds);
        }

        // Lateral acceleration: a_lat = v^2 * |kappa|
        double a_lat_mps2 = v[i] * v[i] * std::abs(kappa);

        TelemetrySample samp;
        samp.distance_m  = s_i;
        samp.time_s      = t[i];
        samp.x           = pos.x;
        samp.y           = pos.y;
        samp.heading_rad = hdg;
        samp.speed_mps   = v[i];
        samp.accel_g     = a_long_mps2 / g;
        samp.lat_accel_g = a_lat_mps2 / g;
        samp.g_total     = std::sqrt(samp.accel_g * samp.accel_g +
                                     samp.lat_accel_g * samp.lat_accel_g);
        samp.sector      = j;
        samp.type        = (track.segment(j).type_name() == "arc")
                               ? SegmentType::arc : SegmentType::straight;
        telem.add_sample(samp);
    }

    telem.set_total_lap_time(total_time);
    return telem;
}

// ═══════════════════════════════════════════════════════════════════════
// FrictionCircleSolver
// ═══════════════════════════════════════════════════════════════════════

FrictionCircleSolver::FrictionCircleSolver(double ds) : ds_{ds} {}

auto FrictionCircleSolver::name() const -> std::string {
    std::ostringstream oss;
    oss << "FrictionCircle (ds=" << ds_ << " m)";
    return oss.str();
}

auto FrictionCircleSolver::a_long_max(double v, double abs_kappa,
                                       double mu_g, double a_max) -> double {
    // Elliptical g-g constraint: (a_lat/(mu*g))^2 + (a_long/a_max)^2 <= 1
    // Gives a_long_max = a_max * sqrt(max(0, 1 - (a_lat/(mu*g))^2)).
    // On straights (kappa=0): full a_max.  At corner apex: 0.
    // The circular form is the special case where a_max == mu*g.
    double a_lat_norm = v * v * abs_kappa / mu_g;  // a_lat / (mu*g)
    double residual = 1.0 - a_lat_norm * a_lat_norm;
    if (residual <= 0.0) return 0.0;
    return a_max * std::sqrt(residual);
}

auto FrictionCircleSolver::solve(const Track& track, const Vehicle& veh) const -> Telemetry {
    const double total_length = track.total_length();
    if (total_length <= 0.0) return Telemetry{};

    const double mu    = veh.mu;
    const double g     = veh.g_mps2;
    const double mu_g  = mu * g;
    const double a_eng = veh.max_accel_mps2;

    // ── Discretize ────────────────────────────────────────────────────
    const auto N_steps = static_cast<std::size_t>(std::ceil(total_length / ds_));
    const double ds = total_length / static_cast<double>(N_steps);
    const std::size_t num_pts = N_steps + 1;

    const std::size_t num_segs = track.segment_count();
    std::vector<double> seg_end(num_segs);
    {
        double acc = 0.0;
        for (std::size_t j = 0; j < num_segs; ++j) {
            acc += track.segment(j).length();
            seg_end[j] = acc;
        }
    }

    auto find_seg = [&](double s) -> std::size_t {
        for (std::size_t j = 0; j < num_segs; ++j) {
            if (s <= seg_end[j] + 1e-12) return j;
        }
        return num_segs - 1;
    };

    auto curvature_at = [&](double s, std::size_t seg_j) -> double {
        double seg_start = (seg_j == 0) ? 0.0 : seg_end[seg_j - 1];
        double local_s = std::clamp(s - seg_start, 0.0, track.segment(seg_j).length());
        return track.segment(seg_j).curvature(local_s);
    };

    // Step 1: station properties — |kappa|, v_cap, segment index
    std::vector<double>      abs_kappa(num_pts);
    std::vector<double>      v_cap(num_pts);
    std::vector<std::size_t> seg_idx(num_pts);
    for (std::size_t i = 0; i < num_pts; ++i) {
        double s = std::min(static_cast<double>(i) * ds, total_length);
        seg_idx[i] = find_seg(s);
        abs_kappa[i] = std::abs(curvature_at(s, seg_idx[i]));
        v_cap[i] = (abs_kappa[i] > 1e-12) ? std::sqrt(mu * g / abs_kappa[i]) : 1e6;
    }

    // ── Forward pass (acceleration-limited by friction circle) ────────
    // a_avail evaluated at the START of each step (Euler forward).
    std::vector<double> v_fwd(num_pts);
    v_fwd[0] = 0.0;
    for (std::size_t i = 1; i < num_pts; ++i) {
        double a_avail = a_long_max(v_fwd[i - 1], abs_kappa[i - 1], mu_g, a_eng);
        double v_cand  = std::sqrt(v_fwd[i - 1] * v_fwd[i - 1] + 2.0 * a_avail * ds);
        v_fwd[i] = std::min(v_cap[i], v_cand);
    }

    // ── Backward pass (braking-limited by friction circle) ────────────
    // a_avail evaluated at the START of the backward step (i+1).
    std::vector<double> v_bwd(num_pts);
    v_bwd[num_pts - 1] = v_cap[num_pts - 1];
    for (std::size_t i = num_pts - 1; i > 0; --i) {
        double a_avail = a_long_max(v_bwd[i], abs_kappa[i], mu_g, a_eng);
        double v_cand  = std::sqrt(v_bwd[i] * v_bwd[i] + 2.0 * a_avail * ds);
        v_bwd[i - 1] = std::min(v_cap[i - 1], v_cand);
    }

    // ── Combined profile ──────────────────────────────────────────────
    std::vector<double> v(num_pts);
    for (std::size_t i = 0; i < num_pts; ++i) {
        v[i] = std::min(v_fwd[i], v_bwd[i]);
    }

    // ── Time integration (trapezoidal) ────────────────────────────────
    constexpr double kVEps = 1e-6;
    std::vector<double> dt_vec(N_steps);
    std::vector<double> t(num_pts, 0.0);
    for (std::size_t i = 0; i < N_steps; ++i) {
        double v_avg = std::max((v[i] + v[i + 1]) / 2.0, kVEps);
        dt_vec[i] = ds / v_avg;
        t[i + 1] = t[i] + dt_vec[i];
    }
    const double total_time = t[num_pts - 1];

    // ── Per-segment summaries (boundary-interpolated) ─────────────────

    auto interp_v = [&](double s_target) -> double {
        if (s_target <= 0.0) return v[0];
        if (s_target >= total_length) return v[N_steps];
        auto idx = static_cast<std::size_t>(s_target / ds);
        if (idx >= N_steps) idx = N_steps - 1;
        double s_idx = static_cast<double>(idx) * ds;
        double alpha = (s_target - s_idx) / ds;
        return v[idx] + alpha * (v[idx + 1] - v[idx]);
    };

    std::vector<double> seg_start_s(num_segs, 0.0);
    for (std::size_t j = 1; j < num_segs; ++j)
        seg_start_s[j] = seg_end[j - 1];

    std::vector<double> seg_v_entry(num_segs);
    std::vector<double> seg_v_exit(num_segs);
    seg_v_entry[0] = v[0];
    for (std::size_t j = 1; j < num_segs; ++j)
        seg_v_entry[j] = interp_v(seg_start_s[j]);
    for (std::size_t j = 0; j + 1 < num_segs; ++j)
        seg_v_exit[j] = interp_v(seg_end[j]);
    seg_v_exit[num_segs - 1] = v[N_steps];

    std::vector<double> seg_v_max(num_segs, 0.0);
    std::vector<bool>   seg_has_sample(num_segs, false);
    for (std::size_t i = 0; i < num_pts; ++i) {
        std::size_t j = seg_idx[i];
        seg_v_max[j] = std::max(seg_v_max[j], v[i]);
        seg_has_sample[j] = true;
    }
    for (std::size_t j = 0; j < num_segs; ++j) {
        if (!seg_has_sample[j])
            seg_v_max[j] = std::max(seg_v_entry[j], seg_v_exit[j]);
    }

    std::vector<double> seg_time(num_segs, 0.0);
    for (std::size_t i = 0; i < N_steps; ++i) {
        std::size_t k_lo = seg_idx[i];
        std::size_t k_hi = seg_idx[i + 1];
        if (k_lo == k_hi) {
            seg_time[k_lo] += dt_vec[i];
        } else {
            double s_lo = static_cast<double>(i) * ds;
            double s_b  = seg_end[k_lo];
            double alpha = (s_b - s_lo) / ds;
            double v_b   = v[i] + alpha * (v[i + 1] - v[i]);
            double v_avg_l = std::max((v[i] + v_b) / 2.0, kVEps);
            double v_avg_r = std::max((v_b + v[i + 1]) / 2.0, kVEps);
            double dt_left  = (alpha * ds) / v_avg_l;
            double dt_right = ((1.0 - alpha) * ds) / v_avg_r;
            double scale = dt_vec[i] / (dt_left + dt_right);
            seg_time[k_lo] += dt_left  * scale;
            seg_time[k_hi] += dt_right * scale;
        }
    }

    {
        double seg_sum = 0.0;
        for (std::size_t j = 0; j < num_segs; ++j) seg_sum += seg_time[j];
        double leak = std::abs(seg_sum - total_time);
        if (leak > 1e-6) {
            std::cerr << "FrictionCircleSolver: time conservation leak = "
                      << leak << " s\n";
        }
    }

    // ── Build Telemetry ───────────────────────────────────────────────
    Telemetry telem;

    for (std::size_t j = 0; j < num_segs; ++j) {
        const auto& seg = track.segment(j);
        SegmentResult sr;
        sr.id      = seg.id();
        sr.length  = seg.length();
        sr.type    = (seg.type_name() == "arc") ? SegmentType::arc : SegmentType::straight;
        sr.v_entry = seg_v_entry[j];
        sr.v_exit  = seg_v_exit[j];
        sr.v_peak  = seg_v_max[j];
        sr.time    = seg_time[j];
        telem.add_segment_result(sr);
    }

    for (std::size_t i = 0; i < num_pts; ++i) {
        double s_i = std::min(static_cast<double>(i) * ds, total_length);
        auto pos   = track.position(s_i);
        double hdg = track.heading(s_i);
        std::size_t j = seg_idx[i];
        double kappa  = curvature_at(s_i, j);

        double a_long_mps2 = 0.0;
        if (i < N_steps) {
            a_long_mps2 = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * ds);
        } else if (N_steps > 0) {
            a_long_mps2 = (v[N_steps] * v[N_steps] - v[N_steps - 1] * v[N_steps - 1]) / (2.0 * ds);
        }

        double a_lat_mps2 = v[i] * v[i] * std::abs(kappa);

        TelemetrySample samp;
        samp.distance_m  = s_i;
        samp.time_s      = t[i];
        samp.x           = pos.x;
        samp.y           = pos.y;
        samp.heading_rad = hdg;
        samp.speed_mps   = v[i];
        samp.accel_g     = a_long_mps2 / g;
        samp.lat_accel_g = a_lat_mps2 / g;
        samp.g_total     = std::sqrt(samp.accel_g * samp.accel_g +
                                     samp.lat_accel_g * samp.lat_accel_g);
        samp.sector      = j;
        samp.type        = (track.segment(j).type_name() == "arc")
                               ? SegmentType::arc : SegmentType::straight;
        telem.add_sample(samp);
    }

    telem.set_total_lap_time(total_time);
    return telem;
}

// ═══════════════════════════════════════════════════════════════════════
// AeroSolver
// ═══════════════════════════════════════════════════════════════════════

AeroSolver::AeroSolver(double ds) : ds_{ds} {}

auto AeroSolver::name() const -> std::string {
    std::ostringstream oss;
    oss << "Aero (ds=" << ds_ << " m)";
    return oss.str();
}

// ── Static helpers ─────────────────────────────────────────────────────

auto AeroSolver::drag_decel(double v, double rho,
                            double CdA, double mass) -> double {
    // F_drag / m = (0.5 * rho * v^2 * CdA) / m
    return 0.5 * rho * CdA * v * v / mass;
}

auto AeroSolver::downforce_per_mass(double v, double rho,
                                    double ClA, double mass) -> double {
    return 0.5 * rho * ClA * v * v / mass;
}

auto AeroSolver::v_cap_aero(double abs_kappa, const Vehicle& veh) -> double {
    if (abs_kappa < 1e-12) return 1e6;

    // Steady-state: v^2*|kappa| = a_lat_max(v) = mu*g + aero_coeff*v^2
    // => v^2 * (|kappa| - aero_coeff) = mu*g
    // => v_cap = sqrt(mu*g / denom)
    double aero_coeff = veh.mu * veh.aero.rho() * veh.aero.cla() / (2.0 * veh.mass_kg);
    double denom = abs_kappa - aero_coeff;
    if (denom <= 0.0) return 100.0;  // safety cap
    return std::sqrt(veh.mu * veh.g_mps2 / denom);
}

auto AeroSolver::a_lat_max_aero(double v, const Vehicle& veh) -> double {
    // mu * N(v) / m = mu*g + mu * 0.5*rho*ClA*v^2 / m
    return veh.mu * veh.g_mps2
         + veh.mu * veh.aero.rho() * veh.aero.cla() * v * v / (2.0 * veh.mass_kg);
}

auto AeroSolver::a_drive_max_aero(double v, double abs_kappa,
                                  const Vehicle& veh) -> double {
    // Elliptical constraint: (a_lat/a_lat_max(v))^2 + (a_drive/a_max)^2 <= 1
    double a_lat = v * v * abs_kappa;
    double a_lat_cap = a_lat_max_aero(v, veh);
    double ratio = a_lat / a_lat_cap;
    double residual = 1.0 - ratio * ratio;
    if (residual <= 0.0) return 0.0;
    return veh.max_accel_mps2 * std::sqrt(residual);
}

// ── Solve ──────────────────────────────────────────────────────────────

auto AeroSolver::solve(const Track& track, const Vehicle& veh) const -> Telemetry {
    const double total_length = track.total_length();
    if (total_length <= 0.0) return Telemetry{};

    const double g     = veh.g_mps2;
    const double rho   = veh.aero.rho();
    const double CdA   = veh.aero.cda();
    const double mass  = veh.mass_kg;

    // ── Discretize ────────────────────────────────────────────────────
    const auto N_steps = static_cast<std::size_t>(std::ceil(total_length / ds_));
    const double ds = total_length / static_cast<double>(N_steps);
    const std::size_t num_pts = N_steps + 1;

    const std::size_t num_segs = track.segment_count();
    std::vector<double> seg_end(num_segs);
    {
        double acc = 0.0;
        for (std::size_t j = 0; j < num_segs; ++j) {
            acc += track.segment(j).length();
            seg_end[j] = acc;
        }
    }

    auto find_seg = [&](double s) -> std::size_t {
        for (std::size_t j = 0; j < num_segs; ++j) {
            if (s <= seg_end[j] + 1e-12) return j;
        }
        return num_segs - 1;
    };

    auto curvature_at = [&](double s, std::size_t seg_j) -> double {
        double seg_start = (seg_j == 0) ? 0.0 : seg_end[seg_j - 1];
        double local_s = std::clamp(s - seg_start, 0.0, track.segment(seg_j).length());
        return track.segment(seg_j).curvature(local_s);
    };

    // Step 1: station properties
    std::vector<double>      abs_kappa(num_pts);
    std::vector<double>      v_cap(num_pts);
    std::vector<std::size_t> seg_idx(num_pts);
    for (std::size_t i = 0; i < num_pts; ++i) {
        double s = std::min(static_cast<double>(i) * ds, total_length);
        seg_idx[i] = find_seg(s);
        abs_kappa[i] = std::abs(curvature_at(s, seg_idx[i]));
        v_cap[i] = v_cap_aero(abs_kappa[i], veh);
    }

    // ── Forward pass (acceleration-limited) ───────────────────────────
    // a_net = a_drive_max - drag_decel.  Can go negative at high speed.
    std::vector<double> v_fwd(num_pts);
    v_fwd[0] = 0.0;
    for (std::size_t i = 1; i < num_pts; ++i) {
        double a_drive = a_drive_max_aero(v_fwd[i - 1], abs_kappa[i - 1], veh);
        double a_drag  = drag_decel(v_fwd[i - 1], rho, CdA, mass);
        double a_net   = a_drive - a_drag;
        double v2      = v_fwd[i - 1] * v_fwd[i - 1] + 2.0 * a_net * ds;
        double v_cand  = std::sqrt(std::max(0.0, v2));
        v_fwd[i] = std::min(v_cap[i], v_cand);
    }

    // ── Backward pass (braking-limited) ───────────────────────────────
    // Drag HELPS braking: total decel magnitude = a_drive + drag_decel.
    std::vector<double> v_bwd(num_pts);
    v_bwd[num_pts - 1] = v_cap[num_pts - 1];
    for (std::size_t i = num_pts - 1; i > 0; --i) {
        double a_drive = a_drive_max_aero(v_bwd[i], abs_kappa[i], veh);
        double a_drag  = drag_decel(v_bwd[i], rho, CdA, mass);
        double a_total = a_drive + a_drag;
        double v_cand  = std::sqrt(v_bwd[i] * v_bwd[i] + 2.0 * a_total * ds);
        v_bwd[i - 1] = std::min(v_cap[i - 1], v_cand);
    }

    // ── Combined profile ──────────────────────────────────────────────
    std::vector<double> v(num_pts);
    for (std::size_t i = 0; i < num_pts; ++i) {
        v[i] = std::min(v_fwd[i], v_bwd[i]);
    }

    // ── Time integration (trapezoidal) ────────────────────────────────
    constexpr double kVEps = 1e-6;
    std::vector<double> dt_vec(N_steps);
    std::vector<double> t(num_pts, 0.0);
    for (std::size_t i = 0; i < N_steps; ++i) {
        double v_avg = std::max((v[i] + v[i + 1]) / 2.0, kVEps);
        dt_vec[i] = ds / v_avg;
        t[i + 1] = t[i] + dt_vec[i];
    }
    const double total_time = t[num_pts - 1];

    // ── Per-segment summaries (boundary-interpolated) ─────────────────

    auto interp_v = [&](double s_target) -> double {
        if (s_target <= 0.0) return v[0];
        if (s_target >= total_length) return v[N_steps];
        auto idx = static_cast<std::size_t>(s_target / ds);
        if (idx >= N_steps) idx = N_steps - 1;
        double s_idx = static_cast<double>(idx) * ds;
        double alpha = (s_target - s_idx) / ds;
        return v[idx] + alpha * (v[idx + 1] - v[idx]);
    };

    std::vector<double> seg_start_s(num_segs, 0.0);
    for (std::size_t j = 1; j < num_segs; ++j)
        seg_start_s[j] = seg_end[j - 1];

    std::vector<double> seg_v_entry(num_segs);
    std::vector<double> seg_v_exit(num_segs);
    seg_v_entry[0] = v[0];
    for (std::size_t j = 1; j < num_segs; ++j)
        seg_v_entry[j] = interp_v(seg_start_s[j]);
    for (std::size_t j = 0; j + 1 < num_segs; ++j)
        seg_v_exit[j] = interp_v(seg_end[j]);
    seg_v_exit[num_segs - 1] = v[N_steps];

    std::vector<double> seg_v_max(num_segs, 0.0);
    std::vector<bool>   seg_has_sample(num_segs, false);
    for (std::size_t i = 0; i < num_pts; ++i) {
        std::size_t j = seg_idx[i];
        seg_v_max[j] = std::max(seg_v_max[j], v[i]);
        seg_has_sample[j] = true;
    }
    for (std::size_t j = 0; j < num_segs; ++j) {
        if (!seg_has_sample[j])
            seg_v_max[j] = std::max(seg_v_entry[j], seg_v_exit[j]);
    }

    std::vector<double> seg_time(num_segs, 0.0);
    for (std::size_t i = 0; i < N_steps; ++i) {
        std::size_t k_lo = seg_idx[i];
        std::size_t k_hi = seg_idx[i + 1];
        if (k_lo == k_hi) {
            seg_time[k_lo] += dt_vec[i];
        } else {
            double s_lo = static_cast<double>(i) * ds;
            double s_b  = seg_end[k_lo];
            double alpha = (s_b - s_lo) / ds;
            double v_b   = v[i] + alpha * (v[i + 1] - v[i]);
            double v_avg_l = std::max((v[i] + v_b) / 2.0, kVEps);
            double v_avg_r = std::max((v_b + v[i + 1]) / 2.0, kVEps);
            double dt_left  = (alpha * ds) / v_avg_l;
            double dt_right = ((1.0 - alpha) * ds) / v_avg_r;
            double scale = dt_vec[i] / (dt_left + dt_right);
            seg_time[k_lo] += dt_left  * scale;
            seg_time[k_hi] += dt_right * scale;
        }
    }

    {
        double seg_sum = 0.0;
        for (std::size_t j = 0; j < num_segs; ++j) seg_sum += seg_time[j];
        double leak = std::abs(seg_sum - total_time);
        if (leak > 1e-6) {
            std::cerr << "AeroSolver: time conservation leak = "
                      << leak << " s\n";
        }
    }

    // ── Build Telemetry ───────────────────────────────────────────────
    Telemetry telem;

    for (std::size_t j = 0; j < num_segs; ++j) {
        const auto& seg = track.segment(j);
        SegmentResult sr;
        sr.id      = seg.id();
        sr.length  = seg.length();
        sr.type    = (seg.type_name() == "arc") ? SegmentType::arc : SegmentType::straight;
        sr.v_entry = seg_v_entry[j];
        sr.v_exit  = seg_v_exit[j];
        sr.v_peak  = seg_v_max[j];
        sr.time    = seg_time[j];
        telem.add_segment_result(sr);
    }

    for (std::size_t i = 0; i < num_pts; ++i) {
        double s_i = std::min(static_cast<double>(i) * ds, total_length);
        auto pos   = track.position(s_i);
        double hdg = track.heading(s_i);
        std::size_t j = seg_idx[i];
        double kappa  = curvature_at(s_i, j);

        // Net longitudinal acceleration (includes drag).
        double a_long_mps2 = 0.0;
        if (i < N_steps) {
            a_long_mps2 = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * ds);
        } else if (N_steps > 0) {
            a_long_mps2 = (v[N_steps] * v[N_steps] - v[N_steps - 1] * v[N_steps - 1]) / (2.0 * ds);
        }

        double a_lat_mps2 = v[i] * v[i] * std::abs(kappa);

        TelemetrySample samp;
        samp.distance_m     = s_i;
        samp.time_s         = t[i];
        samp.x              = pos.x;
        samp.y              = pos.y;
        samp.heading_rad    = hdg;
        samp.speed_mps      = v[i];
        samp.accel_g        = a_long_mps2 / g;
        samp.lat_accel_g    = a_lat_mps2 / g;
        samp.g_total        = std::sqrt(samp.accel_g * samp.accel_g +
                                        samp.lat_accel_g * samp.lat_accel_g);
        samp.F_drag_N       = veh.aero.drag_force(v[i]);
        samp.F_downforce_N  = veh.aero.downforce(v[i]);
        samp.a_lat_max_mps2 = a_lat_max_aero(v[i], veh);
        samp.sector         = j;
        samp.type           = (track.segment(j).type_name() == "arc")
                                  ? SegmentType::arc : SegmentType::straight;
        telem.add_sample(samp);
    }

    telem.set_total_lap_time(total_time);
    return telem;
}

} // namespace lapsim
