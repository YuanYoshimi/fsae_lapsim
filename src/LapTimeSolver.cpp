#include "lapsim/LapTimeSolver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

namespace lapsim {

namespace {

// ── Basic kinematic helpers (moved from Solver.hpp/cpp) ───────────────

struct StraightTimeResult {
    double v_exit  = 0.0;
    double v_peak  = 0.0;
    double time    = 0.0;
};

double compute_corner_speed(double mu, double g, double radius) {
    return std::sqrt(mu * g * radius);
}

StraightTimeResult compute_straight_time(double v_i, double v_f,
                                         double length, double accel) {
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

// ── Physics helpers (duplicated from legacy solvers for independence) ──

double drag_decel(double v, double rho, double CdA, double mass) {
    return 0.5 * rho * CdA * v * v / mass;
}

double v_cap_aero(double abs_kappa, const Vehicle& veh) {
    if (abs_kappa < 1e-12) return 1e6;
    double aero_coeff = veh.mu * veh.aero.rho() * veh.aero.cla() / (2.0 * veh.mass_kg);
    double denom = abs_kappa - aero_coeff;
    if (denom <= 0.0) return 100.0;
    return std::sqrt(veh.mu * veh.g_mps2 / denom);
}

double a_lat_max_aero(double v, const Vehicle& veh) {
    return veh.mu * veh.g_mps2
         + veh.mu * veh.aero.rho() * veh.aero.cla() * v * v / (2.0 * veh.mass_kg);
}

/// Elliptical g-g with constant mu*g lateral cap (FrictionCircle mode).
double a_long_max_fc(double v, double abs_kappa, double mu_g, double a_max) {
    double a_lat_norm = v * v * abs_kappa / mu_g;
    double residual = 1.0 - a_lat_norm * a_lat_norm;
    if (residual <= 0.0) return 0.0;
    return a_max * std::sqrt(residual);
}

/// Elliptical g-g with speed-dependent lateral cap from downforce.
double a_drive_max_aero(double v, double abs_kappa, const Vehicle& veh) {
    double a_lat = v * v * abs_kappa;
    double a_lat_cap = a_lat_max_aero(v, veh);
    double ratio = a_lat / a_lat_cap;
    double residual = 1.0 - ratio * ratio;
    if (residual <= 0.0) return 0.0;
    return veh.max_accel_mps2 * std::sqrt(residual);
}

// ── Track discretization helpers ──────────────────────────────────────

struct StationData {
    std::vector<double>      abs_kappa;
    std::vector<double>      v_cap;
    std::vector<std::size_t> seg_idx;
    std::vector<double>      seg_end;
    std::size_t              N_steps;
    double                   ds;
    std::size_t              num_pts;
};

StationData discretize(const Track& track, const PhysicsConfig& cfg,
                        const Vehicle& veh) {
    const double total_length = track.total_length();
    const std::size_t num_segs = track.segment_count();

    StationData sd;
    sd.N_steps = static_cast<std::size_t>(std::ceil(total_length / cfg.ds));
    sd.ds      = total_length / static_cast<double>(sd.N_steps);
    sd.num_pts = sd.N_steps + 1;

    sd.seg_end.resize(num_segs);
    {
        double acc = 0.0;
        for (std::size_t j = 0; j < num_segs; ++j) {
            acc += track.segment(j).length();
            sd.seg_end[j] = acc;
        }
    }

    auto find_seg = [&](double s) -> std::size_t {
        for (std::size_t j = 0; j < num_segs; ++j)
            if (s <= sd.seg_end[j] + 1e-12) return j;
        return num_segs - 1;
    };

    auto curvature_at = [&](double s, std::size_t seg_j) -> double {
        double seg_start = (seg_j == 0) ? 0.0 : sd.seg_end[seg_j - 1];
        double local_s = std::clamp(s - seg_start, 0.0, track.segment(seg_j).length());
        return track.segment(seg_j).curvature(local_s);
    };

    sd.abs_kappa.resize(sd.num_pts);
    sd.v_cap.resize(sd.num_pts);
    sd.seg_idx.resize(sd.num_pts);

    const double mu = veh.mu;
    const double g  = veh.g_mps2;

    for (std::size_t i = 0; i < sd.num_pts; ++i) {
        double s = std::min(static_cast<double>(i) * sd.ds, total_length);
        sd.seg_idx[i]   = find_seg(s);
        sd.abs_kappa[i] = std::abs(curvature_at(s, sd.seg_idx[i]));

        if (cfg.aero) {
            sd.v_cap[i] = v_cap_aero(sd.abs_kappa[i], veh);
        } else {
            sd.v_cap[i] = (sd.abs_kappa[i] > 1e-12)
                              ? std::sqrt(mu * g / sd.abs_kappa[i]) : 1e6;
        }
    }
    return sd;
}

// ── Compute tire-only longitudinal acceleration at a station ──────────

double compute_a_drive(double v, double abs_kappa,
                       const Vehicle& veh, const PhysicsConfig& cfg) {
    const double mu_g  = veh.mu * veh.g_mps2;
    const double a_max = veh.max_accel_mps2;

    if (cfg.friction_circle && cfg.aero)
        return a_drive_max_aero(v, abs_kappa, veh);
    if (cfg.friction_circle)
        return a_long_max_fc(v, abs_kappa, mu_g, a_max);
    return a_max;
}

// ── Forward/backward passes ───────────────────────────────────────────

std::vector<double> forward_pass(const StationData& sd,
                                 const Vehicle& veh,
                                 const PhysicsConfig& cfg) {
    std::vector<double> v_fwd(sd.num_pts);
    v_fwd[0] = 0.0;
    const double rho  = veh.aero.rho();
    const double CdA  = veh.aero.cda();
    const double mass = veh.mass_kg;

    for (std::size_t i = 1; i < sd.num_pts; ++i) {
        double a_drive = compute_a_drive(v_fwd[i - 1], sd.abs_kappa[i - 1], veh, cfg);
        double a_net = cfg.aero
                           ? a_drive - drag_decel(v_fwd[i - 1], rho, CdA, mass)
                           : a_drive;
        double v2 = v_fwd[i - 1] * v_fwd[i - 1] + 2.0 * a_net * sd.ds;
        v_fwd[i] = std::min(sd.v_cap[i], std::sqrt(std::max(0.0, v2)));
    }
    return v_fwd;
}

std::vector<double> backward_pass(const StationData& sd,
                                  const Vehicle& veh,
                                  const PhysicsConfig& cfg) {
    std::vector<double> v_bwd(sd.num_pts);
    v_bwd[sd.num_pts - 1] = sd.v_cap[sd.num_pts - 1];
    const double rho  = veh.aero.rho();
    const double CdA  = veh.aero.cda();
    const double mass = veh.mass_kg;

    for (std::size_t i = sd.num_pts - 1; i > 0; --i) {
        double a_drive = compute_a_drive(v_bwd[i], sd.abs_kappa[i], veh, cfg);
        double a_total = cfg.aero
                             ? a_drive + drag_decel(v_bwd[i], rho, CdA, mass)
                             : a_drive;
        double v_cand = std::sqrt(v_bwd[i] * v_bwd[i] + 2.0 * a_total * sd.ds);
        v_bwd[i - 1] = std::min(sd.v_cap[i - 1], v_cand);
    }
    return v_bwd;
}

// ── Time integration (trapezoidal) ────────────────────────────────────

struct TimeProfile {
    std::vector<double> t;
    std::vector<double> dt_vec;
    double total_time;
};

TimeProfile integrate_time(const std::vector<double>& v,
                           std::size_t N_steps, double ds) {
    constexpr double kVEps = 1e-6;
    TimeProfile tp;
    tp.dt_vec.resize(N_steps);
    tp.t.resize(N_steps + 1, 0.0);

    for (std::size_t i = 0; i < N_steps; ++i) {
        double v_avg = std::max((v[i] + v[i + 1]) / 2.0, kVEps);
        tp.dt_vec[i] = ds / v_avg;
        tp.t[i + 1] = tp.t[i] + tp.dt_vec[i];
    }
    tp.total_time = tp.t[N_steps];
    return tp;
}

// ── Segment extraction (boundary-interpolated) ───────────────────────

struct SegmentExtraction {
    std::vector<double> v_entry;
    std::vector<double> v_exit;
    std::vector<double> v_peak;
    std::vector<double> seg_time;
};

SegmentExtraction extract_segments(const std::vector<double>& v,
                                   const TimeProfile& tp,
                                   const StationData& sd,
                                   const Track& track) {
    const std::size_t num_segs = track.segment_count();
    const double total_length = track.total_length();

    auto interp_v = [&](double s_target) -> double {
        if (s_target <= 0.0) return v[0];
        if (s_target >= total_length) return v[sd.N_steps];
        auto idx = static_cast<std::size_t>(s_target / sd.ds);
        if (idx >= sd.N_steps) idx = sd.N_steps - 1;
        double s_idx = static_cast<double>(idx) * sd.ds;
        double alpha = (s_target - s_idx) / sd.ds;
        return v[idx] + alpha * (v[idx + 1] - v[idx]);
    };

    std::vector<double> seg_start_s(num_segs, 0.0);
    for (std::size_t j = 1; j < num_segs; ++j)
        seg_start_s[j] = sd.seg_end[j - 1];

    SegmentExtraction se;
    se.v_entry.resize(num_segs);
    se.v_exit.resize(num_segs);
    se.v_peak.resize(num_segs, 0.0);
    se.seg_time.resize(num_segs, 0.0);

    se.v_entry[0] = v[0];
    for (std::size_t j = 1; j < num_segs; ++j)
        se.v_entry[j] = interp_v(seg_start_s[j]);
    for (std::size_t j = 0; j + 1 < num_segs; ++j)
        se.v_exit[j] = interp_v(sd.seg_end[j]);
    se.v_exit[num_segs - 1] = v[sd.N_steps];

    // v_peak from discrete samples
    std::vector<bool> has_sample(num_segs, false);
    for (std::size_t i = 0; i < sd.num_pts; ++i) {
        std::size_t j = sd.seg_idx[i];
        se.v_peak[j] = std::max(se.v_peak[j], v[i]);
        has_sample[j] = true;
    }
    for (std::size_t j = 0; j < num_segs; ++j) {
        if (!has_sample[j])
            se.v_peak[j] = std::max(se.v_entry[j], se.v_exit[j]);
    }

    // Per-segment time with boundary splitting
    constexpr double kVEps = 1e-6;
    for (std::size_t i = 0; i < sd.N_steps; ++i) {
        std::size_t k_lo = sd.seg_idx[i];
        std::size_t k_hi = sd.seg_idx[i + 1];

        if (k_lo == k_hi) {
            se.seg_time[k_lo] += tp.dt_vec[i];
        } else {
            double s_lo = static_cast<double>(i) * sd.ds;
            double s_b  = sd.seg_end[k_lo];
            double alpha = (s_b - s_lo) / sd.ds;
            double v_b   = v[i] + alpha * (v[i + 1] - v[i]);
            double v_avg_l = std::max((v[i] + v_b) / 2.0, kVEps);
            double v_avg_r = std::max((v_b + v[i + 1]) / 2.0, kVEps);
            double dt_left  = (alpha * sd.ds) / v_avg_l;
            double dt_right = ((1.0 - alpha) * sd.ds) / v_avg_r;
            double scale = tp.dt_vec[i] / (dt_left + dt_right);
            se.seg_time[k_lo] += dt_left  * scale;
            se.seg_time[k_hi] += dt_right * scale;
        }
    }
    return se;
}

// ── Build segment results into Telemetry ──────────────────────────────

void build_segment_results(Telemetry& telem, const Track& track,
                           const SegmentExtraction& se) {
    const std::size_t num_segs = track.segment_count();
    for (std::size_t j = 0; j < num_segs; ++j) {
        const auto& seg = track.segment(j);
        SegmentResult sr;
        sr.id      = seg.id();
        sr.length  = seg.length();
        sr.type    = (seg.type_name() == "arc") ? SegmentType::arc : SegmentType::straight;
        sr.v_entry = se.v_entry[j];
        sr.v_exit  = se.v_exit[j];
        sr.v_peak  = se.v_peak[j];
        sr.time    = se.seg_time[j];
        telem.add_segment_result(sr);
    }
}

// ── Populate per-sample telemetry ─────────────────────────────────────

void populate_samples(Telemetry& telem, const Track& track,
                      const std::vector<double>& v,
                      const TimeProfile& tp, const StationData& sd,
                      const Vehicle& veh, const PhysicsConfig& cfg) {
    const double total_length = track.total_length();
    const double g = veh.g_mps2;

    for (std::size_t i = 0; i < sd.num_pts; ++i) {
        double s_i = std::min(static_cast<double>(i) * sd.ds, total_length);
        auto pos   = track.position(s_i);
        double hdg = track.heading(s_i);
        std::size_t j = sd.seg_idx[i];

        auto curvature_at = [&](double s, std::size_t sj) -> double {
            double seg_start = (sj == 0) ? 0.0 : sd.seg_end[sj - 1];
            double local_s = std::clamp(s - seg_start, 0.0,
                                        track.segment(sj).length());
            return track.segment(sj).curvature(local_s);
        };
        double kappa = curvature_at(s_i, j);

        double a_long_mps2 = 0.0;
        if (i < sd.N_steps) {
            a_long_mps2 = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * sd.ds);
        } else if (sd.N_steps > 0) {
            a_long_mps2 = (v[sd.N_steps] * v[sd.N_steps]
                           - v[sd.N_steps - 1] * v[sd.N_steps - 1]) / (2.0 * sd.ds);
        }

        double a_lat_mps2 = v[i] * v[i] * std::abs(kappa);

        TelemetrySample samp;
        samp.distance_m     = s_i;
        samp.time_s         = tp.t[i];
        samp.x              = pos.x;
        samp.y              = pos.y;
        samp.heading_rad    = hdg;
        samp.speed_mps      = v[i];
        samp.accel_g        = a_long_mps2 / g;
        samp.lat_accel_g    = a_lat_mps2 / g;
        samp.g_total        = std::sqrt(samp.accel_g * samp.accel_g +
                                        samp.lat_accel_g * samp.lat_accel_g);
        samp.sector         = j;
        samp.type           = (track.segment(j).type_name() == "arc")
                                  ? SegmentType::arc : SegmentType::straight;

        if (cfg.aero) {
            samp.F_drag_N       = veh.aero.drag_force(v[i]);
            samp.F_downforce_N  = veh.aero.downforce(v[i]);
            samp.a_lat_max_mps2 = a_lat_max_aero(v[i], veh);
        }

        telem.add_sample(samp);
    }
}

// ── Basic kinematic solve (per-segment) ───────────────────────────────

Telemetry solve_basic(const Track& track, const Vehicle& veh,
                      const PhysicsConfig& cfg) {
    if (cfg.friction_circle || cfg.aero) {
        std::cerr << "LapTimeSolver: WARNING — friction_circle and aero "
                     "require continuous_profile; proceeding with basic-only.\n";
    }

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
                std::cerr << "LapTimeSolver(basic): adjacent corners "
                          << track.segment(i).id() << " and "
                          << track.segment(i + 1).id()
                          << " have mismatched speeds (delta="
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

// ── QSS solve (forward/backward with configurable physics) ────────────

Telemetry solve_qss(const Track& track, const Vehicle& veh,
                    const PhysicsConfig& cfg) {
    const double total_length = track.total_length();
    if (total_length <= 0.0) return Telemetry{};

    auto sd    = discretize(track, cfg, veh);
    auto v_fwd = forward_pass(sd, veh, cfg);
    auto v_bwd = backward_pass(sd, veh, cfg);

    // Combined profile
    std::vector<double> v(sd.num_pts);
    for (std::size_t i = 0; i < sd.num_pts; ++i)
        v[i] = std::min(v_fwd[i], v_bwd[i]);

    auto tp = integrate_time(v, sd.N_steps, sd.ds);
    auto se = extract_segments(v, tp, sd, track);

    // Time conservation check
    {
        double seg_sum = 0.0;
        for (auto t : se.seg_time) seg_sum += t;
        double leak = std::abs(seg_sum - tp.total_time);
        if (leak > 1e-6) {
            std::cerr << "LapTimeSolver: time conservation leak = "
                      << leak << " s\n";
        }
    }

    Telemetry telem;
    build_segment_results(telem, track, se);
    populate_samples(telem, track, v, tp, sd, veh, cfg);
    telem.set_total_lap_time(tp.total_time);
    return telem;
}

} // anonymous namespace

// ── Public entry point ────────────────────────────────────────────────

auto LapTimeSolver::solve(const Track& track, const Vehicle& veh,
                          const PhysicsConfig& cfg) const -> Telemetry {
    if (!cfg.continuous_profile)
        return solve_basic(track, veh, cfg);
    return solve_qss(track, veh, cfg);
}

} // namespace lapsim
