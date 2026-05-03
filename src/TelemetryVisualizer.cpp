#include "lapsim/TelemetryVisualizer.hpp"
#include "lapsim/Telemetry.hpp"
#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <numbers>
#include <string>
#include <vector>

// ── Color palette ───────────────────────────────────────────────────────────
namespace {

constexpr Color kBgDark      = {15, 15, 20, 255};
constexpr Color kPanelBg     = {25, 25, 35, 255};
constexpr Color kPanelBorder = {50, 50, 70, 255};
constexpr Color kTrackLine   = {100, 100, 120, 255};
constexpr Color kBoundaryLine = {80, 80, 90, 255};
constexpr Color kRacingLine   = {255, 140, 0, 255};
constexpr Color kAccelColor  = {60, 220, 80, 255};
constexpr Color kBrakeColor  = {230, 60, 60, 255};
constexpr Color kCornerColor = {240, 210, 50, 255};
constexpr Color kCoastColor  = {200, 200, 200, 255};
constexpr Color kCarColor      = {60, 200, 255, 255};
constexpr Color kGhostColor    = {255, 165, 60, 220};
constexpr Color kGhostTrail    = {255, 165, 60, 80};
constexpr Color kGhostTrace    = {255, 165, 60, 180};
constexpr Color kGhostOutline  = {30, 30, 40, 255};
constexpr Color kGridDim       = {40, 40, 55, 255};
constexpr Color kTextLight     = {220, 220, 230, 255};
constexpr Color kTextDim       = {140, 140, 160, 255};
constexpr Color kEllipseYellow = {240, 210, 50, 100};
constexpr Color kPosMarker     = {255, 150, 30, 255};
constexpr Color kDeltaGreen    = {60, 220, 80, 255};
constexpr Color kDeltaRed      = {230, 60, 60, 255};

using lapsim::DriverState;
using lapsim::TelemetrySample;

auto classify_sample(const TelemetrySample& s) -> DriverState {
    constexpr double kG = 9.81;
    constexpr double kThresh = 0.5;
    double a_long = s.accel_g * kG;
    double a_lat  = std::abs(s.lat_accel_g) * kG;
    if (a_long < -kThresh) return DriverState::BRAKING;
    if (a_long >  kThresh) return DriverState::ACCEL;
    if (a_lat  >  kThresh) return DriverState::CORNERING;
    return DriverState::COAST;
}

auto state_color(DriverState ds) -> Color {
    switch (ds) {
        case DriverState::ACCEL:     return kAccelColor;
        case DriverState::BRAKING:   return kBrakeColor;
        case DriverState::CORNERING: return kCornerColor;
        case DriverState::COAST:     return kCoastColor;
    }
    return kCoastColor;
}

auto state_name(DriverState ds) -> const char* {
    switch (ds) {
        case DriverState::ACCEL:     return "ACCEL";
        case DriverState::BRAKING:   return "BRAKING";
        case DriverState::CORNERING: return "CORNERING";
        case DriverState::COAST:     return "COAST";
    }
    return "COAST";
}

struct WPoint { float wx; float wy; };

} // anonymous namespace

// ── Constructor ─────────────────────────────────────────────────────────────
namespace lapsim {

TelemetryVisualizer::TelemetryVisualizer(const Track& track,
                                         const TelemetryReader& primary,
                                         const TelemetryReader* ghost,
                                         const RacingLine* racing_line,
                                         const Config& cfg)
    : track_{track}, primary_{primary}, ghost_{ghost},
      racing_line_{racing_line}, cfg_{cfg} {}

void TelemetryVisualizer::run() {
    // ── Precompute track polyline (same tessellation as RaylibVisualizer) ──
    std::vector<WPoint> polyline;
    std::vector<std::size_t> seg_starts;

    for (std::size_t i = 0; i < track_.segment_count(); ++i) {
        const auto& seg = track_.segment(i);
        seg_starts.push_back(polyline.size());

        int n_pts;
        if (std::abs(seg.curvature(0.0)) < 1e-9) {
            n_pts = 2;
        } else {
            double swept = seg.length() * std::abs(seg.curvature(0.0));
            n_pts = std::max(8, static_cast<int>(32.0 * swept / (2.0 * std::numbers::pi)));
        }

        for (int j = 0; j <= n_pts; ++j) {
            double s = seg.length() * static_cast<double>(j) / static_cast<double>(n_pts);
            auto p = seg.position(s);
            polyline.push_back({static_cast<float>(p.x), static_cast<float>(p.y)});
        }
    }

    // ── Precompute boundary polylines ──
    std::vector<WPoint> left_boundary;
    std::vector<WPoint> right_boundary;

    for (std::size_t i = 0; i < track_.segment_count(); ++i) {
        const auto& seg = track_.segment(i);
        int n_pts;
        if (std::abs(seg.curvature(0.0)) < 1e-9) {
            n_pts = 2;
        } else {
            double swept = seg.length() * std::abs(seg.curvature(0.0));
            n_pts = std::max(8, static_cast<int>(32.0 * swept / (2.0 * std::numbers::pi)));
        }

        for (int j = 0; j <= n_pts; ++j) {
            double s = seg.length() * static_cast<double>(j) / static_cast<double>(n_pts);
            auto lp = seg.left_boundary_point(s, track_.width());
            auto rp = seg.right_boundary_point(s, track_.width());
            left_boundary.push_back({static_cast<float>(lp.x), static_cast<float>(lp.y)});
            right_boundary.push_back({static_cast<float>(rp.x), static_cast<float>(rp.y)});
        }
    }

    // Track bounding box — include boundaries for correct framing
    float wmin_x =  1e30f, wmax_x = -1e30f;
    float wmin_y =  1e30f, wmax_y = -1e30f;
    auto update_bounds = [&](const std::vector<WPoint>& pts) {
        for (const auto& p : pts) {
            wmin_x = std::min(wmin_x, p.wx);
            wmax_x = std::max(wmax_x, p.wx);
            wmin_y = std::min(wmin_y, p.wy);
            wmax_y = std::max(wmax_y, p.wy);
        }
    };
    update_bounds(polyline);
    update_bounds(left_boundary);
    update_bounds(right_boundary);

    // ── Precompute racing line polyline (uniform 0.5 m sampling along
    // centerline arc-length, same density as the offset profile). ──
    std::vector<WPoint> racing_polyline;
    if (racing_line_) {
        constexpr double kRacingDs = 0.5;
        const double L = racing_line_->track_length();
        std::size_t n = static_cast<std::size_t>(std::ceil(L / kRacingDs)) + 1;
        racing_polyline.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            double s = std::min(static_cast<double>(i) * kRacingDs, L);
            auto p = racing_line_->position(s);
            racing_polyline.push_back({static_cast<float>(p.x), static_cast<float>(p.y)});
        }
        update_bounds(racing_polyline);
    }

    float wrange_x = std::max(wmax_x - wmin_x, 1.0f);
    float wrange_y = std::max(wmax_y - wmin_y, 1.0f);

    // Find total distance and max speed from primary samples
    double total_dist = 0.0;
    double max_speed  = 0.0;
    for (const auto& s : primary_.samples()) {
        total_dist = std::max(total_dist, s.distance_m);
        max_speed  = std::max(max_speed,  s.speed_mps);
    }
    if (max_speed < 1.0) max_speed = 30.0;

    // Collect segment boundary distances for speed-trace markers
    std::vector<double> seg_boundary_dists;
    std::vector<std::string> seg_boundary_ids;
    {
        double accum = 0.0;
        for (std::size_t i = 0; i < track_.segment_count(); ++i) {
            seg_boundary_dists.push_back(accum);
            seg_boundary_ids.push_back(track_.segment(i).id());
            accum += track_.segment(i).length();
        }
    }

    // ── State variables ─────────────────────────────────────────────────
    double sim_time       = 0.0;
    double playback_speed = cfg_.initial_playback_speed;
    bool   paused         = true;
    int    last_seg_idx   = -1;
    std::vector<double> split_times;
    double peak_g_lat  = 0.0;
    double peak_g_long = 0.0;

    // ── Window init ─────────────────────────────────────────────────────
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(cfg_.window_width, cfg_.window_height,
               "FSAE Lap Sim - Telemetry Visualizer");
    SetTargetFPS(cfg_.target_fps);

    while (!WindowShouldClose()) {
        // ════════════════════════════════════════════════════════════════
        // INPUT
        // ════════════════════════════════════════════════════════════════
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;

        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD))
            playback_speed = std::min(playback_speed * 2.0, 8.0);
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT))
            playback_speed = std::max(playback_speed / 2.0, 0.125);

        if (IsKeyPressed(KEY_R)) {
            sim_time = 0.0;
            last_seg_idx = -1;
            split_times.clear();
            peak_g_lat  = 0.0;
            peak_g_long = 0.0;
        }

        if (IsKeyPressed(KEY_G) && ghost_)
            cfg_.show_ghost = !cfg_.show_ghost;

        if (IsKeyPressed(KEY_S)) {
            std::filesystem::create_directories("output/screenshots");
            char fname[128];
            std::snprintf(fname, sizeof(fname), "output/screenshots/lap_%04d.png",
                          static_cast<int>(sim_time * 100));
            TakeScreenshot(fname);
        }

        if (paused && primary_.sample_count() > 0) {
            if (IsKeyPressed(KEY_RIGHT)) {
                const auto& samps = primary_.samples();
                for (std::size_t i = 0; i < samps.size() - 1; ++i) {
                    if (samps[i].time_s >= sim_time) {
                        sim_time = samps[std::min(i + 1, samps.size() - 1)].time_s;
                        break;
                    }
                }
            }
            if (IsKeyPressed(KEY_LEFT)) {
                const auto& samps = primary_.samples();
                for (std::size_t i = samps.size(); i > 0; --i) {
                    if (samps[i - 1].time_s < sim_time - 1e-9) {
                        sim_time = samps[i - 1].time_s;
                        break;
                    }
                }
            }
        }

        // ════════════════════════════════════════════════════════════════
        // UPDATE
        // ════════════════════════════════════════════════════════════════
        if (!paused) {
            sim_time += static_cast<double>(GetFrameTime()) * playback_speed;
            if (sim_time > primary_.total_time_s())
                sim_time = primary_.total_time_s();
        }

        auto cur = primary_.at_time(sim_time);

        peak_g_lat  = std::max(peak_g_lat,  std::abs(cur.lat_accel_g));
        peak_g_long = std::max(peak_g_long, std::abs(cur.accel_g));

        int cur_seg = static_cast<int>(cur.sector);
        if (cur_seg != last_seg_idx && last_seg_idx >= 0)
            split_times.push_back(sim_time);
        last_seg_idx = cur_seg;

        DriverState cur_state = classify_sample(cur);

        // ════════════════════════════════════════════════════════════════
        // LAYOUT (responsive to current window size)
        // ════════════════════════════════════════════════════════════════
        int W = GetScreenWidth();
        int H = GetScreenHeight();

        int track_w = static_cast<int>(W * 0.6875);
        int track_h = static_cast<int>(H * 0.8);
        int hud_w   = W - track_w;
        int hud_h   = static_cast<int>(H * 0.478);
        int gg_w    = hud_w;
        int gg_h    = H - hud_h - (H - track_h);
        int trace_h = H - track_h;

        // Coordinate transforms for track panel (fit polyline into panel)
        constexpr float kTrackMargin = 60.0f;
        float tscale = std::min((static_cast<float>(track_w) - 2.0f * kTrackMargin) / wrange_x,
                                (static_cast<float>(track_h) - 2.0f * kTrackMargin) / wrange_y);
        float toff_x = (static_cast<float>(track_w) - wrange_x * tscale) * 0.5f;
        float toff_y = (static_cast<float>(track_h) - wrange_y * tscale) * 0.5f;

        auto to_sx = [&](float wx) -> float {
            return (wx - wmin_x) * tscale + toff_x;
        };
        auto to_sy = [&](float wy) -> float {
            return static_cast<float>(track_h) - ((wy - wmin_y) * tscale + toff_y);
        };

        // ════════════════════════════════════════════════════════════════
        // DRAW
        // ════════════════════════════════════════════════════════════════
        BeginDrawing();
        ClearBackground(kBgDark);

        // ── Track Panel ─────────────────────────────────────────────────
        DrawRectangle(0, 0, track_w, track_h, kPanelBg);
        DrawRectangleLines(0, 0, track_w, track_h, kPanelBorder);

        // Track boundaries (drawn first so centerline is on top)
        for (std::size_t i = 1; i < left_boundary.size(); ++i) {
            DrawLineEx(
                {to_sx(left_boundary[i - 1].wx), to_sy(left_boundary[i - 1].wy)},
                {to_sx(left_boundary[i].wx),     to_sy(left_boundary[i].wy)},
                1.5f, kBoundaryLine);
        }
        for (std::size_t i = 1; i < right_boundary.size(); ++i) {
            DrawLineEx(
                {to_sx(right_boundary[i - 1].wx), to_sy(right_boundary[i - 1].wy)},
                {to_sx(right_boundary[i].wx),     to_sy(right_boundary[i].wy)},
                1.5f, kBoundaryLine);
        }

        // Track centerline
        for (std::size_t i = 1; i < polyline.size(); ++i) {
            DrawLineEx(
                {to_sx(polyline[i - 1].wx), to_sy(polyline[i - 1].wy)},
                {to_sx(polyline[i].wx),     to_sy(polyline[i].wy)},
                3.0f, kTrackLine);
        }

        // Racing line (drawn after centerline so it sits on top)
        for (std::size_t i = 1; i < racing_polyline.size(); ++i) {
            DrawLineEx(
                {to_sx(racing_polyline[i - 1].wx), to_sy(racing_polyline[i - 1].wy)},
                {to_sx(racing_polyline[i].wx),     to_sy(racing_polyline[i].wy)},
                2.5f, kRacingLine);
        }

        // Segment boundary markers (faint tick marks along track)
        for (std::size_t i = 0; i < seg_starts.size(); ++i) {
            std::size_t idx = seg_starts[i];
            if (idx < polyline.size()) {
                float sx = to_sx(polyline[idx].wx);
                float sy = to_sy(polyline[idx].wy);
                DrawCircle(static_cast<int>(sx), static_cast<int>(sy), 3.0f, kPanelBorder);
            }
        }

        // Start marker (green circle + arrow)
        if (track_.segment_count() > 0) {
            auto sp = track_.segment(0).start_point();
            double sh = track_.segment(0).start_heading();
            float sx = to_sx(static_cast<float>(sp.x));
            float sy = to_sy(static_cast<float>(sp.y));

            DrawCircle(static_cast<int>(sx), static_cast<int>(sy), 7.0f, GREEN);

            constexpr float kArrowPx = 24.0f;
            float ex = sx + kArrowPx * static_cast<float>(std::cos(sh));
            float ey = sy - kArrowPx * static_cast<float>(std::sin(sh));
            DrawLineEx({sx, sy}, {ex, ey}, 2.0f, GREEN);
            constexpr float kHead = 7.0f;
            float a1 = static_cast<float>(sh) + 2.6f;
            float a2 = static_cast<float>(sh) - 2.6f;
            DrawLine(static_cast<int>(ex), static_cast<int>(ey),
                     static_cast<int>(ex + kHead * std::cos(a1)),
                     static_cast<int>(ey - kHead * std::sin(a1)), GREEN);
            DrawLine(static_cast<int>(ex), static_cast<int>(ey),
                     static_cast<int>(ex + kHead * std::cos(a2)),
                     static_cast<int>(ey - kHead * std::sin(a2)), GREEN);
        }

        // Colored trail: iterate samples from 0 to sim_time
        for (const auto& s : primary_.samples()) {
            if (s.time_s > sim_time) break;
            float sx = to_sx(static_cast<float>(s.x));
            float sy = to_sy(static_cast<float>(s.y));
            Color c = state_color(classify_sample(s));
            DrawCircleV({sx, sy}, 2.0f, c);
        }

        // Ghost trail (amber wake)
        if (cfg_.show_ghost && ghost_) {
            for (const auto& s : ghost_->samples()) {
                if (s.time_s > sim_time) break;
                float sx = to_sx(static_cast<float>(s.x));
                float sy = to_sy(static_cast<float>(s.y));
                DrawCircleV({sx, sy}, 2.0f, kGhostTrail);
            }
        }

        // Car marker (triangle oriented by heading)
        {
            float cx = to_sx(static_cast<float>(cur.x));
            float cy = to_sy(static_cast<float>(cur.y));
            float h  = static_cast<float>(cur.heading_rad);
            constexpr float kSize = 12.0f;

            Vector2 tip  = {cx + kSize * std::cos(h),
                            cy - kSize * std::sin(h)};
            Vector2 left = {cx + kSize * 0.5f * std::cos(h + 2.4f),
                            cy - kSize * 0.5f * std::sin(h + 2.4f)};
            Vector2 right= {cx + kSize * 0.5f * std::cos(h - 2.4f),
                            cy - kSize * 0.5f * std::sin(h - 2.4f)};
            DrawTriangle(tip, left, right, kCarColor);
        }

        // Ghost car marker (same size as primary, amber, with dark outline)
        if (cfg_.show_ghost && ghost_) {
            auto gs = ghost_->at_time(sim_time);
            float gx = to_sx(static_cast<float>(gs.x));
            float gy = to_sy(static_cast<float>(gs.y));
            float gh = static_cast<float>(gs.heading_rad);
            constexpr float kGSize = 12.0f;
            constexpr float kOutline = 14.0f;

            auto make_tri = [](float cx, float cy, float heading, float sz) {
                Vector2 t = {cx + sz * std::cos(heading),
                             cy - sz * std::sin(heading)};
                Vector2 l = {cx + sz * 0.5f * std::cos(heading + 2.4f),
                             cy - sz * 0.5f * std::sin(heading + 2.4f)};
                Vector2 r = {cx + sz * 0.5f * std::cos(heading - 2.4f),
                             cy - sz * 0.5f * std::sin(heading - 2.4f)};
                return std::array<Vector2, 3>{t, l, r};
            };

            auto outline = make_tri(gx, gy, gh, kOutline);
            DrawTriangle(outline[0], outline[1], outline[2], kGhostOutline);
            auto inner = make_tri(gx, gy, gh, kGSize);
            DrawTriangle(inner[0], inner[1], inner[2], kGhostColor);
        }

        // Panel title
        {
            char title[160];
            if (racing_line_) {
                std::snprintf(title, sizeof(title),
                              "TRACK VIEW | width: %.1f m | length: %.1f m | racing line: ON",
                              track_.width(), track_.total_length());
            } else {
                std::snprintf(title, sizeof(title),
                              "TRACK VIEW | width: %.1f m | length: %.1f m",
                              track_.width(), track_.total_length());
            }
            DrawText(title, 10, 8, 14, kTextLight);
        }
        {
            char tname[128];
            std::snprintf(tname, sizeof(tname), "%s", track_.name().c_str());
            DrawText(tname, 10, 28, 12, kTextDim);
        }

        // Segment labels at midpoints
        for (std::size_t i = 0; i < track_.segment_count(); ++i) {
            const auto& seg = track_.segment(i);
            auto mid = seg.position(seg.length() * 0.5);
            float sx = to_sx(static_cast<float>(mid.x));
            float sy = to_sy(static_cast<float>(mid.y));
            DrawText(seg.id().c_str(),
                     static_cast<int>(sx) + 6, static_cast<int>(sy) - 6,
                     10, kTextDim);
        }

        // ── HUD Panel ───────────────────────────────────────────────────
        int hud_x = track_w;
        int hud_y = 0;
        DrawRectangle(hud_x, hud_y, hud_w, hud_h, kPanelBg);
        DrawRectangleLines(hud_x, hud_y, hud_w, hud_h, kPanelBorder);

        {
            char label[256];
            std::snprintf(label, sizeof(label), "PRIMARY: %s", cfg_.primary_label.c_str());
            DrawText(label, hud_x + 12, hud_y + 8, 14, kCarColor);
        }
        if (cfg_.show_ghost && ghost_) {
            double delta = ghost_->total_time_s() - primary_.total_time_s();
            Color delta_color = (delta >= 0) ? kDeltaGreen : kDeltaRed;
            const char* suffix = (delta >= 0) ? "(primary faster)" : "(ghost faster)";

            char label[256];
            std::snprintf(label, sizeof(label), "GHOST: %s  [%+.3f s]",
                          cfg_.ghost_label.c_str(), delta);
            DrawText(label, hud_x + 12, hud_y + 26, 14, kGhostColor);

            int delta_x = hud_x + 12 + MeasureText(label, 14) + 6;
            char delta_buf[64];
            std::snprintf(delta_buf, sizeof(delta_buf), "%s", suffix);
            DrawText(delta_buf, delta_x, hud_y + 30, 10, delta_color);
        }

        int row = hud_y + (cfg_.show_ghost && ghost_ ? 50 : 32);
        constexpr int kRowH = 22;
        int col1 = hud_x + 14;
        int col2 = hud_x + 120;

        // Time
        {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.3f s", sim_time);
            DrawText("Time:", col1, row, 14, kTextDim);
            DrawText(buf, col2, row, 14, kTextLight);
            row += kRowH;
        }

        // Speed
        {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.1f m/s  (%.1f km/h)",
                          cur.speed_mps, cur.speed_mps * 3.6);
            DrawText("Speed:", col1, row, 14, kTextDim);
            DrawText(buf, col2, row, 14, kTextLight);
            row += kRowH;
        }

        // Distance
        {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.1f m", cur.distance_m);
            DrawText("Distance:", col1, row, 14, kTextDim);
            DrawText(buf, col2, row, 14, kTextLight);
            row += kRowH;
        }

        // Segment
        {
            DrawText("Segment:", col1, row, 14, kTextDim);
            DrawText(cur.segment_id.c_str(), col2, row, 14, kTextLight);
            row += kRowH;
        }

        // Driver state
        {
            DrawText("State:", col1, row, 14, kTextDim);
            DrawText(state_name(cur_state), col2, row, 14, state_color(cur_state));
            row += kRowH + 6;
        }

        // G-force readouts
        {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Long: %+.2f g", cur.accel_g);
            DrawText(buf, col1, row, 13, kTextLight);
            row += kRowH - 2;

            std::snprintf(buf, sizeof(buf), "Lat:  %+.2f g", cur.lat_accel_g);
            DrawText(buf, col1, row, 13, kTextLight);
            row += kRowH - 2;

            std::snprintf(buf, sizeof(buf), "Total: %.2f g", cur.g_total);
            DrawText(buf, col1, row, 13, kTextLight);
            row += kRowH + 4;
        }

        // G-force bar: longitudinal
        {
            int bar_cx = hud_x + hud_w / 2;
            int bar_w  = hud_w - 40;
            int bar_h  = 10;
            DrawRectangle(hud_x + 20, row, bar_w, bar_h, kGridDim);

            float frac = static_cast<float>(cur.accel_g) / 2.0f;
            frac = std::clamp(frac, -1.0f, 1.0f);
            Color bc = (frac >= 0) ? kAccelColor : kBrakeColor;
            int bw = static_cast<int>(std::abs(frac) * (bar_w / 2));
            if (frac >= 0)
                DrawRectangle(bar_cx, row, bw, bar_h, bc);
            else
                DrawRectangle(bar_cx - bw, row, bw, bar_h, bc);
            DrawText("Longitudinal", col1, row - 14, 10, kTextDim);
            row += bar_h + 18;
        }

        // G-force bar: lateral
        {
            int bar_cx = hud_x + hud_w / 2;
            int bar_w  = hud_w - 40;
            int bar_h  = 10;
            DrawRectangle(hud_x + 20, row, bar_w, bar_h, kGridDim);

            float frac = static_cast<float>(cur.lat_accel_g) / 2.0f;
            frac = std::clamp(frac, -1.0f, 1.0f);
            int bw = static_cast<int>(std::abs(frac) * (bar_w / 2));
            if (frac >= 0)
                DrawRectangle(bar_cx, row, bw, bar_h, kCornerColor);
            else
                DrawRectangle(bar_cx - bw, row, bw, bar_h, kCornerColor);
            DrawText("Lateral", col1, row - 14, 10, kTextDim);
            row += bar_h + 18;
        }

        // G-force bar: total
        {
            int bar_w = hud_w - 40;
            int bar_h = 10;
            DrawRectangle(hud_x + 20, row, bar_w, bar_h, kGridDim);

            float frac = static_cast<float>(cur.g_total) / 2.0f;
            frac = std::clamp(frac, 0.0f, 1.0f);
            int bw = static_cast<int>(frac * bar_w);
            DrawRectangle(hud_x + 20, row, bw, bar_h, kCarColor);
            DrawText("Total G", col1, row - 14, 10, kTextDim);
            row += bar_h + 18;
        }

        // Lap progress bar
        {
            double pct = (primary_.total_time_s() > 0)
                         ? sim_time / primary_.total_time_s()
                         : 0.0;
            int bar_w = hud_w - 40;
            int bar_h = 8;
            DrawRectangle(hud_x + 20, row, bar_w, bar_h, kGridDim);
            int filled = static_cast<int>(pct * bar_w);
            DrawRectangle(hud_x + 20, row, filled, bar_h, kCarColor);

            char buf[32];
            std::snprintf(buf, sizeof(buf), "Lap: %.1f%%", pct * 100.0);
            DrawText(buf, col1, row - 14, 10, kTextDim);
            row += bar_h + 14;
        }

        // Peak g values
        {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Peak Long: %.2f g  Lat: %.2f g",
                          peak_g_long, peak_g_lat);
            DrawText(buf, col1, row, 10, kTextDim);
            row += 16;
        }

        // Ghost speed readout
        if (cfg_.show_ghost && ghost_) {
            auto gs = ghost_->at_time(sim_time);
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Ghost: %.1f m/s (%.1f km/h)",
                          gs.speed_mps, gs.speed_mps * 3.6);
            Color ghost_text = kGhostColor;
            ghost_text.a = 255;
            DrawText(buf, col1, row, 11, ghost_text);
        }

        // ── G-G Diagram Panel ───────────────────────────────────────────
        int gg_x = track_w;
        int gg_y = hud_h;
        DrawRectangle(gg_x, gg_y, gg_w, gg_h, kPanelBg);
        DrawRectangleLines(gg_x, gg_y, gg_w, gg_h, kPanelBorder);

        DrawText("G-G DIAGRAM", gg_x + 12, gg_y + 8, 14, kTextLight);

        constexpr int kGGPad = 40;
        int plot_size = std::min(gg_w, gg_h) - 2 * kGGPad;
        if (plot_size < 20) plot_size = 20;
        int plot_cx = gg_x + gg_w / 2;
        int plot_cy = gg_y + kGGPad + plot_size / 2 + 10;
        float half = static_cast<float>(plot_size) / 2.0f;

        // Grid lines every 0.5g within [-2, 2]
        for (int gi = -4; gi <= 4; ++gi) {
            float frac = static_cast<float>(gi) / 4.0f;
            int gx = plot_cx + static_cast<int>(frac * half);
            int gy = plot_cy - static_cast<int>(frac * half);
            DrawLine(gx, plot_cy - static_cast<int>(half),
                     gx, plot_cy + static_cast<int>(half), kGridDim);
            DrawLine(plot_cx - static_cast<int>(half), gy,
                     plot_cx + static_cast<int>(half), gy, kGridDim);
        }

        // Axis lines through origin
        DrawLine(plot_cx, plot_cy - static_cast<int>(half),
                 plot_cx, plot_cy + static_cast<int>(half), kPanelBorder);
        DrawLine(plot_cx - static_cast<int>(half), plot_cy,
                 plot_cx + static_cast<int>(half), plot_cy, kPanelBorder);

        // Friction ellipse (unit circle at 1g radius)
        constexpr int kEllipseSteps = 64;
        for (int ei = 0; ei < kEllipseSteps; ++ei) {
            float th0 = 2.0f * static_cast<float>(std::numbers::pi) *
                        static_cast<float>(ei) / static_cast<float>(kEllipseSteps);
            float th1 = 2.0f * static_cast<float>(std::numbers::pi) *
                        static_cast<float>(ei + 1) / static_cast<float>(kEllipseSteps);
            float r = half / 2.0f;  // 1g maps to half the plot radius (since range is 2g)
            float x0 = plot_cx + r * std::cos(th0);
            float y0 = plot_cy - r * std::sin(th0);
            float x1 = plot_cx + r * std::cos(th1);
            float y1 = plot_cy - r * std::sin(th1);
            DrawLineEx({x0, y0}, {x1, y1}, 1.5f, kEllipseYellow);
        }

        // Trailing dots (last ~120 samples before current time)
        {
            const auto& samps = primary_.samples();
            std::size_t start_i = 0;
            std::size_t end_i   = 0;
            for (std::size_t si = 0; si < samps.size(); ++si) {
                if (samps[si].time_s <= sim_time) end_i = si;
            }
            start_i = (end_i > 120) ? end_i - 120 : 0;

            for (std::size_t si = start_i; si <= end_i && si < samps.size(); ++si) {
                float lat_frac  = static_cast<float>(samps[si].lat_accel_g) / 2.0f;
                float long_frac = static_cast<float>(samps[si].accel_g)     / 2.0f;
                float dx = plot_cx + lat_frac * half;
                float dy = plot_cy - long_frac * half;

                float age = static_cast<float>(end_i - si) / 120.0f;
                unsigned char alpha = static_cast<unsigned char>(255.0f * (1.0f - age * 0.7f));
                Color dc = state_color(classify_sample(samps[si]));
                dc.a = alpha;
                DrawCircleV({dx, dy}, 2.0f, dc);
            }
        }

        // Current sample bright dot
        {
            float lat_frac  = static_cast<float>(cur.lat_accel_g) / 2.0f;
            float long_frac = static_cast<float>(cur.accel_g)     / 2.0f;
            float dx = plot_cx + lat_frac * half;
            float dy = plot_cy - long_frac * half;
            DrawCircleV({dx, dy}, 5.0f, kCarColor);
        }

        // Ghost current sample dot (amber, same size)
        if (cfg_.show_ghost && ghost_) {
            auto gs = ghost_->at_time(sim_time);
            float glat  = static_cast<float>(gs.lat_accel_g) / 2.0f;
            float glong = static_cast<float>(gs.accel_g)     / 2.0f;
            float gdx = plot_cx + glat * half;
            float gdy = plot_cy - glong * half;
            DrawCircleV({gdx, gdy}, 5.0f, kGhostColor);
        }

        // Axis labels
        DrawText("Lat [g]", plot_cx + static_cast<int>(half) - 36,
                 plot_cy + static_cast<int>(half) + 4, 10, kTextDim);
        DrawText("Long [g]", plot_cx + 4,
                 plot_cy - static_cast<int>(half) - 14, 10, kTextDim);

        // ── Speed Trace Panel ───────────────────────────────────────────
        int trace_x = 0;
        int trace_y = track_h;
        DrawRectangle(trace_x, trace_y, W, trace_h, kPanelBg);
        DrawRectangleLines(trace_x, trace_y, W, trace_h, kPanelBorder);

        constexpr int kTracePadL = 50;
        constexpr int kTracePadR = 20;
        constexpr int kTracePadT = 22;
        constexpr int kTracePadB = 20;
        int plot_x0 = trace_x + kTracePadL;
        int plot_y0 = trace_y + kTracePadT;
        int plot_w  = W - kTracePadL - kTracePadR;
        int plot_h2 = trace_h - kTracePadT - kTracePadB;
        if (plot_w < 10) plot_w = 10;
        if (plot_h2 < 10) plot_h2 = 10;

        double speed_ceil = max_speed * 1.1;

        auto dist_to_px = [&](double d) -> float {
            return static_cast<float>(plot_x0) +
                   static_cast<float>(d / total_dist) * static_cast<float>(plot_w);
        };
        auto speed_to_py = [&](double v) -> float {
            return static_cast<float>(plot_y0 + plot_h2) -
                   static_cast<float>(v / speed_ceil) * static_cast<float>(plot_h2);
        };

        DrawText("SPEED TRACE", plot_x0, trace_y + 4, 12, kTextLight);

        // Y-axis tick marks
        for (int vi = 0; vi <= 4; ++vi) {
            double v = speed_ceil * static_cast<double>(vi) / 4.0;
            float py = speed_to_py(v);
            DrawLine(plot_x0, static_cast<int>(py),
                     plot_x0 + plot_w, static_cast<int>(py), kGridDim);
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.0f", v);
            DrawText(buf, plot_x0 - 38, static_cast<int>(py) - 5, 10, kTextDim);
        }

        // Ghost speed profile (amber, drawn first so it's behind primary)
        if (cfg_.show_ghost && ghost_ && ghost_->sample_count() > 1) {
            const auto& gs = ghost_->samples();
            for (std::size_t i = 1; i < gs.size(); ++i) {
                float x0g = dist_to_px(gs[i - 1].distance_m);
                float y0g = speed_to_py(gs[i - 1].speed_mps);
                float x1g = dist_to_px(gs[i].distance_m);
                float y1g = speed_to_py(gs[i].speed_mps);
                DrawLineEx({x0g, y0g}, {x1g, y1g}, 2.0f, kGhostTrace);
            }
        }

        // Primary speed profile colored by driver state
        {
            const auto& samps = primary_.samples();
            for (std::size_t i = 1; i < samps.size(); ++i) {
                float x0s = dist_to_px(samps[i - 1].distance_m);
                float y0s = speed_to_py(samps[i - 1].speed_mps);
                float x1s = dist_to_px(samps[i].distance_m);
                float y1s = speed_to_py(samps[i].speed_mps);
                Color lc = state_color(classify_sample(samps[i]));
                lc.a = 200;
                DrawLineEx({x0s, y0s}, {x1s, y1s}, 1.5f, lc);
            }
        }

        // Current position marker (vertical orange line)
        {
            float marker_x = dist_to_px(cur.distance_m);
            DrawLineEx({marker_x, static_cast<float>(plot_y0)},
                       {marker_x, static_cast<float>(plot_y0 + plot_h2)},
                       2.0f, kPosMarker);
        }

        // Segment boundary vertical lines on speed trace
        for (std::size_t i = 0; i < seg_boundary_dists.size(); ++i) {
            float bx = dist_to_px(seg_boundary_dists[i]);
            DrawLine(static_cast<int>(bx), plot_y0,
                     static_cast<int>(bx), plot_y0 + plot_h2, kGridDim);
            if (i < seg_boundary_ids.size()) {
                DrawText(seg_boundary_ids[i].c_str(),
                         static_cast<int>(bx) + 2, plot_y0 + 2, 8, kTextDim);
            }
        }

        // Y-axis label
        DrawText("m/s", plot_x0 - 38, plot_y0 - 14, 10, kTextDim);

        // ── Controls Help Bar ───────────────────────────────────────────
        {
            char controls[256];
            const char* ghost_status = ghost_
                ? ((cfg_.show_ghost) ? "ON (AMBER)" : "OFF")
                : "N/A";
            std::snprintf(controls, sizeof(controls),
                "[Space] %s  [%s/%s] step  [-/+] speed: %.2fx  "
                "[G] ghost: %s  [R] reset  [S] screenshot",
                paused ? "play" : "pause",
                "\xe2\x86\x90", "\xe2\x86\x92",
                playback_speed,
                ghost_status);
            DrawText(controls, 10, track_h - 20, 10, kTextDim);
        }

        EndDrawing();
    }

    CloseWindow();
}

} // namespace lapsim
