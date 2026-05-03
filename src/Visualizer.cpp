#include "lapsim/Visualizer.hpp"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

namespace lapsim {

RaylibVisualizer::RaylibVisualizer(int width, int height)
    : width_{width}, height_{height} {}

void RaylibVisualizer::run(const Track& track,
                           [[maybe_unused]] const Telemetry& telemetry) {
    // ── Precompute polyline in world space ─────────────────────────────
    struct WPoint { float wx; float wy; };
    std::vector<WPoint> polyline;
    std::vector<std::size_t> seg_starts; // index of first point of each segment

    for (std::size_t i = 0; i < track.segment_count(); ++i) {
        const auto& seg = track.segment(i);
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

    // ── Bounding box ──────────────────────────────────────────────────
    float min_x =  std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y =  std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    for (const auto& p : polyline) {
        min_x = std::min(min_x, p.wx);
        max_x = std::max(max_x, p.wx);
        min_y = std::min(min_y, p.wy);
        max_y = std::max(max_y, p.wy);
    }

    float range_x = std::max(max_x - min_x, 1.0f);
    float range_y = std::max(max_y - min_y, 1.0f);

    constexpr float kMargin = 80.0f;
    float scale = std::min((static_cast<float>(width_)  - 2.0f * kMargin) / range_x,
                           (static_cast<float>(height_) - 2.0f * kMargin) / range_y);

    float offset_x = (static_cast<float>(width_)  - range_x * scale) * 0.5f;
    float offset_y = (static_cast<float>(height_) - range_y * scale) * 0.5f;

    auto to_sx = [&](float wx) -> float { return (wx - min_x) * scale + offset_x; };
    auto to_sy = [&](float wy) -> float {
        return static_cast<float>(height_) - ((wy - min_y) * scale + offset_y);
    };

    // ── Window title ──────────────────────────────────────────────────
    auto val = track.validate();
    char title_buf[256];
    std::snprintf(title_buf, sizeof(title_buf), "%s | %.1f m | %s",
                  track.name().c_str(), track.total_length(),
                  val.is_valid ? "PASS" : "FAIL");

    InitWindow(width_, height_, title_buf);
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // ── Axes through world origin ─────────────────────────────────
        Color axis_color = {200, 200, 200, 255};
        float origin_sx = to_sx(0.0f);
        float origin_sy = to_sy(0.0f);
        DrawLine(0, static_cast<int>(origin_sy), width_,
                 static_cast<int>(origin_sy), axis_color);
        DrawLine(static_cast<int>(origin_sx), 0,
                 static_cast<int>(origin_sx), height_, axis_color);

        // ── Scale bar (10 m) ──────────────────────────────────────────
        float bar_px = 10.0f * scale;
        int bar_y = height_ - 30;
        int bar_x0 = 20;
        DrawLine(bar_x0, bar_y, bar_x0 + static_cast<int>(bar_px), bar_y, DARKGRAY);
        DrawLine(bar_x0, bar_y - 4, bar_x0, bar_y + 4, DARKGRAY);
        DrawLine(bar_x0 + static_cast<int>(bar_px), bar_y - 4,
                 bar_x0 + static_cast<int>(bar_px), bar_y + 4, DARKGRAY);
        DrawText("10 m", bar_x0 + static_cast<int>(bar_px) / 2 - 14,
                 bar_y - 18, 14, DARKGRAY);

        // ── Track outline ─────────────────────────────────────────────
        Color track_color = {30, 60, 160, 255};
        for (std::size_t i = 1; i < polyline.size(); ++i) {
            DrawLineEx(
                {to_sx(polyline[i - 1].wx), to_sy(polyline[i - 1].wy)},
                {to_sx(polyline[i].wx), to_sy(polyline[i].wy)},
                2.5f, track_color);
        }

        // ── Segment labels ────────────────────────────────────────────
        for (std::size_t i = 0; i < track.segment_count(); ++i) {
            const auto& seg = track.segment(i);
            auto mid = seg.position(seg.length() * 0.5);
            float sx = to_sx(static_cast<float>(mid.x));
            float sy = to_sy(static_cast<float>(mid.y));
            DrawText(seg.id().c_str(),
                     static_cast<int>(sx) + 6, static_cast<int>(sy) - 6, 12, DARKGRAY);
        }

        // ── Start marker ─────────────────────────────────────────────
        if (track.segment_count() > 0) {
            auto sp = track.segment(0).start_point();
            double sh = track.segment(0).start_heading();
            float sx = to_sx(static_cast<float>(sp.x));
            float sy = to_sy(static_cast<float>(sp.y));

            DrawCircle(static_cast<int>(sx), static_cast<int>(sy), 6.0f, GREEN);
            DrawText("START", static_cast<int>(sx) + 10,
                     static_cast<int>(sy) - 8, 14, GREEN);

            // Arrow in direction of travel
            constexpr float kArrowPx = 28.0f;
            float ex = sx + kArrowPx * static_cast<float>(std::cos(sh));
            float ey = sy - kArrowPx * static_cast<float>(std::sin(sh));
            DrawLineEx({sx, sy}, {ex, ey}, 2.0f, GREEN);
            // Arrowhead
            constexpr float kHead = 8.0f;
            float a1 = static_cast<float>(sh) + 2.6f;
            float a2 = static_cast<float>(sh) - 2.6f;
            DrawLine(static_cast<int>(ex), static_cast<int>(ey),
                     static_cast<int>(ex + kHead * std::cos(a1)),
                     static_cast<int>(ey - kHead * std::sin(a1)), GREEN);
            DrawLine(static_cast<int>(ex), static_cast<int>(ey),
                     static_cast<int>(ex + kHead * std::cos(a2)),
                     static_cast<int>(ey - kHead * std::sin(a2)), GREEN);
        }

        // ── HUD text ─────────────────────────────────────────────────
        DrawText(title_buf, 10, 10, 18, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
}

} // namespace lapsim
