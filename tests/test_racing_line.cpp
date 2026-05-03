#include "lapsim/RacingLine.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/TrackLoader.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

using namespace lapsim;

namespace {

Track load_interview_track() {
    return TrackLoader::load("tracks/interview_track.yaml");
}

// Cumulative arc-length to the end of segment[k].
double end_of_segment(const Track& t, std::size_t k) {
    double s = 0.0;
    for (std::size_t i = 0; i <= k; ++i) s += t.segment(i).length();
    return s;
}

} // namespace

// ── 10.2.1 Defaults are constructible ───────────────────────────────────────

TEST(RacingLineTest, DefaultParamsConstructible) {
    auto track = load_interview_track();
    EXPECT_NO_THROW({
        RacingLine line(track, RacingLineParams{});
        EXPECT_GT(line.track_length(), 0.0);
    });
}

// ── 10.2.2 Stays within track bounds ────────────────────────────────────────

TEST(RacingLineTest, StaysWithinBoundsAllSamples) {
    auto track = load_interview_track();
    RacingLine line(track, {});

    const double half_w = track.width() / 2.0;
    const double L = line.track_length();
    constexpr int N = 200;
    for (int i = 0; i < N; ++i) {
        double s = (static_cast<double>(i) / (N - 1)) * L;
        double off = line.offset(s);
        EXPECT_LE(std::abs(off), half_w + 1e-6)
            << "offset out of corridor at s=" << s << ": " << off;
    }
}

// ── 10.2.3 Zero-offset params reduce to centerline ──────────────────────────
//
// Loads a temp track WITHOUT YAML racing_offset_m fields so this test isolates
// the algorithmic offset path. Phase 10.5 added YAML overrides on S1/S2/S3
// that bypass apex/entry/exit_offset_frac, so the interview track no longer
// reduces to centerline under zero-frac params.

TEST(RacingLineTest, DegenerateAllZeroOffsetMatchesCenterline) {
    auto path = std::filesystem::temp_directory_path() / "test_no_yaml_offset.yaml";
    {
        std::ofstream f(path);
        f << "track:\n"
             "  name: \"degenerate test\"\n"
             "  width_m: 4.0\n"
             "  start_point: [0.0, 0.0]\n"
             "  start_heading: 0.0\n"
             "  closed: false\n"
             "  segments:\n"
             "    - id: \"S\"\n      type: straight\n      length: 50.0\n"
             "    - id: \"A\"\n      type: arc\n      radius: 15.0\n      swept_angle: 1.5707963267948966\n"
             "    - id: \"S2\"\n      type: straight\n      length: 30.0\n";
    }
    Track track = TrackLoader::load(path.string());

    RacingLineParams pz;
    pz.apex_offset_frac = 0.0;
    pz.entry_offset_frac = 0.0;
    pz.exit_offset_frac = 0.0;
    RacingLine line(track, pz);

    const double L = line.track_length();
    constexpr int N = 100;
    for (int i = 0; i < N; ++i) {
        double s = (static_cast<double>(i) / (N - 1)) * L;
        auto p_line = line.position(s);
        auto p_ctr  = track.position(s);
        EXPECT_NEAR(p_line.x, p_ctr.x, 1e-6) << "at s=" << s;
        EXPECT_NEAR(p_line.y, p_ctr.y, 1e-6) << "at s=" << s;
    }
    std::filesystem::remove(path);
}

// ── 10.2.4 Position is continuous across every segment boundary ─────────────

TEST(RacingLineTest, PositionContinuousAtBoundaries) {
    auto track = load_interview_track();
    RacingLine line(track, {});

    const std::size_t N = track.segment_count();
    for (std::size_t k = 0; k + 1 < N; ++k) {
        double s_b = end_of_segment(track, k);
        auto p_lo = line.position(s_b - 1e-3);
        auto p_hi = line.position(s_b + 1e-3);
        double gap = std::sqrt((p_hi.x - p_lo.x) * (p_hi.x - p_lo.x)
                             + (p_hi.y - p_lo.y) * (p_hi.y - p_lo.y));
        EXPECT_LT(gap, 5e-3)
            << "position discontinuity at boundary after seg " << k
            << " (s=" << s_b << "), gap=" << gap;
    }
}

// ── 10.2.5 Heading is continuous across every segment boundary ──────────────

TEST(RacingLineTest, HeadingContinuousAtBoundaries) {
    // Spec wrote `heading(s ± 0.1)`, but heading() does FD over a 0.2 m chord
    // and that chord is dominated by real fast evolution on tight arcs (R=5
    // C4/C5: ~0.12 rad swing in 0.1 m as offset sweeps to apex), not by an
    // actual boundary kink. We probe boundary-tangent continuity directly
    // using a 1 mm chord on each side, which captures the spec's intent
    // ("catch sign-flip / blend bugs") without rejecting fast lines.
    auto track = load_interview_track();
    RacingLine line(track, {});

    const auto chord_heading = [&](double s_center, double s_other) {
        auto p1 = line.position(s_center);
        auto p2 = line.position(s_other);
        return std::atan2(p2.y - p1.y, p2.x - p1.x);
    };

    const std::size_t N = track.segment_count();
    constexpr double dx = 1e-3;
    for (std::size_t k = 0; k + 1 < N; ++k) {
        double s_b = end_of_segment(track, k);
        double h_lo = chord_heading(s_b - dx, s_b);  // chord ending at boundary
        double h_hi = chord_heading(s_b, s_b + dx);  // chord starting at boundary
        double dh = std::abs(normalize_angle(h_hi - h_lo));
        EXPECT_LT(dh, 0.05)
            << "heading kink at boundary after seg " << k
            << " (s=" << s_b << "), dh=" << dh << " rad";
    }
}

// ── 10.2.6 Curvature is finite and bounded ──────────────────────────────────

TEST(RacingLineTest, CurvatureFinite) {
    auto track = load_interview_track();
    RacingLine line(track, {});

    const double L = line.track_length();
    constexpr int N = 200;
    for (int i = 0; i < N; ++i) {
        double s = (static_cast<double>(i) / (N - 1)) * L;
        double k = line.curvature(s);
        ASSERT_TRUE(std::isfinite(k)) << "non-finite kappa at s=" << s;
        EXPECT_LT(std::abs(k), 10.0) << "kappa too large at s=" << s << ": " << k;
    }
}

// ── 10.5.1 YAML-configured constant straight offset is held mid-segment ────

TEST(RacingLineTest, StraightUsesYamlOffset) {
    auto track = load_interview_track();
    RacingLine line(track, {});

    // Interview track YAML has racing_offset_m: -1.7 on S1, S2, S3.
    // The middle of each straight should sit on the configured value.
    auto mid_of_segment = [&](std::size_t k) {
        double s_start = (k == 0) ? 0.0 : end_of_segment(track, k - 1);
        return s_start + 0.5 * track.segment(k).length();
    };

    EXPECT_NEAR(line.offset(mid_of_segment(0)), -1.7, 1e-3) << "S1 mid";
    EXPECT_NEAR(line.offset(mid_of_segment(4)), -1.7, 1e-3) << "S2 mid";
    EXPECT_NEAR(line.offset(mid_of_segment(6)), -1.7, 1e-3) << "S3 mid";
}

// ── 10.5.2 racing_offset_m on an arc is rejected ───────────────────────────
//
// Compile-time guarantee: Straight has set_racing_offset_m / racing_offset_m;
// Arc does not. The lines below would fail to compile if uncommented:
//
//     auto* a = dynamic_cast<Arc*>(...);
//     a->set_racing_offset_m(1.0);   // error: no member named 'set_racing_offset_m' in 'lapsim::Arc'
//
// Runtime side: TrackLoader prints a warning and skips the field. We exercise
// the warning path via a temp YAML file with racing_offset_m on an arc.

TEST(RacingLineTest, StraightOffsetIgnoredForArcs) {
    auto path = std::filesystem::temp_directory_path() / "test_arc_with_offset.yaml";
    {
        std::ofstream f(path);
        f << "track:\n"
             "  name: \"arc-offset test\"\n"
             "  width_m: 4.0\n"
             "  start_point: [0.0, 0.0]\n"
             "  start_heading: 0.0\n"
             "  closed: false\n"
             "  segments:\n"
             "    - id: \"A1\"\n"
             "      type: arc\n"
             "      radius: 10.0\n"
             "      swept_angle: 1.5707963267948966\n"
             "      racing_offset_m: 1.0\n";  // should be warned & ignored
    }
    Track t = TrackLoader::load(path.string());
    // The track loaded; the field is dropped on arcs (no setter to receive it).
    // Build a RacingLine over it; should not throw.
    EXPECT_NO_THROW({ RacingLine rl(t, {}); });
    std::filesystem::remove(path);
}

// ── 10.5.3 Smooth transition into / out of YAML-offset straight ─────────────

TEST(RacingLineTest, StraightOffsetTransitionsSmoothly) {
    auto track = load_interview_track();
    RacingLine line(track, {});

    // S1 has racing_offset_m: -1.7, length 100. Lap-start treats prev as 0.
    // Blend = min(0.10·L, 5 m) = 5 m at each end.
    EXPECT_NEAR(line.offset(0.0),   0.0, 1e-3)   << "lap start, no prev context";
    EXPECT_NEAR(line.offset(5.0),  -1.7, 1e-3)   << "5 m in, blend complete";
    EXPECT_NEAR(line.offset(95.0), -1.7, 1e-3)   << "5 m before end, still on plateau";
    // s=100 lands on the S1/C1 boundary; both sides see -1.7 because S1's
    // exit blend ends at C1.entry_off and C1's nominal entry_off is also -1.7.
    EXPECT_NEAR(line.offset(100.0), -1.7, 1e-3)  << "S1/C1 boundary";
}

// ── 10.2.7 Apex offset sign matches turn direction ──────────────────────────

TEST(RacingLineTest, ApexInsideTurnDirection) {
    auto track = load_interview_track();
    RacingLine line(track, {});
    const double half_w = track.width() / 2.0;

    // Segment indices on the interview track:
    // 0 S1, 1 C1 (left), 2 C2 (right), 3 C3 (left), 4 S2, 5 C4 (left), 6 S3, 7 C5 (left)
    auto mid_of_segment = [&](std::size_t k) {
        double s_start = (k == 0) ? 0.0 : end_of_segment(track, k - 1);
        return s_start + 0.5 * track.segment(k).length();
    };

    // C1 (left turn) — apex on LEFT (positive offset)
    {
        double s_mid = mid_of_segment(1);
        double off = line.offset(s_mid);
        EXPECT_GT(off, 0.5 * half_w)
            << "C1 (left turn) apex should be on LEFT (positive), got " << off;
    }
    // C2 (right turn) — apex on RIGHT (negative offset)
    {
        double s_mid = mid_of_segment(2);
        double off = line.offset(s_mid);
        EXPECT_LT(off, -0.5 * half_w)
            << "C2 (right turn) apex should be on RIGHT (negative), got " << off;
    }
}
