#include "lapsim/Geometry.hpp"
#include "lapsim/Segment.hpp"
#include "lapsim/Solver.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/Vehicle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace lapsim;

namespace {
constexpr double kPi = std::numbers::pi;
} // namespace

// ── Corner speed formula ──────────────────────────────────────────────

TEST(SolverTest, CornerSpeed) {
    double v = compute_corner_speed(0.7, 9.81, 10.0);
    double expected = std::sqrt(0.7 * 9.81 * 10.0);
    EXPECT_NEAR(v, expected, 1e-6);

    Track track;
    track.set_name("CornerTest");
    track.set_closed(false);

    Vec2 p{0, 0};
    double h = 0.0;
    auto s1 = std::make_unique<Straight>("S1", p, h, 10.0);
    p = s1->end_point(); h = s1->end_heading();
    track.add_segment(std::move(s1));
    auto c1 = std::make_unique<Arc>("C1", p, h, 10.0, kPi);
    track.add_segment(std::move(c1));

    Vehicle veh;
    veh.mu = 0.7; veh.g_mps2 = 9.81; veh.max_accel_mps2 = 14.0;

    BasicSolver solver;
    auto telem = solver.solve(track, veh);
    const auto& results = telem.segment_results();

    ASSERT_EQ(results.size(), 2u);
    double arc_length = kPi * 10.0;
    double corner_time = arc_length / expected;
    EXPECT_NEAR(results[1].time, corner_time, 1e-3);
    EXPECT_NEAR(results[1].v_entry, expected, 1e-6);
}

// ── Pure acceleration straight ────────────────────────────────────────

TEST(SolverTest, PureAccelStraight) {
    auto r = compute_straight_time(0.0, 1e6, 50.0, 10.0);
    double expected_v = std::sqrt(2.0 * 10.0 * 50.0);
    EXPECT_NEAR(r.v_exit, expected_v, 1e-6);
    EXPECT_NEAR(r.v_peak, expected_v, 1e-6);
    EXPECT_NEAR(r.time, expected_v / 10.0, 1e-6);
}

// ── Accel-then-brake straight (v_i == v_f) ────────────────────────────

TEST(SolverTest, AccelBrakeStraight) {
    auto r = compute_straight_time(10.0, 10.0, 100.0, 10.0);
    double expected_peak = std::sqrt(1100.0);
    double expected_time = 2.0 * (expected_peak - 10.0) / 10.0;

    EXPECT_NEAR(r.v_peak, expected_peak, 1e-6);
    EXPECT_NEAR(r.v_exit, 10.0, 1e-6);
    EXPECT_NEAR(r.time, expected_time, 1e-6);
    EXPECT_NEAR(r.time, 4.633, 0.001);
}

// ── Full interview lap (BasicSolver) ──────────────────────────────────

TEST(SolverTest, FullInterviewLap) {
    Vehicle veh;
    veh.mu = 0.7; veh.g_mps2 = 9.81; veh.max_accel_mps2 = 14.0; veh.mass_kg = 250.0;

    Track track;
    track.set_name("Interview");
    track.set_closed(true);

    Vec2 p{0, 0}; double h = kPi;
    auto add_s = [&](const std::string& id, double len) {
        auto seg = std::make_unique<Straight>(id, p, h, len);
        p = seg->end_point(); h = seg->end_heading();
        track.add_segment(std::move(seg));
    };
    auto add_a = [&](const std::string& id, double r, double sw) {
        auto seg = std::make_unique<Arc>(id, p, h, r, sw);
        p = seg->end_point(); h = seg->end_heading();
        track.add_segment(std::move(seg));
    };

    add_s("S1", 100.0);
    add_a("C1", 22.5, kPi);
    add_a("C2", 20.0, -kPi / 2.0);
    add_a("C3", 20.0,  kPi / 2.0);
    add_s("S2", 60.0);
    add_a("C4", 5.0,   kPi / 2.0);
    add_s("S3", 75.0);
    add_a("C5", 5.0,   kPi / 2.0);

    BasicSolver solver;
    auto telem = solver.solve(track, veh);
    double total = telem.lap_time();

    EXPECT_GT(total, 5.0);
    EXPECT_LT(total, 30.0);

    double sum = 0.0;
    for (const auto& r : telem.segment_results()) sum += r.time;
    EXPECT_NEAR(sum, total, 1e-9);
    EXPECT_EQ(telem.segment_results().size(), 8u);
}
