#include "lapsim/Geometry.hpp"
#include "lapsim/LapTimeSolver.hpp"
#include "lapsim/PhysicsConfig.hpp"
#include "lapsim/Segment.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/Vehicle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace lapsim;

namespace {
constexpr double kPi = std::numbers::pi;
} // namespace

// ── Corner speed via basic preset ─────────────────────────────────────

TEST(SolverTest, CornerSpeed) {
    // v_corner = sqrt(mu * g * R) on a 180-deg arc, R=10
    double mu = 0.7, g = 9.81, R = 10.0;
    double expected = std::sqrt(mu * g * R);

    Track track;
    track.set_name("CornerTest");
    track.set_closed(false);

    Vec2 p{0, 0};
    double h = 0.0;
    auto s1 = std::make_unique<Straight>("S1", p, h, 10.0);
    p = s1->end_point(); h = s1->end_heading();
    track.add_segment(std::move(s1));
    auto c1 = std::make_unique<Arc>("C1", p, h, R, kPi);
    track.add_segment(std::move(c1));

    Vehicle veh;
    veh.mu = mu; veh.g_mps2 = g; veh.max_accel_mps2 = 14.0;

    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::basic());
    const auto& results = telem.segment_results();

    ASSERT_EQ(results.size(), 2u);
    double arc_length = kPi * R;
    double corner_time = arc_length / expected;
    EXPECT_NEAR(results[1].time, corner_time, 1e-3);
    EXPECT_NEAR(results[1].v_entry, expected, 1e-6);
}

// ── Full interview lap (basic preset) ─────────────────────────────────

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

    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::basic());
    double total = telem.lap_time();

    EXPECT_GT(total, 5.0);
    EXPECT_LT(total, 30.0);

    double sum = 0.0;
    for (const auto& r : telem.segment_results()) sum += r.time;
    EXPECT_NEAR(sum, total, 1e-9);
    EXPECT_EQ(telem.segment_results().size(), 8u);
}
