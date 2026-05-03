#include "lapsim/Geometry.hpp"
#include "lapsim/LapTimeSolver.hpp"
#include "lapsim/PhysicsConfig.hpp"
#include "lapsim/Segment.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/Vehicle.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

using namespace lapsim;

namespace {

constexpr double kPi = std::numbers::pi;

Vehicle make_vehicle(double mu = 0.7, double g = 9.81, double a = 14.0) {
    Vehicle v;
    v.mu = mu; v.g_mps2 = g; v.max_accel_mps2 = a; v.mass_kg = 250.0;
    return v;
}

struct TrackBuilder {
    Track track;
    Vec2 p{0, 0};
    double h = 0.0;

    TrackBuilder(const std::string& name, double heading = 0.0)
        : h{heading} { track.set_name(name); track.set_closed(false); }

    void straight(const std::string& id, double len) {
        auto seg = std::make_unique<Straight>(id, p, h, len);
        p = seg->end_point(); h = seg->end_heading();
        track.add_segment(std::move(seg));
    }
    void arc(const std::string& id, double r, double sw) {
        auto seg = std::make_unique<Arc>(id, p, h, r, sw);
        p = seg->end_point(); h = seg->end_heading();
        track.add_segment(std::move(seg));
    }
};

Track build_interview_track() {
    TrackBuilder tb("Interview", kPi);
    tb.track.set_closed(true);
    tb.straight("S1", 100.0);
    tb.arc("C1", 22.5, kPi);
    tb.arc("C2", 20.0, -kPi / 2.0);
    tb.arc("C3", 20.0,  kPi / 2.0);
    tb.straight("S2", 60.0);
    tb.arc("C4", 5.0,   kPi / 2.0);
    tb.straight("S3", 75.0);
    tb.arc("C5", 5.0,   kPi / 2.0);
    return std::move(tb.track);
}

} // namespace

// ── QSS on a single straight ─────────────────────────────────────────

TEST(QssTest, SingleStraight) {
    TrackBuilder tb("Straight100");
    tb.straight("S1", 100.0);

    auto veh = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(tb.track, veh, PhysicsConfig::qss());

    double expected_v = std::sqrt(2.0 * 14.0 * 100.0);
    double expected_t = std::sqrt(2.0 * 100.0 / 14.0);

    const auto& res = telem.segment_results();
    ASSERT_EQ(res.size(), 1u);
    EXPECT_NEAR(res[0].v_exit, expected_v, expected_v * 0.01);
    EXPECT_NEAR(telem.lap_time(), expected_t, expected_t * 0.01);
}

// ── QSS on a circular corner ─────────────────────────────────────────

TEST(QssTest, SingleCorner) {
    double R = 10.0;
    double v_corner = std::sqrt(0.7 * 9.81 * R);
    double arc_len  = kPi * R;

    TrackBuilder tb("CornerOnly");
    tb.straight("S1", 20.0);
    tb.arc("C1", R, kPi);

    auto veh = make_vehicle();
    auto cfg = PhysicsConfig::qss();
    cfg.ds = 0.25;
    LapTimeSolver solver;
    auto telem = solver.solve(tb.track, veh, cfg);

    const auto& res = telem.segment_results();
    ASSERT_EQ(res.size(), 2u);

    double analytic_time = arc_len / v_corner;
    EXPECT_NEAR(res[1].time, analytic_time, analytic_time * 0.01);
    EXPECT_NEAR(res[1].v_peak, v_corner, v_corner * 0.01);
}

// ── QSS on the interview track ───────────────────────────────────────

TEST(QssTest, InterviewTrack_SaneLapTime) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::qss());

    EXPECT_GT(telem.lap_time(), 20.0);
    EXPECT_LT(telem.lap_time(), 35.0);

    double seg_sum = 0.0;
    for (const auto& r : telem.segment_results()) seg_sum += r.time;
    EXPECT_NEAR(seg_sum, telem.lap_time(), 1e-3);
}

TEST(QssTest, InterviewTrack_CornerVcap) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::qss());

    for (std::size_t j = 0; j < track.segment_count(); ++j) {
        const auto& seg = track.segment(j);
        if (seg.type_name() != "arc") continue;

        double R = 1.0 / std::abs(seg.curvature(0.0));
        double v_cap = std::sqrt(veh.mu * veh.g_mps2 * R);

        const auto& res = telem.segment_results();
        EXPECT_LE(res[j].v_peak, v_cap * 1.01)
            << "Segment " << seg.id() << " v_peak " << res[j].v_peak
            << " exceeds cap " << v_cap;
    }
}

// ── Determinism test ──────────────────────────────────────────────────

TEST(QssTest, Determinism) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto t1 = solver.solve(track, veh, PhysicsConfig::qss());
    auto t2 = solver.solve(track, veh, PhysicsConfig::qss());

    EXPECT_EQ(t1.lap_time(), t2.lap_time());
    ASSERT_EQ(t1.sample_count(), t2.sample_count());
    for (std::size_t i = 0; i < t1.sample_count(); ++i) {
        EXPECT_EQ(t1.sample(i).speed_mps, t2.sample(i).speed_mps);
    }
}

// ── Refinement test ───────────────────────────────────────────────────

TEST(QssTest, Refinement) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto cfg_coarse = PhysicsConfig::qss();
    cfg_coarse.ds = 1.0;
    auto cfg_fine = PhysicsConfig::qss();
    cfg_fine.ds = 0.1;

    auto t_coarse = solver.solve(track, veh, cfg_coarse);
    auto t_fine   = solver.solve(track, veh, cfg_fine);

    double diff = std::abs(t_coarse.lap_time() - t_fine.lap_time());
    double tol  = t_fine.lap_time() * 0.02;
    EXPECT_LT(diff, tol)
        << "Coarse=" << t_coarse.lap_time() << " Fine=" << t_fine.lap_time();
}
