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

// ── Pure straight: FC matches QSS (kappa=0 → full a_max) ─────────────

TEST(FrictionCircleTest, PureStraight_MatchesQss) {
    TrackBuilder tb("Straight100");
    tb.straight("S1", 100.0);

    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto telem_fc  = solver.solve(tb.track, veh, PhysicsConfig::fc());
    auto telem_qss = solver.solve(tb.track, veh, PhysicsConfig::qss());

    EXPECT_NEAR(telem_fc.lap_time(), telem_qss.lap_time(), 1e-9);

    double a = veh.max_accel_mps2;
    double expected_v = std::sqrt(2.0 * a * 100.0);
    double expected_t = std::sqrt(2.0 * 100.0 / a);
    const auto& res = telem_fc.segment_results();
    ASSERT_EQ(res.size(), 1u);
    EXPECT_NEAR(res[0].v_exit, expected_v, expected_v * 0.01);
    EXPECT_NEAR(telem_fc.lap_time(), expected_t, expected_t * 0.01);
}

// ── Pure corner steady state ──────────────────────────────────────────

TEST(FrictionCircleTest, PureCorner_SteadyState) {
    double R = 10.0;
    double mu = 0.7, g = 9.81;
    double v_corner = std::sqrt(mu * g * R);
    double arc_len  = kPi * R;

    TrackBuilder tb("Corner");
    tb.straight("S1", 30.0);
    tb.arc("C1", R, kPi);

    auto veh = make_vehicle();
    auto cfg = PhysicsConfig::fc();
    cfg.ds = 0.25;
    LapTimeSolver solver;
    auto telem = solver.solve(tb.track, veh, cfg);

    const auto& res = telem.segment_results();
    ASSERT_EQ(res.size(), 2u);

    double analytic_time = arc_len / v_corner;
    EXPECT_NEAR(res[1].time, analytic_time, analytic_time * 0.02);
    EXPECT_NEAR(res[1].v_peak, v_corner, v_corner * 0.01);
}

// ── Corner entry: FC brakes earlier than QSS ──────────────────────────

TEST(FrictionCircleTest, CornerEntry_BrakesEarlier) {
    TrackBuilder tb("CornerEntry");
    tb.straight("S1", 100.0);
    tb.arc("C1", 10.0, kPi);

    auto veh = make_vehicle();
    auto cfg_qss = PhysicsConfig::qss();
    cfg_qss.ds = 0.25;
    auto cfg_fc = PhysicsConfig::fc();
    cfg_fc.ds = 0.25;

    LapTimeSolver solver;
    auto t_qss = solver.solve(tb.track, veh, cfg_qss);
    auto t_fc  = solver.solve(tb.track, veh, cfg_fc);

    ASSERT_EQ(t_qss.sample_count(), t_fc.sample_count());

    bool fc_slower = false;
    for (std::size_t i = 0; i < t_qss.sample_count(); ++i) {
        double s = t_qss.sample(i).distance_m;
        if (s > 90.0 && s < 100.0) {
            if (t_fc.sample(i).speed_mps < t_qss.sample(i).speed_mps - 0.01) {
                fc_slower = true;
                break;
            }
        }
    }
    EXPECT_TRUE(fc_slower)
        << "FrictionCircle should brake earlier than QSS before corner entry";
}

// ── Friction budget never exceeded on interview track ─────────────────

TEST(FrictionCircleTest, InterviewTrack_EllipseBudgetRespected) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    double mu_g = veh.mu * veh.g_mps2;
    double a_max = veh.max_accel_mps2;

    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::fc());

    double max_u = 0.0;
    for (std::size_t i = 0; i < telem.sample_count(); ++i) {
        const auto& s = telem.sample(i);
        double a_lat_mps2  = s.lat_accel_g * veh.g_mps2;
        double a_long_mps2 = s.accel_g * veh.g_mps2;
        double u = std::sqrt((a_lat_mps2 / mu_g) * (a_lat_mps2 / mu_g) +
                             (a_long_mps2 / a_max) * (a_long_mps2 / a_max));
        max_u = std::max(max_u, u);
    }

    EXPECT_LE(max_u, 1.0 + 0.05)
        << "Max ellipse utilization = " << max_u;
}

// ── Determinism ───────────────────────────────────────────────────────

TEST(FrictionCircleTest, Determinism) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto t1 = solver.solve(track, veh, PhysicsConfig::fc());
    auto t2 = solver.solve(track, veh, PhysicsConfig::fc());

    EXPECT_EQ(t1.lap_time(), t2.lap_time());
    ASSERT_EQ(t1.sample_count(), t2.sample_count());
    for (std::size_t i = 0; i < t1.sample_count(); ++i)
        EXPECT_EQ(t1.sample(i).speed_mps, t2.sample(i).speed_mps);
}

// ── Lap time sanity: FC slower than QSS ───────────────────────────────

TEST(FrictionCircleTest, InterviewTrack_LapTimeSanity) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto t_fc  = solver.solve(track, veh, PhysicsConfig::fc());
    auto t_qss = solver.solve(track, veh, PhysicsConfig::qss());

    EXPECT_GT(t_fc.lap_time(), t_qss.lap_time())
        << "Elliptical FC should be slower than QSS";

    EXPECT_GT(t_fc.lap_time(), 24.5);
    EXPECT_LT(t_fc.lap_time(), 26.0);
}
