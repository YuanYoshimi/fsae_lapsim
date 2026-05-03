#include "lapsim/Aero.hpp"
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

Vehicle make_vehicle() {
    Vehicle v;
    v.mu = 0.7; v.g_mps2 = 9.81; v.max_accel_mps2 = 14.0;
    v.mass_kg = 250.0;
    v.aero = Aero(1.225, 1.0, 1.5);
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

// ── Aero force unit tests (testing the Aero class, not solvers) ───────

TEST(AeroTest, DragForce) {
    Aero aero(1.225, 1.0, 1.5);
    EXPECT_NEAR(aero.drag_force(30.0), 551.25, 0.01);
}

TEST(AeroTest, Downforce) {
    Aero aero(1.225, 1.0, 1.5);
    EXPECT_NEAR(aero.downforce(30.0), 826.875, 0.01);
}

// ── Interview track: aero beats FC ────────────────────────────────────

TEST(AeroTest, InterviewTrack_LapTimeWithAero) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto t_aero = solver.solve(track, veh, PhysicsConfig::aero_preset());
    auto t_fc   = solver.solve(track, veh, PhysicsConfig::fc());

    EXPECT_GT(t_aero.lap_time(), 24.0);
    EXPECT_LT(t_aero.lap_time(), 26.0);
    EXPECT_LT(t_aero.lap_time(), t_fc.lap_time())
        << "Aero (downforce) should help overall vs plain friction circle";
}

// ── Downforce helps corners ───────────────────────────────────────────

TEST(AeroTest, DownforceHelpsCorner) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto t_aero = solver.solve(track, veh, PhysicsConfig::aero_preset());
    auto t_fc   = solver.solve(track, veh, PhysicsConfig::fc());

    const auto& res_aero = t_aero.segment_results();
    const auto& res_fc   = t_fc.segment_results();

    double c1_aero = 0.0, c1_fc = 0.0;
    for (const auto& r : res_aero) if (r.id == "C1") c1_aero = r.time;
    for (const auto& r : res_fc)   if (r.id == "C1") c1_fc = r.time;
    EXPECT_LT(c1_aero, c1_fc)
        << "C1 with aero (" << c1_aero << ") should be faster than without (" << c1_fc << ")";
}

// ── Drag slows peak speed on straights ────────────────────────────────

TEST(AeroTest, DragSlowsHighSpeed) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto t_aero = solver.solve(track, veh, PhysicsConfig::aero_preset());
    auto t_fc   = solver.solve(track, veh, PhysicsConfig::fc());

    double s1_peak_aero = 0.0, s1_peak_fc = 0.0;
    for (const auto& r : t_aero.segment_results()) if (r.id == "S1") s1_peak_aero = r.v_peak;
    for (const auto& r : t_fc.segment_results())   if (r.id == "S1") s1_peak_fc = r.v_peak;

    EXPECT_LT(s1_peak_aero, s1_peak_fc)
        << "S1 peak with aero (" << s1_peak_aero << ") should be lower than without (" << s1_peak_fc << ")";
}

// ── Determinism ───────────────────────────────────────────────────────

TEST(AeroTest, Determinism) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    LapTimeSolver solver;

    auto t1 = solver.solve(track, veh, PhysicsConfig::aero_preset());
    auto t2 = solver.solve(track, veh, PhysicsConfig::aero_preset());

    EXPECT_EQ(t1.lap_time(), t2.lap_time());
    ASSERT_EQ(t1.sample_count(), t2.sample_count());
    for (std::size_t i = 0; i < t1.sample_count(); ++i)
        EXPECT_EQ(t1.sample(i).speed_mps, t2.sample(i).speed_mps);
}
