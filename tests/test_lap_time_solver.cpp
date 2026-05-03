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
    v.mu = 0.7;
    v.g_mps2 = 9.81;
    v.max_accel_mps2 = 14.0;
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

// ── Each preset produces non-empty Telemetry ──────────────────────────

TEST(LapTimeSolverTest, BasicPresetProducesTelemetry) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::basic());

    EXPECT_GT(telem.lap_time(), 0.0);
    EXPECT_EQ(telem.segment_results().size(), 8u);
    EXPECT_EQ(telem.sample_count(), 0u);
}

TEST(LapTimeSolverTest, QssPresetProducesTelemetry) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::qss());

    EXPECT_GT(telem.lap_time(), 0.0);
    EXPECT_GT(telem.sample_count(), 0u);
}

TEST(LapTimeSolverTest, FcPresetProducesTelemetry) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::fc());

    EXPECT_GT(telem.lap_time(), 0.0);
    EXPECT_GT(telem.sample_count(), 0u);
}

TEST(LapTimeSolverTest, AeroPresetProducesTelemetry) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::aero_preset());

    EXPECT_GT(telem.lap_time(), 0.0);
    EXPECT_GT(telem.sample_count(), 0u);
}

// ── Lap times within 1.0 s of canonical (loose gate for 8.2) ─────────

TEST(LapTimeSolverTest, BasicLapTimeLoose) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::basic());
    EXPECT_NEAR(telem.lap_time(), 25.290, 1.0);
}

TEST(LapTimeSolverTest, QssLapTimeLoose) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::qss());
    EXPECT_NEAR(telem.lap_time(), 25.157, 1.0);
}

TEST(LapTimeSolverTest, FcLapTimeLoose) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::fc());
    EXPECT_NEAR(telem.lap_time(), 25.424, 1.0);
}

TEST(LapTimeSolverTest, AeroLapTimeLoose) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::aero_preset());
    EXPECT_NEAR(telem.lap_time(), 25.085, 1.0);
}

// ── Regression gate (1e-3 s tolerance) ────────────────────────────────
// These prove the consolidation didn't change physics.

TEST(LapTimeSolverTest, LapTimeRegressionBasic) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::basic());
    EXPECT_NEAR(telem.lap_time(), 25.290, 1e-3)
        << "Basic preset lap time regression failed";
}

TEST(LapTimeSolverTest, LapTimeRegressionQss) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::qss());
    EXPECT_NEAR(telem.lap_time(), 25.157, 1e-3)
        << "QSS preset lap time regression failed";
}

TEST(LapTimeSolverTest, LapTimeRegressionFc) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::fc());
    EXPECT_NEAR(telem.lap_time(), 25.424, 1e-3)
        << "FC preset lap time regression failed";
}

TEST(LapTimeSolverTest, LapTimeRegressionAero) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::aero_preset());
    EXPECT_NEAR(telem.lap_time(), 25.085, 1e-3)
        << "Aero preset lap time regression failed";
}
