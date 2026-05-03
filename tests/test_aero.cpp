#include "lapsim/Aero.hpp"
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

Vehicle make_vehicle() {
    Vehicle v;
    v.mu = 0.7; v.g_mps2 = 9.81; v.max_accel_mps2 = 14.0;
    v.mass_kg = 250.0;
    v.aero = Aero(1.225, 1.0, 1.5);
    return v;
}

Vehicle make_vehicle_no_aero() {
    Vehicle v;
    v.mu = 0.7; v.g_mps2 = 9.81; v.max_accel_mps2 = 14.0;
    v.mass_kg = 250.0;
    v.aero = Aero(1.225, 0.0, 0.0);
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

// ── Aero force unit tests ─────────────────────────────────────────────

TEST(AeroTest, DragForce) {
    Aero aero(1.225, 1.0, 1.5);
    // F_d = 0.5 * 1.225 * 30^2 * 1.0 = 0.5 * 1.225 * 900 = 551.25 N
    EXPECT_NEAR(aero.drag_force(30.0), 551.25, 0.01);
}

TEST(AeroTest, Downforce) {
    Aero aero(1.225, 1.0, 1.5);
    // F_l = 0.5 * 1.225 * 30^2 * 1.5 = 826.875 N
    EXPECT_NEAR(aero.downforce(30.0), 826.875, 0.01);
}

// ── v_cap_aero degenerates to sqrt(mu*g/kappa) without aero ──────────

TEST(AeroTest, VCapAero_NoAero) {
    auto veh = make_vehicle_no_aero();
    double kappa = 1.0 / 20.0;
    double expected = std::sqrt(veh.mu * veh.g_mps2 / kappa);
    EXPECT_NEAR(AeroSolver::v_cap_aero(kappa, veh), expected, 1e-6);
}

// ── v_cap_aero with downforce on R=20 ────────────────────────────────

TEST(AeroTest, VCapAero_WithAero) {
    auto veh = make_vehicle();
    double kappa = 1.0 / 20.0;
    // aero_coeff = mu * rho * ClA / (2*m) = 0.7*1.225*1.5/(2*250) = 0.0025725
    // denom = 0.05 - 0.0025725 = 0.0474275
    // v_cap = sqrt(mu*g / denom) = sqrt(6.867 / 0.0474275)
    double aero_coeff = veh.mu * veh.aero.rho() * veh.aero.cla() / (2.0 * veh.mass_kg);
    double denom = kappa - aero_coeff;
    double expected = std::sqrt(veh.mu * veh.g_mps2 / denom);
    double v_no_aero = std::sqrt(veh.mu * veh.g_mps2 / kappa);

    double result = AeroSolver::v_cap_aero(kappa, veh);
    EXPECT_NEAR(result, expected, 1e-3);
    EXPECT_GT(result, v_no_aero) << "Downforce should increase corner cap";
}

// ── a_drive_max_aero on straight gives full a_max ─────────────────────

TEST(AeroTest, ADriveMax_Straight) {
    auto veh = make_vehicle();
    double result = AeroSolver::a_drive_max_aero(20.0, 0.0, veh);
    EXPECT_NEAR(result, 14.0, 1e-6);
}

// ── Interview track lap time with aero ────────────────────────────────

TEST(AeroTest, InterviewTrack_LapTimeWithAero) {
    auto track = build_interview_track();
    auto veh = make_vehicle();

    AeroSolver aero_solver(0.5);
    auto t_aero = aero_solver.solve(track, veh);

    FrictionCircleSolver fc(0.5);
    auto t_fc = fc.solve(track, veh);

    EXPECT_GT(t_aero.lap_time(), 24.0);
    EXPECT_LT(t_aero.lap_time(), 26.0);
    EXPECT_LT(t_aero.lap_time(), t_fc.lap_time())
        << "Aero (downforce) should help overall vs plain friction circle";
}

// ── Downforce helps corners ───────────────────────────────────────────

TEST(AeroTest, DownforceHelpsCorner) {
    auto track = build_interview_track();
    auto veh = make_vehicle();

    AeroSolver aero_solver(0.5);
    FrictionCircleSolver fc(0.5);

    auto t_aero = aero_solver.solve(track, veh);
    auto t_fc   = fc.solve(track, veh);

    const auto& res_aero = t_aero.segment_results();
    const auto& res_fc   = t_fc.segment_results();

    // C1 time should decrease with aero (downforce adds grip).
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

    AeroSolver aero_solver(0.5);
    FrictionCircleSolver fc(0.5);

    auto t_aero = aero_solver.solve(track, veh);
    auto t_fc   = fc.solve(track, veh);

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
    AeroSolver solver(0.5);

    auto t1 = solver.solve(track, veh);
    auto t2 = solver.solve(track, veh);

    EXPECT_EQ(t1.lap_time(), t2.lap_time());
    ASSERT_EQ(t1.sample_count(), t2.sample_count());
    for (std::size_t i = 0; i < t1.sample_count(); ++i)
        EXPECT_EQ(t1.sample(i).speed_mps, t2.sample(i).speed_mps);
}
