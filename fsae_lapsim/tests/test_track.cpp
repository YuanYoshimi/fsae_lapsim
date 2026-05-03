#include "lapsim/Geometry.hpp"
#include "lapsim/Segment.hpp"
#include "lapsim/Track.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace lapsim;

namespace {
constexpr double kEps = 1e-9;
constexpr double kPi  = std::numbers::pi;
} // namespace

// ── Straight ────────────────────────────────────────────────────────────────

TEST(StraightTest, PositionAtMidpointAndEnd) {
    Straight seg("S1", {0.0, 0.0}, 0.0, 100.0);

    auto p0   = seg.position(0.0);
    auto pmid = seg.position(50.0);
    auto pend = seg.position(100.0);

    EXPECT_NEAR(p0.x,   0.0, kEps);
    EXPECT_NEAR(p0.y,   0.0, kEps);
    EXPECT_NEAR(pmid.x, 50.0, kEps);
    EXPECT_NEAR(pmid.y, 0.0, kEps);
    EXPECT_NEAR(pend.x, 100.0, kEps);
    EXPECT_NEAR(pend.y, 0.0, kEps);
}

TEST(StraightTest, HeadingConstant) {
    Straight seg("S1", {5.0, 3.0}, 1.23, 50.0);

    EXPECT_NEAR(seg.heading(0.0),  1.23, kEps);
    EXPECT_NEAR(seg.heading(25.0), 1.23, kEps);
    EXPECT_NEAR(seg.heading(50.0), 1.23, kEps);
}

TEST(StraightTest, EndPointMatchesPosition) {
    Straight seg("S1", {1.0, 2.0}, kPi / 4.0, 10.0);
    auto ep = seg.end_point();
    auto pp = seg.position(seg.length());

    EXPECT_NEAR(ep.x, pp.x, kEps);
    EXPECT_NEAR(ep.y, pp.y, kEps);
}

// ── Arc: 90° left turn ─────────────────────────────────────────────────────

TEST(ArcTest, LeftTurn90_EndPoint) {
    // Start (0,0) heading +x, R=10, swept +pi/2
    // Expected end: (10, 10) heading pi/2
    Arc seg("C1", {0.0, 0.0}, 0.0, 10.0, kPi / 2.0);

    EXPECT_NEAR(seg.length(), 10.0 * kPi / 2.0, kEps);

    auto ep = seg.end_point();
    EXPECT_NEAR(ep.x, 10.0, 1e-8);
    EXPECT_NEAR(ep.y, 10.0, 1e-8);
    EXPECT_NEAR(normalize_angle(seg.end_heading() - kPi / 2.0), 0.0, 1e-8);
}

TEST(ArcTest, LeftTurn90_Midpoint) {
    Arc seg("C1", {0.0, 0.0}, 0.0, 10.0, kPi / 2.0);
    double half_s = seg.length() / 2.0;
    auto pmid = seg.position(half_s);

    // At 45°: center is (0,10), angle from center = -pi/4
    // position = (0,10) + 10*(cos(-pi/4), sin(-pi/4)) = (5*sqrt(2), 10-5*sqrt(2))
    double expected_x = 5.0 * std::sqrt(2.0);
    double expected_y = 10.0 - 5.0 * std::sqrt(2.0);

    EXPECT_NEAR(pmid.x, expected_x, 1e-8);
    EXPECT_NEAR(pmid.y, expected_y, 1e-8);
    EXPECT_NEAR(normalize_angle(seg.heading(half_s) - kPi / 4.0), 0.0, 1e-8);
}

TEST(ArcTest, LeftTurn90_StartPoint) {
    Arc seg("C1", {0.0, 0.0}, 0.0, 10.0, kPi / 2.0);
    auto p0 = seg.position(0.0);

    EXPECT_NEAR(p0.x, 0.0, kEps);
    EXPECT_NEAR(p0.y, 0.0, kEps);
    EXPECT_NEAR(seg.heading(0.0), 0.0, kEps);
}

// ── Arc: 90° right turn ────────────────────────────────────────────────────

TEST(ArcTest, RightTurn90_EndPoint) {
    // Start (0,0) heading +x, R=10, swept -pi/2
    // Expected end: (10, -10) heading -pi/2
    Arc seg("C2", {0.0, 0.0}, 0.0, 10.0, -kPi / 2.0);

    EXPECT_NEAR(seg.length(), 10.0 * kPi / 2.0, kEps);

    auto ep = seg.end_point();
    EXPECT_NEAR(ep.x, 10.0, 1e-8);
    EXPECT_NEAR(ep.y, -10.0, 1e-8);
    EXPECT_NEAR(normalize_angle(seg.end_heading() + kPi / 2.0), 0.0, 1e-8);
}

// ── Arc: 180° left turn ────────────────────────────────────────────────────

TEST(ArcTest, LeftTurn180_EndPoint) {
    // Start (0,0) heading +x, R=5, swept +pi
    // Expected end: (0, 10) heading pi (= -x direction)
    Arc seg("C3", {0.0, 0.0}, 0.0, 5.0, kPi);

    EXPECT_NEAR(seg.length(), 5.0 * kPi, 1e-8);

    auto ep = seg.end_point();
    EXPECT_NEAR(ep.x, 0.0, 1e-8);
    EXPECT_NEAR(ep.y, 10.0, 1e-8);
    EXPECT_NEAR(normalize_angle(seg.end_heading() - kPi), 0.0, 1e-8);
}

// ── Track validation: closing track ────────────────────────────────────────

TEST(TrackValidationTest, TwoSemicirclesClose) {
    // Two 180° left arcs of radius 5 form a full circle.
    Track track;
    track.set_name("Circle");
    track.set_closed(true);

    // Arc1: (0,0) heading 0, R=5, swept pi -> end (0,10) heading pi
    track.add_segment(std::make_unique<Arc>("A1", Vec2{0.0, 0.0}, 0.0, 5.0, kPi));
    // Arc2: (0,10) heading pi, R=5, swept pi -> end (0,0) heading 2pi ≈ 0
    track.add_segment(std::make_unique<Arc>("A2", Vec2{0.0, 10.0}, kPi, 5.0, kPi));

    auto val = track.validate();

    EXPECT_TRUE(val.is_valid);
    EXPECT_TRUE(val.closes_loop);
    EXPECT_LT(val.max_position_gap, 1e-4);
    EXPECT_LT(val.max_heading_gap, 1e-4);
    EXPECT_TRUE(val.errors.empty());
}

// ── Track validation: broken track ─────────────────────────────────────────

TEST(TrackValidationTest, BrokenTrackReportsGap) {
    Track track;
    track.set_name("Broken");
    track.set_closed(false);

    // Straight1: (0,0) heading 0, length 10 -> end (10, 0)
    track.add_segment(std::make_unique<Straight>("S1", Vec2{0.0, 0.0}, 0.0, 10.0));
    // Straight2: deliberately starts at (20,0) — 10 m gap
    track.add_segment(std::make_unique<Straight>("S2", Vec2{20.0, 0.0}, 0.0, 10.0));

    auto val = track.validate();

    EXPECT_FALSE(val.is_valid);
    EXPECT_GT(val.max_position_gap, 9.0);
    ASSERT_FALSE(val.errors.empty());

    // Error message should mention both segment ids
    bool found_s1 = false;
    bool found_s2 = false;
    for (const auto& err : val.errors) {
        if (err.find("S1") != std::string::npos) found_s1 = true;
        if (err.find("S2") != std::string::npos) found_s2 = true;
    }
    EXPECT_TRUE(found_s1);
    EXPECT_TRUE(found_s2);
}

// ── Track: position and heading queries ────────────────────────────────────

TEST(TrackTest, GlobalPositionQuery) {
    Track track;
    track.set_name("Simple");
    track.set_closed(false);

    // Two straights, each 50m, both heading +x
    track.add_segment(std::make_unique<Straight>("S1", Vec2{0.0, 0.0}, 0.0, 50.0));
    track.add_segment(std::make_unique<Straight>("S2", Vec2{50.0, 0.0}, 0.0, 50.0));

    EXPECT_NEAR(track.total_length(), 100.0, kEps);

    auto p0  = track.position(0.0);
    auto p50 = track.position(50.0);
    auto p75 = track.position(75.0);

    EXPECT_NEAR(p0.x,  0.0, kEps);
    EXPECT_NEAR(p50.x, 50.0, kEps);
    EXPECT_NEAR(p75.x, 75.0, kEps);

    EXPECT_NEAR(track.heading(0.0),  0.0, kEps);
    EXPECT_NEAR(track.heading(50.0), 0.0, kEps);
    EXPECT_NEAR(track.heading(75.0), 0.0, kEps);
}
