#include "lapsim/Aero.hpp"
#include "lapsim/Geometry.hpp"
#include "lapsim/LapTimeSolver.hpp"
#include "lapsim/PhysicsConfig.hpp"
#include "lapsim/Segment.hpp"
#include "lapsim/Telemetry.hpp"
#include "lapsim/TelemetryReader.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/Vehicle.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
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
        p = seg->end_point();
        h = seg->end_heading();
        track.add_segment(std::move(seg));
    }
    void arc(const std::string& id, double r, double sw) {
        auto seg = std::make_unique<Arc>(id, p, h, r, sw);
        p = seg->end_point();
        h = seg->end_heading();
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

const std::string kTestDir = "test_reader_tmp/";

Telemetry generate_test_csv(const Track& track, const Vehicle& veh,
                            const std::string& csv_path) {
    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::aero_preset());

    TelemetryMetadata meta;
    meta.solver_name = "aero";
    meta.mu          = veh.mu;
    meta.g_mps2      = veh.g_mps2;
    meta.a_max_mps2  = veh.max_accel_mps2;
    meta.mass_kg     = veh.mass_kg;
    meta.CdA         = veh.aero.cda();
    meta.ClA         = veh.aero.cla();
    meta.rho         = veh.aero.rho();
    telem.set_metadata(meta);

    std::vector<std::string> seg_ids;
    for (std::size_t i = 0; i < track.segment_count(); ++i)
        seg_ids.push_back(track.segment(i).id());
    telem.assign_segment_ids(seg_ids);

    telem.write_csv(csv_path);
    return telem;
}

} // namespace

// ── ReadKnownCsv ──────────────────────────────────────────────────────

TEST(TelemetryReaderTest, ReadKnownCsv) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    std::string csv = kTestDir + "known.csv";
    auto telem = generate_test_csv(track, veh, csv);

    TelemetryReader reader;
    ASSERT_TRUE(reader.load(csv));

    EXPECT_EQ(reader.sample_count(), telem.sample_count());

    EXPECT_NEAR(reader.samples().front().distance_m, 0.0, 1e-3);
    EXPECT_NEAR(reader.samples().back().distance_m,
                track.total_length(), 1.0);

    std::filesystem::remove_all(kTestDir);
}

// ── AtTimeEndpoints ───────────────────────────────────────────────────

TEST(TelemetryReaderTest, AtTimeEndpoints) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    std::string csv = kTestDir + "time.csv";
    auto telem = generate_test_csv(track, veh, csv);

    TelemetryReader reader;
    ASSERT_TRUE(reader.load(csv));

    auto first = reader.at_time(0.0);
    EXPECT_NEAR(first.speed_mps, 0.0, 1.0);

    auto last = reader.at_time(reader.total_time_s());
    EXPECT_NEAR(last.speed_mps,
                reader.samples().back().speed_mps, 0.1);

    std::filesystem::remove_all(kTestDir);
}

// ── AtTimeMidpoint ────────────────────────────────────────────────────

TEST(TelemetryReaderTest, AtTimeMidpoint) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    std::string csv = kTestDir + "midpoint.csv";
    auto telem = generate_test_csv(track, veh, csv);

    TelemetryReader reader;
    ASSERT_TRUE(reader.load(csv));

    double max_speed = 0.0;
    for (const auto& s : reader.samples())
        max_speed = std::max(max_speed, s.speed_mps);

    auto mid = reader.at_time(reader.total_time_s() / 2.0);
    EXPECT_GT(mid.speed_mps, 0.0);
    EXPECT_LT(mid.speed_mps, max_speed + 1.0);

    std::filesystem::remove_all(kTestDir);
}

// ── AtDistanceEndpoints ───────────────────────────────────────────────

TEST(TelemetryReaderTest, AtDistanceEndpoints) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    std::string csv = kTestDir + "dist.csv";
    auto telem = generate_test_csv(track, veh, csv);

    TelemetryReader reader;
    ASSERT_TRUE(reader.load(csv));

    auto first = reader.at_distance(0.0);
    EXPECT_NEAR(first.time_s, 0.0, 0.1);

    auto last = reader.at_distance(track.total_length());
    EXPECT_NEAR(last.time_s, reader.total_time_s(), 1.0);

    std::filesystem::remove_all(kTestDir);
}

// ── MalformedCsv ──────────────────────────────────────────────────────

TEST(TelemetryReaderTest, MalformedCsv) {
    auto track = build_interview_track();
    auto veh   = make_vehicle();
    std::string csv = kTestDir + "malformed.csv";
    auto telem = generate_test_csv(track, veh, csv);

    std::ifstream ifs(csv);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(ifs, line)) lines.push_back(line);
    ifs.close();

    std::string bad_csv = kTestDir + "malformed2.csv";
    std::ofstream ofs(bad_csv);
    ofs << lines[0] << "\n";
    ofs << lines[1] << "\n";
    ofs << "garbage,data\n";
    for (std::size_t i = 2; i < lines.size(); ++i)
        ofs << lines[i] << "\n";
    ofs.close();

    TelemetryReader reader;
    ASSERT_TRUE(reader.load(bad_csv));
    EXPECT_GT(reader.sample_count(), 0u);

    std::filesystem::remove_all(kTestDir);
}

// ── HeaderValidation ──────────────────────────────────────────────────

TEST(TelemetryReaderTest, HeaderValidation) {
    std::filesystem::create_directories(kTestDir);
    std::string bad_csv = kTestDir + "bad_header.csv";

    std::ofstream ofs(bad_csv);
    ofs << "col_a,col_b,col_c\n";
    ofs << "1,2,3\n";
    ofs.close();

    TelemetryReader reader;
    EXPECT_FALSE(reader.load(bad_csv));
    EXPECT_EQ(reader.sample_count(), 0u);

    std::filesystem::remove_all(kTestDir);
}
