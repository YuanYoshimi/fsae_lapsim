#include "lapsim/Aero.hpp"
#include "lapsim/Geometry.hpp"
#include "lapsim/LapTimeSolver.hpp"
#include "lapsim/PhysicsConfig.hpp"
#include "lapsim/Segment.hpp"
#include "lapsim/Telemetry.hpp"
#include "lapsim/Track.hpp"
#include "lapsim/Vehicle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

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

std::vector<std::string> read_lines(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream ifs(path);
    std::string line;
    while (std::getline(ifs, line)) lines.push_back(line);
    return lines;
}

Telemetry run_aero_with_export(const Track& track, const Vehicle& veh) {
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

    return telem;
}

const std::string kTestDir = "test_output_tmp/";

} // namespace

// ── CSV: header + row count ───────────────────────────────────────────

TEST(TelemetryExportTest, CsvHeaderAndRowCount) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    auto telem = run_aero_with_export(track, veh);

    std::string path = kTestDir + "test_csv.csv";
    telem.write_csv(path);

    auto lines = read_lines(path);
    ASSERT_GT(lines.size(), 1u);

    EXPECT_TRUE(lines[0].find("sample_idx") != std::string::npos);
    EXPECT_TRUE(lines[0].find("driver_state") != std::string::npos);

    EXPECT_EQ(lines.size(), telem.sample_count() + 1);

    std::filesystem::remove_all(kTestDir);
}

// ── CSV: first and last s values ──────────────────────────────────────

TEST(TelemetryExportTest, CsvFirstLastDistance) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    auto telem = run_aero_with_export(track, veh);

    std::string path = kTestDir + "test_dist.csv";
    telem.write_csv(path);

    auto lines = read_lines(path);
    ASSERT_GT(lines.size(), 2u);

    auto parse_s = [](const std::string& line) -> double {
        std::istringstream ss(line);
        std::string token;
        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        return std::stod(token);
    };

    double first_s = parse_s(lines[1]);
    double last_s  = parse_s(lines.back());
    EXPECT_NEAR(first_s, 0.0, 1e-3);
    EXPECT_NEAR(last_s, track.total_length(), 1.0);

    std::filesystem::remove_all(kTestDir);
}

// ── JSON: structural check ────────────────────────────────────────────

TEST(TelemetryExportTest, JsonStructure) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    auto telem = run_aero_with_export(track, veh);

    std::string path = kTestDir + "test.json";
    telem.write_json(path);

    std::ifstream ifs(path);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                         std::istreambuf_iterator<char>());

    EXPECT_FALSE(content.empty());
    EXPECT_EQ(content.front(), '{');
    EXPECT_TRUE(content.find("\"metadata\"") != std::string::npos);
    EXPECT_TRUE(content.find("\"samples\"") != std::string::npos);
    EXPECT_TRUE(content.find("\"segment_summary\"") != std::string::npos);
    EXPECT_TRUE(content.find("\"solver_name\"") != std::string::npos);

    std::filesystem::remove_all(kTestDir);
}

// ── Segments CSV: 8 rows for interview track ──────────────────────────

TEST(TelemetryExportTest, SegmentsCsvRowCountAndColumns) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    auto telem = run_aero_with_export(track, veh);

    std::string path = kTestDir + "test_segs.csv";
    telem.write_segments_csv(path);

    auto lines = read_lines(path);
    EXPECT_EQ(lines.size(), 9u);
    EXPECT_TRUE(lines[0].find("segment_id") != std::string::npos);
    EXPECT_TRUE(lines[0].find("mean_ellipse_util") != std::string::npos);

    int col_count = 1;
    for (char c : lines[0]) if (c == ',') ++col_count;
    EXPECT_EQ(col_count, 11);

    std::filesystem::remove_all(kTestDir);
}

// ── Basic preset: no per-sample CSV/JSON, segments CSV still written ──

TEST(TelemetryExportTest, BasicSolverSkipsCsvJson) {
    auto track = build_interview_track();
    auto veh = make_vehicle();

    LapTimeSolver solver;
    auto telem = solver.solve(track, veh, PhysicsConfig::basic());
    EXPECT_EQ(telem.sample_count(), 0u);

    std::string csv_path = kTestDir + "basic_telem.csv";
    std::string seg_path = kTestDir + "basic_segs.csv";

    TelemetryMetadata meta;
    meta.solver_name = "basic";
    meta.mu = veh.mu; meta.g_mps2 = veh.g_mps2;
    meta.a_max_mps2 = veh.max_accel_mps2; meta.mass_kg = veh.mass_kg;
    telem.set_metadata(meta);
    telem.write_segments_csv(seg_path);

    auto seg_lines = read_lines(seg_path);
    EXPECT_EQ(seg_lines.size(), 9u);

    telem.write_csv(csv_path);
    auto csv_lines = read_lines(csv_path);
    EXPECT_EQ(csv_lines.size(), 1u);

    std::filesystem::remove_all(kTestDir);
}

// ── DriverState classification ────────────────────────────────────────

TEST(TelemetryExportTest, DriverStateClassification) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    auto telem = run_aero_with_export(track, veh);

    bool found_accel = false;
    for (std::size_t i = 1; i < telem.sample_count() && i < 20; ++i) {
        if (telem.sample(i).distance_m < 50.0 &&
            telem.sample(i).distance_m > 1.0) {
            EXPECT_EQ(telem.classify(telem.sample(i)), DriverState::ACCEL);
            found_accel = true;
            break;
        }
    }
    EXPECT_TRUE(found_accel);

    bool found_corner = false;
    for (std::size_t i = 0; i < telem.sample_count(); ++i) {
        const auto& s = telem.sample(i);
        if (s.distance_m > 183.0 && s.distance_m < 190.0 &&
            s.type == SegmentType::arc) {
            auto ds = telem.classify(s);
            EXPECT_EQ(ds, DriverState::CORNERING)
                << "At s=" << s.distance_m << " a_long=" << s.accel_g
                << " a_lat=" << s.lat_accel_g;
            found_corner = true;
            break;
        }
    }
    EXPECT_TRUE(found_corner);
}

// ── CSV round-trip: row count and total time ──────────────────────────

TEST(TelemetryExportTest, CsvRoundTrip) {
    auto track = build_interview_track();
    auto veh = make_vehicle();
    auto telem = run_aero_with_export(track, veh);

    std::string path = kTestDir + "test_rt.csv";
    telem.write_csv(path);

    auto lines = read_lines(path);
    ASSERT_EQ(lines.size(), telem.sample_count() + 1);

    auto parse_t = [](const std::string& line) -> double {
        std::istringstream ss(line);
        std::string token;
        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        std::getline(ss, token, ',');
        return std::stod(token);
    };

    double last_t = parse_t(lines.back());
    EXPECT_NEAR(last_t, telem.lap_time(), 1e-4);

    std::filesystem::remove_all(kTestDir);
}
