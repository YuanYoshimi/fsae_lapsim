#include "lapsim/Solver.hpp"
#include "lapsim/TrackLoader.hpp"
#include "lapsim/Vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

namespace {

constexpr double kRadToDeg = 180.0 / std::numbers::pi;

void print_geo_row(const lapsim::Segment& seg) {
    auto sp = seg.start_point();
    auto ep = seg.end_point();
    std::printf("  %-4s | %-8s | %8.3f | (%8.3f, %8.3f) | (%8.3f, %8.3f) | %10.3f | %10.3f\n",
                seg.id().c_str(), seg.type_name().c_str(), seg.length(),
                sp.x, sp.y, ep.x, ep.y,
                seg.start_heading() * kRadToDeg, seg.end_heading() * kRadToDeg);
}

void print_geo_header() {
    std::printf("  %-4s | %-8s | %8s | %21s | %21s | %10s | %10s\n",
                "id", "type", "length", "start_pt", "end_pt", "start_hdg", "end_hdg");
    std::printf("  -----+----------+----------+-----------------------"
                "+-----------------------+------------+-----------\n");
}

void print_sim_header() {
    std::printf("  %-4s | %-8s | %9s | %12s | %12s | %12s | %8s\n",
                "id", "type", "length(m)", "v_entry(m/s)", "v_exit(m/s)", "v_peak(m/s)", "time(s)");
    std::printf("  -----+----------+-----------+--------------"
                "+--------------+--------------+----------\n");
}

void print_sim_row(const lapsim::SegmentResult& r) {
    const char* ts = (r.type == lapsim::SegmentType::arc) ? "arc" : "straight";
    std::printf("  %-4s | %-8s | %9.3f | %12.3f | %12.3f | %12.3f | %8.3f\n",
                r.id.c_str(), ts, r.length, r.v_entry, r.v_exit, r.v_peak, r.time);
}

const lapsim::SegmentResult* find_result(const std::vector<lapsim::SegmentResult>& res,
                                         const std::string& id) {
    for (const auto& r : res)
        if (r.id == id) return &r;
    return nullptr;
}

} // namespace

int main(int argc, char* argv[]) {
    std::string vehicle_yaml = "config/default.yaml";
    std::string track_yaml   = "tracks/interview_track.yaml";
    std::string solver_name  = "aero";
    std::string csv_path     = "output/telemetry.csv";
    std::string json_path    = "output/telemetry.json";
    std::string seg_csv_path = "output/segments.csv";

    int pos_arg = 0;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.starts_with("--solver=")) {
            solver_name = arg.substr(9);
        } else if (arg.starts_with("--csv=")) {
            csv_path = arg.substr(6);
        } else if (arg.starts_with("--json=")) {
            json_path = arg.substr(7);
        } else if (arg.starts_with("--segments=")) {
            seg_csv_path = arg.substr(11);
        } else {
            if (pos_arg == 0) vehicle_yaml = arg;
            else if (pos_arg == 1) track_yaml = arg;
            ++pos_arg;
        }
    }

    auto vehicle = lapsim::Vehicle::from_yaml(vehicle_yaml);
    auto track   = lapsim::TrackLoader::load(track_yaml);

    // ── Track summary ─────────────────────────────────────────────────
    std::printf("\n=== Track: %s ===\n", track.name().c_str());
    std::printf("Segments:     %zu\n", track.segment_count());
    std::printf("Total length: %.3f m\n", track.total_length());
    auto val = track.validate();
    std::printf("Validation:   %s\n", val.is_valid ? "PASS" : "FAIL");

    // ── Vehicle summary ───────────────────────────────────────────────
    std::printf("\n=== Vehicle ===\n");
    std::printf("  mu:        %.3f\n", vehicle.mu);
    std::printf("  g:         %.3f m/s^2\n", vehicle.g_mps2);
    std::printf("  max_accel: %.3f m/s^2\n", vehicle.max_accel_mps2);
    std::printf("  mass:      %.1f kg\n", vehicle.mass_kg);
    std::printf("  Aero: rho=%.3f kg/m^3, CdA=%.3f m^2, ClA=%.3f m^2\n",
                vehicle.aero.rho(), vehicle.aero.cda(), vehicle.aero.cla());

    // ── Geometry table ────────────────────────────────────────────────
    std::printf("\n--- Geometry (traversal order) ---\n");
    print_geo_header();
    for (std::size_t i = 0; i < track.segment_count(); ++i)
        print_geo_row(track.segment(i));

    // ── Solver ────────────────────────────────────────────────────────
    std::unique_ptr<lapsim::Solver> solver;
    if (solver_name == "basic") {
        solver = std::make_unique<lapsim::BasicSolver>();
    } else if (solver_name == "qss") {
        solver = std::make_unique<lapsim::QssSolver>(0.5);
    } else if (solver_name == "friction_circle") {
        solver = std::make_unique<lapsim::FrictionCircleSolver>(0.5);
    } else {
        solver = std::make_unique<lapsim::AeroSolver>(0.5);
    }

    std::printf("\n=== Solver: %s ===\n", solver->name().c_str());
    auto telem = solver->solve(track, vehicle);
    const auto& results = telem.segment_results();

    // ── Interview-format table ────────────────────────────────────────
    static const std::vector<std::string> interview_order = {
        "S1", "C1", "C2", "C3", "S2", "C4", "S3", "C5"
    };

    std::printf("\n--- Simulation results (interview order) ---\n");
    print_sim_header();
    for (const auto& sid : interview_order) {
        const auto* r = find_result(results, sid);
        if (r) print_sim_row(*r);
    }
    std::printf("\n  TOTAL LAP TIME: %.3f s\n", telem.lap_time());

    // ── One-line summary table ────────────────────────────────────────
    std::printf("\n--- Interview summary ---\n  ");
    for (const auto& sid : interview_order) std::printf("%-6s| ", sid.c_str());
    std::printf("Total\n  ");
    for (const auto& sid : interview_order) {
        const auto* r = find_result(results, sid);
        if (r) std::printf("%-6.2f| ", r->time);
    }
    std::printf("%.2f\n", telem.lap_time());

    // ── Verification ──────────────────────────────────────────────────
    double seg_sum = 0.0;
    for (const auto& r : results) seg_sum += r.time;
    double profile_total = telem.lap_time();
    bool match = std::abs(seg_sum - profile_total) < 1e-6;
    std::printf("\n  Sum of segment times: %.6f s | Profile-integrated total: %.6f s | Match: %s (tolerance 1e-6)\n",
                seg_sum, profile_total, match ? "Y" : "N");

    // ── Friction Budget Audit (elliptical utilization) ────────────────
    if (telem.sample_count() > 0) {
        double mu_g  = vehicle.mu * vehicle.g_mps2;
        double a_max = vehicle.max_accel_mps2;
        double mass  = vehicle.mass_kg;

        double max_u = 0.0;
        int over_count = 0;
        double sum_u_corner = 0.0, sum_u_straight = 0.0;
        int n_corner = 0, n_straight = 0;

        for (std::size_t i = 0; i < telem.sample_count(); ++i) {
            const auto& s = telem.sample(i);
            double a_lat_mps2 = s.lat_accel_g * vehicle.g_mps2;

            // Use speed-dependent a_lat_max if available, else static mu*g.
            double a_lat_cap = (s.a_lat_max_mps2 > 0.0) ? s.a_lat_max_mps2 : mu_g;

            // Recover tire-only longitudinal component by removing drag.
            double a_drive_mps2 = s.accel_g * vehicle.g_mps2
                                + s.F_drag_N / mass;

            double u = std::sqrt((a_lat_mps2 / a_lat_cap) * (a_lat_mps2 / a_lat_cap) +
                                 (a_drive_mps2 / a_max) * (a_drive_mps2 / a_max));
            max_u = std::max(max_u, u);
            if (u > 1.0 + 1e-3) ++over_count;
            if (s.type == lapsim::SegmentType::arc) {
                sum_u_corner += u; ++n_corner;
            } else {
                sum_u_straight += u; ++n_straight;
            }
        }

        std::printf("\n=== Friction Budget Audit (elliptical) ===\n");
        std::printf("  Max ellipse utilization:       %.4f  (limit = 1.000)\n", max_u);
        std::printf("  Samples exceeding limit by >1e-3: %d  (should be 0)\n", over_count);
        std::printf("  Mean utilization in corners:   %.4f\n",
                    n_corner > 0 ? sum_u_corner / n_corner : 0.0);
        std::printf("  Mean utilization on straights: %.4f\n",
                    n_straight > 0 ? sum_u_straight / n_straight : 0.0);

        // ── ASCII g-g diagram (normalized elliptical coords) ────────
        constexpr int GG = 21;
        constexpr int MID = 10;
        char grid[GG][GG];
        for (auto& row : grid)
            for (auto& c : row) c = '.';

        for (int r = 0; r < GG; ++r)
            for (int c = 0; c < GG; ++c) {
                double nx = (c - MID) / static_cast<double>(MID);
                double ny = (MID - r) / static_cast<double>(MID);
                double d2 = nx * nx + ny * ny;
                if (d2 >= 0.82 && d2 <= 1.18) grid[r][c] = 'o';
            }

        for (int i = 0; i < GG; ++i) {
            if (grid[MID][i] == '.') grid[MID][i] = '-';
            if (grid[i][MID] == '.') grid[i][MID] = '|';
        }
        grid[MID][MID] = '+';

        for (std::size_t i = 0; i < telem.sample_count(); ++i) {
            const auto& s = telem.sample(i);
            double a_lat_mps2  = s.lat_accel_g * vehicle.g_mps2;
            double a_lat_cap   = (s.a_lat_max_mps2 > 0.0) ? s.a_lat_max_mps2 : mu_g;
            double a_drive_mps2 = s.accel_g * vehicle.g_mps2 + s.F_drag_N / mass;
            double nx = a_lat_mps2 / a_lat_cap;
            double ny = a_drive_mps2 / a_max;
            int col = MID + static_cast<int>(std::round(nx * MID));
            int row = MID - static_cast<int>(std::round(ny * MID));
            col = std::clamp(col, 0, GG - 1);
            row = std::clamp(row, 0, GG - 1);
            grid[row][col] = '#';
        }

        std::printf("\n=== g-g Diagram (normalized elliptical: a_lat/a_lat_max right, a_drive/a_max up) ===\n");
        for (int r = 0; r < GG; ++r) {
            const char* label = "      ";
            if (r == 0)       label = " +1.0 ";
            else if (r == MID) label = "    0 ";
            else if (r == GG - 1) label = " -1.0 ";
            std::printf("%s", label);
            for (int c = 0; c < GG; ++c) std::putchar(grid[r][c]);
            std::putchar('\n');
        }
        std::printf("       -1.0      0       +1.0\n");
    }

    // ── Aero Audit (when aero telemetry is populated) ─────────────────
    bool has_aero = telem.sample_count() > 1 && telem.sample(1).a_lat_max_mps2 > 0.0;
    if (has_aero) {
        double peak_drag = 0.0, peak_df = 0.0;
        double v_at_peak_drag = 0.0, v_at_peak_df = 0.0;
        double sum_drag = 0.0, sum_df = 0.0;
        double drag_energy_J = 0.0;
        double sum_alm_corner = 0.0, max_alm_corner = 0.0;
        int n_corner_alm = 0;
        double mu_g = vehicle.mu * vehicle.g_mps2;

        for (std::size_t i = 0; i < telem.sample_count(); ++i) {
            const auto& s = telem.sample(i);
            sum_drag += s.F_drag_N;
            sum_df   += s.F_downforce_N;
            if (s.F_drag_N > peak_drag) {
                peak_drag = s.F_drag_N;
                v_at_peak_drag = s.speed_mps;
            }
            if (s.F_downforce_N > peak_df) {
                peak_df = s.F_downforce_N;
                v_at_peak_df = s.speed_mps;
            }
            if (s.type == lapsim::SegmentType::arc && s.a_lat_max_mps2 > 0.0) {
                sum_alm_corner += s.a_lat_max_mps2;
                max_alm_corner = std::max(max_alm_corner, s.a_lat_max_mps2);
                ++n_corner_alm;
            }

            if (i + 1 < telem.sample_count()) {
                const auto& s1 = telem.sample(i + 1);
                double dt = s1.time_s - s.time_s;
                double power_avg = (s.F_drag_N * s.speed_mps
                                  + s1.F_drag_N * s1.speed_mps) / 2.0;
                drag_energy_J += power_avg * dt;
            }
        }

        auto n = static_cast<double>(telem.sample_count());
        std::printf("\n=== Aero Audit ===\n");
        std::printf("  Peak drag force:        %.1f N at v = %.2f m/s\n", peak_drag, v_at_peak_drag);
        std::printf("  Peak downforce:         %.1f N at v = %.2f m/s\n", peak_df, v_at_peak_df);
        std::printf("  Mean drag (whole lap):  %.1f N\n", sum_drag / n);
        std::printf("  Mean downforce:         %.1f N\n", sum_df / n);
        std::printf("  Drag energy lost:       %.3f kJ over the lap\n", drag_energy_J / 1000.0);
        std::printf("  Effective grip boost in corners:\n");
        std::printf("    Mean a_lat_max in corners: %.2f m/s^2 (vs static mu*g = %.2f)\n",
                    n_corner_alm > 0 ? sum_alm_corner / n_corner_alm : mu_g, mu_g);
        std::printf("    Max a_lat_max in corners:  %.2f m/s^2\n", max_alm_corner);
    }

    // ── Straight v_peak detail ────────────────────────────────────────
    std::printf("\n--- Straight v_peak detail ---\n");
    for (const auto& r : results) {
        if (r.type == lapsim::SegmentType::straight && r.length > 0.0)
            std::printf("  %s: v_i=%.3f  v_f=%.3f  v_peak=%.3f\n",
                        r.id.c_str(), r.v_entry, r.v_exit, r.v_peak);
    }

    std::printf("\n--- Corner v_peak detail ---\n");
    for (const auto& r : results) {
        if (r.type == lapsim::SegmentType::arc)
            std::printf("  %s: v_entry=%.3f  v_exit=%.3f  v_peak=%.3f\n",
                        r.id.c_str(), r.v_entry, r.v_exit, r.v_peak);
    }

    // ── Metadata + segment IDs for export ─────────────────────────────
    {
        lapsim::TelemetryMetadata meta;
        meta.solver_name = solver->name();
        meta.ds_m        = 0.5;
        meta.mu          = vehicle.mu;
        meta.g_mps2      = vehicle.g_mps2;
        meta.a_max_mps2  = vehicle.max_accel_mps2;
        meta.mass_kg     = vehicle.mass_kg;
        meta.CdA         = vehicle.aero.cda();
        meta.ClA         = vehicle.aero.cla();
        meta.rho         = vehicle.aero.rho();
        telem.set_metadata(meta);

        std::vector<std::string> seg_ids;
        for (std::size_t i = 0; i < track.segment_count(); ++i)
            seg_ids.push_back(track.segment(i).id());
        telem.assign_segment_ids(seg_ids);
    }

    // ── Export files ────────────────────────────────────────────────────
    std::printf("\n--- Telemetry Export ---\n");
    bool has_samples = telem.sample_count() > 0;
    if (!has_samples) {
        std::printf("  BasicSolver does not produce per-sample telemetry. Use --solver=qss,\n"
                    "  friction_circle, or aero for CSV output. Segments CSV still written.\n");
    }
    if (!csv_path.empty() && has_samples) {
        telem.write_csv(csv_path);
        auto abs = std::filesystem::absolute(csv_path);
        std::printf("  CSV:      %s\n", abs.string().c_str());
    }
    if (!json_path.empty() && has_samples) {
        telem.write_json(json_path);
        auto abs = std::filesystem::absolute(json_path);
        std::printf("  JSON:     %s\n", abs.string().c_str());
    }
    if (!seg_csv_path.empty()) {
        telem.write_segments_csv(seg_csv_path);
        auto abs = std::filesystem::absolute(seg_csv_path);
        std::printf("  Segments: %s\n", abs.string().c_str());
    }

    // ── CSV preview ─────────────────────────────────────────────────────
    if (!csv_path.empty() && telem.sample_count() > 0) {
        std::ifstream ifs(csv_path);
        if (ifs.is_open()) {
            std::vector<std::string> lines;
            std::string line;
            while (std::getline(ifs, line)) lines.push_back(line);

            std::printf("\n--- CSV Preview (%zu rows incl. header) ---\n", lines.size());
            std::size_t show = 4; // header + 3 data rows
            for (std::size_t i = 0; i < std::min(show, lines.size()); ++i)
                std::printf("  %s\n", lines[i].c_str());
            if (lines.size() > show * 2) {
                std::printf("  ...\n");
                for (std::size_t i = lines.size() - 3; i < lines.size(); ++i)
                    std::printf("  %s\n", lines[i].c_str());
            }
        }
    }

    std::printf("\n");
    return 0;
}
