#include "lapsim/Telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <numbers>

namespace lapsim {

// ── Basic accessors ───────────────────────────────────────────────────

void Telemetry::add_sample(const TelemetrySample& sample) {
    samples_.push_back(sample);
}

auto Telemetry::sample_count() const -> std::size_t {
    return samples_.size();
}

auto Telemetry::sample(std::size_t i) const -> const TelemetrySample& {
    return samples_.at(i);
}

void Telemetry::add_segment_result(const SegmentResult& r) {
    seg_results_.push_back(r);
}

auto Telemetry::segment_results() const -> const std::vector<SegmentResult>& {
    return seg_results_;
}

void Telemetry::set_total_lap_time(double t) {
    total_lap_time_ = t;
}

auto Telemetry::lap_time() const -> double {
    if (total_lap_time_ >= 0.0) return total_lap_time_;
    if (samples_.empty()) return 0.0;
    return samples_.back().time_s;
}

// ── Metadata ──────────────────────────────────────────────────────────

void Telemetry::set_metadata(const TelemetryMetadata& m) { meta_ = m; }

auto Telemetry::metadata() const -> const TelemetryMetadata& { return meta_; }

// ── Segment ID assignment ─────────────────────────────────────────────

void Telemetry::assign_segment_ids(const std::vector<std::string>& ids) {
    for (auto& s : samples_) {
        if (s.sector < ids.size())
            s.segment_id = ids[s.sector];
    }
}

// ── Driver state ──────────────────────────────────────────────────────

void Telemetry::set_driver_state_threshold(double thresh_mps2) {
    driver_thresh_mps2_ = thresh_mps2;
}

auto Telemetry::classify(const TelemetrySample& s) const -> DriverState {
    double g = (meta_.g_mps2 > 0.0) ? meta_.g_mps2 : 9.81;
    double a_long = s.accel_g * g;
    double a_lat  = std::abs(s.lat_accel_g) * g;

    if (a_long < -driver_thresh_mps2_) return DriverState::BRAKING;
    if (a_long >  driver_thresh_mps2_) return DriverState::ACCEL;
    if (a_lat  >  driver_thresh_mps2_) return DriverState::CORNERING;
    return DriverState::COAST;
}

auto driver_state_name(DriverState ds) -> const char* {
    switch (ds) {
        case DriverState::ACCEL:     return "ACCEL";
        case DriverState::BRAKING:   return "BRAKING";
        case DriverState::CORNERING: return "CORNERING";
        case DriverState::COAST:     return "COAST";
    }
    return "UNKNOWN";
}

// ── Helpers ───────────────────────────────────────────────────────────

namespace {

void ensure_parent_dir(const std::string& path) {
    auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty())
        std::filesystem::create_directories(parent);
}

double compute_ellipse_util(const TelemetrySample& s,
                            const TelemetryMetadata& m) {
    double g     = (m.g_mps2 > 0.0) ? m.g_mps2 : 9.81;
    double mu_g  = m.mu * g;
    double a_max = m.a_max_mps2;
    double mass  = m.mass_kg;

    if (a_max <= 0.0 || mu_g <= 0.0 || mass <= 0.0) return 0.0;

    double a_lat_mps2 = s.lat_accel_g * g;
    double a_lat_cap  = (s.a_lat_max_mps2 > 0.0) ? s.a_lat_max_mps2 : mu_g;
    double a_drive    = s.accel_g * g + s.F_drag_N / mass;

    double lat_r = a_lat_mps2 / a_lat_cap;
    double lon_r = a_drive / a_max;
    return std::sqrt(lat_r * lat_r + lon_r * lon_r);
}

} // namespace

// ── CSV export ────────────────────────────────────────────────────────

void Telemetry::write_csv(const std::string& path) const {
    ensure_parent_dir(path);

    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) return;

    std::fprintf(fp,
        "sample_idx,s_m,t_s,x_m,y_m,heading_deg,v_mps,v_kph,"
        "a_long_mps2,a_lat_mps2,g_long,g_lat,g_total,"
        "F_drag_N,F_downforce_N,a_lat_max_mps2,ellipse_utilization,"
        "sector_idx,segment_id,segment_type,driver_state\n");

    constexpr double kRadToDeg = 180.0 / std::numbers::pi;
    double g = (meta_.g_mps2 > 0.0) ? meta_.g_mps2 : 9.81;

    for (std::size_t i = 0; i < samples_.size(); ++i) {
        const auto& s = samples_[i];
        double heading_deg = s.heading_rad * kRadToDeg;
        double v_kph       = s.speed_mps * 3.6;
        double a_long_mps2 = s.accel_g * g;
        double a_lat_mps2  = s.lat_accel_g * g;
        double u           = compute_ellipse_util(s, meta_);
        const char* seg_t  = (s.type == SegmentType::arc) ? "CORNER" : "STRAIGHT";
        const char* ds_str = driver_state_name(classify(s));

        std::fprintf(fp,
            "%zu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,"
            "%zu,%s,%s,%s\n",
            i, s.distance_m, s.time_s, s.x, s.y, heading_deg, s.speed_mps, v_kph,
            a_long_mps2, a_lat_mps2, s.accel_g, s.lat_accel_g, s.g_total,
            s.F_drag_N, s.F_downforce_N, s.a_lat_max_mps2, u,
            s.sector, s.segment_id.c_str(), seg_t, ds_str);
    }

    std::fclose(fp);
}

// ── JSON export ───────────────────────────────────────────────────────

void Telemetry::write_json(const std::string& path) const {
    ensure_parent_dir(path);

    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) return;

    constexpr double kRadToDeg = 180.0 / std::numbers::pi;
    double g = (meta_.g_mps2 > 0.0) ? meta_.g_mps2 : 9.81;

    // Top-level object
    std::fprintf(fp, "{\n");

    // Metadata
    std::fprintf(fp, "  \"metadata\": {\n");
    std::fprintf(fp, "    \"solver_name\": \"%s\",\n", meta_.solver_name.c_str());
    std::fprintf(fp, "    \"ds_m\": %.4f,\n", meta_.ds_m);
    std::fprintf(fp, "    \"total_lap_time_s\": %.6f,\n", lap_time());
    std::fprintf(fp, "    \"mu\": %.4f,\n", meta_.mu);
    std::fprintf(fp, "    \"g\": %.4f,\n", meta_.g_mps2);
    std::fprintf(fp, "    \"a_max_mps2\": %.4f,\n", meta_.a_max_mps2);
    std::fprintf(fp, "    \"mass_kg\": %.2f,\n", meta_.mass_kg);
    std::fprintf(fp, "    \"CdA\": %.4f,\n", meta_.CdA);
    std::fprintf(fp, "    \"ClA\": %.4f,\n", meta_.ClA);
    std::fprintf(fp, "    \"rho\": %.4f,\n", meta_.rho);

    // Segment summary
    std::fprintf(fp, "    \"segment_summary\": [\n");
    for (std::size_t j = 0; j < seg_results_.size(); ++j) {
        const auto& sr = seg_results_[j];
        const char* type = (sr.type == SegmentType::arc) ? "CORNER" : "STRAIGHT";
        std::fprintf(fp, "      {\"id\": \"%s\", \"type\": \"%s\", \"length_m\": %.3f, \"time_s\": %.6f}%s\n",
                     sr.id.c_str(), type, sr.length, sr.time,
                     (j + 1 < seg_results_.size()) ? "," : "");
    }
    std::fprintf(fp, "    ]\n");
    std::fprintf(fp, "  },\n");

    // Samples array
    std::fprintf(fp, "  \"samples\": [\n");
    for (std::size_t i = 0; i < samples_.size(); ++i) {
        const auto& s = samples_[i];
        double heading_deg = s.heading_rad * kRadToDeg;
        double v_kph       = s.speed_mps * 3.6;
        double a_long_mps2 = s.accel_g * g;
        double a_lat_mps2  = s.lat_accel_g * g;
        double u           = compute_ellipse_util(s, meta_);
        const char* seg_t  = (s.type == SegmentType::arc) ? "CORNER" : "STRAIGHT";
        const char* ds_str = driver_state_name(classify(s));

        std::fprintf(fp, "    {");
        std::fprintf(fp, "\"idx\": %zu, ", i);
        std::fprintf(fp, "\"s_m\": %.6f, ", s.distance_m);
        std::fprintf(fp, "\"t_s\": %.6f, ", s.time_s);
        std::fprintf(fp, "\"x_m\": %.6f, ", s.x);
        std::fprintf(fp, "\"y_m\": %.6f, ", s.y);
        std::fprintf(fp, "\"heading_deg\": %.6f, ", heading_deg);
        std::fprintf(fp, "\"v_mps\": %.6f, ", s.speed_mps);
        std::fprintf(fp, "\"v_kph\": %.6f, ", v_kph);
        std::fprintf(fp, "\"a_long_mps2\": %.6f, ", a_long_mps2);
        std::fprintf(fp, "\"a_lat_mps2\": %.6f, ", a_lat_mps2);
        std::fprintf(fp, "\"g_long\": %.6f, ", s.accel_g);
        std::fprintf(fp, "\"g_lat\": %.6f, ", s.lat_accel_g);
        std::fprintf(fp, "\"g_total\": %.6f, ", s.g_total);
        std::fprintf(fp, "\"F_drag_N\": %.6f, ", s.F_drag_N);
        std::fprintf(fp, "\"F_downforce_N\": %.6f, ", s.F_downforce_N);
        std::fprintf(fp, "\"a_lat_max_mps2\": %.6f, ", s.a_lat_max_mps2);
        std::fprintf(fp, "\"ellipse_utilization\": %.6f, ", u);
        std::fprintf(fp, "\"sector_idx\": %zu, ", s.sector);
        std::fprintf(fp, "\"segment_id\": \"%s\", ", s.segment_id.c_str());
        std::fprintf(fp, "\"segment_type\": \"%s\", ", seg_t);
        std::fprintf(fp, "\"driver_state\": \"%s\"", ds_str);
        std::fprintf(fp, "}%s\n", (i + 1 < samples_.size()) ? "," : "");
    }
    std::fprintf(fp, "  ]\n");
    std::fprintf(fp, "}\n");

    std::fclose(fp);
}

// ── Segments CSV export ───────────────────────────────────────────────

void Telemetry::write_segments_csv(const std::string& path) const {
    ensure_parent_dir(path);

    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) return;

    std::fprintf(fp,
        "segment_id,type,length_m,v_entry_mps,v_exit_mps,"
        "v_peak_mps,time_s,mean_g_lat,mean_g_long,max_g_total,"
        "mean_ellipse_util\n");

    for (const auto& sr : seg_results_) {
        const char* type = (sr.type == SegmentType::arc) ? "CORNER" : "STRAIGHT";

        double sum_g_lat = 0.0, sum_g_long = 0.0, max_g_tot = 0.0;
        double sum_util = 0.0;
        std::size_t n = 0;
        std::size_t seg_j = 0;
        for (std::size_t k = 0; k < seg_results_.size(); ++k) {
            if (seg_results_[k].id == sr.id) { seg_j = k; break; }
        }
        for (const auto& s : samples_) {
            if (s.sector == seg_j) {
                sum_g_lat  += std::abs(s.lat_accel_g);
                sum_g_long += s.accel_g;
                max_g_tot   = std::max(max_g_tot, s.g_total);
                sum_util   += compute_ellipse_util(s, meta_);
                ++n;
            }
        }

        double mean_g_lat  = (n > 0) ? sum_g_lat / static_cast<double>(n) : 0.0;
        double mean_g_long = (n > 0) ? sum_g_long / static_cast<double>(n) : 0.0;
        double mean_util   = (n > 0) ? sum_util / static_cast<double>(n) : 0.0;

        std::fprintf(fp, "%s,%s,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                     sr.id.c_str(), type, sr.length,
                     sr.v_entry, sr.v_exit, sr.v_peak, sr.time,
                     mean_g_lat, mean_g_long, max_g_tot, mean_util);
    }

    std::fclose(fp);
}

// ── Legacy stub ───────────────────────────────────────────────────────

void Telemetry::export_csv(const std::string& path) const {
    write_csv(path);
}

} // namespace lapsim
