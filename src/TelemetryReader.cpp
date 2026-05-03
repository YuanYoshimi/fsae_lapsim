#include "lapsim/TelemetryReader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numbers>
#include <sstream>
#include <string>
#include <unordered_map>

namespace lapsim {

namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;

/// Split a string by a delimiter character.
auto split(const std::string& s, char delim) -> std::vector<std::string> {
    std::vector<std::string> tokens;
    std::istringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, delim))
        tokens.push_back(tok);
    return tokens;
}

/// Build a column-name → index map from the CSV header line.
auto build_column_map(const std::vector<std::string>& cols)
    -> std::unordered_map<std::string, std::size_t> {
    std::unordered_map<std::string, std::size_t> m;
    for (std::size_t i = 0; i < cols.size(); ++i)
        m[cols[i]] = i;
    return m;
}

/// Safe column lookup; returns -1 if not found.
auto col_idx(const std::unordered_map<std::string, std::size_t>& m,
             const std::string& name) -> int {
    auto it = m.find(name);
    return (it != m.end()) ? static_cast<int>(it->second) : -1;
}

/// Safe double parse from token vector.
auto safe_double(const std::vector<std::string>& tokens, int idx) -> double {
    if (idx < 0 || idx >= static_cast<int>(tokens.size())) return 0.0;
    try { return std::stod(tokens[static_cast<std::size_t>(idx)]); }
    catch (...) { return 0.0; }
}

/// Safe size_t parse from token vector.
auto safe_sizet(const std::vector<std::string>& tokens, int idx) -> std::size_t {
    if (idx < 0 || idx >= static_cast<int>(tokens.size())) return 0;
    try { return static_cast<std::size_t>(std::stoull(tokens[static_cast<std::size_t>(idx)])); }
    catch (...) { return 0; }
}

/// Safe string fetch from token vector.
auto safe_string(const std::vector<std::string>& tokens, int idx) -> std::string {
    if (idx < 0 || idx >= static_cast<int>(tokens.size())) return {};
    return tokens[static_cast<std::size_t>(idx)];
}

/// Extract a JSON numeric value from a line like:  "key": 1.23,
auto extract_json_double(const std::string& line) -> double {
    auto colon = line.find(':');
    if (colon == std::string::npos) return 0.0;
    std::string val = line.substr(colon + 1);
    // Strip trailing comma, whitespace, quotes
    std::string cleaned;
    for (char c : val) {
        if (c == ',' || c == '"' || c == ' ' || c == '\t') continue;
        cleaned += c;
    }
    try { return std::stod(cleaned); }
    catch (...) { return 0.0; }
}

/// Extract a JSON string value from a line like:  "key": "value",
auto extract_json_string(const std::string& line) -> std::string {
    auto colon = line.find(':');
    if (colon == std::string::npos) return {};
    std::string val = line.substr(colon + 1);
    auto first_q = val.find('"');
    if (first_q == std::string::npos) return {};
    auto second_q = val.find('"', first_q + 1);
    if (second_q == std::string::npos) return {};
    return val.substr(first_q + 1, second_q - first_q - 1);
}

} // namespace

// ── load ──────────────────────────────────────────────────────────────

auto TelemetryReader::load(const std::string& csv_path) -> bool {
    samples_.clear();

    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        std::cerr << "TelemetryReader: cannot open \"" << csv_path << "\"\n";
        return false;
    }

    std::string header_line;
    if (!std::getline(ifs, header_line)) {
        std::cerr << "TelemetryReader: empty file \"" << csv_path << "\"\n";
        return false;
    }

    auto header_cols = split(header_line, ',');
    auto cmap = build_column_map(header_cols);

    const std::vector<std::string> required = {
        "sample_idx", "s_m", "t_s", "x_m", "y_m", "heading_deg",
        "v_mps", "v_kph", "a_long_mps2", "a_lat_mps2", "g_long", "g_lat",
        "g_total", "F_drag_N", "F_downforce_N", "a_lat_max_mps2",
        "ellipse_utilization", "sector_idx", "segment_id", "segment_type",
        "driver_state"
    };
    for (const auto& name : required) {
        if (cmap.find(name) == cmap.end()) {
            std::cerr << "TelemetryReader: missing column \"" << name
                      << "\" in \"" << csv_path << "\"\n";
            return false;
        }
    }

    int i_s   = col_idx(cmap, "s_m");
    int i_t   = col_idx(cmap, "t_s");
    int i_x   = col_idx(cmap, "x_m");
    int i_y   = col_idx(cmap, "y_m");
    int i_hdg = col_idx(cmap, "heading_deg");
    int i_v   = col_idx(cmap, "v_mps");
    int i_gl  = col_idx(cmap, "g_long");
    int i_gla = col_idx(cmap, "g_lat");
    int i_gt  = col_idx(cmap, "g_total");
    int i_fd  = col_idx(cmap, "F_drag_N");
    int i_fdf = col_idx(cmap, "F_downforce_N");
    int i_alm = col_idx(cmap, "a_lat_max_mps2");
    int i_sec = col_idx(cmap, "sector_idx");
    int i_sid = col_idx(cmap, "segment_id");
    int i_sty = col_idx(cmap, "segment_type");

    std::string line;
    std::size_t line_num = 1;
    while (std::getline(ifs, line)) {
        ++line_num;
        if (line.empty()) continue;

        auto tokens = split(line, ',');

        try {
            TelemetrySample sample;
            sample.distance_m     = safe_double(tokens, i_s);
            sample.time_s         = safe_double(tokens, i_t);
            sample.x              = safe_double(tokens, i_x);
            sample.y              = safe_double(tokens, i_y);
            sample.heading_rad    = safe_double(tokens, i_hdg) * kDegToRad;
            sample.speed_mps      = safe_double(tokens, i_v);
            sample.accel_g        = safe_double(tokens, i_gl);
            sample.lat_accel_g    = safe_double(tokens, i_gla);
            sample.g_total        = safe_double(tokens, i_gt);
            sample.F_drag_N       = safe_double(tokens, i_fd);
            sample.F_downforce_N  = safe_double(tokens, i_fdf);
            sample.a_lat_max_mps2 = safe_double(tokens, i_alm);
            sample.sector         = safe_sizet(tokens, i_sec);
            sample.segment_id     = safe_string(tokens, i_sid);

            std::string seg_type  = safe_string(tokens, i_sty);
            sample.type = (seg_type == "CORNER") ? SegmentType::arc
                                                 : SegmentType::straight;

            samples_.push_back(sample);
        } catch (...) {
            std::cerr << "TelemetryReader: skipping malformed line "
                      << line_num << "\n";
        }
    }

    return !samples_.empty();
}

// ── load_metadata_json ────────────────────────────────────────────────

void TelemetryReader::load_metadata_json(const std::string& json_path) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) return;

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.find("\"solver_name\"") != std::string::npos)
            meta_.solver_name = extract_json_string(line);
        else if (line.find("\"mu\"") != std::string::npos)
            meta_.mu = extract_json_double(line);
        else if (line.find("\"g\"") != std::string::npos &&
                 line.find("\"g_long\"") == std::string::npos &&
                 line.find("\"g_lat\"") == std::string::npos &&
                 line.find("\"g_total\"") == std::string::npos)
            meta_.g_mps2 = extract_json_double(line);
        else if (line.find("\"a_max_mps2\"") != std::string::npos)
            meta_.a_max_mps2 = extract_json_double(line);
        else if (line.find("\"mass_kg\"") != std::string::npos)
            meta_.mass_kg = extract_json_double(line);
        else if (line.find("\"CdA\"") != std::string::npos)
            meta_.CdA = extract_json_double(line);
        else if (line.find("\"ClA\"") != std::string::npos)
            meta_.ClA = extract_json_double(line);
        else if (line.find("\"rho\"") != std::string::npos)
            meta_.rho = extract_json_double(line);
    }
}

// ── accessors ─────────────────────────────────────────────────────────

auto TelemetryReader::samples() const -> const std::vector<TelemetrySample>& {
    return samples_;
}

auto TelemetryReader::metadata() const -> const TelemetryMetadata& {
    return meta_;
}

auto TelemetryReader::total_time_s() const -> double {
    return samples_.empty() ? 0.0 : samples_.back().time_s;
}

auto TelemetryReader::sample_count() const -> std::size_t {
    return samples_.size();
}

// ── at_time ───────────────────────────────────────────────────────────

auto TelemetryReader::at_time(double t_s) const -> TelemetrySample {
    if (samples_.empty()) return {};

    t_s = std::clamp(t_s, 0.0, total_time_s());

    auto it = std::lower_bound(
        samples_.begin(), samples_.end(), t_s,
        [](const TelemetrySample& s, double t) { return s.time_s < t; });

    if (it == samples_.begin()) return samples_.front();
    if (it == samples_.end())   return samples_.back();

    const auto& b = *it;
    const auto& a = *std::prev(it);

    double dt = b.time_s - a.time_s;
    double alpha = (dt > 0.0) ? (t_s - a.time_s) / dt : 0.0;
    return lerp_sample(a, b, alpha);
}

// ── at_distance ───────────────────────────────────────────────────────

auto TelemetryReader::at_distance(double s_m) const -> TelemetrySample {
    if (samples_.empty()) return {};

    double s_max = samples_.back().distance_m;
    s_m = std::clamp(s_m, 0.0, s_max);

    auto it = std::lower_bound(
        samples_.begin(), samples_.end(), s_m,
        [](const TelemetrySample& s, double d) { return s.distance_m < d; });

    if (it == samples_.begin()) return samples_.front();
    if (it == samples_.end())   return samples_.back();

    const auto& b = *it;
    const auto& a = *std::prev(it);

    double ds = b.distance_m - a.distance_m;
    double alpha = (ds > 0.0) ? (s_m - a.distance_m) / ds : 0.0;
    return lerp_sample(a, b, alpha);
}

// ── lerp_sample ───────────────────────────────────────────────────────

auto TelemetryReader::lerp_sample(const TelemetrySample& a,
                                   const TelemetrySample& b,
                                   double alpha) -> TelemetrySample {
    auto mix = [alpha](double va, double vb) { return va + alpha * (vb - va); };

    TelemetrySample out;
    out.distance_m     = mix(a.distance_m,     b.distance_m);
    out.time_s         = mix(a.time_s,         b.time_s);
    out.x              = mix(a.x,              b.x);
    out.y              = mix(a.y,              b.y);
    out.heading_rad    = mix(a.heading_rad,    b.heading_rad);
    out.speed_mps      = mix(a.speed_mps,      b.speed_mps);
    out.accel_g        = mix(a.accel_g,        b.accel_g);
    out.lat_accel_g    = mix(a.lat_accel_g,    b.lat_accel_g);
    out.g_total        = mix(a.g_total,        b.g_total);
    out.F_drag_N       = mix(a.F_drag_N,       b.F_drag_N);
    out.F_downforce_N  = mix(a.F_downforce_N,  b.F_downforce_N);
    out.a_lat_max_mps2 = mix(a.a_lat_max_mps2, b.a_lat_max_mps2);
    out.sector         = a.sector;
    out.type           = a.type;
    out.segment_id     = a.segment_id;
    return out;
}

} // namespace lapsim
