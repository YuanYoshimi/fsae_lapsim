#pragma once

#include "lapsim/Telemetry.hpp"

#include <string>
#include <vector>

namespace lapsim {

/// Reads telemetry CSV files produced by Phase 6 write_csv().
/// Parses header to validate columns, stores samples in memory.
class TelemetryReader {
public:
    /// Load a CSV file. Prints warnings for malformed lines but never crashes.
    /// @param csv_path Path to the telemetry CSV.
    /// @return true if at least some samples were loaded.
    [[nodiscard]] auto load(const std::string& csv_path) -> bool;

    /// Optionally load metadata from a companion JSON file.
    /// If csv_path is "output/telemetry.csv", tries "output/telemetry.json".
    void load_metadata_json(const std::string& json_path);

    [[nodiscard]] auto samples() const -> const std::vector<TelemetrySample>&;
    [[nodiscard]] auto metadata() const -> const TelemetryMetadata&;
    [[nodiscard]] auto total_time_s() const -> double;
    [[nodiscard]] auto sample_count() const -> std::size_t;

    /// Interpolate a sample at a given simulated time.
    /// Clamps to [0, total_time]. Linear interpolation between neighbors.
    [[nodiscard]] auto at_time(double t_s) const -> TelemetrySample;

    /// Interpolate a sample at a given arc-length distance.
    [[nodiscard]] auto at_distance(double s_m) const -> TelemetrySample;

private:
    std::vector<TelemetrySample> samples_;
    TelemetryMetadata meta_;

    /// Linearly interpolate between two samples.
    [[nodiscard]] static auto lerp_sample(const TelemetrySample& a,
                                           const TelemetrySample& b,
                                           double alpha) -> TelemetrySample;
};

} // namespace lapsim
