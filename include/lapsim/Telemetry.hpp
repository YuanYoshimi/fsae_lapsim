#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace lapsim {

/// Segment classification for solver results.
enum class SegmentType { straight, arc };

/// Driver state classification for telemetry analysis.
enum class DriverState { ACCEL, BRAKING, CORNERING, COAST };

/// Per-segment solver result.
struct SegmentResult {
    std::string id;
    SegmentType type   = SegmentType::straight;
    double length      = 0.0;   ///< Arc length [m]
    double v_entry     = 0.0;   ///< Entry speed [m/s]
    double v_exit      = 0.0;   ///< Exit speed [m/s]
    double v_peak      = 0.0;   ///< Peak speed on segment [m/s]
    double time        = 0.0;   ///< Time to traverse [s]
};

/// A single point-in-time telemetry sample along the lap.
struct TelemetrySample {
    double distance_m      = 0.0;  ///< Arc-length from start [m]
    double time_s          = 0.0;  ///< Elapsed time [s]
    double x               = 0.0;  ///< World x [m]
    double y               = 0.0;  ///< World y [m]
    double heading_rad     = 0.0;  ///< Heading [rad]
    double speed_mps       = 0.0;  ///< Vehicle speed [m/s]
    double accel_g         = 0.0;  ///< Longitudinal acceleration [g] (net, incl. drag)
    double lat_accel_g     = 0.0;  ///< Lateral acceleration [g]
    double g_total         = 0.0;  ///< Total acceleration magnitude [g]
    double F_drag_N        = 0.0;  ///< Aerodynamic drag force [N]
    double F_downforce_N   = 0.0;  ///< Aerodynamic downforce [N]
    double a_lat_max_mps2  = 0.0;  ///< Speed-dependent lateral grip cap [m/s^2] (0 = static mu*g)
    std::size_t sector     = 0;    ///< Segment index
    SegmentType type       = SegmentType::straight;
    std::string segment_id;        ///< Segment name ("S1", "C1", etc.)
};

/// Solver/vehicle metadata stored alongside telemetry for export.
struct TelemetryMetadata {
    std::string solver_name;
    double ds_m        = 0.5;
    double mu          = 0.0;
    double g_mps2      = 9.81;
    double a_max_mps2  = 0.0;
    double mass_kg     = 0.0;
    double CdA         = 0.0;
    double ClA         = 0.0;
    double rho         = 0.0;
};

/// Container for a full lap of telemetry data with CSV/JSON export.
class Telemetry {
public:
    Telemetry() = default;

    // ── Point samples ─────────────────────────────────────────────────

    void add_sample(const TelemetrySample& sample);

    [[nodiscard]] auto sample_count() const -> std::size_t;
    [[nodiscard]] auto sample(std::size_t i) const -> const TelemetrySample&;

    // ── Segment-level results ─────────────────────────────────────────

    void add_segment_result(const SegmentResult& r);

    [[nodiscard]] auto segment_results() const -> const std::vector<SegmentResult>&;

    // ── Lap time ──────────────────────────────────────────────────────

    void set_total_lap_time(double t);

    [[nodiscard]] auto lap_time() const -> double;

    // ── Metadata ──────────────────────────────────────────────────────

    void set_metadata(const TelemetryMetadata& m);
    [[nodiscard]] auto metadata() const -> const TelemetryMetadata&;

    // ── Segment ID assignment (post-solve, avoids modifying solvers) ──

    void assign_segment_ids(const std::vector<std::string>& ids);

    // ── Driver state classification ───────────────────────────────────

    /// Threshold [m/s^2] for accel/brake/cornering detection.
    void set_driver_state_threshold(double thresh_mps2);

    [[nodiscard]] auto classify(const TelemetrySample& s) const -> DriverState;

    // ── Export ─────────────────────────────────────────────────────────

    void write_csv(const std::string& path = "output/telemetry.csv") const;
    void write_json(const std::string& path = "output/telemetry.json") const;
    void write_segments_csv(const std::string& path = "output/segments.csv") const;

    /// Legacy stub kept for backward compatibility.
    void export_csv(const std::string& path) const;

private:
    std::vector<TelemetrySample> samples_;
    std::vector<SegmentResult> seg_results_;
    double total_lap_time_ = -1.0;
    TelemetryMetadata meta_;
    double driver_thresh_mps2_ = 0.5;
};

[[nodiscard]] auto driver_state_name(DriverState ds) -> const char*;

} // namespace lapsim
