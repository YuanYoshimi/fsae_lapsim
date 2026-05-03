#include "lapsim/RacingLine.hpp"
#include "lapsim/TelemetryReader.hpp"
#include "lapsim/TelemetryVisualizer.hpp"
#include "lapsim/TrackLoader.hpp"

#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

int main(int argc, char* argv[]) {
    std::string track_yaml  = "tracks/interview_track.yaml";
    std::string primary_csv = "output/telemetry.csv";
    std::string ghost_csv;
    bool show_racing_line = false;

    // Separate flags from positional args. Flags can appear in any order.
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        if (a == "--racing-line") {
            show_racing_line = true;
        } else if (a == "--help" || a == "-h") {
            std::printf("usage: lapsim_viz [track.yaml] [primary.csv] [ghost.csv] [--racing-line]\n");
            return 0;
        } else {
            positional.emplace_back(a);
        }
    }
    if (positional.size() > 0) track_yaml  = positional[0];
    if (positional.size() > 1) primary_csv = positional[1];
    if (positional.size() > 2) ghost_csv   = positional[2];

    auto track = lapsim::TrackLoader::load(track_yaml);

    lapsim::TelemetryReader primary;
    if (!primary.load(primary_csv)) {
        std::fprintf(stderr, "Failed to load primary CSV: %s\n", primary_csv.c_str());
        return 1;
    }

    auto json_companion = std::filesystem::path(primary_csv).replace_extension(".json").string();
    if (std::filesystem::exists(json_companion))
        primary.load_metadata_json(json_companion);

    std::printf("Loaded %zu samples for primary (solver: %s, lap: %.3f s)\n",
                primary.sample_count(),
                primary.metadata().solver_name.empty() ? "unknown" : primary.metadata().solver_name.c_str(),
                primary.total_time_s());

    lapsim::TelemetryReader ghost_reader;
    lapsim::TelemetryReader* ghost_ptr = nullptr;
    if (!ghost_csv.empty()) {
        if (ghost_reader.load(ghost_csv)) {
            auto ghost_json = std::filesystem::path(ghost_csv).replace_extension(".json").string();
            if (std::filesystem::exists(ghost_json))
                ghost_reader.load_metadata_json(ghost_json);

            ghost_ptr = &ghost_reader;
            std::printf("Loaded %zu samples for ghost (solver: %s, lap: %.3f s)\n",
                        ghost_reader.sample_count(),
                        ghost_reader.metadata().solver_name.empty() ? "unknown" : ghost_reader.metadata().solver_name.c_str(),
                        ghost_reader.total_time_s());
        } else {
            std::fprintf(stderr, "Warning: could not load ghost CSV: %s\n", ghost_csv.c_str());
        }
    }

    std::optional<lapsim::RacingLine> racing_line;
    if (show_racing_line) {
        racing_line.emplace(track, lapsim::RacingLineParams{});
        std::printf("Racing line: ON (default params)\n");
    }

    lapsim::TelemetryVisualizer::Config cfg;
    cfg.show_ghost = (ghost_ptr != nullptr);

    auto make_label = [](const lapsim::TelemetryReader& r, const std::string& csv_path) {
        const auto& name = r.metadata().solver_name;
        if (!name.empty()) return name;
        return std::filesystem::path(csv_path).filename().string();
    };
    cfg.primary_label = make_label(primary, primary_csv);
    if (ghost_ptr)
        cfg.ghost_label = make_label(ghost_reader, ghost_csv);

    const lapsim::RacingLine* racing_ptr = racing_line.has_value() ? &racing_line.value() : nullptr;
    lapsim::TelemetryVisualizer viz(track, primary, ghost_ptr, racing_ptr, cfg);
    viz.run();

    return 0;
}
