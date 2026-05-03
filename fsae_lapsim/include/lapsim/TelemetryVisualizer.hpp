#pragma once

#include "lapsim/TelemetryReader.hpp"
#include "lapsim/Track.hpp"

#include <string>

namespace lapsim {

class TelemetryVisualizer {
public:
    struct Config {
        int window_width = 1600;
        int window_height = 900;
        int target_fps = 60;
        double initial_playback_speed = 1.0;
        bool show_ghost = false;
        std::string primary_label;
        std::string ghost_label;
    };

    TelemetryVisualizer(const Track& track,
                         const TelemetryReader& primary,
                         const TelemetryReader* ghost,
                         const Config& cfg);

    void run();

private:
    const Track& track_;
    const TelemetryReader& primary_;
    const TelemetryReader* ghost_;
    Config cfg_;
};

} // namespace lapsim
