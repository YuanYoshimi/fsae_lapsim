#pragma once

#include "lapsim/Telemetry.hpp"
#include "lapsim/Track.hpp"

namespace lapsim {

/// Abstract interface for rendering telemetry + track data.
class Visualizer {
public:
    virtual ~Visualizer() = default;

    /// Display the visualisation (blocking until window close). (stub)
    virtual void run(const Track& track, const Telemetry& telemetry) = 0;
};

/// Raylib-based 2-D visualizer with HUD overlay.
class RaylibVisualizer final : public Visualizer {
public:
    /// @param width  Window width in pixels.
    /// @param height Window height in pixels.
    explicit RaylibVisualizer(int width = 1280, int height = 720);

    void run(const Track& track, const Telemetry& telemetry) override;

private:
    int width_;
    int height_;
};

} // namespace lapsim
