# FSAE Lap Time Simulator

A quasi-steady-state lap time simulator for Formula SAE vehicles, written in modern C++20.

## Overview

This simulator computes the fastest possible lap time for an FSAE car around a
given track by combining forward and backward integration passes constrained by
the friction circle. It supports configurable vehicle parameters, multiple tire
models, aerodynamic effects, and outputs telemetry as CSV or via a real-time 2-D
visualizer.

## Building

```bash
# Configure (fetches dependencies automatically via CMake FetchContent)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build everything (library, CLI, visualizer, tests)
cmake --build build -j$(nproc)

# Run tests
ctest --test-dir build --output-on-failure

# Run the CLI
./build/lapsim_cli config/default.yaml

# Run the visualizer
./build/lapsim_viz config/default.yaml
```

### Requirements

* CMake >= 3.20
* A C++20 compiler (GCC 11+, Clang 14+, MSVC 19.30+)
* Internet connection for the first build (FetchContent downloads dependencies)

### Dependencies (fetched automatically)

| Library    | Purpose                |
|------------|------------------------|
| yaml-cpp   | YAML config/track I/O  |
| raylib     | 2-D visualisation      |
| GoogleTest | Unit testing framework |

## Project Structure

```
fsae_lapsim/
├── CMakeLists.txt          # Build system
├── README.md
├── config/default.yaml     # Vehicle parameters
├── tracks/                 # Track definition files
├── include/lapsim/         # Public headers
├── src/                    # Library implementation
├── apps/                   # Executables (CLI + visualizer)
├── tests/                  # GoogleTest test suites
└── output/                 # Simulation output (CSV, etc.)
```

## Visualizer

An animated 4-panel telemetry playback tool that reads CSV output from any solver
and renders a motorsport-engineer-style display. Supports an optional ghost car
for solver-vs-solver comparison. Primary car renders in cyan, ghost in amber.
Solver names appear at the top of the HUD panel.

### Running the Visualizer

```bash
# Basic: replay a single telemetry CSV
./build/lapsim_viz tracks/interview_track.yaml output/telemetry.csv

# With ghost car (compare two solvers):
./build/lapsim_cli config/default.yaml --solver=qss  --csv=output/qss.csv
./build/lapsim_cli config/default.yaml --solver=aero --csv=output/aero.csv
./build/lapsim_viz tracks/interview_track.yaml output/aero.csv output/qss.csv
```

### Layout

| Panel | Position | Content |
|-------|----------|---------|
| Track View | Left (1100x720) | Top-down track with animated car, colored trail, segment labels |
| Telemetry HUD | Right top (500x430) | Time, speed, segment, driver state, G-force bars, lap progress |
| G-G Diagram | Right bottom (500x290) | Lateral vs longitudinal g with friction ellipse and history trail |
| Speed Trace | Bottom (1600x180) | Speed vs distance colored by driver state, ghost overlay |

### Controls

| Key | Action |
|-----|--------|
| `Space` | Toggle pause/play |
| `←` / `→` | Step backward/forward by one sample (when paused) |
| `-` / `+` | Halve/double playback speed (0.125x to 8x) |
| `R` | Reset to t=0, clear trail and sector splits |
| `G` | Toggle ghost visibility (requires ghost CSV, ghost shown in amber) |
| `S` | Save screenshot to `output/screenshots/` |
| `ESC` | Quit |

### Screenshots

Screenshots are saved to `output/screenshots/` with filenames based on the
current simulation time.

## Completed Phases

| Phase | Description |
|-------|-------------|
| **0** | Project scaffolding, build system, stub classes |
| **1** | Track geometry — segments, arcs, YAML loader, validation |
| **2** | Basic point-mass solver with per-segment results |
| **3** | Forward/backward QSS solver with full velocity profile |
| **4** | Friction circle solver (elliptical combined grip constraint) |
| **5** | Aerodynamics solver (drag + downforce, speed-dependent grip) |
| **6** | Telemetry export — CSV, JSON, segments CSV, driver state |
| **7** | Animated visualizer — 4-panel playback with ghost comparison |

## License

Internal / private project.
