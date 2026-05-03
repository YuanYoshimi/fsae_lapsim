# FSAE Lap Sim — Project Notes

Living document tracking project state, key decisions, and known issues.
Updated at the end of each phase.

---

## Current State (as of 2026-05-02)

- **Tests:** 80/80 passing
- **Solver:** LapTimeSolver with PhysicsConfig presets (basic, qss, fc, aero)
- **Visualizer:** 4-panel raylib viewer with ghost-car comparison + track boundaries
- **Most recent phase:** Phase 9 complete (track width + boundaries)
- **Next phase:** Phase 10 (racing line representation)

---

## Lap Times (current canonical values)

| Preset | Lap Time | Physics Captured |
|---|---|---|
| basic | 25.290 s | Constant corner speed, kinematic straights |
| qss | 25.157 s | Continuous v(s) profile (forward + backward) |
| fc | 25.424 s | Combined longitudinal + lateral grip (elliptical) |
| aero | 25.085 s | Speed-dependent grip (downforce) + drag |

---

## Track Geometry

- 8 segments, 384.23 m total, validates closed (zero gap)
- CCW lap from origin, initial heading π (−x direction)
- Track width: 4.0 m (constant, FSAE autocross typical)
- All in `tracks/interview_track.yaml`

| ID | Type | Spec | Length (m) |
|---|---|---|---|
| S1 | Straight | top | 100.00 |
| C1 | Arc | R=22.5, +180° | 70.69 |
| C2 | Arc | R=20.0, −90° (concave) | 31.42 |
| C3 | Arc | R=20.0, +90° | 31.42 |
| S2 | Straight | bottom | 60.00 |
| C4 | Arc | R=5.0, +90° | 7.85 |
| S3 | Straight | right side | 75.00 |
| C5 | Arc | R=5.0, +90° | 7.85 |

---

## Vehicle Parameters (config/default.yaml)

- mu = 0.7
- g = 9.81 m/s²
- a_max = 14 m/s²
- mass = 250 kg
- rho = 1.225 kg/m³
- CdA = 1.0 m²
- ClA = 1.5 m²

---

## Architecture

- C++20, CMake (FetchContent), raylib + yaml-cpp + GoogleTest
- Modular: Geometry → Track → Solver → Telemetry → Visualizer
- Single LapTimeSolver with PhysicsConfig (3 bool flags + ds) replaces old strategy pattern
- TireModel and Aero also abstract for future extension

---

## Phase History

### Phase 0: Scaffolding (DONE)

CMake project, header skeletons, stub implementations, raylib window opens.

### Phase 1: Track geometry (DONE)

Vec2, Straight + Arc segment math, Track validation, YAML loader, outline rendering. Caught 30 m closure gap from misread of original drawing (removed phantom S2 between C1 and C2).

### Phase 2: Basic solver (DONE)

Hand-verified at 25.290 s. C1→C2 velocity discontinuity flagged as known limit (resolved in Phase 3).

### Phase 3: QSS solver (DONE)

Forward/backward passes with ds=0.5 m. Initial bug: segment-results extraction used wrong sample indices at boundaries. Fixed via boundary interpolation. Time conservation verified to 1e-6 s.

### Phase 4: Friction circle (DONE)

Initially circular, capped acceleration at mu*g = 6.87 m/s² which ignored a_max = 14 m/s². Switched to elliptical formula (a_lat/(mu*g))² + (a_long/a_max)² ≤ 1. Lap time 25.424 s (slightly slower than QSS — that's the cost of physics).

### Phase 5: Aero (DONE)

Drag and downforce, speed-dependent v_cap. Closed-form solution for v_cap_aero. Lap time 25.085 s (downforce gain exceeds drag cost on this track). C1 alone gains 0.16 s from downforce.

### Phase 6: Telemetry export (DONE)

CSV + JSON + per-segment summary CSV. Added DriverState classification, mean_ellipse_util column, BasicSolver skips per-sample CSV with clear message.

### Phase 7: Visualizer (DONE)

4-panel raylib viewer. Track view with car triangle and color-coded trail. HUD with PRIMARY/GHOST labels and time delta. G-G diagram with friction ellipse and history dots. Speed trace with current-position marker. Ghost car in amber, primary in cyan.

### Phase 8: Solver Consolidation (DONE)

Replaced four parallel solver classes (BasicSolver, QssSolver, FrictionCircleSolver, AeroSolver) with a single `LapTimeSolver` class that branches on a `PhysicsConfig` struct with three boolean flags (`continuous_profile`, `friction_circle`, `aero`). Four named presets reproduce canonical lap times bit-identically. CLI uses `--preset=basic|qss|fc|aero` (default: aero) and `--ds=<float>`. Old `Solver` abstract base class and all four concrete classes deleted. Test count 54→72, all passing.

### Phase 9: Track Width + Boundaries (DONE)

Added constant track width (4.0 m) to geometry layer. YAML `width_m` field parsed by TrackLoader with 4.0 m default + stderr warning. Segment base class gained virtual `left/right_boundary_point(s, width)` — Straight uses perpendicular offset, Arc uses radial offset with inside/outside logic depending on turn direction. Guard throws `std::runtime_error` if inside radius < 0.1 m. Track delegates via arc-length lookup. Visualizer renders left/right boundaries as dim gray (80,80,90) lines at 1.5px before the centerline; bounding box includes boundary extent. CLI header shows width. All four lap times unchanged (no solver/physics touched). Test count 72→80.

---

## Known Limitations (and how I'd address them)

- **No racing line** — currently solving along the geometric centerline. Phases 10-11 add this.
- **Constant μ (no load sensitivity)** — would add Pacejka tire model in TireModel strategy slot.
- **No weight transfer** — would extend point mass to 4-wheel model.
- **Constant a_max (no torque curve)** — would add P/(m·v) constraint.
- **Fixed racing line within track width** — true SQP optimization is Level 3 work; current plan is hand-coded out-in-out parameterization.
- **No tire temperature/wear** — would need thermal model + wear integration.
- **No banking or grade** — would modify normal force expression.
- **First-order Euler integration** — ~3-4% overshoot at transitions; RK2/RK4 would tighten it.

---

## Known Bugs / Tech Debt

(none currently — list here as they appear)

---

## Build & Run Quick Reference

```bash
# Build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(sysctl -n hw.ncpu)

# Run tests
ctest --test-dir build --output-on-failure

# Run a preset
./build/lapsim_cli config/default.yaml --preset=aero --csv=output/aero.csv

# Generate all four CSVs
for p in basic qss fc aero; do
  ./build/lapsim_cli config/default.yaml --preset=$p --csv=output/$p.csv
done

# Visualize one solver
./build/lapsim_viz tracks/interview_track.yaml output/aero.csv

# Visualize with ghost
./build/lapsim_viz tracks/interview_track.yaml output/aero.csv output/qss.csv
```

---

## Workflow Rules I'm Following

1. Small focused changes per Cursor prompt
2. Read AI-generated code before accepting
3. Tests pass before every commit
4. New chat per phase with explicit handoff
5. Commit after every successful sub-task
6. Update this NOTES.md at the end of each phase
