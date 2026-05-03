---
description: FSAE Lap Time Simulator — project context, architecture, workflow, phase plan
alwaysApply: true
---

# FSAE Lap Time Simulator — AI Workflow Rules

## Project context

This is a C++20 quasi-steady-state lap time simulator for an FSAE Vehicle Dynamics interview. It uses CMake with FetchContent for yaml-cpp, raylib, and GoogleTest. The project is organized into 7 completed phases with 4 solvers (Basic, QSS, FrictionCircle, Aero), per-sample telemetry, CSV/JSON export, and a 4-panel raylib visualizer with ghost-car comparison.

Current state: 54/54 tests passing. Lap times: Basic 25.290 s, QSS 25.157 s, FrictionCircle 25.424 s, Aero 25.085 s. Track is 8 segments totaling 384.23 m, validates closed.

For full project state at any time, read NOTES.md in the repo root.

## Architectural principles (do not violate)

1. Modular separation: Track/Geometry knows nothing about physics. Physics knows nothing about rendering. Visualizer reads CSVs, never calls solvers.
2. Strategy pattern for swappable components: Solver and TireModel are abstract base classes with concrete implementations. New variants are added as new classes, not by modifying existing ones.
3. Configuration is data, not code. All vehicle and track parameters live in YAML, parsed at runtime. No magic numbers in source.
4. Every solver must produce a Telemetry that satisfies time conservation to 1e-6 s (sum of segment times == profile-integrated total).

## Code standards

- Modern C++20 idioms: std::unique_ptr for ownership, std::optional, enum class, [[nodiscard]] on getters, const-correctness, std::filesystem for paths, std::numbers::pi.
- Headers use #pragma once.
- All public APIs documented with brief Doxygen-style comments.
- Compile cleanly with -Wall -Wextra -Wpedantic — no warnings allowed.
- Comment WHY, not WHAT. The code shows what; comments explain rationale.
- Functions over 50 lines are a smell. Split them.

## Workflow rules

1. Small focused changes only. One logical change per prompt. If the user asks for something that requires touching unrelated areas, flag it and ask whether to split.
2. Read existing code before editing. Use the view tool first.
3. Tests are non-negotiable. Every new feature ships with tests. Every change must leave all tests passing.
4. Do NOT modify files outside the explicit scope of the current task. If you must, tell the user before doing it.
5. After completing any change, report:
   - What files were modified or created
   - What tests were added
   - Build and test results
   - Anything unexpected or worth flagging

## Things to NEVER do

- Combine multiple unrelated changes in one edit.
- Refactor "while you're here" without being asked.
- Loosen test tolerances without flagging it.
- Add new dependencies without asking first.
- Modify the four existing solvers' physics without explicit instruction.
- Touch the track YAML's geometry without explicit instruction.
- Use raw new/delete; use smart pointers or stack allocation.
- Use printf-style formatting where std::format would do (C++20 has it).

## Phase plan (track-width feature)

Currently in Phase 8 (track width). Coming phases:

- Phase 8: Track width geometry, boundary rendering. No solver changes.
- Phase 9: RacingLine class, hand-coded geometric line, line curvature.
- Phase 10: Solvers accept paths (centerline OR racing line), measure delta.
- Phase 11: Parameter sweep, visualizer polish, README/PDF updates.

Each phase produces a working buildable codebase with all tests passing.

## When the user asks for help

- Default to terse, technical responses. The user is an FSAE engineering candidate, not a beginner.
- Show diffs or specific code snippets, not whole-file rewrites, when making targeted changes.
- If you spot a real issue or improvement opportunity adjacent to the user's request, mention it BUT do not implement it unless asked.
- When uncertain, ask one focused clarifying question rather than guessing and producing a wrong answer.
