# Changelog

## Unreleased

- `ROADMAP.md`: 20 prioritised next steps with scope, dependencies, acceptance criteria and estimates.
- `docs/verification.md`: what is implemented, verified, partial and unverified, the checks run on a fresh
  clone, and a manual checklist for a real window. The README gains a status summary.
- `AGENTS.md` now states the repository's own rules (boundaries, verification commands) instead of the
  instructions it was first written under.

## 1.0.0

The first release. Everything below is new in it; the history of how it grew is in `git log`.

### Simulation

- A deterministic grid-warehouse simulation: robots with an explicit state machine (idle, navigating,
  loading, unloading, charging, unavailable), a battery and charging model, and a motion model with
  acceleration, braking and turning that does not depend on the step length.
- Tasks with priorities, a central queue with exclusive claims, and a greedy dispatcher that weighs
  availability, path distance and priority.
- Shortest routes with A\* (deterministic, allocation-free when warm); collision avoidance with a
  space-time reservation table; cooperative prioritized planning with waiting; detection of blocked
  robots and deadlocks, with recovery.
- Four built-in layouts (`demo`, `compact`, `fulfilment`, and `large` for hundreds of robots) and a
  versioned, validated JSON map format.
- Optional planning threads (`--threads N`) that change how long a run takes and nothing else.
- Deterministic recording and replay with state checksums, and simulated performance metrics with a
  dashboard and CSV export.

### Programs

- `warehousesim_cli`: headless runs, `--robots`, `--spawn`, `--threads`, `--metrics`, `--record`,
  `--replay`, and `--audit`, which checks every step against the simulator's invariants.
- `warehousesim_gui`: the fleet interface (select robots, add tasks, change priorities, pause, speed,
  restart, metrics dashboard), driven through a command queue.
- `warehousesim_editor`: draw and save maps, with undo and redo.
- `--version` on all three.

### Quality

- 1,500+ tests: unit tests per module, end-to-end runs over layouts, fleet sizes and task loads,
  randomised halls, collision invariants on every step, replay of command-driven runs, and the
  examples; mutation-checked.
- AddressSanitizer, UBSan (with the float checks) and ThreadSanitizer configurations, clang-tidy and
  clang-format at pinned versions, and a performance regression gate with a committed baseline.
- Six example warehouses in `examples/`.
- CI for Linux and macOS (experimental GCC and MSVC jobs), and a release workflow that builds, tests and
  packages.

### Fixed during the release checks

- **A robot could move two cells in one tick.** A blocked robot could push aside a blocker that had
  already taken its own step in the same tick. Found by the invariant auditor in 5 of 328 audited
  campaign runs (crowded maps with 16 to 64 robots); the three reports read in full showed no overlap or
  swap. Fixed, with a
  regression test, and the engine revision is now 2: a recording made by revision 1 replays exactly
  unless its run hit this, and says so if it does not.
- `SimulationRunner::warehouse()` returned the layout as it was when the runner was built, so it did not
  show walls put up or taken down by commands while the run went on. It now returns the current layout.
- The GUI program could not be built with CMake at all: the editor program and the editor library had
  the same target name.
- A packaged GUI would not have started on another machine: SDL3 is now linked statically.
- UBSan findings were printed and ignored; they now fail the test.
- 22 clang-tidy findings (moves with no effect, a missing `reserve`, a sort whose comparator the
  analyser could not follow, widening multiplications, a helper that could return a dangling reference),
  none of them a wrong result.
- A packaged GUI would have needed `libSDL3` beside it; a downloaded SDL3 is now linked statically.

### Found and not fixed

- Fleets much bigger than the work can use do worse than smaller ones: in the audited campaign, 16 robots
  at a light load completed fewer tasks than 4 in 9 of 10 layouts. Collisions never happen; robots block
  each other until a task is given up. The likely cause, not proven, is that idle robots are not parked
  (docs/benchmarks.md).

### Known limitations

See [docs/architecture.md](docs/architecture.md#limitations): planning is neither complete nor optimal;
one-cell corridors with three or more robots can deadlock (safely); idle robots are not parked, so a
light load on a big fleet can leave some tasks stuck behind them; state checksums are verified on
macOS (arm64) only until CI has run elsewhere.
