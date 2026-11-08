# WarehouseSim: project boundaries and working agreement

- Work only in `/Users/mk-rune/test_projects/07-warehousesim`. This is one of fourteen independent repositories.
- Do not read, modify, clean up, or run commands in sibling repositories or the user's configuration.
- Work autonomously: choose sensible defaults and document assumptions; do not ask questions or delegate.
- Never install anything globally or use sudo, Homebrew, launchd, host credentials, or host Docker sockets.
- Dependencies, toolchains, caches, browser binaries, SDKs, temporary files and test data stay in this repository. Preserve the supplied project-local environment. Python packages go in `.venv`.
- `.agent-local/` is ignored, project-local build/runtime state. `.opencode/` is managed orchestration configuration; do not change or remove it or bypass the sandbox.
- Use existing system tools read-only, or install pinned tools under this repository. Do not change shell profiles, user settings, Keychain, or global Git settings.
- Bind test servers to loopback; use ports 21600–21699. Limit build/test workers to two unless a required benchmark explicitly needs more.
- Docker-dependent integration tests must remain reproducible but be reported as unverified here if they require access outside this repository. Do not bypass this boundary.
- Do not touch attached hardware, enroll services, flash firmware, install agents, or alter OS settings. Supply manual checklists for such steps.
- Implement only the current numbered prompt, preserving prior behavior. Run appropriate checks, fix failures, and report genuine limitations without fabricating test or benchmark results.
- Before finishing every numbered turn: stop its servers/watchers, remove scratch outputs, update relevant documentation, and make a local conventional-commit Git commit. Do not push.
- End each turn with a short summary of changes, checks actually run, remaining limitations, and commit hash. The orchestrator will provide the next prompt only after this turn ends.

Extend this file with the project's language-specific architecture and verification commands while retaining these boundaries.

## WarehouseSim (C++20) architecture and engineering rules

### Layout and dependency direction

Each module is a static library `warehousesim::<module>` in `src/<module>/`, with public headers under
`src/<module>/include/warehousesim/<module>/` and sources next to them. Dependencies only point inward:

| Module       | Responsibility                                        | May depend on                        |
|--------------|-------------------------------------------------------|--------------------------------------|
| `core`       | points, ids, clock, deterministic RNG                 | std only                             |
| `world`      | grid model: cell types, static layout, robot occupancy, predefined layouts | core            |
| `robots`     | robot entity: pose, battery, task, path, state machine | core                                |
| `planning`   | `PathPlanner` interface, A* with options, results, reused workspace | core, world            |
| `tasks`      | task lifecycle, assignment, task generation           | core, robots                         |
| `simulation` | tick engine, runner thread, JSON, demo scenario       | all of the above, nlohmann_json      |
| `testing`    | header-only test/bench support: reference BFS, map generators, allocation counter | planning (types only) |
| `view`       | camera, scene (draw list), input -> view logic; no SDL | core, world, simulation             |
| `rendering`  | SDL3 window: events in, draw list out (GUI only)      | view, SDL3                           |

`apps/cli` (headless) and `apps/gui` (SDL3) are thin `main()`s. Tests are in `tests/`.

### Clean architecture

- Inner modules never include outer ones; the build enforces this through `target_link_libraries`. Do not
  add a link just to reach a header — move the type inward instead.
- Only `simulation` knows about JSON, and only `rendering` (private) knows about SDL. The simulation
  library, `view`, the CLI and every test must build and run with no display and no SDL.
- Everything the GUI shows is decided in `view`: `buildScene()` turns simulation state plus the `Camera`
  into a plain `DrawList`, and `ViewController` turns an SDL-free `InputEvents` into camera, pause and
  speed changes. `rendering` only translates SDL events to `InputEvents` and executes draw commands, so
  keep it free of decisions that can be tested.
- The simulation advances by a fixed timestep (`core::FixedTimestep`, driven by `SimulationRunner` on its
  own thread), never by frame time. Frame rate, window resizing and rendering speed must not change what
  the simulation computes. Robot behaviour is out of scope for the view code: it only displays state.
- Robot behaviour lives in `robots` (the entity and its state machine) and `simulation` (the decisions
  that need the world, planner and tasks). A robot changes state only through `Robot`'s named operations,
  which validate the transition table and throw `InvalidTransition` without changing anything; never
  assign state fields directly. Keep `Robot::inconsistency()` and `isValidTransition` in step with any
  new state or field, and extend the tests in `tests/robots_test.cpp` (they run in a target linked
  against `core` and `robots` only). Battery and energy are integer units; do not introduce
  floating-point accumulation into anything that affects simulation outcomes (motion progress is the one
  exception and is compared with an epsilon).
- The default `RobotSpec` is the idealised robot that the pre-state-machine tests assume. Do not change
  its defaults; give scenarios their own spec instead.
- The path planner depends only on `core` and `world`; keep it free of robots, tasks, simulation and
  rendering (its tests build into `warehousesim_planning_tests`, linked against `planning` and `testing`
  only). Its results must stay deterministic: ordering keys must be unique and independent of the
  standard library, so changing tie-breaking is a deliberate act that also updates the golden hash in
  `ResultsAreReproducibleAcrossPlatforms`. A* behaviour changes need a matching check against the
  reference search in `tests/astar_reference_test.cpp`. Do not reintroduce per-search allocation:
  `planning_alloc_test.cpp` must stay green.
- Benchmarks (`bench/`) never assert on timing, since that flakes. They cross-check results instead, and
  numbers are only quoted from an optimised build. `src/` must never depend on `testing/`.
- Depend on interfaces at module seams (`PathPlanner`, `TaskAssigner`) and inject implementations through
  constructors. No global or singleton state.
- Keep static layout (`Warehouse`) and dynamic robot occupancy (`Occupancy`) separate; mutate them
  together only through `world::World`, which keeps robots off obstacles and one per cell. New world code
  must not depend on simulation, JSON or SDL: `tests/world_*`, `layouts`, `occupancy`, `warehouse` and `core`
  tests are built into `warehousesim_world_tests`, linked against `core` and `world` only.
- New predefined layouts go in `src/world/layouts.cpp` and must pass `tests/layouts_test.cpp` (one
  connected walkable region, a dropoff, and a pickup source).

### Deterministic tests and behaviour

- Same warehouse, seed, config and inputs must give the same state after the same number of ticks.
- Randomness comes only from `core::Rng` with an explicit seed. Do not use `std::rand`, `std::random_device`,
  `std::*_distribution`, or unordered-container iteration order to make decisions.
- Simulated time is `core::Tick` / `SimClock`. Simulation logic never reads wall-clock time; only
  `SimulationRunner` and the GUI sleep.
- Tests must not depend on real time for correctness. Threaded tests wait on `waitForTick` (with a
  generous timeout), never on fixed sleeps; sleeps are allowed only to check that something does *not* happen.
- Tests are not allowed to touch the network, the display, or files outside the build tree.
- Fix bugs by adding a regression test first or alongside. Never weaken or delete a test to make it pass.

### Memory safety and resource cleanup

- RAII everywhere: `std::unique_ptr`, containers, `std::scoped_lock`/`unique_lock`. No raw `new`/`delete`,
  no `malloc`, no manual `SDL_Destroy*` outside the deleters in `rendering/sdl_window.cpp`.
- No owning raw pointers, no `const_cast`, no C-style casts, no uninitialised members. Use `std::span`,
  `std::optional` and `std::string_view` instead of pointer+length pairs.
- Validate at boundaries (constructors, `fromRows`, JSON, CLI arguments) by throwing or returning
  `std::optional`; do not re-check invariants the types already guarantee.
- Every thread is joined before its owner is destroyed (`SimulationRunner::~SimulationRunner`). Shared state is
  guarded by one mutex; readers get value-copy `Snapshot`s.
- SDL init/quit, window and renderer lifetimes are owned by `SdlWindow` and unwind correctly if
  construction throws.
- Code must compile warning-free with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`
  and stay clean under AddressSanitizer, UBSan and ThreadSanitizer.

### Style

- `.clang-format` (Google-based, 100 columns) is authoritative; run `clang-format -i` on changed files.
- `.clang-tidy` is curated; fix findings rather than adding `NOLINT`, and justify any `NOLINT` inline.
- Public headers use `#pragma once`, are self-contained, and expose the minimum needed.

### Verification after every change

Run from the repository root (dependencies are fetched by CMake at pinned versions or found on the system;
keep all build trees under `build/`, which is git-ignored):

```sh
cmake --preset default                  # Debug + SDL3 GUI + tests, warnings as errors
cmake --build --preset default
ctest --preset default
build/default/apps/warehousesim_cli --ticks 200 --seed 1   # headless smoke test
build/default/apps/warehousesim_cli --layout compact --ticks 200   # also: fulfilment
SDL_VIDEODRIVER=dummy build/default/apps/warehousesim_gui --frames 30   # GUI smoke test, no window
# (ctest also runs the headless CLI on every layout and checks that equal seeds give equal output)
cmake --preset asan && cmake --build --preset asan && ctest --preset asan   # ASan + UBSan
cmake --preset tsan && cmake --build --preset tsan && ctest --preset tsan   # ThreadSanitizer
cmake --preset tidy && cmake --build --preset tidy                          # clang-tidy
find src apps tests \( -name '*.cpp' -o -name '*.hpp' \) | xargs clang-format --dry-run --Werror
```

Use `--preset headless` when SDL3 cannot be built or fetched. Report any check you could not run, and why,
rather than skipping it silently. CI (`.github/workflows/ci.yml`) runs the same checks.

### Dependencies

Pinned in `cmake/WarehouseSimDependencies.cmake` (found via `find_package` first, otherwise fetched):
nlohmann_json 3.12.0, GoogleTest 1.17.0, SDL3 3.2.0. Install tools only inside this repository (`.tools/`,
`.venv/`); never globally. Do not add a dependency for a few lines of code.
