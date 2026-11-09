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
| `core`       | points, ids, clock, deterministic RNG, `PointSet`, `ThreadPool` (bounded, safe shutdown) | std only (and threads) |
| `world`      | grid model: cell types, static layout, robot occupancy (cell plus claimed cells), predefined layouts (`demo`, `compact`, `fulfilment`, and `large`, 100x64, for fleets of hundreds), `Warehouse::version()`, `checkLayout` (can it host a simulation?) | core |
| `robots`     | robot entity: pose, motion model, battery, task, path, state machine | core                  |
| `planning`   | `PathPlanner` interface, A* with options, results, reused workspace; space-time A* (`SpaceTimeAStar`) behind a `SpaceTimeConstraints` interface; `PathBatchExecutor` (many independent path questions on a `ThreadPool`) | core, world |
| `tasks`      | `TaskQueue` lifecycle and exclusive claims, `Dispatcher` (greedy), distance estimators, task generation | core, world, robots, planning |
| `traffic`    | space-time `ReservationTable`, `TrafficController` (collision avoidance), `CooperativeRouter` (prioritized planning), `ProgressMonitor` (blocking and deadlock detection) | core, world, robots, planning |
| `metrics`    | simulated performance figures: `MetricsAccumulator` (plain observations in), `FleetMetrics` (counts and derived rates), CSV and dashboard text | core only |
| `simulation` | tick engine, runner thread and command queue (`Command`, `apply`), JSON, scenarios (`ScenarioOptions`), `MetricsCollector`, benchmark sweeps, run recording and replay (`RunJournal`, `Recorder`, `replay`, the recording file, `stateChecksum`), planning threads (`usePlanningThreads`) | all of the above, metrics, nlohmann_json |
| `testing`    | header-only test/bench support: reference BFS, map generators, allocation counter | planning (types only) |
| `mapio`      | the versioned JSON map file: schema validation, reading, canonical writing | core, world, nlohmann_json (private) |
| `editor`     | `MapEditor` editing model (tools, strokes, undo, resize), `editor_files` (save/load/predefined) | core, world, mapio (private) |
| `view`       | camera, scenes (draw lists), interpolation, input -> view logic (`ViewController`, `FleetController`, `FleetSession`, `EditorController`); no SDL | core, world, simulation, editor |
| `rendering`  | SDL3 window: events in, draw list out (GUI only)      | view, SDL3                           |

`apps/cli` (headless), `apps/gui` (SDL3, the fleet interface) and `apps/editor` (SDL3) are thin `main()`s. Tests are in `tests/`; `bench/` holds the planning, fleet and scaling benchmarks.

### Clean architecture

- Inner modules never include outer ones; the build enforces this through `target_link_libraries`. Do not
  add a link just to reach a header — move the type inward instead.
- Only `simulation` (snapshots) and `mapio` (map files) know about JSON, and only `rendering` (private)
  knows about SDL. The simulation library, `mapio`, `editor`, `view`, the CLI and every test must build and
  run with no display and no SDL.
- The frame rate must never reach the simulation. The GUI draws `view::blend(frame.previous,
  frame.latest, frame.alpha)` from `SimulationRunner::frame()`; `alpha` comes from `core::FixedTimestep`, the
  picture is a function of real time only (`FrameRateTest`), and nothing outside the runner and the GUI
  reads a clock.
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
  floating-point accumulation into anything that affects simulation outcomes. The motion model (speed,
  progress along a segment, heading) is the exception: it is closed-form `double` arithmetic, compared
  with an epsilon (see the motion rules below).
- The default `RobotSpec` is the idealised robot that the pre-state-machine tests assume. Do not change
  its defaults; give scenarios their own spec instead. Acceleration, deceleration and turn rate are
  optional (unset = instant) for the same reason.
- Motion keeps the logical cell and the continuous pose apart. Occupancy, planning and tasks use
  `Robot::position()` only; `Robot::pose()` is for display. A robot moves only through
  `turnTowardNext()`/`advance()` (or `drive()`), only into cells it has been granted
  (`Robot::authority()`), and `Simulation` grants a cell only by claiming it in `World`. Keep the
  invariant that the world's holds for a robot are exactly its position plus the first `authority()` cells
  of its path (`MotionAudit` in `tests/movement_test.cpp` checks it on every tick), give claims back with
  `syncHolds()` whenever a robot's route is cut short, and never write a robot's position or pose from
  outside `Robot`. A deceleration must stop the robot within one cell (`RobotSpec::validate`); the
  `wantedAuthority()` bound of three cells depends on it. `brakingViolations()` must stay 0 in every test
  that does not set out to provoke one.
- Motion arithmetic must not depend on the step length (`MotionPropertyTest`, `RobotTimestepTest`): add
  closed-form pieces, not per-step approximations. Floating point is compiled with `-ffp-contract=off` so
  that results are plain IEEE on every platform; keep it, and compare doubles with a tolerance in tests
  unless the values are exactly representable.
- A robot withdrawn mid-move (cancel, disable, flat battery) coasts to a stop and only slows down; do not
  make it stop dead or speed up. It is `Idle`/`Unavailable` at once but not `isAvailable()` until stopped.
- The path planner depends only on `core` and `world`; keep it free of robots, tasks, simulation and
  rendering (its tests build into `warehousesim_planning_tests`, linked against `planning` and `testing`
  only). Its results must stay deterministic: ordering keys must be unique and independent of the
  standard library, so changing tie-breaking is a deliberate act that also updates the golden hash in
  `ResultsAreReproducibleAcrossPlatforms`. A* behaviour changes need a matching check against the
  reference search in `tests/astar_reference_test.cpp`. Do not reintroduce per-search allocation:
  `planning_alloc_test.cpp` must stay green.
- Benchmarks (`bench/`) never assert on timing, since that flakes. They cross-check results instead, and
  numbers are only quoted from an optimised build. `src/` must never depend on `testing/`.
- Tasks change status only through `TaskQueue`, and a task is claimed only by `TaskQueue::tryAssign`,
  which succeeds for exactly one caller. A `Dispatcher` is a pure decider: it returns a `DispatchPlan` and
  never touches the queue or the robots; `Simulation::assignTasks` applies the plan through `tryAssign` and
  only to available robots, which is what guarantees exclusivity. Keep it that way: never assign a task by
  writing to a `Task`, and never trust a plan. The dispatch order (priority, then age, then id) and the
  robot tie-break (lowest id) must stay total and independent of container order; tests for this build into
  `warehousesim_task_tests`, linked against `tasks` and `testing` only, so `tasks` must not reach for
  simulation, view or SDL. Dispatcher changes need a matching update of the independent reference greedy in
  `tests/dispatcher_test.cpp`. Cancelling through `Simulation::cancelTask` must also stop the robot that
  holds the task.
- Collision avoidance is two layers and the lower one is the authority. Robots never overlap or swap
  because `Simulation` only lets a robot drive into cells it has claimed in `World` (one holder per cell);
  `traffic` only decides, ahead of time, how many cells a robot may claim this step. Never let the table
  grant a cell the world refuses, never make safety depend on it, and keep `reservations = false` working.
  `traffic` depends on `core`, `world`, `robots` and `planning` only (its tests build into
  `warehousesim_traffic_tests`), and must stay deterministic: conflicts are reported earliest step first, then
  lowest robot id, and robots plan in ascending id order (planning out of order throws). The table copies the
  walkable cells when it is built, so the layout may only change through `Simulation::placeObstacle` and
  `clearObstacle`, which rebuild it and repair the routes that cross the change. `robots::forecastHolds()`
  must keep using the robot's real motion code (`stepUnobstructed`), not an approximation of it. Anything
  that moves a robot other than on its own turn (`stepAside`) must tell the controller (`relocate`) and
  must use up that robot's step. Keep `CollisionAvoidancePropertyTest` and `TrafficAudit` (no overlap, no
  swap, at most a cell of movement per tick) green on every layout, and the demo-hall standstill test: a
  change to blocking or sidestepping rules is checked against many seeds, not one (the throughput of the
  demo hall varies by about +-3 tasks per 500 ticks between seeds, so single-seed comparisons mislead).
- Cooperative planning is decoupled prioritized planning, and it only ever adds to the layers above it:
  `cooperative = false` and `reservations = false` must keep working, and safety never depends on it.
  `planning::SpaceTimeAStar` knows reservations only through `SpaceTimeConstraints` (so `planning` still
  needs nothing but `core` and `world`; its tests are in `warehousesim_planning_tests`). Everything it
  decides must be reproducible from the simulation state: search work is counted in node expansions, never
  in time; priority is the robot id, changed only by the temporary boost that ends a deadlocked ring; ties
  are broken by explicit keys, never by container or hash order. `CooperativeRouter` replaces a route only
  if the old one cannot be followed or the new one is better, and within `maxDetourCells`/`maxDodgeCells`;
  keep the per-step and per-search work limits honoured (`NoStepSpendsMoreSearchWorkThanItsLimit...`).
  `ProgressMonitor` only observes: recovery (sidestep, boost, abandon) lives in `Simulation`
  (`cooperation.cpp`). Known limits are pinned by tests named `...StaySafe` (safe and reported, not
  solved); when one of them is ever solved, change that test and the "Limits" list in the README on purpose,
  and do not relax `TrafficAudit` to make a scenario pass. Scenarios in `tests/cooperative_planning_test.cpp`
  share helpers with `collision_avoidance_test.cpp` via `tests/scenario_support.hpp`.
- Depend on interfaces at module seams (`PathPlanner`, `DistanceEstimator`, `Dispatcher`) and inject implementations through
  constructors. No global or singleton state.
- Keep static layout (`Warehouse`) and dynamic robot occupancy (`Occupancy`) separate; mutate them
  together only through `world::World`, which keeps robots off obstacles and one per cell. New world code
  must not depend on simulation, JSON or SDL: `tests/world_*`, `layouts`, `occupancy`, `warehouse` and `core`
  tests are built into `warehousesim_world_tests`, linked against `core` and `world` only.
- New predefined layouts go in `src/world/layouts.cpp` and must pass `tests/layouts_test.cpp` (one
  connected walkable region, a dropoff, and a pickup source) and `world::checkLayout`.
- Map files and the editor sit on top of the world model and never the other way round: `world` must not
  learn about files, JSON, editing, undo or windows (it only offers pure functions of a layout, like
  `checkLayout` and `kMaxMapSide`). The editor edits a `Warehouse` through its public interface only.
- The map file format is a contract. `mapio/map_file.hpp` is its definition and the README table must say
  the same; `MapFileFormatTest.TheTextIsPinnedExactlyForASmallMap` pins the exact bytes. Reading stays
  strict (unknown and duplicate keys, wrong types, out-of-range numbers, cells outside the grid or listed
  twice, deep nesting and big documents are all refused with a JSON Pointer, never an exception, and never
  a crash on any input) and writing stays canonical (`parse(serialize(d)) == d`, equal documents give equal
  text). Anything that old readers cannot understand needs a new `version`, with the old one still read
  or refused by name; do not loosen a rule to make an input pass. A save is all or nothing (temporary file,
  then rename). `MapEditor::resize` and `replace` must keep every side within `1..world::kMaxMapSide`.
- The interface never mutates the simulation, it asks. `view::FleetController` is given snapshots, never a
  `Simulation` or a `SimulationRunner`, and returns a `FleetOutput`: `simulation::Command` values to post and
  the playback state to set. `FleetSession` is the only glue and `SimulationRunner` the only owner of the
  `Simulation` while it runs: commands go in through `post()`, are applied by the worker between two steps
  through `simulation::apply()` and are recorded in `commandLog()` with the tick and outcome. Keep it so:
  a new operator action is a new `Command` alternative with an `apply` overload that validates and refuses
  (never throws for any value), a controller test, a runner test and a replay test. Pause and speed are not
  commands (they decide when steps happen, not what they compute) and must never be recorded as one. Nothing
  in `view` or `apps` may call a mutating method of `Simulation`, and a command must not take a draw from
  the simulation's random number generators (`TaskCommandsDoNotDisturbTheRandomTaskStream`). The state at any
  tick must stay a function of the scenario, the seed and the command log (`RunnerDeterminismTest`,
  `FleetDeterminismTest`); a change that breaks replay is a bug, not a flaky test.
- Performance figures are simulated figures, and say so. `metrics` knows nothing of the simulation: it
  sees `TickObservation`s (plain data) and `simulation::MetricsCollector` is the one place that reads a
  `Simulation` to make them, through its public const interface. Measuring must never change a run
  (`AMeasuredRunIsTheSameRunStepForStepAsAnUnmeasuredOne`); a metric must never read a clock, call the
  planner or draw a random number. Every robot is in exactly one `Activity` per tick (the budget adds up,
  `checkRuns` and the tests assert it), a figure with a zero denominator is undefined (empty CSV field,
  `n/a`, never 0 or NaN), and a new figure needs a definition in the README table, a CSV column (the runs
  header is pinned by `RunsCsvTest.TheColumnsAreAContractAndAppearInThisOrder`: change it on purpose and
  update the README), a hand-checked test and a mutation check. Every output says `simulated`
  (`basis` column, dashboard title and disclaimer); do not describe a figure as a property of a real robot,
  warehouse or controller, in code, docs or reports. Waiting time comes from `Simulation::heldBack()` (the
  engine refusing a robot its next cell this step), never from a robot's own blocked-tick counter, which
  restarts whenever it tries to resolve a block. Counters read by the collector must be running totals that
  never go down (`TheTrafficCountsKeepCountingWhenTheLayoutIsEdited`); the accumulator refuses a gap or a
  count that falls rather than guess.
- The benchmark is deterministic and checks itself: `warehousesim_fleet_bench` fails on a collision, an
  unaccounted task or robot-tick, or a first case that does not reproduce. Fleet size is chosen through
  `ScenarioOptions` (unset must stay exactly the legacy fleet: `NoOptionsIsExactlyTheScenarioAsItAlwaysWas`),
  and the task stream depends on the seed alone so that fleets are offered the same work. When the baseline
  table in the README is regenerated, regenerate all of it with the same command and say what it was
  built with; never edit a number by hand.
- A run is a function of its `ScenarioSpec` (`Simulation::origin()`) and the `Command`s applied to it, each
  after a whole number of steps. A recording (`recording.hpp`) is that plus the task events and a state
  checksum every so often, and `replay()` plays it back from tick 0 and reports the first difference. Keep
  it so:
  - Everything that changes a `Simulation` from outside is a `Command` with an `apply` overload that validates
    and refuses (never throws) and an entry in the file (`commandJson`/`readCommand`, the pinned text in
    `RecordingFormatTest`). A public mutator of `Simulation` that no command wraps cannot be recorded: add
    the command, or keep it out of the public interface. `RunJournal` refuses to start on a simulation that
    was changed before recording began, or that was built with its own planner or dispatcher.
  - `Simulation::stateChecksum()` (`state_checksum.cpp`) covers what decides the next step or what a
    snapshot shows. New state goes into it in the same change, with a test in `state_checksum_test.cpp` that
    makes it differ. It hashes `double` bit patterns, so the arithmetic must stay exact IEEE:
    `-ffp-contract=off`, no fast-math, and no libm transcendental (`sin`, `cos`, `pow`, `atan2`, ...) in
    `src/` outside `view`. Never decide anything from the iteration order of an `unordered_*` or from an
    address, and never let a clock or a file reach `Simulation`; a thread reaches it only the way the planning
    threads rule below allows.
  - Changing what a simulation computes on purpose means, in the same change: raising `kEngineRevision`,
    updating `ThePinnedScenarioHasThePinnedChecksums` and regenerating the fixture `kEarlierRecording` in
    `replay_test.cpp`. A change that needs neither is behaviour-preserving, which is what to aim for.
  - The file is a contract like the map file: `recording_file.hpp` documents it, reading stays strict (JSON
    Pointer problems, never a throw or a crash on any input, bounded size, depth and counts) and writing
    stays canonical. A field added to `SimulationConfig` or `RobotSpec` must be added to the format
    (`TheFormatCoversEveryFieldOfTheConfigurationAndTheRobotModel` stops compiling until it is), and what
    old readers cannot read needs a new `version`. Nothing about the machine or the time may go into a
    recording: ticks only.
  - `replay()` throws for no recording, compares as it goes (checkpoints, whether each command is accepted as
    recorded, every task event) and stops at the first difference with its tick and a reason. The way
    metrics are measured never changes what is replayed.
- Planning threads change how long a step takes and nothing else. `Simulation::usePlanningThreads(n)` lets the
  independent path questions of a step (the way to the pickup of each newly assigned task, the ways to the free
  chargers, the routes repaired after `placeObstacle`) be answered by `planning::PathBatchExecutor` on a
  `core::ThreadPool`. Keep it so:
  - Only pure questions go into a batch: a function of the layout and two cells and of nothing else. Anything
    that depends on robots, reservations, tasks or on the answer to another question (the cooperative
    searches, which are ordered by priority; `tryRepath`, which avoids the robots where they stand) is asked
    on the stepping thread, in order.
  - A worker never touches a `Simulation`. The executor reads a private copy of the layout (taken when
    `Warehouse::version()` changes), has a planner of its own for each participant, and writes only the answer
    slot of its question. `Simulation` applies the answers itself, on its own thread, in the order the questions
    were asked: there is one path by which state changes. The work a batch did is counted with the planner's
    (`batchWork_`, `plannerWork()`), so the checksum is the same for every thread count.
  - A new batch needs a test that compares 0 and N threads tick by tick (`PlanningThreadsTest`), a mutation
    check, and a line in `warehousesim_scaling_bench`, which fails if a thread count changes a state or a route.
  - Threads are not part of the scenario: they are not recorded, and a recording made on any number of threads
    replays on any other (`ReplayOptions::planningThreads`). Do not make a recording or a checksum depend on them.
  - `core::ThreadPool` is bounded in threads and in queued tasks (`submit` blocks while the queue is full).
    `shutdown()` and the destructor refuse new tasks, wake producers that are blocked, run everything already
    accepted and join. A task must not call `parallelFor` or `shutdown` on its own pool (both throw), and
    `parallelFor` rethrows the exception of the lowest index after all the other calls have run.
    `ThreadPoolTest` and the mutants that hang (a producer never woken, a worker that abandons tasks) are what
    pin this down; run them under TSan after any change to the pool.
- Optimisations keep behaviour: the golden checksums of a run (`ThePinnedScenarioHasThePinnedChecksums`,
  `PlanningThreadsTest`, and the one-thread-against-N comparisons) must not move. `Warehouse::version()` only
  says whether a layout may have changed, so caches that key on it (`SpaceTimeAStar`, `PathBatchExecutor`)
  stay correct; never decide anything from its value. A distance field is charged for as before whether it was
  kept or not (`recent_`), because the planning limits were tuned that way: kept fields save time, not work.
  If a speed-up must change what a run computes (for instance, counting fewer expansions), it is a change of
  the engine: raise `kEngineRevision` and regenerate the fixtures, in the same change.
- Everything the editor decides is in `editor` (the model) and `view::EditorController` (input to edits):
  both are tested without a display, with plain `InputEvents`. `rendering` only translates SDL events into
  `PointerAction`s and `EditorCommand`s, in the order they happened, and must not decide anything. Anything
  that throws the map away (load, a predefined layout, quit) must ask first when there are unsaved changes
  (`mayDiscard`), and a map file that fails to load must leave the editor exactly as it was. The apps never
  start the editor on a blank map in place of a file that exists but cannot be read.

### Deterministic tests and behaviour

- Same warehouse, seed, config and inputs must give the same state after the same number of ticks.
- Randomness comes only from `core::Rng` with an explicit seed. Do not use `std::rand`, `std::random_device`,
  `std::*_distribution`, or unordered-container iteration order to make decisions.
- Simulated time is `core::Tick` / `SimClock`. Simulation logic never reads wall-clock time; only
  `SimulationRunner` and the GUI sleep.
- Tests must not depend on real time for correctness. Threaded tests wait on `waitForTick` (with a
  generous timeout), never on fixed sleeps; sleeps are allowed only to check that something does *not* happen.
- Tests are not allowed to touch the network, the display, or files outside the build tree. Tests that
  need files use `tests/scratch_dir.hpp` (`ScratchDir`), which creates a uniquely named directory below
  the working directory (the build tree under ctest) and removes it again.
- Fix bugs by adding a regression test first or alongside. Never weaken or delete a test to make it pass.

### Memory safety and resource cleanup

- RAII everywhere: `std::unique_ptr`, containers, `std::scoped_lock`/`unique_lock`. No raw `new`/`delete`,
  no `malloc`, no manual `SDL_Destroy*` outside the deleters in `rendering/sdl_window.cpp`.
- No owning raw pointers, no `const_cast`, no C-style casts, no uninitialised members. Use `std::span`,
  `std::optional` and `std::string_view` instead of pointer+length pairs.
- Validate at boundaries (constructors, `fromRows`, JSON, CLI arguments) by throwing or returning
  `std::optional`; do not re-check invariants the types already guarantee.
- Every thread is joined before its owner is destroyed (`SimulationRunner::~SimulationRunner`,
  `ThreadPool::~ThreadPool`). Shared state is guarded by one mutex; readers get value-copy `Snapshot`s. Apart
  from those two, no module starts a thread.
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
SDL_VIDEODRIVER=dummy build/default/apps/warehousesim_gui --frames 30   # fleet interface smoke test, no window
build/release/bench/warehousesim_fleet_bench                              # the baseline table of the README
build/release/bench/warehousesim_scaling_bench                            # a step with and without planning threads (README)
build/default/apps/warehousesim_cli --layout large --robots 100 --ticks 300 --threads 2   # 100 robots, 2 planning threads
build/default/apps/warehousesim_cli --layout compact --ticks 500 --metrics --metrics-csv build/m   # dashboard + CSV
build/default/apps/warehousesim_cli --layout compact --ticks 500 --seed 2 --record build/run.json   # record ...
build/default/apps/warehousesim_cli --replay build/run.json                                         # ... and check the replay
SDL_VIDEODRIVER=dummy build/default/apps/warehousesim_editor --file build/smoke-map.json --layout compact --frames 30
# (ctest also runs the headless CLI on every layout and on a map file, checks that equal seeds give
# equal output, and that invalid maps are refused)
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
