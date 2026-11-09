# WarehouseSim

A deterministic C++20 simulation of autonomous robots moving goods through a grid warehouse, with a
headless library and CLI plus a separate SDL3 visualisation.

```
core ← world ← planning ┐
core ← robots ──────────┴← tasks ┐
        world, robots ← traffic ─┴← simulation ┐
        core ← metrics ──────────────┘   ↑       ├← view (no SDL) ← rendering (SDL3) ← apps/gui, apps/editor
                                  apps/cli     │
                                (headless)     │
        world ← mapio (map files) ← editor ────┘
   bench/fleet_bench: headless, on simulation + metrics
```

| Module       | Contents                                                                  |
|--------------|---------------------------------------------------------------------------|
| `core`       | `Point`, strong ids, `SimClock`/`Tick`, seedable `Rng` (SplitMix64), `PointSet`, `ThreadPool` (bounded, safe shutdown) |
| `world`      | grid model, occupancy, predefined layouts (see below); no SDL or JSON     |
| `robots`     | `Robot` entity: pose, speed, battery, task, path and an explicit state machine; `forecastHolds()` |
| `traffic`    | space-time `ReservationTable` and the `TrafficController` that fills it (see Collision avoidance) |
| `planning`   | `PathPlanner` interface, deterministic 4-connected A* (see Path planning), `PathBatchExecutor`: many independent path questions on a thread pool (see Performance and planning threads) |
| `tasks`      | `TaskQueue` lifecycle and exclusive claims, greedy `Dispatcher`, distance estimators, seeded `TaskGenerator` |
| `testing`    | header-only test/benchmark support: reference BFS, map generators, allocation counter |
| `metrics`    | simulated performance figures from plain observations: aggregation, derived rates, CSV and dashboard text (see Performance metrics); depends on `core` only |
| `simulation` | `Simulation` tick engine, `SimulationRunner` thread with its command queue, JSON, scenarios, `MetricsCollector`, benchmark sweeps, run recording and replay (see Recording and replay), the `InvariantAuditor` behind `--audit` |
| `mapio`      | the versioned JSON map file: schema validation, reading, canonical writing (see Map editor) |
| `editor`     | `MapEditor` editing model (tools, strokes, undo, resize) and its file glue; no SDL, no GUI |
| `view`       | `Camera`, draw lists (`buildScene()`, `buildFleetScene()`, `buildEditorScene()`), `ViewController`, `FleetController`, `FleetSession`, `EditorController`; no SDL, fully tested |
| `rendering`  | SDL3 `SdlWindow` (RAII): turns SDL events into `InputEvents`, runs a draw list |

## Documentation

| Document | For |
|---|---|
| [docs/user-guide.md](docs/user-guide.md) | running the programs: layouts, maps, fleets and loads, the GUI, recording, auditing |
| [docs/architecture.md](docs/architecture.md) | the modules, one step of the engine, the algorithms, determinism, limitations, extension points |
| [docs/developer-guide.md](docs/developer-guide.md) | toolchain, presets, the test tiers, sanitizers, static analysis, benchmarks, CI, releasing |
| [docs/audit.md](docs/audit.md) | the audit of memory ownership, concurrency, event ordering, collision safety and numerical stability |
| [docs/benchmarks.md](docs/benchmarks.md) | the final benchmark results and the scenario campaign, with the machine and build they came from |
| [docs/verification.md](docs/verification.md) | what is implemented, verified, partial and unverified, with the checks and results behind each, and a manual checklist for a real window |
| [ROADMAP.md](ROADMAP.md) | the prioritised next steps, with scope, dependencies, acceptance criteria and estimates |
| [examples/README.md](examples/README.md) | six example warehouses and what each shows |
| [CHANGELOG.md](CHANGELOG.md) | what each release added |

## Status

Version 1.0.0. Checked locally on **macOS 27.2, Apple M4 (arm64), Apple clang 21**, and in hosted CI (third run,
commit `f03538d`) on **Ubuntu 24.04 (x86-64) with clang and with GCC 13** and **macOS 26 (arm64)**. The full
table, with the checks behind each row and a checklist for a real window, is
[docs/verification.md](docs/verification.md).

- **Verified** (checks run on macOS, and passing): the engine and its modules; collision safety (no
  overlap, swap or doubly held cell in 666 333 audited states); planning, tasks, metrics, the map file and the
  editing model, recording and replay, planning threads, the command line, the benchmarks and the regression
  gate, the release archive. 1 538 tests pass in the Debug build with the GUI and in the Release build, and 1 533
  under AddressSanitizer + UBSan and under ThreadSanitizer (0 reports); clang-tidy (0 findings) and
  clang-format are clean; macOS `leaks` finds no leak in 18 programs. The hosted runs passed the same suite on
  Ubuntu with clang, with AddressSanitizer (leak detection on), ThreadSanitizer and clang-tidy, on macOS 26,
  and with GCC 13.
- **Partial** (works, but a requirement is only partly met): cooperative planning is neither complete nor
  optimal. Fleets bigger than the work needs make less progress than smaller ones, and one-cell corridors with
  three or more robots can deadlock, safely. The fleet interface cannot cancel a task, take a robot out of
  service or edit the layout. Recordings cannot be authored from the CLI or watched in a window.
- **Implemented, unverified**: the GUI and the editor in interactive use (they start, render and exit on the
  real macOS window system, but keys, mouse and appearance have not been checked); Windows with MSVC (the
  Debug headless build compiles and passes 1 533 of 1 538 tests in hosted CI; the five failures were in the
  tests, are fixed and have not run since; the Release configuration, packaging, the GUI and the editor have
  not run on Windows), and Valgrind.

What comes next, in priority order, is in [ROADMAP.md](ROADMAP.md).

The rest of this file is the reference for each part of the simulator.

## Build

Requires CMake ≥ 3.24, a C++20 compiler and Ninja (or another generator). nlohmann_json, GoogleTest and
SDL3 are located with `find_package` and otherwise downloaded at pinned versions by `FetchContent` (a
downloaded SDL3 is linked statically). The tools the project is checked with are pinned in
[docs/developer-guide.md](docs/developer-guide.md#toolchain) (`cmake` 3.31.6, `ninja` 1.11.1.4,
`clang-format` 21.1.8, `clang-tidy` 21.1.6) and installed into a project-local `.venv`.

```sh
cmake --preset default && cmake --build --preset default && ctest --preset default
```

Presets: `default` (Debug, GUI, tests), `headless` (no SDL3), `release` (optimised with link-time
optimisation), `asan` (AddressSanitizer, UBSan and the float checks, no recovery), `tsan`, `tidy`
(clang-tidy on every target). `cmake --build --preset package` makes the release archive: the programs,
the example maps and the documents (`cmake --install build/release --prefix DIR` installs the same).
Building SDL3 from source on Linux needs the X11 development packages (see `.github/workflows/ci.yml`).
`warehousesim_cli`, `warehousesim_gui` and `warehousesim_editor` all take `--version`.

## World model

The `world` module is independent of SDL, JSON and the simulation (its tests build as the separate
`warehousesim_world_tests` executable linked against `core` and `world` only).

- **Coordinates and size:** `core::Point` (x = column, y = row, origin top-left, y grows south),
  `core::Direction` (north/east/south/west) and the validated `world::GridSize`.
- **Cell types:** `Free` `.`, `Wall` `#`, `Shelf` `S`, `Pickup` `P`, `Dropoff` `D`, `Charging` `C`. Walls and
  shelves are static obstacles; the other four are walkable.
- **Static layout:** `Warehouse` holds the cell grid: bounds checks, `at`/`set`, `neighbors`,
  `walkableNeighbors`, `cellsOfType`, ASCII `fromRows`/`toRows`. It knows nothing about robots.
- **Dynamic occupancy:** `Occupancy` maps cell ↔ robot (one robot per cell, one cell per robot) and
  knows nothing about walls. Mutators return a status (`PlaceStatus`, `MoveStatus`) and change nothing
  when they refuse.
- **Both together:** `World` is the only way to mutate either half, so a robot can never stand on an
  obstacle or share a cell. It adds `isFree`, `freeNeighbors`, `checkMove`/`moveRobot` (one orthogonal
  step onto a walkable, unoccupied cell), and layout edits `setCell`/`placeObstacle`/`clearCell`
  (turning an occupied cell into an obstacle is refused).
- **Predefined layouts** (`world::predefinedLayouts()`, also `--layout` in the apps):
  `demo` (20x12 open hall), `compact` (12x8 walled room, all six cell types), `fulfilment` (28x16 walled
  hall: pickup column, dropoff row, chargers, six shelf rows), `large` (100x64 walled hall with two-cell
  aisles, 147 chargers and 25 dropoffs, built for fleets of a hundred robots and more; `--robots` chooses
  the fleet, up to 512). A `Warehouse` also has a `version()`: it changes with every `set()` and is shared
  by copies, so that a cache can tell without looking at every cell that a layout cannot have changed.
- **Readiness:** `world::checkLayout()` says whether a layout can host a simulation: it needs a walkable
  cell, a dropoff and a source of goods (a pickup, or a free cell beside a shelf), and it warns if the
  walkable cells fall into separate regions. `world::kMaxMapSide` (512) is the longest side accepted for
  a layout that comes from outside the program. The map editor is built on this module without the
  module knowing about it.

## Run

```sh
build/default/apps/warehousesim_cli --ticks 200 --seed 1     # headless; prints the final snapshot as JSON
build/default/apps/warehousesim_cli --layout compact         # demo, compact or fulfilment
build/default/apps/warehousesim_gui --layout fulfilment --speed 2 --fps 60
SDL_VIDEODRIVER=dummy build/default/apps/warehousesim_gui --frames 30   # render without a display
build/default/apps/warehousesim_editor --file my-hall.json               # edit (or start) a map file
build/default/apps/warehousesim_cli --map my-hall.json --ticks 500       # run a map made with it
build/default/apps/warehousesim_cli --layout compact --ticks 3000 --metrics --metrics-csv out/compact
build/release/bench/warehousesim_fleet_bench                              # layouts x fleet sizes x seeds
build/default/apps/warehousesim_cli --layout large --robots 100 --spawn 1 --ticks 300 --threads 2   # 100 robots, 2 planning threads
build/release/bench/warehousesim_scaling_bench                            # wall-clock cost of a step, with and without threads
build/default/apps/warehousesim_cli --layout compact --ticks 600 --seed 7 --record run.json   # also write down the run
build/default/apps/warehousesim_cli --replay run.json                    # play it back and check it
build/default/apps/warehousesim_cli --map examples/cross-dock.json --robots 12 --spawn 3 --ticks 1500 --audit
build/release/bench/warehousesim_regression_bench                        # the performance gate against its baseline
python3 scripts/campaign.py build/release/apps/warehousesim_cli          # layouts x fleets x loads x seeds, all audited
```

`--audit` checks every step against the invariants in `simulation/audit.hpp` (collision safety, task
ownership, event order, finite numbers) and exits with 1, listing what, if one is broken. The
`examples/` directory has six maps to run, edit and learn from.

Both `warehousesim_cli` and `warehousesim_gui` take `--map FILE` instead of `--layout NAME` (not both).
A map that is missing, invalid or cannot host a simulation is refused with every problem listed and
exit status 1; an unknown layout or conflicting options exit with status 2.

`warehousesim_gui` options: `--seed N`, `--layout NAME` or `--map FILE`, `--tick-ms N` (fixed simulation step, default
100 ms of real time at speed 1), `--speed X` (initial playback speed, 1/16 to 64), `--fps N` (frame cap,
default 60), `--frames N` (exit after N frames; 0 = until closed), `--metrics-csv BASE` (where `E` writes
the performance figures; the folder of BASE must exist).

`warehousesim_cli` and `warehousesim_gui` also take `--robots N` (a fleet of N instead of the layout's usual
four or fewer, 1 to 512, as many as the layout has room for), `--spawn N` (a new random task every N ticks;
default 5; 0 for none) and `--threads N` (plan routes on N worker threads besides the stepping one; the run
is the same for every N, see Performance and planning threads). `--robots` and `--spawn` belong to the
scenario, so a `--replay` takes them from the file and refuses them on the command line; `--threads` is
allowed with `--replay` because it only changes how long the replay takes.

`warehousesim_cli` also takes `--metrics` (print the simulated performance dashboard to stderr after the
run; the JSON on stdout is the same with or without it) and `--metrics-csv BASE` (write the three CSV files),
and `--record FILE` and `--replay FILE` (see Recording and replay). `warehousesim_gui --record FILE` writes
the recording of the session, with every command given through the interface, when the window closes.

`warehousesim_gui` is the fleet management interface: the map on the left, a panel on the right with the
robots, the selected robot, the active and finished tasks, and a prompt. See "Fleet interface" below for
what it can do; the controls are:

| Input | Action |
|-------|--------|
| click a robot or a row of the robot list; Tab / Shift+Tab | select a robot and inspect it |
| click a row of a task list; `T` / Shift+`T` | select a task |
| `N`, click a pickup cell, click a dropoff cell | add a delivery task |
| `1` `2` `3` `4` | priority low / normal / high / urgent of the task being made, else of the selected waiting task |
| Space | pause / resume |
| `[` / `]` | halve / double playback speed (1/16x to 64x) |
| `R` | start the scenario over (same layout and seed) |
| mouse wheel, `+` / `-` | zoom about the pointer / window centre |
| drag with the middle or right button, arrow keys, WASD | pan |
| `0` | fit the whole warehouse |
| `P` / `O` | show / hide the planned routes of all robots (and a red cross on the cell a held-back robot may not enter) / the cells robots hold |
| `M` | show / hide the performance dashboard (simulated figures, see Performance metrics) |
| `E` | write the performance figures to `BASE.runs.csv`, `BASE.robots.csv` and `BASE.series.csv` (needs `--metrics-csv BASE`) |
| Esc | cancel task entry; else clear the selection; else quit |
| closing the window, Ctrl-C | quit (the worker thread is joined and SDL shut down) |

## Path planning

`planning::AStarPlanner` finds shortest routes on the warehouse grid. It depends only on `core` and `world`
(its tests build as `warehousesim_planning_tests`, linked against `planning` and the header-only `testing` helpers): it knows nothing about robots,
task assignment or rendering. `PathPlanner::findPath(warehouse, start, goal, blocked)` returns the cells to
walk (excluding `start`, including `goal`), or `nullopt`. `blocked` holds cells that are temporarily
impassable, typically where other robots stand; `start` may be in it, `goal` may not.

- **Obstacles and moves.** Walls and shelves are never entered; pickup, dropoff, charging and free cells are
  ordinary terrain. A step goes one cell in one of the allowed directions at cost 1. `AStarOptions::directions`
  is a `core::DirectionSet` (default: all four), so a planner can be confined to rows, to columns, or to
  one-way travel such as East/South only. An empty set is rejected.
- **Heuristic.** Manhattan distance to the goal. With unit costs and orthogonal moves it is admissible
  (never over-estimates, for any subset of directions) and consistent, so each cell is expanded at most once and the
  first route found is shortest. Both properties are checked against true distances in the tests.
- **Reproducible tie-breaking.** The open list is ordered by `(f, h, row-major cell index)` and neighbours are
  tried north, east, south, west. Those keys are unique, so the answer does not depend on how the standard
  library implements its heap: the same query gives the same path on every platform. On an open grid the
  path runs along the start's row first, then down the goal's column. A golden hash over 400 random queries
  pins this down (`ResultsAreReproducibleAcrossPlatforms`).
- **Failures are explicit.** `AStarPlanner::search()` returns a `SearchResult` with a `SearchStatus`
  (`Found`, `InvalidStart`, `InvalidGoal`, `GoalBlocked`, `Unreachable`, `BudgetExceeded`), the path cost and
  the number of cells expanded. A goal or start that is sealed in is rejected before any searching; a goal on the
  far side of a wall makes A* exhaust the start's side of the map, which `AStarOptions::maxExpansions` can bound
  (status `BudgetExceeded` means "gave up", not "unreachable").
- **Allocation.** Cost/parent arrays, the blocked-cell marks and the open list are kept in the planner and
  invalidated with a generation counter rather than cleared. After the first search on a grid of a given size,
  `search()` with a reused path buffer allocates nothing and `findPath()` allocates exactly the returned
  path (`planning_alloc_test.cpp` counts `operator new`). A planner therefore is not thread-safe: use one per
  thread (the simulation owns one).

Checks: besides scenario tests (open grids, mazes, serpentine and one-cell-wide corridors, blocked/invalid
goals, sealed and unreachable targets, restricted moves, search budgets), A* is compared with an independent
breadth-first search (`testing/reference_search.hpp`) on ~12,000 random queries over thousands of random maps,
mazes and move sets: same reachability, same optimal length, legal route.

### Benchmark

```sh
cmake --preset release && cmake --build --preset release
build/release/bench/warehousesim_planning_bench                # 16x16 .. 512x512
build/release/bench/warehousesim_planning_bench --max-size 256 --csv
```

Scenarios per map size: `open` (empty grid, corner to corner), `scattered25` (25% random obstacles), `maze`
(perfect maze, one long winding route), `unreachable` (a wall splits the map: A* must exhaust the start's
half) and `sealed-goal` (rejected without searching). Columns: **warm** = reused planner and path buffer,
**cold** = a new planner for every query (pays for allocating the workspace), **bfs** = the reference
breadth-first search (which allocates a distance field per call), and heap allocations per warm search. It
always cross-checks A* against the reference and exits non-zero on a mismatch; `ctest` runs it with `--quick`.
Only trust timings from an optimised build; the program says which kind it is.

Measured on an Apple M4 (arm64, macOS), Apple clang 21 with `-O3 -DNDEBUG`, one thread, median of 11 samples,
0 heap allocations per warm search in every row:

| map | scenario | path | expanded | warm A* | cold A* | BFS |
|-----|----------|-----:|---------:|--------:|--------:|----:|
| 64x64 | open | 126 | 127 | 2.8 us | 4.2 us | 26.6 us |
| 64x64 | scattered25 | 126 | 404 | 9.6 us | 10.6 us | 25.1 us |
| 256x256 | open | 510 | 511 | 14.8 us | 22.1 us | 415 us |
| 256x256 | scattered25 | 510 | 3,926 | 144 us | 130 us | 678 us |
| 256x256 | maze | 8,760 | 24,788 | 457 us | 464 us | 410 us |
| 256x256 | unreachable | - | 32,768 | 3.72 ms | 3.76 ms | 0.22 ms |
| 512x512 | open | 1,022 | 1,023 | 34.2 us | 63.7 us | 1.65 ms |
| 512x512 | scattered25 | 1,022 | 14,803 | 864 us | 903 us | 3.35 ms |
| 512x512 | maze | 19,432 | 46,506 | 915 us | 903 us | 2.09 ms |
| 512x512 | unreachable | - | 131,072 | 16.7 ms | 16.6 ms | 0.84 ms |
| 512x512 | sealed-goal | - | 0 | 0.01 us | 30.5 us | 1.66 ms |

A* is far ahead when the heuristic helps (open and lightly obstructed maps, and sealed goals), roughly level on
mazes, and about 20x slower than plain BFS when asked to exhaust a large region for an unreachable goal: that
is the cost of the priority queue (about 130 ns per expanded cell there against about 6 ns for BFS's FIFO
queue), and the reason to use `maxExpansions` for speculative queries. Against the previous implementation (a fresh `std::priority_queue` and cost/parent vectors on every call), measured
the same way with `findPath`, the reused workspace was up to 50% faster on open maps (512x512: 79.6 us to 39.9 us;
256x256: 25.9 us to 17.5 us), 8-16% faster on scattered-obstacle maps (256x256: 96 us to 80 us), and level on
mazes and unreachable goals, which are dominated by heap operations. The paths it returns are identical:
a check over 30,000 random queries found no difference.

## Tasks and dispatching

A `tasks::Task` has a unique `TaskId`, a `pickup` and a `dropoff` cell, a `TaskPriority` (`Low`, `Normal`,
`High`, `Urgent`), the tick it was created (`createdAt`), a `TaskStatus` and the `assignee` robot that holds
or last held it.

```
Pending -> Assigned -> InProgress -> Completed
```

`Assigned` goes back to `Pending` if the robot gives the task up before collecting it. `Failed` is reachable
from `Pending` (the dispatcher found the task impossible), `Assigned` and `InProgress` (the carrying robot was
lost). `Cancelled` is reachable from every open state. `Completed`, `Failed` and `Cancelled` are final.

`TaskQueue` is the one place tasks live and change status. `submit` adds a task, `cancel` withdraws it and
`complete` finishes it; a transition the lifecycle does not allow throws `std::logic_error`, except
`tryAssign` and `cancel`, which return `false` because losing that race is an expected outcome. Waiting
tasks are kept in dispatch order: highest priority first, then oldest, then lowest id, a total order that
does not depend on insertion history.

**A task is exclusive.** `TaskQueue::tryAssign(task, robot)` is the only way to claim one and succeeds for
exactly one caller. A dispatcher only *proposes* a plan; `Simulation` applies it through `tryAssign` and only
to robots that are still available, so a faulty dispatcher still cannot give one task to two robots or two
tasks to one robot (`SimulationDispatchTest.AFaultyDispatcherCannotGiveOneTaskToTwoRobotsOrTwoTasksToOne`).

`GreedyDispatcher` is deterministic and does not change the queue or the robots:

1. Pending tasks are taken in queue order, so an urgent task is never starved by a nearer, less important one.
2. Each task goes to the *available* robot (`Robot::isAvailable()`: idle and charged enough) with the
   smallest estimated distance to its pickup; ties go to the lowest robot id. Each robot gets at most one task
   per plan, whatever order the fleet vector is in.
3. Robots that cannot reach the pickup are skipped. A task whose delivery leg (pickup to dropoff) is
   impossible, or whose pickup no robot in the fleet can ever reach, is *rejected* and ends up `Failed` so it
   does not sit in the queue forever. A task only a busy robot can reach stays `Pending`.

Distances come from a `DistanceEstimator`: `PathDistance` (default) is the real A* route length, cached per
(from, to) pair including "unreachable" answers; `ManhattanDistance` is free but cannot see walls, so it never
rejects anything.

`Simulation::submitTask(pickup, dropoff, priority)` adds a task and `cancelTask(id)` withdraws one. A robot
working on a cancelled task stops and becomes idle, and cargo it carried is dropped. Cancelling a task that has
already finished returns `false`; an unknown id throws `std::out_of_range`. The bundled scenarios give spawned
tasks random priorities (`SimulationConfig::varyTaskPriority`).

Limitations: assignment is greedy per task in priority order, not a globally optimal matching, and it never
preempts a robot that is already working, so an urgent task waits for the next free robot. Estimates ignore
other robots (they move). `PathDistance` assumes the layout does not change; call `reset()` if it does.

## Robots

`robots::Robot` is a self-contained entity (it depends only on `core`; it never looks at the warehouse,
planner, task list or renderer). Each robot has:

| Property | Where |
|----------|-------|
| stable id (never changes, 0 is invalid) | `id()` |
| logical cell (the last cell it passed) and the `Direction` it faces | `position()`, `heading()` |
| continuous position and heading, which are between cells while it drives | `pose()`, `headingAngle()`, `segmentProgress()` |
| maximum speed, current speed and velocity (cells per tick; a robot enters at most one cell per tick, so `maxSpeed` is in (0, 1]) | `maxSpeed()`, `speed()`, `velocity()` |
| what its last step of motion was (stopped, turning, accelerating, cruising, braking) | `motion()` |
| the cells it has been granted and may drive into | `authority()` |
| cargo aboard, and cargo delivered | `carriesCargo()`, `deliveries()` |
| battery (integer units, saturating, so exact on every platform) | `battery()`, `needsCharge()` |
| assigned task, destination, planned path | `task()`, `destination()`, `path()` |
| operational state | `state()`, `goal()` |

`RobotSpec` holds the per-model constants: speed, acceleration, deceleration and turn rate, battery
capacity and start charge, the energy used per idle tick, per cell entered and per loading/unloading tick,
the charge rate, a low-battery reserve, and the number of ticks spent loading and unloading. **The default
spec is an idealised robot** (one cell per tick, speeds and turns that happen instantly, instant handling,
no energy use); with it the simulation reproduces the pre-state-machine behaviour tick for tick (see
Movement, below, for how that was checked).

State machine (`robots::isValidTransition`; a state never transitions to itself):

```
Idle        -> Navigating | Charging | Unavailable
Navigating  -> Loading | Unloading | Charging | Idle | Unavailable
Loading     -> Navigating | Idle | Unavailable
Unloading   -> Idle | Unavailable
Charging    -> Idle | Navigating | Unavailable
Unavailable -> Idle
```

`Navigating` carries a `NavigationGoal`: `Pickup`, `Dropoff` or `Charger`. A task cycle is
Idle → Navigating(Pickup) → Loading → Navigating(Dropoff) → Unloading → Idle; a charging trip is
Idle → Navigating(Charger) → Charging → Idle. Every change goes through a named operation
(`beginTask`, `beginLoading`, `finishLoading`, `beginUnloading`, `finishUnloading`, `beginChargerTrip`,
`beginCharging`, `stopCharging`, `abortTask`, `makeUnavailable`, `restore`, ...). An operation that is not
allowed from the current state, or whose preconditions fail (not arrived yet, empty battery, dwell not
finished), throws `InvalidTransition` and leaves the robot unchanged; malformed arguments (a path that
does not end at the destination, non-adjacent steps) throw `std::invalid_argument`.
`Robot::inconsistency()` checks the invariants (goal and destination exist exactly while navigating, a task
is held exactly while heading to a pickup/dropoff or handling cargo, a path exists only while navigating or
while a withdrawn robot is still braking to a stop, speed, progress, authority and dwell are zero when they
do not apply, the battery is within bounds).

### Movement

Motion is fixed-step (one tick is one step; distances are in cells, times in ticks) and keeps two things
apart on purpose. The **logical** position is a grid cell: `position()` is the last cell the robot passed,
and it is all that occupancy, planning and tasks ever see. The **continuous** position is `pose()`, the
robot's centre in world space (cell `(x, y)` is centred on `(x + 0.5, y + 0.5)`), which is between two cells
while it drives. Only the picture uses the pose.

- **Limits.** `RobotSpec::maxSpeed`, `acceleration` and `deceleration` (cells per tick squared) and
  `turnRate` (radians per tick). Unset means instant, which is the idealised default. A deceleration must
  stop the robot from top speed within one cell (`validate()` checks it).
- **Profile.** Along a straight run to a stop point the robot speeds up, cruises and brakes so as to come to
  rest exactly at the stop point (`robots::advanceAlongRun`). The profile is closed-form, so the robot
  follows the same trajectory however a stretch of time is divided into steps (`MotionPropertyTest`
  integrates the same run in steps from 1.0 down to 0.05 and compares).
- **Stops.** A robot stops at its destination, at the end of the cells it has been granted, and, if it has a
  finite `turnRate`, at every corner. It turns on the spot, the short way round (a U-turn goes clockwise),
  and the time the turn takes comes out of the step; a robot that is moving never turns. A robot with
  instant turns drives straight through corners.
- **Authority.** A robot may only drive into cells it holds. `Occupancy` therefore holds, for each robot,
  its cell plus the cells it has claimed ahead, still at most one robot per cell; `Robot::wantedAuthority()`
  asks for just enough to cover the distance it can drive in the step plus the distance it needs to stop
  (at most three cells), and `Simulation` claims them in the world. This is what guarantees that a robot
  never enters a cell another robot holds, that it can always stop in time, and that two robots' centres are
  never closer than one cell (all audited on every tick, on every layout, by `MovementTest`). It is not
  cooperative planning: nothing is negotiated, a robot that cannot get its next cell simply waits for it.
- **Withdrawn mid-move.** Cancelling a task, disabling a robot or running out of battery between two cells
  does not stop the robot dead. It changes state at once (`Idle` or `Unavailable`) but keeps the cells it
  needs and brakes to the first waypoint it can stop at without exceeding its deceleration, only ever
  slowing down on the way; it is not available for work until it has stopped.
- **Cargo.** `carriesCargo()` is true from the end of loading to the end of unloading, `deliveries()` counts
  hand-overs at dropoffs. Dropped cargo (a withdrawn job) is not a delivery.
- **Energy.** `moveCostPerCell` is charged for each cell entered, so a journey costs the same whatever
  speed profile covered it; idle and handling drains are per tick.

**Compared with the previous commit.** With the default spec the new engine is byte-for-byte the same on
all three layouts and three seeds (4 robots, 1500 ticks each: positions, states, headings, batteries,
speeds and task counts). A single robot with a fractional `maxSpeed` also matches, except that it turns to
its next direction as soon as it has driven into the next segment (rather than a tick later, when its pose
would already disagree), and that one whose battery dies mid-cell coasts to a stop instead of freezing
there. Several slow robots (`maxSpeed < 1`) differ when they contest a cell, because a cell is now claimed
before a robot drives into it rather than checked when it arrives.

`Simulation` is the only place that drives robots. Each tick a robot, in id order: finishes whatever its
state allows (arrive, finish loading/unloading/charging, start a charging trip if idle and low), is taken
out of service if its battery is flat, lets one tick of time pass in its state (drain, dwell countdown,
charging) and advances along its path. Rules worth knowing:

- Only an idle robot with enough charge (`Robot::isAvailable()`) is assigned a task. An idle robot below
  its reserve goes to the nearest free charging station, or charges where it stands. If the layout has no
  charging station the reserve is ignored, since a robot could never recharge.
- A robot whose battery is empty becomes `Unavailable`: a task it had not collected goes back to `Pending`,
  one it was carrying fails. `Simulation::disableRobot` / `restoreRobot` do the same on request (restoring
  swaps in a fresh battery if the old one is flat).
- Loading, unloading, charging and unavailable robots are never pushed aside by other robots; idle and
  lower-priority navigating robots still are. A robot waiting on an occupied charger picks another one.
- `RobotSpec` on `SimulationConfig::robot` applies to every robot. The bundled scenarios use two ticks of
  loading and unloading everywhere, and an energy model (and recharging) on the layouts that have chargers.

Robots are shown by the visualisation but contain no drawing code: the view reads
`simulation::RobotSnapshot` (state, goal, heading, pose, speed, motion, battery, task, cargo, route, held
cells and what it is waiting for).

## Collision avoidance

Two layers keep robots apart. The lower one is physical and always on: a robot only drives into cells it
has claimed in `World`, which refuses a cell somebody holds, so robots can never overlap or swap cells
(every test that runs traffic audits that on every tick). The upper one, new in this stage, looks ahead so
that robots mostly never get that far.

- **`traffic::ReservationTable`** holds space-time reservations: `Occupy{cell, steps}` (a robot is on a
  cell for a range of simulation steps) and `Traverse{from, to, step}` (it moves along an edge). Asking
  whether a request fits returns the clash, if any: `Blocked` (a wall or outside the grid), `Vertex` (two
  robots on one cell in the same step) or `Swap` (two robots exchanging cells along an edge). The answer is
  independent of storage order: earliest step first, then the lowest robot id, so everything built on it
  is reproducible. A robot's own reservations never clash with each other.
- **`robots::forecastHolds()`** runs the robot's own motion code (turning, braking distance, the cells it
  has to hold ahead of itself) on a copy of the robot, as if nothing were in the way, to say which cells it
  will hold at each of the next steps. The table therefore reserves what the robot will really do, not an
  approximation of it.
- **`traffic::TrafficController`** rebuilds the table every step. `beginStep()` reserves what is already a
  fact: every robot's cell and the cells it holds, and robots with nowhere to go for the whole horizon.
  Then robots plan in ascending id order (priority is the id, as in the rest of the engine; planning out
  of order throws). A robot whose forecast fits is reserved as it is. One whose forecast clashes with a
  reservation made earlier is cut back to the cells before the first clash and reserves the cell it will
  wait on instead. It prefers a waiting cell that no robot with priority has on its route at all, however
  far ahead, so a robot waits outside a corridor rather than in it. If there is no such cell (it is
  already on the other's route) the plan says `inTheWay` and the robot is left to the physical layer.
  The default horizon is 12 steps; looking 48 ahead held robots back for clashes that never came.
- **In `Simulation`,** the plan limits how many cells a robot may claim this step. A held-back robot
  waits; if it has waited `maxBlockedTicks` and the cell is still free it asks the planner for a way round
  (the conflicting cell and every occupied cell count as walls) and takes it only if it is at most
  `maxDetourCells` (4) longer, since vehicles that stop at every corner lose more on a detour than on a
  short wait. If the cell is already occupied the older rules apply: the blocker steps aside if it can
  (idle or lower priority), otherwise the robot routes around it, and two robots face to face with a
  blocker that cannot move are sorted out by the one with a free neighbour stepping aside. A robot moved
  aside has used its step, and a robot that has already taken its step is not moved aside in it, so no
  robot moves more than a cell per tick (the second half is engine revision 2: the audit found robots
  that moved twice, see `ARobotThatHasMovedIsNeverPushedAsideInTheSameTick` and docs/audit.md).
- **Switching it off:** `SimulationConfig::reservations = false` leaves only the physical layer (and the
  deadlock-breaking rules above); the tests use it to show that robots still never collide but meet
  head-on.
- **What you can see:** the GUI draws each robot's planned route and held cells (`P` and `O` toggle them) and
  a red cross on the cell a held-back robot may not enter. The JSON has `path`, `holds` and `waiting` (cell,
  kind of clash, the robot in the way) per robot and a top-level `traffic` object with `replans` and the
  clashes planning has found, counted once per robot per step in which it was held back.

What it buys, honestly: on the bundled layouts the table does not change throughput measurably. Over 30
seeds and 3000 ticks, with the reservations on or off, tasks completed per 500 ticks (after the first
1000 ticks) were 33.0 vs 32.6
(`demo`), 15.9 vs 15.7 (`compact`) and 24.7 vs 24.4 (`fulfilment`), with no failed tasks either way; the
slowest 500-tick window was 28 vs 26, 13 vs 13 and 20 vs 20. Those differences are inside the seed-to-seed
spread (about +-3 per window on `demo`). What it does is make robots hold back *before* a clash instead of
meeting face to face: in `CollisionAvoidanceTest` two robots from opposite rooms of a one-cell corridor
never meet inside it (the lower-priority one waits in its room), whereas with `reservations = false` they
do. The scenario tests cover crossing paths, opposing corridor traffic, intersections of one-cell roads and
simultaneous arrivals, with an audit on every tick that no two robots overlap or swap cells.

Limits: the table looks 12 steps ahead, so a robot that has already entered a corridor when a robot with
priority is still far down it cannot wait outside, and falls back on the physical rules (one of them is
moved aside if it can be). Priority is the robot id, so a low-priority robot can be held back for a long
time by a busy high-priority one. Three or more robots locked in a one-cell corridor with no free
neighbour between them can still deadlock. The next section adds a search that finds a way round and a
way out of deadlocks, and lists what is still not guaranteed.

## Cooperative planning

The table can say "wait" but cannot find a way *round*, or make a robot get out of another's way. This
stage adds **prioritized (decoupled) planning**: robots plan one after another in order of priority
(lowest id first, as everywhere in the engine), each in space and time around the reservations of the
robots before it. It is cheap and simple, and it works well in practice, but it is neither complete nor
optimal; see the limits below.

- **`planning::SpaceTimeAStar`** searches over (cell, step) states within a window of at most 256 steps
  (the simulation uses the reservation horizon, 12). A robot may move to a neighbouring cell or *wait*
  where it is, if the `SpaceTimeConstraints` it is given say the cell is free at that step and the next
  (a robot holds the cell it is entering) and the move is not a swap with another robot. The heuristic
  is the true distance on the empty map (a breadth-first distance field per goal, kept for the last 24
  goals and keyed by a hash of the walkable cells, so a changed layout never reuses a stale field). The
  goal only counts as reached if nobody needs it again before the window ends (otherwise the robot
  would park in somebody's way) and it is *terminal*: a plan never passes through it, because a robot
  stops the first time it gets there. If the window ends first the result is `Partial`, the state
  nearest the goal. Ties are broken by (f, h, state index) and, between plans of equal time, by fewer
  moves, so waiting is preferred to wandering. Work is counted in states expanded.
- **`traffic::CooperativeRouter`** serves a robot the table holds back. The plan starts where the cells
  the robot has already been granted end, at the step it gets there (`robots::forecastHolds`: a moving
  vehicle cannot stop at once). First `followRoute` measures the current route against the
  reservations (can it be followed, waiting where needed, and when does it arrive?), then the search
  runs. The new route replaces the old one only if the old one cannot be followed at all (a *dodge*: the
  robot is in somebody's way and steps into a side cell; at most `maxDodgeCells` = 12 cells longer) or
  the new one arrives sooner or gets nearer (at most `maxDetourCells` = 4 longer). Beyond the window a
  plain A* tail, steering clear of robots standing still, completes the route. Granted cells are never
  taken back.
- **`traffic::ProgressMonitor`** counts, for each robot that wants to move, how many ticks it has stayed
  within 2 cells of where it began to wait, and whom it waits for. A robot waits for at most one other,
  so rings of robots waiting for each other (deadlocks) are found by following those links.
- **In `Simulation`** (`cooperation.cpp`):
  - *Work per update is limited.* All searches of a step share `planningWorkPerStep` (4000) and one
    search may spend `planningWorkPerSearch` (1500). Work is counted in states expanded, never in time,
    so the limit cannot make a run depend on the machine (`PathPlanner::work()` reports the cells the
    plain planner has expanded, so `peakStepWork` counts that too). A search that does not fit waits for the next
    step, where the robots put off longest go first, each with a share of its own; one that runs out of
    work is repeated on the next tick with twice the allowance, up to eight times the first.
  - *Replanning.* A held-back robot searches when it first waits and then every `replanInterval` (4)
    ticks. Routes also change with the layout: `Simulation::placeObstacle` and `clearObstacle` rebuild
    the table, and a robot whose route crosses a new wall gets a new one from the last cell it was
    granted; with none, a task not yet collected goes back to the queue and a carried one fails. Tasks
    that start or end on the wall are cancelled. Cancelling or reassigning a task replaces the route as
    before.
  - *Prolonged blocking.* A robot that wants to move and does not for `prolongedBlockTicks` (20) ticks is
    counted (`prolongedBlocks`) and its searches may spend twice the work.
  - *Deadlock recovery.* A ring whose robots have all been stuck for `deadlockTicks` (6) is a deadlock.
    The first time, the lowest-priority robot that is not busy in place (loading, unloading, charging,
    out of service), has not moved this step and has a free neighbour is moved aside one cell. If the
    ring is still there after as many ticks again, and again, the lowest-priority robot
    in it that has not had its turn is put first in the order of priority for `priorityBoostTicks` (40),
    so that the others plan around its route. A ring in which nobody has room is counted as unresolved.
  - *Giving up.* A robot stuck for `abandonTicks` (150) hands its task back: to the queue if not yet
    collected, failed if it carries cargo.
- **Two smaller changes to the layers below.** A robot at rest that cannot take its next cell reserves
  what it will do, which is to wait a little (up to 6 steps) and then go, instead of standing where it is
  for the whole horizon, so the robots after it can see that it will move off. A robot asked to step
  aside prefers a cell off the route of the robot that wants its cell, so it is not pushed along in
  front of it.
- **Determinism.** The same simulation state gives the same decisions: priority is the id (changed only
  by the boost), work is counted rather than timed, ties have explicit keys, and nothing depends on
  container order. `TheSameSimulationRunTwiceDecidesTheSameThingsEveryTick` compares snapshots on every
  tick at two work limits.
- **Switching it off.** `SimulationConfig::cooperative = false` leaves the table and the physical rules
  without any search (the two smaller changes above still apply); `reservations = false` leaves the
  physical layer only. Safety never depends on this stage: `TrafficAudit` (no overlap, no swap, at most a
  cell of movement per tick, no task lost) runs on every tick of every scenario test.
- **What you can see.** The snapshot and JSON have a `planning` object (`searches`, `reroutes`, `dodges`,
  `partialPlans`, `outOfBudget`, `deferred`, `work`, `peakStepWork`, `obstacleRepairs`, `prolongedBlocks`,
  `deadlocks`, `deadlocksRecovered`, `deadlocksUnresolved`, `priorityBoosts`, `abandoned`) and each robot
  has `stuckTicks`.

What it buys, measured: on the bundled layouts, 3000 ticks and seeds 1 to 10, the previous stage completed
195.4 (188 to 200) tasks on average on `demo`, 94.5 (91 to 100) on `compact` and 146.8 (139 to 157) on
`fulfilment`; this one completes 217.7 (213 to 225), 107.7 (105 to 110) and 155.7 (149 to 164), with no
failed task in either. That is the effect of the search, the delayed reservations and the sidestep
preference together (they were not measured apart), and on `fulfilment` the ranges overlap. In the
scenario tests, two robots meeting in a one-cell corridor with a niche pass at the niche in less time
than when they meet and sort it out afterwards, and eight robots sharing a long corridor with niches all
get through. The scenarios cover narrow corridors, intersections, several destinations and conflicting
priorities, and 20 random crossings of a corridor with a niche all end with every task done.

Limits, honestly:

- **Not complete.** Priority is fixed (the id), so a lower-priority robot always adapts, even when it
  would be cheaper for the other. A solution that needs the robot with priority to give way first is
  missed; the boost that ends a deadlocked ring is the only exception.
- **Not optimal.** The search is optimal for one robot given the reservations of the ones before it, not
  for all of them together. Detours are capped (`maxDetourCells`, `maxDodgeCells`), so a robot sometimes
  waits where a longer route would have been faster.
- **Short sight.** It plans 12 steps ahead. A conflict further off is dealt with only when it comes into
  the window, and the tail of a long route ignores other robots (except those standing still).
- **Bounded work.** Under the work limit a search can be put off or cut short; the table and the physical
  rules keep robots safe meanwhile, but they may wait longer.
- **Nothing sends an idle robot away.** Robots that stand still are obstacles to planning. An idle robot
  boxed in by robots that wait for its cell (four robots delivering to one corner cell) or pushed ahead
  of another robot into a dead end that holds that robot's goal stays there, and so do the robots around
  it. `RobotsQueuingAroundAnIdleRobotOnTheirSharedGoalStaySafe` and
  `AnIdleRobotPushedIntoADeadEndOntoAnotherRobotsGoalStaysSafe` pin that these cases stay safe and are
  reported (`prolongedBlocks`, `abandoned`), not that they are solved; after `abandonTicks` the task
  goes back to the queue and, in the dead-end scenario, to the same robot again. The tests that expect
  every task to finish use goals of their own for each robot for this reason.
- **Rings with no room** are counted as unresolved and left to `abandonTicks`, and three or more robots
  locked in a one-cell corridor with no free neighbour can still deadlock.

## Visualisation

The window shows the grid with walls, shelves, pickup `P`, dropoff `D` and charging `C` cells (letters
appear when zoomed in), task markers, robot placeholders (a coloured square per robot by state: blue idle, orange
heading to a pickup, red heading to a dropoff, lime heading to a charger, amber loading, magenta unloading,
green charging, pale grey unavailable; a nub on the side it faces, a box on top while it carries cargo, and the id and a battery gauge when zoomed in), column/row indices along the top and left edges,
and a status bar with the tick, playback speed, zoom, task counts and the cell under the pointer.
Two overlays, both on at start and toggled with `P` and `O`, show each robot's planned route (a line in the
robot's colour through the cell centres to a marker on its goal, plus a red cross on the cell a held-back
robot may not enter) and the cells robots hold (an outline in the robot's colour, which includes the
cells a moving vehicle has claimed ahead of itself).

- **Fixed timestep.** The simulation runs on a worker thread; `core::FixedTimestep` turns elapsed real time
  into whole simulation steps of constant size, so frame rate, window resizing and a slow renderer never
  change what is simulated. Playback speed only scales how many steps happen per real second, and time
  spent paused is not replayed afterwards. A stall longer than 200 steps' worth is dropped, not replayed.
- **Smooth motion at any frame rate.** The simulation only ever advances in whole steps. For each frame
  `SimulationRunner::frame()` returns the last two states and how far into the next step real time has got
  (`alpha`), and `view::blend()` interpolates the robots' poses and headings between them. What is drawn is
  therefore a function of real time alone, whatever the frame rate (`FrameRateTest` replays the same run at
  10 to 240 fps and compares), at the price of showing the simulation up to one step behind. Pausing
  freezes the picture where it is and resuming carries on from there.
- **Camera.** `view::Camera` maps world space (cells, y down) to screen pixels (`screen = (world - centre) *
  zoom + window centre`), zooms about the pointer, pans in pixels, keeps its centre inside the warehouse,
  and re-fits the whole grid on resize until the user pans or zooms.
- **Testable drawing.** `view::buildScene()` produces a backend-neutral `DrawList` (rectangles, lines,
  text); `rendering::SdlWindow` only executes it. The scene, camera and input handling are unit-tested
  without SDL.
- **Resources.** `SdlWindow` owns SDL init, window and renderer through RAII (destroyed renderer, window,
  then `SDL_Quit`), including when construction throws part-way.
- **Headless.** `warehousesim_cli`, the simulation library and all tests need no display or SDL.

Text is drawn with SDL's built-in 8x8 debug font. Hovering a robot shows its state, goal and battery in
the status bar.

## Fleet interface

`warehousesim_gui` lets an operator look at the fleet and steer it without ever touching the simulation.

- **Selecting and inspecting.** Click a robot on the map or in the list (or Tab through them) and the panel
  shows its state and goal, cell and heading, speed and motion phase, battery, task and its pickup and
  dropoff, whether it carries cargo and how many deliveries it has made, the length and end of its route and
  the next cells, and whether it is held back (where, and by whom) or has not moved for some ticks. The
  selected robot gets a white frame, its route is drawn thick *even when routes are off*, and the pickup and
  dropoff of its task are doubled in white. Clicking a task row (or `T`) does the same for that task's two
  cells. Clicking empty floor clears the selection.
- **Seeing routes.** `P` shows every robot's planned route; a selected robot's route is always shown.
  `O` outlines the cells robots hold.
- **Adding a delivery task.** `N`, then click a pickup, then click a dropoff (any open floor, not the same
  cell); `1`-`4` before or in between sets the priority. A wall, a shelf or the same cell is refused on the
  spot and entry goes on. The added task is selected so that its priority can be changed at once.
- **Changing a priority.** Select a task that is still *waiting* and press `1`-`4`. A task a robot has
  claimed, or a finished one, cannot change priority (priority only orders tasks that wait); the panel says
  so, and so does the simulation if the command reaches it anyway.
- **Active and finished tasks.** The panel lists the active tasks (waiting ones in the order the dispatcher
  will take them, then the ones a robot has) and the last 12 finished ones, newest first, with the counters
  of done, failed and cancelled tasks.
- **Pause, speed and restart.** Space, `[` `]` and `R`. A restart rebuilds the scenario from the start with
  the same seed (and layout, whether predefined or from a map file); every manually submitted task is gone with it.
- **Telling things apart.** Robots are filled squares coloured by state (with a heading nub, a battery gauge
  and a cargo box); obstacles are filled cells in wall and shelf colours that no robot colour comes near
  (`NoRobotColourCanPassForAFloorOrAnObstacleOrAStationCell`); destinations are *diamonds* (amber round a
  pickup until the cargo is collected, cyan round a dropoff), a shape nothing else has; planned paths are
  thin lines through the cell centres in the robot's colour. A legend at the bottom of the panel shows each.

### Commands

The interface and the simulation meet in one place, and it is narrow.

```
 InputEvents -> FleetController -> FleetOutput{commands, paused, speed}
                                        |
                       FleetSession  -- post(Command) / setPaused / setSpeed -->  SimulationRunner
                                        ^                                           | worker thread:
                                        '-- commandLog(), frame() <-----------------' applies each command
                                                                                      between two steps
```

- A `simulation::Command` is a plain value: `SubmitTask{pickup, dropoff, priority}`,
  `SetTaskPriority{task, priority}`, `CancelTask{task}`, `SetRobotService{robot, inService}`,
  `PlaceObstacle{cell, type}`, `ClearObstacle{cell}` or `ResetScenario{seed?}`. The panel offers the first
  two and the restart; the others are the same operations the engine always had, now as commands so that a
  recording can hold them. `FleetController` (no SDL, no simulation type but the snapshot it is shown) only
  *describes* what the user wants as commands. It cannot touch a `Simulation`, because it is never given
  one.
- `SimulationRunner::post()` queues a command (at most 256 wait; more are refused and the panel says so).
  The worker applies the queue in order, **between two steps** and also while paused, through
  `simulation::apply()`, which validates (a cell outside the map or not floor, the same pickup and dropoff,
  an unknown or no longer waiting task are all refused with a reason and change nothing; nothing throws).
  Each command is recorded in `commandLog()` with the tick it was applied at and its outcome, and the
  interface reads that log to show "task #7 added" or why not.
- Pausing, resuming and the speed are *not* commands: they decide when steps happen, never what a step
  computes (`setPaused`/`setSpeed` as before).
- **Determinism.** What a step computes depends only on the simulation state. Commands change that state
  only at step boundaries, so the state at any tick is a function of the scenario, its seed and the command
  log. The tick a command lands at depends on timing, but the log says which: replaying the log on a fresh
  simulation (`RunnerDeterminismTest`, `FleetDeterminismTest`) gives the same state as the live run, and a
  session of only looking, selecting, zooming and hovering, or of pausing and changing speed at random,
  leaves the run byte-identical to a direct one. A manually submitted task takes no draw from the random task
  generator, so the generated stream is the same with or without it.
- `FleetSession` is the frame loop's glue (read what to draw, tell the controller what became of its
  commands, run it, forward its output); `apps/gui/main.cpp` only adds SDL around it, so the glue is tested
  with a real runner and no window (`fleet_session_test.cpp`).

Limits: the interface selects, adds tasks and changes priorities, but cannot cancel a task, take a robot out
of service or edit the layout (these are commands, and a recording holds them, but they are not on the
panel). A restart
always keeps the seed it started with unless a `ResetScenario{seed}` command says otherwise (the GUI has no
key for that). The panel is left out of windows smaller than 720 x 400. Text is the SDL debug font, so the
panel is 39 characters wide and long messages wrap into two lines.

## Performance metrics

Every figure in this section is **simulated**: it measures a model (ticks, grid cells, an abstract energy
unit) running with a given seed, and says nothing about how a real robot or a real warehouse would perform.
The dashboard, the CSV files and the benchmark all say so on their face: the dashboard's title ends
"(simulated)" and its last line reads "Simulated results. Not a claim about real robots.", and every CSV row
has a `basis` column that is always `simulated`.

### What is measured

A `metrics::MetricsAccumulator` is told the state after every tick as plain data (`metrics::TickObservation`)
and keeps counts only. The adapter `simulation::MetricsCollector` builds those observations from a
running `Simulation` by reading its public state: measuring never writes to the simulation, and a measured
run is step-for-step the same run as an unmeasured one (`AMeasuredRunIsTheSameRunStepForStepAs...`).
Each robot is in **exactly one activity per tick** (the one it ended the tick in): *moving* (driving to a
pickup, a dropoff or a charger and given its next cell, so also while accelerating or turning), *waiting*
(driving but refused its next cell), *handling* (loading or unloading), *charging*, *idle* or *out of
service*. All the time figures come from that budget.

| Figure | Definition | CSV column(s) |
|--------|------------|---------------|
| Tasks completed | tasks delivered during the interval (also failed, cancelled, and open at the end) | `tasks_completed`, `tasks_failed`, `tasks_cancelled`, `tasks_open_at_end` |
| Tasks per simulated hour | completed ÷ simulated hours, where hours = ticks × `seconds_per_tick` ÷ 3600 (1 tick = 1 s unless told otherwise; the scale is arbitrary, not calibrated to any robot) | `tasks_per_sim_hour`, `tasks_per_robot_hour` |
| Average task completion time | mean over completed tasks of (tick delivered − tick submitted), in ticks and simulated seconds. It **includes the wait in the queue**, so with a backlog it measures how long the served tasks had waited more than the trip | `avg_completion_ticks`, `avg_completion_sim_s`, `max_completion_ticks` |
| Robot utilization | (moving + waiting + handling robot-ticks) ÷ (all robot-ticks − out-of-service). A charger trip is moving; charging is not busy | `utilization` |
| Average travel distance | cells entered ÷ robots, and cells entered ÷ tasks completed (empty trips and charger trips included) | `cells_per_robot`, `cells_per_completed_task` |
| Idle time | robot-ticks idle, and as a fraction of all robot-ticks | `idle_robot_ticks`, `idle_fraction` |
| Waiting time | robot-ticks held back from the next cell, and as a fraction | `waiting_robot_ticks`, `waiting_fraction` |
| Collision conflicts prevented | the number of **episodes** in which a robot was held back from a cell that another robot held or had reserved (consecutive ticks of waiting are one episode). `conflict_detections` is the raw number of clashes the reservation table found while planning, step by step | `conflicts_prevented`, `conflict_detections` |
| Collisions that occurred | an audit that does not trust the thing that prevents them: two robots in one cell, or two robots that changed places, in a tick. Always 0 in the bundled scenarios | `collisions_occurred` |
| Replanning frequency | routes replaced while a robot was under way (after a clash, to get round a robot, or after the layout changed), and per simulated hour; `searches` counts the cooperative searches run, successful or not | `replans`, `replans_per_sim_hour`, `reroutes`, `repaths`, `obstacle_repairs`, `searches` |
| Estimated energy | idle ticks × idle drain + cells entered × cost per cell + handling ticks × handling drain, under a stated `EnergyModel` (default 1 / 15 / 2, the drain of the bundled robots that have a battery). Charging draws nothing | `energy_units_est`, `energy_units_per_task_est`, `energy_idle_est`, `energy_moving_est`, `energy_handling_est` |

A figure whose denominator is zero (no time observed, no robot, no task completed) is **undefined**, not 0:
an empty CSV field and `n/a` on the dashboard, never `nan`. A zero numerator over a real denominator is a
real 0. "Conflicts prevented" means traffic control stopped a robot entering a cell another robot held or
had reserved; the world's own claims would also have refused the cell, so it is a count of clashes the
planner avoided, not of crashes a world without it would have had.

The estimate uses one stated model for every layout rather than each robot's own drain, so that the demo
hall (whose robots have no battery model) is not reported as using no energy; `simulation::energyModelOf()`
gives the simulation's own, and a test keeps the standard model equal to the bundled robots' drain.

Time is simulated time only: nothing in the metrics, the collector or the benchmark reads a clock. A run
that is observed from the middle (`MetricsCollector` built on a simulation already at tick N) gives exactly
the difference of two measurements from the start (`MeasuringFromTheMiddleGivesTheDifferenceOf...`).

### Dashboard and CSV

- **Dashboard.** In the GUI, `M` shows the figures over the top left of the map (every number above, the
  energy model stated beside the estimate, the disclaimer last) and one bar for each sampling window of the
  run so far — tasks completed in each window of 100 ticks, the latest that fit. It reads the runner's live
  figures, which cover every step the worker took; a restart (`R`) starts the measurement over too. In a
  window too small for it, it says so on one line. `warehousesim_cli --metrics` prints the same text to
  stderr.
- **CSV.** `E` in the GUI, or `warehousesim_cli --metrics-csv BASE`, writes `BASE.runs.csv` (one row of
  every figure above), `BASE.robots.csv` (one row per robot with its time budget in ticks) and
  `BASE.series.csv` (one row per sampling window). Numbers are written with fixed decimals in the C locale,
  fields are quoted per RFC 4180, each file is written whole to a temporary name and renamed over the real
  one, and the same run always gives the same bytes. The runs file is the contract (its header is pinned by
  a test); `warehousesim_fleet_bench --csv FILE` writes the same columns, one row per run.

### Benchmark

`warehousesim_fleet_bench` runs every combination of layout, fleet size and seed for a fixed number of
ticks with the same task rate, and prints one table with the seeds pooled (counts added up, so every
figure is total over total). Defaults: all three layouts × fleets of 1, 2, 4 and 8 robots × seeds 1, 2, 3 ×
3000 ticks (50 simulated minutes at 1 tick = 1 s), a new random task every 5 ticks. Fleet size is chosen
with `ScenarioOptions::robots`: robots start on the charging stations first and then on open floor, spaced
at least two cells apart, in reading order; asking for the number a layout already gets changes nothing,
and the task stream depends on the seed alone, so fleets of different sizes are offered the same work.

It is a benchmark and not just a printout because it **fails** (exit 1) if any run has two robots overlap or
change places, if a task is unaccounted for (completed + failed + cancelled + open ≠ submitted), if a robot
is not accounted for in every tick, or if running the first case twice gives different results.
`ctest` runs `--quick` (150 ticks, fleets 1 and 2, seed 1) as `fleet_bench_smoke`.

Baseline (this commit; the full default sweep, 36 runs, takes about 2 seconds; all figures simulated):

| layout | robots | runs | tasks/h | tasks/robot-h | mean done (s) | backlog | utilization % | waiting % | idle % | charging % | cells/task | conflicts/h | replans/h | energy/task |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| demo | 1 | 3 | 77.6 | 77.6 | 542.2 | 535.3 | 97.8 | 0.0 | 2.2 | 0.0 | 24.4 | 0.0 | 0.0 | 375.5 |
| demo | 2 | 3 | 145.6 | 72.8 | 295.1 | 478.7 | 97.9 | 1.8 | 2.1 | 0.0 | 25.2 | 33.6 | 54.8 | 387.1 |
| demo | 4 | 3 | 260.0 | 65.0 | 420.9 | 383.3 | 97.9 | 4.4 | 2.1 | 0.0 | 26.6 | 164.0 | 323.6 | 407.6 |
| demo | 8 | 3 | 444.0 | 55.5 | 395.3 | 230.0 | 97.9 | 9.9 | 2.1 | 0.0 | 27.4 | 746.0 | 1306.8 | 420.9 |
| compact | 1 | 3 | 73.2 | 73.2 | 555.3 | 539.0 | 95.0 | 0.0 | 2.0 | 2.9 | 26.2 | 0.0 | 0.0 | 401.7 |
| compact | 2 | 3 | 127.6 | 63.8 | 441.0 | 493.7 | 95.6 | 3.4 | 1.8 | 2.6 | 27.6 | 61.6 | 126.4 | 423.6 |
| compact | 4 | 3 | 215.6 | 53.9 | 325.2 | 420.3 | 96.1 | 9.3 | 1.7 | 2.2 | 28.7 | 344.8 | 547.2 | 440.4 |
| compact | 8 | 3 | 291.6 | 36.5 | 474.6 | 357.0 | 96.9 | 20.1 | 1.6 | 1.5 | 33.0 | 1712.8 | 2265.6 | 504.5 |
| fulfilment | 1 | 3 | 51.6 | 51.6 | 388.9 | 557.0 | 95.5 | 0.0 | 1.4 | 3.1 | 38.8 | 0.0 | 0.0 | 591.3 |
| fulfilment | 2 | 3 | 100.4 | 50.2 | 562.5 | 516.3 | 95.5 | 1.4 | 1.5 | 3.0 | 38.8 | 21.2 | 51.2 | 591.4 |
| fulfilment | 4 | 3 | 190.0 | 47.5 | 220.0 | 441.7 | 95.5 | 3.3 | 1.6 | 2.9 | 39.5 | 106.8 | 227.6 | 602.3 |
| fulfilment | 8 | 3 | 332.4 | 41.5 | 508.6 | 323.0 | 95.9 | 7.7 | 1.8 | 2.4 | 41.3 | 580.4 | 1109.6 | 629.6 |

`backlog` is the mean number of tasks still open at the end of a run. The table was first produced with
Apple clang at `-O1 -ffp-contract=off`, and the CMake `release` preset (`-O3`, link-time optimisation) gives
the same figures in every row of this table and of the next one (compared row by row). A simulation's output does not depend on timing or on the optimisation level, but
other compilers and platforms have not been compared (see [docs/verification.md](docs/verification.md)), and
the numbers are simulation outputs, not performance measurements of the code.

How to read it, and how not to:

- **Every row is saturated.** The scenarios offer a task every 5 ticks, which is 720 tasks per simulated hour
  at 1 tick = 1 s; no fleet here gets near that (the best is 444), so the backlog is in the hundreds and the
  robots are busy 95–98% of the time. Utilization therefore tells almost nothing in this table: it is high
  because there is always work, and it counts waiting as busy. To see a fleet that keeps up, lower the rate
  (`--spawn 25`; the second table below).
- **More robots give more throughput, with diminishing returns.** Tasks per hour rise with the fleet in all
  three layouts (1 → 8 robots: demo ×5.7, fulfilment ×6.4, compact ×4.0) while tasks per robot-hour fall
  (demo 77.6 → 55.5, compact 73.2 → 36.5, fulfilment 51.6 → 41.5).
- **The small room congests first.** With 8 robots the 12×8 `compact` room has robots waiting 20.1% of the
  time (demo 9.9%, fulfilment 7.7%), preventing about 1700 conflicts per simulated hour, driving more cells
  per task (33.0 against 26.2 for one robot) and getting the least out of the extra robots. The table does not
  say why; the campaign in [docs/benchmarks.md](docs/benchmarks.md#scenario-campaign) runs larger fleets and
  finds the cause only as a likely one.
- **`mean done` is not a trip time and is not monotonic in the fleet size.** With hundreds of open tasks it
  mostly reflects how long the tasks that happened to be served had waited (priority and age decide who is
  served), and the three seeds are few. Compare it only between runs that are not backlogged.
- **Three seeds are a baseline, not a confidence interval.** The numbers are exact for these seeds; no claim
  is made about how much they vary with others (the demo hall's throughput varies by about ±3 tasks per
  500 ticks between seeds, see the collision-avoidance notes).

The same sweep with work the fleet can keep up with (`warehousesim_fleet_bench --spawn 25 --layouts
demo,fulfilment`, a task every 25 ticks, 144 per simulated hour):

| layout | robots | runs | tasks/h | tasks/robot-h | mean done (s) | backlog | utilization % | waiting % | idle % | charging % | cells/task | conflicts/h | replans/h | energy/task |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| demo | 1 | 3 | 74.8 | 74.8 | 515.5 | 57.7 | 97.9 | 0.0 | 2.1 | 0.0 | 25.9 | 0.0 | 0.0 | 396.9 |
| demo | 2 | 3 | 138.4 | 69.2 | 84.6 | 4.7 | 96.1 | 1.8 | 3.9 | 0.0 | 26.1 | 31.2 | 67.2 | 401.3 |
| demo | 4 | 3 | 142.8 | 35.7 | 50.3 | 1.0 | 49.1 | 1.8 | 50.9 | 0.0 | 24.6 | 81.6 | 90.4 | 428.0 |
| demo | 8 | 3 | 129.6 | 16.2 | 60.9 | 2.0 | 36.9 | 12.6 | 63.1 | 0.0 | 26.9 | 212.0 | 155.6 | 551.8 |
| fulfilment | 1 | 3 | 50.8 | 50.8 | 480.7 | 77.7 | 95.5 | 0.0 | 1.4 | 3.1 | 39.5 | 0.0 | 0.0 | 601.0 |
| fulfilment | 2 | 3 | 100.0 | 50.0 | 361.4 | 36.7 | 94.7 | 1.2 | 2.4 | 2.9 | 38.7 | 16.8 | 50.4 | 590.7 |
| fulfilment | 4 | 3 | 142.0 | 35.5 | 70.0 | 1.7 | 69.6 | 2.0 | 28.4 | 2.1 | 38.9 | 62.8 | 156.0 | 619.9 |
| fulfilment | 8 | 3 | 140.0 | 17.5 | 69.8 | 2.3 | 35.7 | 2.6 | 63.5 | 0.8 | 38.6 | 78.4 | 144.4 | 718.2 |

Here throughput stops at the offered load (about 140–143 tasks per hour for 4 and 8 robots), the backlog
drops to one or two tasks, mean completion time falls to 50–70 s, and utilization tells something at last:
it halves or worse as robots are added past what the work needs. Extra robots then only cost: in the demo hall
8 robots wait 12.6% of the time (against 1.8% for 4), complete fewer tasks per hour than 4 (129.6 against
142.8; a difference this small can be seed noise, but the campaign finds much larger drops at 16 and 64
robots, in nine of ten layouts at 16, see [docs/benchmarks.md](docs/benchmarks.md#scenario-campaign)), and
the energy estimate per task rises because idle robots draw power in the model (idle drain 1 per tick).

### Limits

- The time scale (1 tick = 1 s) and the energy model are stand-ins chosen for the model; nothing calibrates
  them to a real vehicle, so absolute values (tasks per hour, energy units) carry no real-world meaning.
  Ratios between runs of the same model do.
- Energy is an estimate from the time budget and distance, not what the simulated batteries lost (the two
  agree exactly when the estimate is given the robots' own drain, which a test checks).
- Waiting is what the engine refused: a robot that stands still because its path is empty, or while it
  rotates, is not "waiting" here. A robot that is refused its cell and is then pushed aside by another
  robot's sidestep in the same step is still counted as waiting in that step, although it ended it one
  cell away (seen in the 8-robot runs of all three layouts; in each such step the robot was at rest). A robot that drove part of
  a step and was refused only after it had stopped is moving in that step, not waiting.
- The sampling windows (100 ticks, the latest 240 kept) bound the memory of a long run; only the dashboard
  chart and the series CSV use them.
- A restart or a different seed is a different run: the figures start over, they are never carried across.

## Performance and planning threads

This section is about how long the program takes to compute, not about simulated results: everything in
it is a wall-clock time or a count of what the program did. The times are from one machine (Apple M4,
10 hardware threads), with binaries compiled directly with Apple clang 21 (`-O3 -DNDEBUG`, no CMake), and are medians of the number
of runs stated. [docs/benchmarks.md](docs/benchmarks.md) repeats the whole-step and thread measurements with
the CMake `release` preset (link-time optimisation) and says which of these it reproduces. The figures say
nothing about real robots or other machines; rerun `warehousesim_scaling_bench` on a Release build to get
your own.

### How it was measured

A small sampling profiler (a `SIGPROF` timer and `backtrace`, in a scratch driver that is not part of the
repository, because the system's `sample` tool gave nothing here) on a build with `-O2` and frame
pointers; percentages are shares of the samples on the stepping thread. Allocations were counted by
replacing `operator new` in a scratch driver, after 500 warm-up steps. Timings are the median of five
runs of the old and the new engine, alternated on the same configurations and the same planning limits;
every pair was checked to end in the same state (tasks done and failed, and the state checksum).

### What the profile showed, and what was done

The hall used is the new `large` layout (100x64, 6400 cells) with 128 robots and a task every tick, which
keeps the fleet busy. "Before" is the engine of the previous commit with the `large` layout added and the
engine error below fixed (both needed to run these cases at all).

The profile of the engine as it was (6 000 steps, 1.15 ms a step), and what each finding led to:

| Cost centre | Share of a step | What was done |
|---|---:|---|
| `SpaceTimeAStar::distanceField`, itself: it hashed every cell of the layout on each lookup to tell whether the layout had changed | 24 % | `Warehouse::version()` names the layout as it is; the hash is only made again after an edit |
| `ReservationTable::release` and `check`: release walked every cell that held anything and the whole `std::map` of edges; every question built a `Conflict` | 14 % and 9 % | per-robot lists of what it holds; edges in flat per-cell arrays instead of a `std::map`; `isFree` and `isTraversable` answer yes or no without building a `Conflict` |
| Heap allocation (`operator new` and the allocator under it) | 15 % | see the allocation table below: 91 times fewer allocations |
| `forecastHolds`: a copy of the robot and a vector of vectors for each forecast, several forecasts per robot per step | 13 % | a reusable `ForecastBuffer` and working copy, request buffers kept between calls |
| Repeated grid queries: the dispatcher ran one A* for each (robot, pickup) pair, the charger search one for each charger | 5 % in A* | one breadth-first sweep from the goal serves every start (`PathDistance`, with an open-addressing table); space-time distance fields are kept for up to 512 goals (at most 32 MB) and are still *charged* for exactly as before, so the planning limits mean what they meant |
| Blocked-cell sets (`std::unordered_set<Point>`, one allocation for each cell, built for most questions) | 2 % | `core::PointSet`: two allocations however many cells, and it walks its points in the order they were inserted |
| Route counts of the traffic controller (`std::unordered_map`) | under 1 % | an array indexed by cell |
| Everything else: the dispatcher, motion, world claims | small | left alone |

What was left of a step in a profile taken once the single-thread changes were in (20 000 steps on the
same hall and fleet, old limits, 0.16 ms a step): the traffic controller planning each robot in priority order 55 % (forecasting 23 %,
table checks and reservations 20 %), A* for new routes 13 %, `nearestFreeCharger` 8 %, the dispatcher 7 %,
the sweeps for distance fields 6 %. Almost all of it has to happen in order: each robot plans against the
reservations of the robots before it.

**Heap allocations per step**, same runs, same planning limits:

| Case | Before | After |
|---|---:|---:|
| `compact`, 20 robots | 3 533 allocations, 428 kB | 119 allocations, 6.5 kB |
| `fulfilment`, 24 robots | 3 034 allocations, 372 kB | 96 allocations, 5.0 kB |
| `large`, 128 robots | 17 247 allocations, 2.0 MB | 189 allocations, 46 kB |
| `large`, 256 robots | 37 052 allocations, 4.6 MB | 441 allocations, 154 kB |

The changes behind it: forecasts and the requests built from them go into buffers that are kept between
calls; the traffic controller's route counts are an array instead of an `unordered_map`; blocked cells are a
`core::PointSet` (two allocations however many cells, instead of one per cell); `PathDistance` keeps its
answers in one open-addressing table; the cells to steer clear of are made only when a plan falls short of
its destination and needs them.

**Single-thread speed**, the same simulation as `warehousesim_cli` runs (in a scratch driver), median of
five, with the planning limits the engine always had (1 500 work per search, 4 000 per step). "Same run"
means the same tasks done and failed and the same state checksum:

| Case | Before ms/step | After ms/step | Faster | Same run |
|---|---:|---:|---:|---|
| `demo`, 4 robots | 0.015 | 0.006 | 2.5x | yes |
| `compact`, 20 robots | 0.120 | 0.061 | 2.0x | yes |
| `fulfilment`, 16 robots | 0.092 | 0.038 | 2.4x | yes |
| `fulfilment`, 24 robots | 0.156 | 0.070 | 2.2x | yes |
| `large`, 64 robots | 0.528 | 0.142 | 3.7x | yes |
| `large`, 100 robots | 0.959 | 0.233 | 4.1x | yes |
| `large`, 128 robots | 1.320 | 0.297 | 4.4x | yes |
| `large`, 192 robots, a task every 2 ticks | 1.223 | 0.200 | 6.1x | yes |
| `large`, 256 robots | 2.929 | 0.492 | 6.0x | yes |

And with the limits the `large` hall gets (below), where robots really do search for ways round each other:

| Case | Before ms/step | After ms/step | Faster | Same run |
|---|---:|---:|---:|---|
| `large`, 100 robots | 1.396 | 0.291 | 4.8x | yes |
| `large`, 128 robots | 1.956 | 0.386 | 5.1x | yes |
| `large`, 128 robots, a task every 5 ticks | 0.539 | 0.144 | 3.7x | yes |
| `large`, 256 robots | 3.550 | 0.664 | 5.3x | yes |
| `large`, 256 robots, a task every 2 ticks | 2.028 | 0.455 | 4.5x | yes |

Nothing about what a run computes changed: the three bundled halls give byte-identical output and
recordings to the previous commit (3 layouts x 4 seeds x 2 000 steps), and a recording from the previous
engine replays on this one. The one exception is the new `large` layout, which has no history.

**Planning limits for a big hall.** A search in space and time starts from the distance from every cell to
its goal, a sweep that costs a cell of work for each cell, and the usual limit for one search is 1 500. On
a hall of more than 1 500 cells no search could ever begin, and on `large` robots never looked for a way
round each other (every search ended "out of budget"). A scenario on a hall of 1 501 to 40 000 cells now
adds the sweep to the limit of one search and scales the limit of a step the same way (`large`: 7 900 and
21 066). The bundled small halls keep their limits. On `large` with 128 robots, over 3 000 steps with seeds
1 and 2, tasks done went from 1 257 and 1 186 to 1 485 and 1 545, and tasks failed from 35 and 34 to 15
and 1.

**An engine error found on the way.** A robot sent to the free charger it was standing on has arrived
before it sets out; the step then had nothing to drive and threw `InvalidTransition: the robot has no path to
follow`. It showed after about a thousand steps with 128 robots on `large` (seeds 1 and 2: steps 1 056 and
862), and also on `compact` with 20 robots and a task every 2 ticks (seed 5, step 3 031); runs that did not
throw are unchanged. Fixed in
`Simulation::moveAlongPath`, with a regression test.

### Planning threads

`--threads N` (CLI and GUI; `Simulation::usePlanningThreads(N)`, `SimulationRunner::usePlanningThreads(N)`,
`ReplayOptions::planningThreads`) lets the independent path questions of a step be answered by N worker
threads besides the stepping one. They are the ways to the pickups of the tasks assigned in the step, the
ways to every free charger when a robot needs one, and the routes repaired after `placeObstacle`. Each is a
function of the layout and two cells, nothing else.

- **Immutable snapshot.** `planning::PathBatchExecutor` answers from a private copy of the layout, taken
  when `Warehouse::version()` changes. The caller can go on editing its own between batches.
- **One planner per thread.** The stepping thread and each worker have an `AStarPlanner` (and its scratch
  memory) of their own; the only shared writes are the answer slots, one per question.
- **One path by which state changes.** A worker never touches a `Simulation`. The stepping thread applies the
  answers itself, in the order the questions were asked, so the state after any number of steps does not
  depend on how the work was shared out. The expansions a batch made are added to the planner's own
  count (it is part of the state checksum), so even that is the same for every thread count.
- **Bounded pool.** `core::ThreadPool` has a fixed number of threads and a bounded queue (`submit` waits while
  it is full). `shutdown()` and the destructor refuse new tasks, wake any producer that is waiting, run what
  was already accepted and join; calling `parallelFor` or `shutdown` from one of the pool's own tasks throws
  instead of deadlocking; if calls of a `parallelFor` throw, the exception of the lowest index is rethrown
  after the others have run. The runner joins its planning threads when it is destroyed or restarted.
- **Not part of the scenario.** The thread count is not recorded and changes no checksum: a recording made on
  one number of threads replays on any other.
- Batches smaller than 16 questions stay on the stepping thread (`minParallelQueries`): between 1 and 32
  made no measurable difference to a step, 64 and more gave back part of the gain.

What it checks: `PlanningThreadsTest` compares every tick of runs on 0, 1, 2 and 3 threads on three halls,
a hundred robots on `large`, obstacles placed and cleared in the middle of a run, a change of the thread
count in the middle of a run, and recordings made and replayed on different counts; the runner tests start
and stop runners with threads over and over; the suites are clean under ASan, UBSan and TSan. The pool's
tests were also run against deliberately broken pools (a producer never woken, workers that drop queued
tasks, two threads joining the same worker, an unlocked read), and every one is caught.

**What it gives.** `warehousesim_scaling_bench` on the final code (`large`, a task every tick, seed 1,
1 500 steps per sample, median of 3; "planning threads" are besides the stepping thread; the state checksum
is the same for every thread count and the benchmark fails if it is not):

| robots | planning threads | ms / step | speed-up | batches / step | questions / batch |
|---:|---:|---:|---:|---:|---:|
| 100 | 0 | 0.290 | 1.00x | - | - |
| 100 | 1 | 0.277 | 1.05x | 0.50 | 24.6 |
| 100 | 2 | 0.276 | 1.05x | 0.50 | 24.6 |
| 128 | 0 | 0.372 | 1.00x | - | - |
| 128 | 1 | 0.356 | 1.04x | 0.60 | 24.6 |
| 128 | 2 | 0.350 | 1.06x | 0.60 | 24.6 |
| 256 | 0 | 0.770 | 1.00x | - | - |
| 256 | 1 | 0.751 | 1.03x | 0.90 | 20.3 |
| 256 | 2 | 0.745 | 1.03x | 0.90 | 20.3 |

The gain on a whole step is small: 3 to 6 %, and an earlier complete run gave 1 to 6 %. The profile
says why: about 7 % of a step is spent on questions that can be answered independently, so even free
threads could not give more than that (Amdahl); the rest is the traffic controller and the cooperative
search, which plan robot after robot against each other's reservations. Dropping an obstacle on the busiest
cell of a fleet of 100 to 256 robots repairs 7 to 11 routes, below the batch size worth sending to the pool, so
there is nothing to gain there either (0.96x to 1.04x, which is noise):

| robots | planning threads | ms / obstacle | routes repaired | speed-up |
|---:|---:|---:|---:|---:|
| 100 | 0 / 1 / 2 | 0.083 / 0.084 / 0.087 | 7.5 | 1.00x / 0.99x / 0.96x |
| 128 | 0 / 1 / 2 | 0.101 / 0.098 / 0.099 | 9.3 | 1.00x / 1.04x / 1.02x |
| 256 | 0 / 1 / 2 | 0.102 / 0.101 / 0.103 | 10.8 | 1.00x / 1.00x / 0.98x |

Where the pool does pay is when there is a lot of independent work at once, which is what a batch of
routes is. Raw batches of random questions on the same hall (200 batches per case, same routes as with no
threads in every case):

| questions / batch | 0 threads us | 1 thread us | 2 threads us | speed-up with 1 / 2 |
|---:|---:|---:|---:|---:|
| 16 | 27.0 | 14.1 | 17.2 | 1.91x / 1.57x |
| 64 | 112.6 | 57.0 | 51.4 | 1.98x / 2.19x |
| 256 | 677.5 | 324.6 | 233.8 | 2.09x / 2.90x |
| 1 024 | 3 439.7 | 1 808.4 | 1 216.1 | 1.90x / 2.83x |

With two workers and the calling thread that is 2.8 to 2.9x on three threads for batches of 256 or more.
So the feature is real and safe, and worth turning on for a use that asks many routes at once; for a
simulation step as it is today it is a small, free gain, not a fix. Only up to two workers were used
(the project limit for build and test workers); how it scales further is not measured.

### Rendering

The main thread's work for a frame before a pixel is drawn (the runner's last two states, blended by how
far real time has got, turned into draw commands for a 1280x720 window with routes and held cells shown),
on the same busy fleets, median of 200 frames:

| robots | `frame()` us | blend us | scene us | draw commands | share of a 60 fps frame |
|---:|---:|---:|---:|---:|---:|
| 100 | 4.9 | 4.4 | 69.0 | 8 328 | 0.47 % |
| 128 | 6.1 | 6.4 | 72.2 | 9 714 | 0.51 % |
| 256 | 8.8 | 16.2 | 74.0 | 9 862 | 0.59 % |

The stepping thread pays 5 to 10 microseconds per step to publish its state under the lock. This is far
from a bottleneck, so the view code was left alone. Executing 8 000 to 10 000 draw commands is SDL's, and
cannot be measured without a window: that part is **not measured**. The runner now takes one snapshot
per step, not two: the state before the last step is the one it already holds.

### Limits

- Planning threads give 3 to 6 % on a whole step here; most of a step is ordered by priority and cannot be
  shared out without changing what the simulation computes.
- The next cheap gain is known and **not taken**: the nearest free charger could be found with one sweep
  from the robot instead of one A* for each charger (8 % of a step on `large`, which has 147). It would
  change the expansions counted as planning work, which are in the state checksum and the JSON, so it is a
  change of the engine (raise `kEngineRevision`, regenerate the fixtures) and was left for a change that
  means to do that.
- Memory: a space-time planner keeps distance fields for up to 512 goals and 32 MB; `PathDistance` keeps 32
  fields (about 33 MB on a 512x512 map). Before, the estimator kept no fields.
- The numbers are from one machine and one compiler, for two worker threads at most.

## Recording and replay

A run can be written down and played again, step for step, from its initial state, and the replay says
where it goes wrong if it ever does. This is what the determinism of the engine buys: a run is a function of
what it was built from and of what was done to it from outside, each thing after a whole number of steps.
Those two things are all a recording needs, plus what lets a replay be checked.

### What a recording holds

| Part | Contents |
|------|----------|
| `scenario` | the layout as it was built (before any wall was put up), where each robot starts, the **random seed**, and every setting: the whole `SimulationConfig` and the robot model (`RobotSpec`) |
| `commands` | every **user command**, in the order it was applied, with the tick it was applied at (after that many steps, before the next) and whether the simulation accepted it: `submitTask`, `setTaskPriority`, `cancelTask`, `setRobotService`, `placeObstacle`, `clearObstacle`. A refused command is kept too, so that the replay refuses it as well |
| `taskEvents` | every **task created** (by the random generator, `source: engine`, or by a command) and every **task cancelled** (by a command, including the cancellation that follows from putting a wall on a cell a task starts or ends on), with the tick, the task number and, for a new task, its pickup, dropoff and priority |
| `checkpoints`, `end` | `Simulation::stateChecksum()` at tick 0, every `checkpointInterval` ticks (default 100) and at the end of the run |
| `ticks`, `engine` | how many steps were taken, and the revision of the engine that made it (`kEngineRevision`) |

Time in a recording is simulated time, the tick; the steps are fixed (one tick each). There is no clock,
user name, path or machine in the file. The order of events is stable by construction: commands are listed
in the order they were applied; task events by tick, at one tick those of commands before those of the
step, creations before cancellations, tasks in order of number. A file whose order is any other is refused.
A 600-tick run of a bundled layout takes about 15 KB.

### Using it

```sh
warehousesim_cli --layout compact --ticks 600 --seed 7 --record run.json [--checkpoint-interval 50]
warehousesim_cli --replay run.json                  # prints the same final state as the run did
warehousesim_cli --replay run.json --ticks 250      # only the first 250 ticks (an earlier state)
warehousesim_cli --replay run.json --no-verify      # just play it, compare nothing
warehousesim_gui --record session.json              # written when the window closes
```

A replay takes the layout, seed and settings from the file (so `--layout`, `--map`, `--seed` and `--record`
are refused with it), plays it and compares it with the recording as it goes. Exit status 0 means the
replay matched (it says how many checkpoints, and the end state); 1 means it did not, or the file is not a
usable recording, with the reason on stderr and no final state on stdout. `--metrics` and `--metrics-csv`
work on a replay too.

In code: `simulation::Recorder` runs a simulation and records it (all changes go through it, so it cannot
miss one), `SimulationRunner::startRecording()` / `recording()` does the same for the run in a window
(commands are recorded with the tick the worker applied them at; a restart starts the recording over on the
new run, as it does the command log), and `simulation::replay(recording, options)` plays one back. Files
are read and written with `parseRecording` / `serializeRecording` / `loadRecordingFile` /
`saveRecordingFile`.

### Finding where a replay goes wrong

`replay()` returns the state and the performance figures of the replay, and, if anything differed, a
`Divergence`: the tick and what differs, in words. Checked as it goes, so the first difference is the one
reported, and the replay stops there:

- the **state checksum** at every checkpoint (tick 0 first, which also catches a recording from an
  engine that builds its scenario differently) and at the end;
- whether each **command** was accepted or refused as recorded (a recorded "accepted" that the replay
  refuses reports the reason the engine gave);
- every **task event**, one by one ("the recording has task #7 created at tick 40 ..., but the replay made
  task #7 created at tick 40 ..."), and that none the recording has is missing or extra.

So a change to any input, setting, command or the engine itself is found at the first tick where it shows,
not just at the end, and usually at the event before the first checksum that would show it. A divergence
from a recording of another `engine` revision says so. A recording that cannot be replayed at all
(not consistent, a scenario that cannot be built, an engine error) comes back as a divergence at tick 0
or wherever it happened; `replay()` throws for no recording.

The checksum covers everything that decides the next step or that a snapshot shows: tick, layout, every
robot (pose, speed, battery, route, motion, counters), every task ever made, the traffic, planning and
recovery bookkeeping and the state of the random stream. It is FNV-1a over a canonical byte stream (fixed
width, little-endian, doubles as IEEE bit patterns), so it is the same on every platform when the numbers
are. It is not cryptographic: it finds accidental divergence, not tampering, and a recording is trusted
input as far as being what it says it is (what is read from a file is validated and bounded).

### The file

A versioned JSON document (version 1, `format: "warehousesim-recording"`) with a strict schema: unknown and
duplicate keys, wrong types, out-of-range numbers, inconsistent order and a scenario that cannot be built
are refused with a JSON Pointer to the problem (`/commands/3/tick: tick 400 is after the end of the run`),
never an exception or a crash. A reader that meets a newer version says so. The writer is canonical (equal
recordings give equal text; reading then writing changes nothing) and saves through a temporary file and a
rename. The keys are documented at the top of `recording_file.hpp`, and `RecordingFormatTest` pins the exact
text of a small recording and of a recording made earlier that must keep replaying.

Everything but `scenario` and `ticks` may be left out of a file written manually. Commands with no
`taskEvents`, `checkpoints` or `end` form a *scenario script*: it is played as it says and only the commands'
`accepted` is compared.

### Sources of nondeterminism and how each is handled

| Source | Where it could enter | How it is handled |
|--------|---------------------|-------------------|
| Random numbers | the task generator | only `core::Rng` (SplitMix64: its output is fully specified, unlike `std::*_distribution`), seeded by `config.seed`, recorded; no `std::rand` or `std::random_device`. The stream's state is in the checksum, so a different seed is seen at tick 0 |
| Wall-clock time, thread schedule | the runner steps in real time; clicks arrive whenever | the simulation never reads a clock. Steps are whole and fixed. Commands are applied only *between* steps, one at a time, in the order posted, and recorded with the tick they landed on: the timing decides *which* tick, and the recording keeps it. Pause, speed, frame rate, zoom and selection are not commands and never reach the state (`PausingAndChangingTheSpeed...`, `FleetDeterminismTest`) |
| Floating point | the motion model uses `double` | only exactly specified IEEE-754 operations (`+ - * /`, `sqrt`, `fmod`, `floor`, `ceil`, `fabs`); no `-ffast-math`, and `-ffp-contract=off` so that no fused multiply-add changes a result; no libm transcendental in the simulation (`sin`, `cos`, `pow` are in the view code only). The checksum takes the bits |
| Container order | hash containers | no decision depends on the iteration order of an `unordered_*`: the few left are only looked up (the metrics accumulator's cell index, the reservation table's per-robot lists by robot id). The planner's blocked set is a `core::PointSet`, which walks its points in the order they were inserted, the same on every platform; the traffic controller's route counts and the distance estimator's answers are arrays and an open-addressing table that are only looked up. Everything else is a `vector`, a `deque` or an ordered container, and ties are broken by id, never by position in memory |
| Memory addresses, uninitialised memory | ordering or hashing by pointer; an unset member | nothing is ordered or hashed by address; members are initialised; the suites run clean under ASan, UBSan and TSan |
| Integer width, byte order, locale | the checksum, the file | fixed-width types and a little-endian byte stream; numbers in the file are written locale-independently, doubles with the shortest text that reads back to the same bits (`EveryFieldOfTheConfigurationSurvivesTheRoundTrip`) |
| Parallelism | planning threads (`--threads`); several replays at once | the threads only answer questions that are a function of the layout and two cells; the stepping thread applies the answers in the order asked, so no thread count changes a state, a checksum or a recording (`PlanningThreadsTest`, run under TSan; see Performance and planning threads). A replay shares no state with another (`ReplaysOnOtherThreads...`) |
| Things done to the simulation that the recording does not know | a wall put up, a robot disabled, a task made, before recording began; a planner or dispatcher of one's own | `RunJournal` compares the simulation with what its origin builds and refuses to start on any difference; it refuses a simulation built with its own planner or dispatcher, because a replay builds the default ones |
| The engine itself changing | a new release | the `engine` revision, a pinned checksum (`ThePinnedScenarioHasThePinnedChecksums`) and a recording made earlier that must keep replaying (`ARecordingMadeEarlierStillReplays...`) force a deliberate change to be made on purpose; a recording of another revision that diverges says so |

### Limits

- A recording starts at tick 0, before the first task, of a simulation with the default planner and
  dispatcher. A restart in the window starts a new recording; the part before it is not kept.
- The CLI has no way to give commands, so a recording from it has none; commands come from the window
  (`--record` on the GUI), from code (`Recorder`), or from a file edited manually (a scenario script).
- There is no way to *watch* a replay in the window: `--replay` is headless and prints the final state.
- Replaying takes as long as running: a file may ask for up to 10^9 ticks and 4096 robots, and nothing
  limits the time it takes. Files are limited to 64 MiB and layouts to 512 x 512.
- Bit-for-bit agreement between machines rests on the arithmetic being IEEE as built. It has been run only
  on Apple clang 21 (arm64), where the plain and the ASan+UBSan builds give the same checksums; the
  recording kept as a fixture in the tests is how another platform's CI finds out.

## Map editor and map files

`warehousesim_editor` is an interactive editor for warehouse layouts, built with the GUI (SDL3).

```sh
warehousesim_editor --file my-hall.json                 # opens the file if it exists, else starts a new 20 x 12 map
warehousesim_editor --file my-hall.json --width 40 --height 25     # a new blank map of that size
warehousesim_editor --file my-hall.json --layout fulfilment        # a new map that starts as a predefined layout
```

An existing file is opened as it is (`--layout`, `--width` and `--height` only apply to a new map and are
refused otherwise). A file that exists but is not a valid map is an error that lists its problems; the
editor never starts on a blank map in its place, because a later save would overwrite it.

| Input | Action |
|-------|--------|
| left button (drag) | paint with the selected tool; a whole drag is one undo step and fast drags leave no gaps |
| right button (drag) | erase |
| `1` `2` `3` `4` `5` `6` | tool: wall, shelf, pickup, dropoff, charging station, erase |
| Shift + arrow keys | make the map one cell wider / narrower / taller / shorter (hold for key repeat) |
| Ctrl+Z, Ctrl+Y (Ctrl+Shift+Z) | undo / redo (the last 100 steps; on a Mac use Cmd) |
| Ctrl+S, Ctrl+O | save to / load from the `--file` path |
| `L` | load the next predefined layout (`demo`, `compact`, `fulfilment`, then around) |
| Delete or Backspace | clear the map (undoable) |
| middle button drag, WASD or arrows, wheel or `+`/`-`, `0` | pan, zoom, fit the whole map |
| Esc, closing the window | quit |

The bottom bar shows the file (with `*` when there are unsaved changes), the size, zoom, tool and the cell
under the pointer, the palette, the last message, and whether the map is ready for a simulation. Loading
a file or a predefined layout over unsaved changes, and quitting with them, need the same key twice in a
row; the first press only asks. Anything else in between takes the question back. Shrinking the map crops
it and says how many objects were cut off (undo brings them back). The editor does not run the
simulation: save the map and run it with `warehousesim_gui --map FILE`.

### The file format

A map is one JSON object, version 1 (`mapio/map_file.hpp` is the authority; this is the same text):

```json
{
  "format": "warehousesim-map",
  "version": 1,
  "name": "my hall",
  "width": 12,
  "height": 8,
  "walls": [
    [0, 0], [1, 0], [2, 0]
  ],
  "shelves": [[3, 2], [4, 2]],
  "pickups": [[1, 1]],
  "dropoffs": [[10, 6]],
  "chargers": [[1, 6], [2, 6]]
}
```

| Key | Required | Rule |
|-----|----------|------|
| `format` | yes | exactly `"warehousesim-map"` |
| `version` | yes | the integer `1`; any other number is refused as unsupported (and nothing else about such a document is judged, since another version may have another shape) |
| `name` | no | text, at most 80 bytes, no control characters |
| `width`, `height` | yes | integers from 1 to 512 |
| `walls`, `shelves`, `pickups`, `dropoffs`, `chargers` | no | lists of `[x, y]` pairs of integers; `x` is the column and `y` the row, both from 0 at the top left |

A cell may appear once, in one list only; a cell in no list is open floor. Reading is strict and
reports every problem it can find (up to 20, then a count) with a JSON Pointer to the place:

```
/width: must be from 1 to 512, not 0
/walls/3: cell (25, 2) is outside the 20 x 12 grid
/shelves/0: cell (4, 4) is already listed as a wall at /walls/1
/robots: unknown key; the keys of a map are: format, version, name, width, height, walls, ...
```

Refused: text that is not JSON (or has trailing text, or invalid UTF-8), a document that is not an object,
unknown or duplicate keys, wrong types (including `2.0` or `"20"` for an integer), dimensions outside
1..512, cells outside the grid, cells listed twice, malformed pairs, over-long or control-character
names, documents nested more than 4 levels deep (refused before anything is built from them) and
documents over 4 MB. Whether a map can *host a simulation* is a separate question, answered by
`world::checkLayout()`: a map with no dropoff or nothing to collect is valid and can be saved, but the
editor says it is not ready and `--map` refuses to run it.

Writing is canonical: cells are listed row by row, the lists wrap at 100 columns, and equal documents give
identical text, so `parse(serialize(d)) == d` and saving a loaded map changes nothing. A save writes
`FILE.tmp` completely and then renames it over `FILE`, so a failure never leaves a half-written map
under the real name. A change that old readers cannot understand gets a new `version`.

A map holds the layout only. Robot start positions are derived when it is run: on its charging stations
(up to four) if it has any, otherwise on open floor, spaced out. Robots then have a battery model only
if the map has chargers.

### How it is built

`world` knows nothing about editing or files (`world::checkLayout` and `kMaxMapSide` are the only
additions, and are pure functions of a layout). `mapio` is the only place that knows JSON for maps
(nlohmann_json is private to it). `editor::MapEditor` edits a `world::Warehouse` through its public
interface and has no idea of windows, mouse or files; `editor_files` connects it to `mapio` and the
predefined layouts. `view::EditorController` turns SDL-free `InputEvents` (pointer presses, moves and
releases in the order they happened, and key commands) into edits, and `buildEditorScene()` draws it as
the same plain `DrawList` the simulation view uses. `rendering::SdlWindow::pollEvents(PollMode::Editor)`
only translates SDL events into those types, so everything the editor does is tested without a display.

Limits: the editor has no text entry, so the file path comes from `--file` and the name from the file
(or the file's name); the map changes size one cell at a time; there is no rectangle or fill tool (only
freehand drags); undo keeps 100 steps; a layout whose walkable cells form separate regions is
accepted with a warning, and tasks between the regions can never be done.

## Behaviour notes

- Scenarios have up to four robots (the demo layout starts them on row 0; other layouts on charging
  stations) and spawn a random task every 5 ticks, with a random priority: from a `Pickup` cell if the
  layout has any, otherwise from an aisle cell beside a shelf, to a random `Dropoff`.
- The bundled robots drive like vehicles: top speed 1 cell per tick, acceleration 0.5, deceleration 1 and a
  quarter turn per tick, stopping at every corner. That costs throughput against the idealised robot: in
  3000 ticks they complete 213 to 225 tasks on `demo`, 105 to 110 on `compact` and 149 to 164 on `fulfilment`
  (seeds 1 to 10, with cooperative planning), against 315 to 343, 159 to 160 and 227 to 239 with idealised
  robots (seeds 1 to 3, measured before collision avoidance).
- Robots act in id order each tick (see Robots above). A robot blocked for 3 ticks makes an idle or lower-priority (higher id)
  blocker step aside, or routes around a higher-priority one, or, face to face with a blocker that cannot
  move, steps aside itself. Dropoffs are single cells at the edge of the demo layout, so heavy demand queues
  up there; no seed of the demo hall comes to a standstill (`DemoScenarioNeverDeadlocks` for seeds 1 to 6,
  `NoSeedOfTheDemoHallEverComesToAStandstill` for 1 to 30; before the face-to-face rule and the immediate
  claim of a cell a blocker has just left, seeds such as 9 with the reservations off and 28 with them on
  locked up for good).
- Same seed ⇒ same run, bit for bit (`SameSeedIsDeterministic`, and the CLI output hashes match).
- Layouts in 1-wide corridors can still deadlock if robots need each other's cells and none has a free
  neighbour to step to.
- Robots that have finished a task stay where they are, and a fleet bigger than the work can use does worse
  than a smaller one: robots never collide, but a crowd of idle ones near the dropoffs can block the robots
  still working until a task is given up (after 150 stuck ticks). Measured on `fulfilment`, 16 robots, a
  task every 25 ticks, 1500 ticks, seed 1: 54 of 60 tasks done, 4 still open, 2 given up
  (`EndToEndTest.ALightLoadIsMostlyServedAndEveryFailureIsAnExplainedAbandonment`). Across the audited
  campaign it is much worse on the small halls: 16 robots at a task every 25 ticks did less than 4 in 9 of
  10 layouts, and 64 robots on `demo` completed 85 tasks against 339 for 16 at heavy load
  ([docs/benchmarks.md](docs/benchmarks.md#scenario-campaign)). Idle robots by the dropoff row is the
  likely cause (seen in one run, not isolated by an experiment). A parking behaviour is the first thing to
  try; it would change what the simulation computes (a new engine revision), so it is not part of this
  release.

Project rules for contributors are in [AGENTS.md](AGENTS.md).

## License

WarehouseSim is released under the [MIT License](LICENSE).
