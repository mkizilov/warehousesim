# WarehouseSim

A deterministic C++20 simulation of autonomous robots moving goods through a grid warehouse, with a
headless library and CLI plus a separate SDL3 visualisation.

```
core ← world ← planning ┐
core ← robots ──────────┴← tasks ← simulation ← view (no SDL) ← rendering (SDL3) ← apps/gui
                                       ↑
                                  apps/cli (headless)
```

| Module       | Contents                                                                  |
|--------------|---------------------------------------------------------------------------|
| `core`       | `Point`, strong ids, `SimClock`/`Tick`, seedable `Rng` (SplitMix64)       |
| `world`      | grid model, occupancy, predefined layouts (see below); no SDL or JSON     |
| `robots`     | `Robot` entity: pose, speed, battery, task, path and an explicit state machine |
| `planning`   | `PathPlanner` interface, deterministic 4-connected A* (see Path planning) |
| `tasks`      | `TaskQueue` lifecycle and exclusive claims, greedy `Dispatcher`, distance estimators, seeded `TaskGenerator` |
| `testing`    | header-only test/benchmark support: reference BFS, map generators, allocation counter |
| `simulation` | `Simulation` tick engine, `SimulationRunner` thread, JSON, demo scenario  |
| `view`       | `Camera`, `buildScene()` draw lists, `ViewController`; no SDL, fully tested |
| `rendering`  | SDL3 `SdlWindow` (RAII): turns SDL events into `InputEvents`, runs a draw list |

## Build

Requires CMake ≥ 3.24, a C++20 compiler and Ninja (or another generator). nlohmann_json, GoogleTest and
SDL3 are located with `find_package` and otherwise downloaded at pinned versions by `FetchContent`.

```sh
cmake --preset default && cmake --build --preset default && ctest --preset default
```

Presets: `default` (Debug, GUI, tests), `release`, `headless` (no SDL3), `asan`, `tsan`, `tidy`.
Building SDL3 from source on Linux needs the X11 development packages (see `.github/workflows/ci.yml`).

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
  hall: pickup column, dropoff row, chargers, six shelf rows).

## Run

```sh
build/default/apps/warehousesim_cli --ticks 200 --seed 1     # headless; prints the final snapshot as JSON
build/default/apps/warehousesim_cli --layout compact         # demo, compact or fulfilment
build/default/apps/warehousesim_gui --layout fulfilment --speed 2 --fps 60
SDL_VIDEODRIVER=dummy build/default/apps/warehousesim_gui --frames 30   # render without a display
```

`warehousesim_gui` options: `--seed N`, `--layout NAME`, `--tick-ms N` (fixed simulation step, default
100 ms of real time at speed 1), `--speed X` (initial playback speed, 1/16 to 64), `--fps N` (frame cap,
default 60), `--frames N` (exit after N frames; 0 = until closed).

| Input | Action |
|-------|--------|
| Space | pause / resume |
| `[` / `]` | halve / double playback speed (1/16x to 64x) |
| mouse wheel, `+` / `-` | zoom about the pointer / window centre |
| drag with any mouse button, arrow keys, WASD | pan |
| `0` | fit the whole warehouse |
| Esc, closing the window, Ctrl-C | quit (the worker thread is joined and SDL shut down) |

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
| position and orientation (`Direction` it faces) | `position()`, `heading()` |
| maximum speed, current speed and velocity (cells per tick; a robot enters at most one cell per tick, so `maxSpeed` is in (0, 1] and 0.5 means one cell every second tick) | `maxSpeed()`, `speed()`, `velocity()` |
| battery (integer units, saturating, so exact on every platform) | `battery()`, `needsCharge()` |
| assigned task, destination, planned path | `task()`, `destination()`, `path()` |
| operational state | `state()`, `goal()` |

`RobotSpec` holds the per-model constants: speed, battery capacity and start charge, the energy used per
idle tick, per cell entered and per loading/unloading tick, the charge rate, a low-battery reserve, and
the number of ticks spent loading and unloading. **The default spec is an idealised robot** (one cell per
tick, instant handling, no energy use); with it the simulation reproduces the pre-state-machine behaviour
tick for tick (checked against the previous commit on all three layouts and three seeds).

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
is held exactly while heading to a pickup/dropoff or handling cargo, a path exists only while navigating,
speed and dwell are zero when they do not apply, the battery is within bounds).

`Simulation` is the only place that drives robots. Each tick a robot, in id order: finishes whatever its
state allows (arrive, finish loading/unloading/charging, start a charging trip if idle and low), is taken
out of service if its battery is flat, lets one tick of time pass in its state (drain, dwell countdown,
charging) and advances along its path. Rules worth knowing:

- Only an idle robot with enough charge (`Robot::isAvailable()`) is assigned a task. An idle robot below
  its reserve goes to the nearest free charging station, or charges where it stands. If the layout has no
  charging station the reserve is ignored, since a robot could never recharge.
- A robot whose battery is empty becomes `Unavailable`: a task it had not collected goes back to `Pending`,
  one it was carrying fails. `Simulation::disableRobot` / `restoreRobot` do the same by hand (restoring
  swaps in a fresh battery if the old one is flat).
- Loading, unloading, charging and unavailable robots are never pushed aside by other robots; idle and
  lower-priority navigating robots still are. A robot waiting on an occupied charger picks another one.
- `RobotSpec` on `SimulationConfig::robot` applies to every robot. The bundled scenarios use two ticks of
  loading and unloading everywhere, and an energy model (and recharging) on the layouts that have chargers.

Robots are shown by the visualisation but contain no drawing code: the view reads
`simulation::RobotSnapshot` (state, goal, heading, speed, battery, task).

## Visualisation

The window shows the grid with walls, shelves, pickup `P`, dropoff `D` and charging `C` cells (letters
appear when zoomed in), task markers, robot placeholders (a coloured square per robot by state: blue idle, orange
heading to a pickup, red heading to a dropoff, lime heading to a charger, amber loading, magenta unloading,
green charging, grey unavailable; a nub on the edge it faces, and the id and a battery gauge when zoomed in), column/row indices along the top and left edges,
and a status bar with the tick, playback speed, zoom, task counts and the cell under the pointer.

- **Fixed timestep.** The simulation runs on a worker thread; `core::FixedTimestep` turns elapsed real time
  into whole simulation steps of constant size, so frame rate, window resizing and a slow renderer never
  change what is simulated. Playback speed only scales how many steps happen per real second, and time
  spent paused is not replayed afterwards. A stall longer than 200 steps' worth is dropped, not replayed.
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

## Behaviour notes

- Scenarios have up to four robots (the demo layout starts them on row 0; other layouts on charging
  stations) and spawn a random task every 5 ticks, with a random priority: from a `Pickup` cell if the
  layout has any, otherwise from an aisle cell beside a shelf, to a random `Dropoff`.
- Robots act in id order each tick (see Robots above). A robot blocked for 3 ticks makes an idle or lower-priority (higher id)
  blocker step aside, or routes around a higher-priority one. Dropoffs are single cells reachable from
  one side in the demo layout, so heavy demand queues up; nothing deadlocks (see `DemoScenarioNeverDeadlocks`).
- Same seed ⇒ same run, bit for bit (`SameSeedIsDeterministic`, and the CLI output hashes match).
- Layouts in 1-wide corridors can still deadlock if two robots need each other's cell with no free neighbour.

Project rules for contributors and agents are in [AGENTS.md](AGENTS.md).
