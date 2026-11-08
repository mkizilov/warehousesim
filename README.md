# WarehouseSim

A deterministic C++20 simulation of autonomous robots moving goods through a grid warehouse, with a
headless library and CLI plus a separate SDL3 visualisation.

```
core ← world ← planning ┐
core ← robots ──────────┴← tasks ┐
        world, robots ← traffic ─┴← simulation ┐
                                       ↑       ├← view (no SDL) ← rendering (SDL3) ← apps/gui, apps/editor
                                  apps/cli     │
                                (headless)     │
        world ← mapio (map files) ← editor ────┘
```

| Module       | Contents                                                                  |
|--------------|---------------------------------------------------------------------------|
| `core`       | `Point`, strong ids, `SimClock`/`Tick`, seedable `Rng` (SplitMix64)       |
| `world`      | grid model, occupancy, predefined layouts (see below); no SDL or JSON     |
| `robots`     | `Robot` entity: pose, speed, battery, task, path and an explicit state machine; `forecastHolds()` |
| `traffic`    | space-time `ReservationTable` and the `TrafficController` that fills it (see Collision avoidance) |
| `planning`   | `PathPlanner` interface, deterministic 4-connected A* (see Path planning) |
| `tasks`      | `TaskQueue` lifecycle and exclusive claims, greedy `Dispatcher`, distance estimators, seeded `TaskGenerator` |
| `testing`    | header-only test/benchmark support: reference BFS, map generators, allocation counter |
| `simulation` | `Simulation` tick engine, `SimulationRunner` thread, JSON, scenarios      |
| `mapio`      | the versioned JSON map file: schema validation, reading, canonical writing (see Map editor) |
| `editor`     | `MapEditor` editing model (tools, strokes, undo, resize) and its file glue; no SDL, no GUI |
| `view`       | `Camera`, `buildScene()` and `buildEditorScene()` draw lists, `ViewController`, `EditorController`; no SDL, fully tested |
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
```

Both `warehousesim_cli` and `warehousesim_gui` take `--map FILE` instead of `--layout NAME` (not both).
A map that is missing, invalid or cannot host a simulation is refused with every problem listed and
exit status 1; an unknown layout or conflicting options exit with status 2.

`warehousesim_gui` options: `--seed N`, `--layout NAME` or `--map FILE`, `--tick-ms N` (fixed simulation step, default
100 ms of real time at speed 1), `--speed X` (initial playback speed, 1/16 to 64), `--fps N` (frame cap,
default 60), `--frames N` (exit after N frames; 0 = until closed).

| Input | Action |
|-------|--------|
| Space | pause / resume |
| `[` / `]` | halve / double playback speed (1/16x to 64x) |
| mouse wheel, `+` / `-` | zoom about the pointer / window centre |
| drag with any mouse button, arrow keys, WASD | pan |
| `0` | fit the whole warehouse |
| `P` / `O` | show / hide the planned routes (and a red cross on the cell a held-back robot may not enter) / the cells robots hold |
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
  one it was carrying fails. `Simulation::disableRobot` / `restoreRobot` do the same by hand (restoring
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
  aside has used its step, so no robot ever moves more than a cell per tick.
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
green charging, grey unavailable; a nub on the side it faces, a box on top while it carries cargo, and the id and a battery gauge when zoomed in), column/row indices along the top and left edges,
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

Project rules for contributors and agents are in [AGENTS.md](AGENTS.md).
