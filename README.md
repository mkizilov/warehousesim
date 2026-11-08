# WarehouseSim

A deterministic C++20 simulation of autonomous robots moving goods through a grid warehouse, with a
headless library and CLI plus a separate SDL3 visualisation.

```
core ← world ← planning ┐
core ← robots ──────────┴← tasks ┐
        world, robots ← traffic ─┴← simulation ← view (no SDL) ← rendering (SDL3) ← apps/gui
                                       ↑
                                  apps/cli (headless)
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
neighbour between them can still deadlock.
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

## Behaviour notes

- Scenarios have up to four robots (the demo layout starts them on row 0; other layouts on charging
  stations) and spawn a random task every 5 ticks, with a random priority: from a `Pickup` cell if the
  layout has any, otherwise from an aisle cell beside a shelf, to a random `Dropoff`.
- The bundled robots drive like vehicles: top speed 1 cell per tick, acceleration 0.5, deceleration 1 and a
  quarter turn per tick, stopping at every corner. That costs throughput against the idealised robot: in
  3000 ticks they complete 188 to 197 tasks on `demo`, 91 to 100 on `compact` and 148 to 157 on `fulfilment`
  (seeds 1 to 3), against 315 to 343, 159 to 160 and 227 to 239 with idealised robots (measured before
  collision avoidance).
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
