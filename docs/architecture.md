# Architecture

WarehouseSim simulates a fleet of robots moving goods through a grid warehouse. It is a C++20 library
with a headless command line, a graphical fleet interface and a map editor on top. This document says
how it is put together, which algorithms it uses, what they cannot do, and where to extend it. The
README has the detail of each part; [the audit](audit.md) says what was checked about it.

## Modules

Each module is a static library `warehousesim::<module>` in `src/<module>/`, with public headers in
`src/<module>/include/warehousesim/<module>/`. Dependencies only point inward, and the build enforces it
(a module can only include headers of modules it links).

```
core ← world ← planning ┐
core ← robots ──────────┴← tasks ┐
        world, robots, planning ← traffic ─┴← simulation ← view (no SDL) ← rendering (SDL3) ← apps
        core ← metrics ────────────────────────┘     ↑
        world ← mapio (map files) ← editor ──────────┘   apps/cli: simulation only, no display
```

| Module | What it is |
|---|---|
| `core` | points, strong ids, the tick clock, the seeded `Rng`, `PointSet`, the fixed-timestep stepper, `ThreadPool` |
| `world` | the grid: cell types, `Warehouse` (static layout), `Occupancy` (who holds which cell), `World` (the only way to change either), predefined layouts, `checkLayout` |
| `robots` | one `Robot`: pose, motion model, battery, task, path and an explicit state machine; `forecastHolds` |
| `planning` | `PathPlanner`, A\*, space-time A\*, `PathBatchExecutor` (independent questions on a thread pool) |
| `tasks` | `TaskQueue` (lifecycle and exclusive claims), the greedy `Dispatcher`, distance estimators, the task generator |
| `traffic` | `ReservationTable`, `TrafficController`, `CooperativeRouter`, `ProgressMonitor` |
| `metrics` | simulated performance figures from plain observations; CSV and dashboard text |
| `simulation` | `Simulation` (the engine), `SimulationRunner` (a thread and a command queue), scenarios, JSON, recording and replay, the invariant auditor, benchmark sweeps |
| `mapio`, `editor` | the versioned JSON map file; the editing model behind the map editor |
| `view`, `rendering` | camera, scenes (draw lists) and input logic with no SDL; the thin SDL3 layer |

Three rules do most of the work of keeping it understandable:

1. **The engine knows nothing of windows.** Everything on screen is decided in `view` and tested without
   a display; `rendering` only turns SDL events into plain input and runs a draw list.
2. **Changes go through one door.** Tasks change only through `TaskQueue`, cells only through `World`, a
   robot only through its named operations, and the running simulation only through `Command`s applied
   between two steps. Nothing mutates state "on the side".
3. **Same inputs, same run.** There is no wall-clock time, no `std::rand`, no iteration over hash order
   in anything that decides a result. See [Determinism](#determinism).

## One step

`Simulation::step()` advances the world by one tick (one tick is the unit of simulated time):

1. `spawnTask()`: the task generator may add a random task (seeded, so reproducible).
2. `assignTasks()`: the dispatcher proposes who takes which waiting task; the proposal is applied only
   through `TaskQueue::tryAssign`, which succeeds for exactly one caller. The route to each pickup is
   worked out for all assignments at once (this is where planning threads help).
3. `updateRobots()`: the reservation table is built from where every robot is. Robots then take their
   turns, those with a temporary priority boost first, the rest in id order. Each robot finishes
   whatever is due (loading, unloading, charging), lets a tick pass, reserves its route, searches for a
   way round a clash if the table holds it back, and moves along its path into cells it has claimed. A
   cell a robot has just left is available to the robots after it in the same tick.
4. `supervise()`: stuck robots are counted; rings of robots waiting for each other are broken.
5. The clock advances.

## Algorithms

| Concern | What is used | Properties |
|---|---|---|
| Shortest routes | A\* on the 4-connected grid, Manhattan heuristic | optimal, deterministic tie-breaking `(f, h, cell index)`, reused workspace (no allocation when warm), reports why it failed |
| Task order | `TaskQueue` | priority (urgent first), then age, then id; total order |
| Who does a task | greedy `Dispatcher`: tasks in queue order, each to the available robot with the shortest path to the pickup, ties to the lowest id | not a globally optimal assignment; rejects tasks that can never be done |
| Collision safety | `World` holds: a robot drives only into cells it has claimed, one holder per cell | the authority; independent of everything below |
| Collision avoidance | `ReservationTable` (space-time: cell × step, and edge swaps), filled by `TrafficController` in priority order over a 12-step horizon | reports a vertex clash, a swap and a blocked cell, earliest first, lowest id first |
| Getting round others | `CooperativeRouter`: space-time A\* with waiting, against the reservations of higher-priority robots | decoupled prioritized planning: fast, not complete, not optimal |
| Blocking and deadlock | `ProgressMonitor` counts stuck robots and finds rings; `Simulation` moves a robot aside, then boosts one, then gives up the task | recovers the common cases; see limits |
| Motion | closed-form kinematics: acceleration, braking to stop exactly, a turn rate; energy per cell entered | independent of the step length; a deceleration must stop the robot within one cell |
| Battery | integer units, low-battery trips to the nearest free charger | no floating-point accumulation in outcomes |
| Metrics | per-tick activity of every robot (exactly one each), counts and rates | zero denominators are "undefined", never 0 or NaN |
| Recording | layout + seed + settings + commands + task events + a state checksum at intervals | replay stops at the first difference and says where |

The values that tune them (horizons, work limits, detour limits, deadlock timers) live in
`SimulationConfig`; the README says what each is for and how it was chosen.

## Determinism

A run is a function of its `ScenarioSpec` (layout, robot starts, configuration including the seed) and
the `Command`s applied to it, each after a whole number of steps. How that is kept true:

| Source of variation | How it is handled |
|---|---|
| Random numbers | only `core::Rng` (SplitMix64, fully specified), seeded and part of the checksum |
| Time | the simulation never reads a clock; the runner and the GUI sleep, nothing else |
| Floating point | IEEE-exact operations only (`+ - * / sqrt fmod floor ceil`); `-ffp-contract=off`; no libm transcendental in `src/` outside `view`; the checksum hashes bit patterns |
| Container order | no decision depends on `unordered_*` iteration order; ties are broken by id |
| Threads | planning threads answer only pure questions; answers are applied on the stepping thread in the order asked, so the thread count changes no state, checksum or recording |

## Concurrency

Exactly two components start threads: `SimulationRunner` (one worker that steps the simulation under a
mutex and publishes value-copy snapshots) and `core::ThreadPool` (a bounded pool used by
`PathBatchExecutor`). Every thread is joined before its owner is destroyed. A worker never touches a
`Simulation`; the executor reads a private copy of the layout, has a planner of its own for each
thread, and writes only the answer slot of its question.

## Supported inputs

Layouts are predefined (`demo`, `compact`, `fulfilment`, `large`) or a versioned JSON map file made with
the editor (`examples/` has six). A map needs a dropoff and a source of goods (a pickup cell or a free
cell beside a shelf); `world::checkLayout` says what is missing. Maps are at most 512 × 512; a scenario starts
at most 512 robots, and no more than the layout has room for.

## Limitations

These are properties of the design, each pinned by a test or measured, not bugs waiting to be found.

- **Not complete, not optimal.** Prioritized planning (priority is the robot id, with a temporary boost)
  plans 12 steps ahead and caps detours. A solvable scenario can still make no progress, and a found
  route is not always the best one. The greedy dispatcher does not minimise total travel.
- **One-cell corridors with three or more robots can deadlock.** Safety holds (nobody overlaps or swaps),
  progress does not. The stuck robots are counted, the ring is broken when somebody has room, and a task
  stuck for 150 ticks is given up (back to the queue, or failed if the robot carries cargo). The
  `StaySafe` tests pin the cases that are safe and reported but not solved.
- **A fleet bigger than the work can use does worse than a smaller one.** Idle robots stay where they
  finished; nothing sends them to a parking area. Robots never collide, but they can block each other
  until a task is given up. In the audited campaign ([benchmarks](benchmarks.md#scenario-campaign)) 16
  robots at a task every 25 ticks completed fewer tasks than 4 in 9 of 10 layouts, and 64 robots on the
  small halls collapsed (`demo`: 85 tasks against 339 for 16 robots at heavy load). Idle robots standing
  by the dropoff row is the likely cause, seen in one run and not isolated by an experiment; parking is
  the first thing to try, and has not been tried. Size the fleet to the load, not to the hall.
- **The fleet interface judges clicks on the layout it was built with.** After `PlaceObstacle` or
  `ClearObstacle` its messages can be out of date; the engine still refuses what cannot be done.
- **The layout can change only through `placeObstacle` and `clearObstacle`**, which rebuild the
  reservation table and repair routes. There is no other layout-editing API on a running simulation.
- **A recording starts at tick 0** of a simulation with the default planner and dispatcher.
- **Replaying takes as long as running.**
- **Simulated figures are not real-world figures.** One tick is a stand-in for a second, and the energy
  model is a stand-in; ratios between runs mean something, absolute values do not.
- **Checksums are verified on Apple clang (arm64) only** until CI has run on other platforms; the pinned
  checksum test and the replay fixture are how another platform finds out whether it agrees.

## Extension points

| To add | Where | What to keep true |
|---|---|---|
| A layout | `world/layouts.cpp`, or a map file | passes `tests/layouts_test.cpp` and `world::checkLayout` |
| A cell type | `world/cell_type`, then the map file format and the scenes | new `version` if old readers cannot read it |
| A route planner | implement `planning::PathPlanner` and pass it to the `Simulation` constructor | deterministic; a simulation with its own planner cannot be recorded or use planning threads |
| A dispatching policy | implement `tasks::Dispatcher` (a pure decider returning a `DispatchPlan`) | never touches the queue or the robots; keep the reference greedy in `dispatcher_test.cpp` in step |
| Distance estimates | implement `tasks::DistanceEstimator` | depends only on the static layout |
| A space-time constraint source | implement `planning::SpaceTimeConstraints` | search work counted in expansions, never time |
| An operator action | a new `simulation::Command` alternative with an `apply` overload | validates and refuses, never throws; add the file encoding, a controller test, a runner test and a replay test |
| A metric | `metrics` (a new field of the observation and a definition in the README table) | a CSV column, a hand-checked test, and "simulated" in every output |
| A robot model | `RobotSpec` | defaults of the idealised robot do not change; a new field goes into the recording format |
| Something shown on screen | `view` scenes and controllers | decisions in `view`, not in `rendering` |
| An invariant | `simulation/audit.cpp` | a unit test that hand-builds a state breaking it |

[AGENTS.md](../AGENTS.md) has the full rules for contributors, including what to update together.
