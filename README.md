# WarehouseSim

A deterministic C++20 simulation of autonomous robots moving goods through a grid warehouse, with a
headless library and CLI plus a separate SDL3 visualisation.

```
core ← world ← planning ┐
core ← robots ← tasks   ├← simulation ← view (no SDL) ← rendering (SDL3) ← apps/gui
                        ┘      ↑
                          apps/cli (headless)
```

| Module       | Contents                                                                  |
|--------------|---------------------------------------------------------------------------|
| `core`       | `Point`, strong ids, `SimClock`/`Tick`, seedable `Rng` (SplitMix64)       |
| `world`      | grid model, occupancy, predefined layouts (see below); no SDL or JSON     |
| `robots`     | `Robot` entity: pose, speed, battery, task, path and an explicit state machine |
| `planning`   | `PathPlanner` interface, deterministic 4-connected A*                     |
| `tasks`      | `TaskBoard` lifecycle, `NearestRobotAssigner`, seeded `TaskGenerator`     |
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
  stations) and spawn a random task every 5 ticks: from a `Pickup` cell if the layout has any, otherwise
  from an aisle cell beside a shelf, to a random `Dropoff`.
- Robots act in id order each tick (see Robots above). A robot blocked for 3 ticks makes an idle or lower-priority (higher id)
  blocker step aside, or routes around a higher-priority one. Dropoffs are single cells reachable from
  one side in the demo layout, so heavy demand queues up; nothing deadlocks (see `DemoScenarioNeverDeadlocks`).
- Same seed ⇒ same run, bit for bit (`SameSeedIsDeterministic`, and the CLI output hashes match).
- Layouts in 1-wide corridors can still deadlock if two robots need each other's cell with no free neighbour.

Project rules for contributors and agents are in [AGENTS.md](AGENTS.md).
