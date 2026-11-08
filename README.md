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
| `robots`     | `Robot` state machine (idle → to pickup → to dropoff) and path following  |
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

## Visualisation

The window shows the grid with walls, shelves, pickup `P`, dropoff `D` and charging `C` cells (letters
appear when zoomed in), task markers, robot placeholders (a coloured square per robot: blue idle, orange
heading to a pickup, red carrying; the id when zoomed in), column/row indices along the top and left edges,
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

Text is drawn with SDL's built-in 8x8 debug font. Robots here are placeholders showing existing
simulation state; no new robot behaviour was added with the visualisation.

## Behaviour notes

- Scenarios have up to four robots (the demo layout starts them on row 0; other layouts on charging
  stations) and spawn a random task every 5 ticks: from a `Pickup` cell if the layout has any, otherwise
  from an aisle cell beside a shelf, to a random `Dropoff`.
- Robots act in id order each tick. A robot blocked for 3 ticks makes an idle or lower-priority (higher id)
  blocker step aside, or routes around a higher-priority one. Dropoffs are single cells reachable from
  one side in the demo layout, so heavy demand queues up; nothing deadlocks (see `DemoScenarioNeverDeadlocks`).
- Same seed ⇒ same run, bit for bit (`SameSeedIsDeterministic`, and the CLI output hashes match).
- Layouts in 1-wide corridors can still deadlock if two robots need each other's cell with no free neighbour.

Project rules for contributors and agents are in [AGENTS.md](AGENTS.md).
