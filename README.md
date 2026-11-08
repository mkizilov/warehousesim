# WarehouseSim

A deterministic C++20 simulation of autonomous robots moving goods through a grid warehouse, with a
headless library and CLI plus a separate SDL3 visualisation.

```
core ← world ← planning ┐
core ← robots ← tasks   ├← simulation ← rendering (SDL3) ← apps/gui
                        ┘                              apps/cli ← simulation
```

| Module       | Contents                                                                  |
|--------------|---------------------------------------------------------------------------|
| `core`       | `Point`, strong ids, `SimClock`/`Tick`, seedable `Rng` (SplitMix64)       |
| `world`      | `Warehouse` grid (floor, shelf, station, wall), ASCII layouts             |
| `robots`     | `Robot` state machine (idle → to pickup → to dropoff) and path following  |
| `planning`   | `PathPlanner` interface, deterministic 4-connected A*                     |
| `tasks`      | `TaskBoard` lifecycle, `NearestRobotAssigner`, seeded `TaskGenerator`     |
| `simulation` | `Simulation` tick engine, `SimulationRunner` thread, JSON, demo scenario  |
| `rendering`  | SDL3 `SdlWindow` (RAII), SDL-free `Viewport` math                         |

## Build

Requires CMake ≥ 3.24, a C++20 compiler and Ninja (or another generator). nlohmann_json, GoogleTest and
SDL3 are located with `find_package` and otherwise downloaded at pinned versions by `FetchContent`.

```sh
cmake --preset default && cmake --build --preset default && ctest --preset default
```

Presets: `default` (Debug, GUI, tests), `release`, `headless` (no SDL3), `asan`, `tsan`, `tidy`.
Building SDL3 from source on Linux needs the X11 development packages (see `.github/workflows/ci.yml`).

## Run

```sh
build/default/apps/warehousesim_cli --ticks 200 --seed 1     # prints the final snapshot as JSON
build/default/apps/warehousesim_gui --seed 1 --tick-ms 100   # Space pauses, Escape quits
SDL_VIDEODRIVER=dummy build/default/apps/warehousesim_gui --frames 30   # render without a display
```

## Behaviour notes

- The demo scenario has four robots and spawns a random shelf → station task every 5 ticks.
- Robots act in id order each tick. A robot blocked for 3 ticks makes an idle or lower-priority (higher id)
  blocker step aside, or routes around a higher-priority one. Stations are single cells reachable from
  one side in the demo layout, so heavy demand queues up; nothing deadlocks (see `DemoScenarioNeverDeadlocks`).
- Same seed ⇒ same run, bit for bit (`SameSeedIsDeterministic`, and the CLI output hashes match).
- Layouts in 1-wide corridors can still deadlock if two robots need each other's cell with no free neighbour.

Project rules for contributors and agents are in [AGENTS.md](AGENTS.md).
