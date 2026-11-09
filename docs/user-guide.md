# User guide

WarehouseSim simulates a fleet of robots moving goods through a warehouse laid out on a grid. You give
it a layout, a fleet size and a rate of work; it tells you how the fleet copes. There are three
programs:

| Program | For |
|---|---|
| `warehousesim_cli` | headless runs, batch experiments, recording and replay; needs no display |
| `warehousesim_gui` | watching and steering a run: select robots, add tasks, change priorities, pause, speed up, restart |
| `warehousesim_editor` | drawing your own warehouse and saving it as a map file |

Everything the simulator reports is **simulated**: ticks of a model, cells of a grid and units of an
abstract energy model. It says nothing about how a real robot or a real warehouse would perform.

## Getting it

You need a C++20 compiler, CMake 3.24 or newer and Ninja. Dependencies (nlohmann_json, GoogleTest, SDL3)
are downloaded at pinned versions the first time you configure.

```sh
cmake --preset release            # optimised, with link-time optimisation and the GUI
cmake --build --preset release
ctest --preset release            # optional: the test suite
build/release/apps/warehousesim_cli --version
```

`cmake --preset headless` builds without SDL3 (no GUI or editor) if you only want the command line.
`cmake --build --preset package` makes `build/release/warehousesim-<version>-<system>-<cpu>.tar.gz`
(`.zip` on Windows) with the programs, the example maps and these documents; unpack it anywhere, the
programs need nothing else.

## First run

```sh
warehousesim_cli --layout fulfilment --ticks 1000 --seed 1
```

prints the final state as JSON (robots, tasks, counters). The same layout, seed and options always give
the same output, byte for byte. Try a bigger fleet and a heavier load:

```sh
warehousesim_cli --layout large --robots 100 --spawn 1 --ticks 1000 --metrics
```

`--robots N` is the fleet (1 to 512, and no more than the layout has room for; the CLI says how many
that is), `--spawn N` makes a
new random task every N ticks (0 for none), and `--metrics` prints a dashboard of throughput,
utilisation, waiting time, conflicts avoided and energy to stderr.

### Checking a run

`--audit` checks the simulation after every step against the rules it must never break: no two robots
in one cell or crossing through each other, nobody inside a wall or moving further than a cell a tick,
every task owned by exactly one robot, every task accounted for, no number gone wrong. It prints
`audited N states: no violations` and exits 1, listing what, if there is a violation.

```sh
warehousesim_cli --map examples/cross-dock.json --robots 12 --spawn 3 --ticks 1500 --audit
```

### Recording and replaying

```sh
warehousesim_cli --layout compact --ticks 600 --seed 7 --record run.json
warehousesim_cli --replay run.json                  # prints the same final state as the run did
```

A recording holds the layout, the seed, the settings, every command and a checksum of the state at
intervals. A replay plays it back from tick 0 and stops at the first difference, saying at which tick
and why. The graphical program records a session with `--record FILE` and writes it when you close the
window.

### Trying many scenarios at once

`scripts/campaign.py` runs every built-in layout and example map at several fleet sizes, loads and seeds,
with `--audit` on each, and prints tables of tasks completed:

```sh
python3 scripts/campaign.py build/release/apps/warehousesim_cli            # about 40 seconds
python3 scripts/campaign.py build/release/apps/warehousesim_cli --quick    # a small one
```

It exits with 1 if any run breaks an invariant. Throughput is for reading, not asserted.

### Using more than one thread

`--threads N` lets N extra threads answer the independent route questions of a step. It changes how
long a run takes and nothing else: the output, the checksums and a recording are identical for every N.
On a hundred-robot hall the gain on a whole step is small (a few percent); see
[benchmarks](benchmarks.md).

## Layouts and maps

Four layouts are built in: `demo` (open hall), `compact` (a walled room with every kind of cell),
`fulfilment` (pickup column, dropoff row, shelf rows, chargers) and `large` (100 × 64 for hundreds of
robots). `--layout NAME` selects one.

Your own warehouse is a map file; `--map FILE` runs it. The `examples/` directory has six, each with
the command that suits it:

| Map | What it shows | Try |
|---|---|---|
| `small-depot.json` | the smallest map worth running | `--robots 4 --spawn 5` |
| `fulfilment-aisles.json` | long shelf blocks, a pickup column, a dropoff row, chargers | `--robots 16 --spawn 3` |
| `cross-dock.json` | goods flow both ways, so robots meet head-on | `--robots 12 --spawn 3` |
| `choke-point.json` | every task crosses one two-cell gate | `--robots 10 --spawn 4` |
| `single-lane.json` | a one-cell corridor with a passing place: the hard case | `--robots 3 --spawn 8`, then `--robots 10` |
| `charging-hub.json` | more robots than chargers, so they queue for energy | `--robots 12 --spawn 4` |

A map needs a dropoff and somewhere goods come from (a pickup cell, or a free cell beside a shelf). A map
that is missing, invalid or unusable is refused with every problem listed (exit status 1).

### Drawing a map

```sh
warehousesim_editor --file my-hall.json --layout fulfilment      # start from a built-in layout
warehousesim_editor --file my-hall.json --width 40 --height 25   # or from a blank map
warehousesim_cli --map my-hall.json --robots 8 --ticks 2000 --audit
```

Left button paints with the selected tool, right button erases. `1`–`6` choose wall, shelf, pickup,
dropoff, charging station or erase; Shift + arrows resize; Ctrl/Cmd+Z and Y undo and redo; Ctrl/Cmd+S
saves; `L` loads the next built-in layout. The bottom bar tells you whether the map is ready for a
simulation. The README lists every key.

## The graphical fleet interface

```sh
warehousesim_gui --layout fulfilment --robots 16 --spawn 3
```

| Input | Action |
|---|---|
| click a robot or a list row; Tab | select a robot and inspect it |
| `N`, then click a pickup, then a dropoff | add a delivery task (`1`–`4` set its priority first) |
| `1`–`4` with a waiting task selected | change that task's priority |
| Space; `[` and `]` | pause and resume; halve and double the speed |
| `R` | restart the scenario |
| `P`, `O` | show or hide planned routes and held cells |
| `M`, `E` | show the metrics dashboard; export the figures to CSV (needs `--metrics-csv BASE`) |
| wheel, drag, arrows, `0` | zoom, pan, fit the whole map |
| Esc | cancel, then deselect, then quit |

Robots are filled squares coloured by state, obstacles are filled cells, destinations are diamonds,
planned paths are thin lines. Your actions are commands that the simulation applies between two steps,
so watching or steering never changes what a run computes, and a recorded session replays exactly.

## Exit codes

| Code | Meaning |
|---|---|
| 0 | success; with `--audit`, no violations; with `--replay`, the replay matched |
| 1 | a problem with the input or the run (bad map, unreadable file, failed audit, diverging replay) |
| 2 | bad command line (unknown option, conflicting options, unknown layout) |

## When something looks wrong

- **Robots stand still in a corridor.** One-cell corridors with three or more robots can deadlock. The
  run stays safe (the audit proves it), the stuck robots are counted, and after 150 ticks a stuck task
  is given up. Give the corridor a passing place or a second lane.
- **Tasks fail, or a bigger fleet does less.** Robots that have finished a task stay where they are, and a
  crowd of them near the dropoffs can block the robots still working, until a task is given up. In the
  campaign ([benchmarks](benchmarks.md#scenario-campaign)) 16 robots at a light load did less than 4 in
  nine of ten layouts. Pick the smallest fleet that carries your load: `warehousesim_fleet_bench` and
  `scripts/campaign.py` show where more robots stop helping, which depends on the load as well as the
  hall (at a task every tick 16 robots beat 4 in 8 of 10 layouts; at a task every 25 ticks they lost in
  9). `--metrics` shows the share of time robots spend waiting.
- **A replay diverges.** It names the tick. The usual cause is a different build or platform whose
  arithmetic differs; the checksum of a recording is exact.
- **The GUI does not start on a server.** It needs a display; `SDL_VIDEODRIVER=dummy` runs it without one
  (for tests).
