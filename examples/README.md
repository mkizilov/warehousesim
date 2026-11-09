# Example warehouses

Six maps in the versioned map format (see the README, "Map editor and map files"). Each is valid, in the
editor's own canonical form, and is run by the test suite: audited step by step, run again, run on
planning threads, and recorded and replayed. Open one in the editor to change it:

```sh
warehousesim_editor --file examples/cross-dock.json
warehousesim_cli --map examples/cross-dock.json --robots 12 --spawn 3 --ticks 1500 --audit
warehousesim_gui --map examples/cross-dock.json --robots 12 --spawn 3
```

Legend: `#` wall, `S` shelf, `P` pickup, `D` dropoff, `C` charging station, `.` floor.

| Map | Size | What it shows | Suggested |
|---|---|---|---|
| `small-depot` | 16 × 9 | the smallest map worth running: two pickups, two dropoffs, four shelf blocks, one charger | `--robots 4 --spawn 5` |
| `fulfilment-aisles` | 32 × 18 | a pickup column on the west wall, a dropoff row on the south, eight long shelf blocks with aisles between them, four chargers | `--robots 16 --spawn 3` |
| `cross-dock` | 40 × 14 | pickups and dropoffs at both ends, so goods and robots flow both ways and meet head-on | `--robots 12 --spawn 3` |
| `choke-point` | 21 × 11 | two rooms joined by a two-cell gate; every task crosses it | `--robots 10 --spawn 4` |
| `single-lane` | 23 × 9 | a one-cell corridor of length 7 with a passing place halfway: the hard case | `--robots 3 --spawn 8` |
| `charging-hub` | 24 × 12 | eight chargers along the south wall and a fleet that wants more, so robots queue for energy | `--robots 12 --spawn 4` |

`--robots` and `--spawn` are not part of a map: they say how many robots start and how often a random
task appears. The same map behaves very differently with them. Measured over 1500 ticks, seed 1, with
`--spawn 3` (a task every 3 ticks, more than any of these fleets can carry):

| Map | 3 robots | 10 robots |
|---|---|---|
| `small-depot` | 65 done | 111 done |
| `fulfilment-aisles` | 56 done | 156 done |
| `cross-dock` | 64 done | 180 done |
| `choke-point` | 60 done | 145 done |
| `single-lane` | 55 done | 69 done, 2 failed |
| `charging-hub` | 59 done | 149 done |

Every one of those runs passed `--audit` with no violation. `single-lane` is the instructive row:
tripling the fleet barely raises the throughput (one lane, one robot at a time) and two tasks were given
up after their robots had been stuck for 150 ticks. The corridor is where the planner's limits show
(see docs/architecture.md, "Limitations"): the run is safe, not fast.

To make your own, start from the closest example in the editor. A map needs a dropoff and somewhere goods
come from (a pickup cell, or a free cell beside a shelf).
