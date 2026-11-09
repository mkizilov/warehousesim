# Benchmarks and scenario campaign

The final figures of version 1.0.0, with what they were measured on. Two kinds of number appear here and
they must not be mixed up:

- **exact**: tasks done, search work, state checksums, heap allocations, anything counted by the
  simulation. The same build gives the same figures on any machine of the same arithmetic, and the
  regression gate checks them;
- **wall-clock**: milliseconds and microseconds. They belong to one machine, one build and one moment.

Everything the simulator reports is **simulated**: ticks of a model (one tick stands in for a second), cells
of a grid and units of an abstract energy model. Nothing here is a claim about how a real robot or a real
warehouse performs.

## Where these were measured

| | |
|---|---|
| Machine | Apple M4, 10 hardware threads, macOS (arm64) |
| Compiler | Apple clang 21.0.0.21000401, libc++ |
| Build | `cmake --preset release`: `-O3 -DNDEBUG -flto=thin -ffp-contract=off`, CMake 3.31.6, Ninja |
| Workers | at most two build or test workers; at most two planning threads besides the stepping thread |
| Quietness | **not quiet**: the 1-minute load average read 3.6 to 4.5 throughout, from processes that are not this project's (they could not be listed from here). Repeated runs of the same scenario agreed within about 3 %; read the wall-clock figures as that good and no better |

Link-time optimisation was compared with a build without it (the same sources, `-O3`), alternating the
two builds twice, on the five regression scenarios: the build with LTO was 4 to 5 % faster on the two
`large` scenarios (`large`, 100 robots: 0.285 against 0.299 ms a step; 256 robots: 0.741 against 0.777)
and level or a few per cent faster on the small ones. Both computed exactly the committed baseline, so
the preset keeps LTO.

## The regression gate

`warehousesim_regression_bench` (median of 5; steps are after a warm-up of 200):

| scenario | steps | ms / step | allocs / step | done | failed | search work | state checksum |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| demo, 4 robots | 2800 | 0.006 | 7.9 | 213 | 0 | 59 471 | `f1c46bfa9aa2d45b` |
| compact, 20 robots | 3300 | 0.063 | 112.0 | 166 | 24 | 1 082 913 | `47415ac8ff12ec36` |
| fulfilment, 24 robots | 1300 | 0.071 | 100.5 | 192 | 0 | 448 927 | `d6881fdbd9e481c9` |
| large, 100 robots | 1300 | 0.293 | 233.3 | 678 | 1 | 22 505 437 | `84e4843d771d4352` |
| large, 256 robots | 600 | 0.771 | 619.5 | 533 | 9 | 11 045 679 | `0097f72971c48a95` |

The columns from "allocs" on are what `bench/regression_baseline.txt` holds. Repeated measurements of the
`large`, 100-robot scenario over the session ranged from 0.278 to 0.293 ms a step. Saving the times of one
run and comparing a later run on the same machine against them (`--save-times`, `--times`) passed with a
limit of 1.5x.

## Planning threads

`warehousesim_scaling_bench`, `large` (100 x 64), a task every tick, seed 1, 1 500 steps a sample, median of
3. Threads are besides the stepping thread. The state checksum is the same for every thread count (the
benchmark fails if it is not).

| robots | planning threads | ms / step | speed-up | batches / step | questions / batch | tasks done |
|---:|---:|---:|---:|---:|---:|---:|
| 100 | 0 | 0.286 | 1.00x | - | - | 678 |
| 100 | 1 | 0.275 | 1.04x | 0.50 | 24.6 | 678 |
| 100 | 2 | 0.273 | 1.05x | 0.50 | 24.6 | 678 |
| 128 | 0 | 0.367 | 1.00x | - | - | 785 |
| 128 | 1 | 0.356 | 1.03x | 0.60 | 24.6 | 785 |
| 128 | 2 | 0.350 | 1.05x | 0.60 | 24.6 | 785 |
| 256 | 0 | 0.768 | 1.00x | - | - | 1 060 |
| 256 | 1 | 0.753 | 1.02x | 0.90 | 20.3 | 1 060 |
| 256 | 2 | 0.748 | 1.03x | 0.90 | 20.3 | 1 060 |

Threads give 2 to 5 % on a whole step. Only about 7 % of a step is independent work, so no number of
threads could give much more; the rest is the traffic controller and the cooperative search, which plan
robot after robot against each other's reservations (see the README, "Performance and planning threads").
On the raw batches of independent path questions they do help:

| questions / batch | 0 threads | 1 thread | 2 threads |
|---:|---:|---:|---:|
| 16 | 25.6 us | 15.1 us (1.70x) | 13.1 us (1.96x) |
| 64 | 98.6 us | 55.0 us (1.79x) | 41.0 us (2.40x) |
| 256 | 677.2 us | 316.7 us (2.14x) | 216.9 us (3.12x) |
| 1 024 | 3 504.9 us | 1 801.8 us (1.95x) | 1 198.6 us (2.92x) |

Preparing a frame for drawing (snapshot, interpolation and the draw list, not the SDL calls) takes under
0.6 % of a 60 fps frame at 100 to 256 robots. Executing the draw commands in SDL is not measured.

These agree with the README's figures, which were measured with directly compiled binaries: its
scaling table gave 0.290, 0.372 and 0.770 ms a step for 100, 128 and 256 robots on one thread, against
0.286, 0.367 and 0.768 here. Its table of "single-thread speed" with the engine's fixed planning limits
(0.233, 0.297 and 0.492 ms) is a different measurement: the shipped `large` scenario scales the limits with
the hall, as the README's second table (0.291 ms for 100 robots) and every figure here do.

## Fleet benchmark (simulated)

`warehousesim_fleet_bench`: 3 layouts x 4 fleet sizes x 3 seeds, 3 000 ticks each, a task every 5 ticks.
No robot overlapped or swapped places, every task was accounted for and the first case reproduced exactly.
Every row is the same, figure for figure, as the README's baseline table (compared row by row on a fresh
clone), as are the 8 rows of its light-load table (`--spawn 25 --layouts demo,fulfilment`). Utilisation is 95 to 98 % everywhere
(the load saturates these fleets), which is why throughput is the figure to read:

| layout | robots | tasks / h | tasks / robot-h | waiting % | conflicts / h | replans / h |
|---|---:|---:|---:|---:|---:|---:|
| demo | 1 | 77.6 | 77.6 | 0.0 | 0.0 | 0.0 |
| demo | 2 | 145.6 | 72.8 | 1.8 | 33.6 | 54.8 |
| demo | 4 | 260.0 | 65.0 | 4.4 | 164.0 | 323.6 |
| demo | 8 | 444.0 | 55.5 | 9.9 | 746.0 | 1 306.8 |
| compact | 1 | 73.2 | 73.2 | 0.0 | 0.0 | 0.0 |
| compact | 8 | 291.6 | 36.5 | 20.1 | 1 712.8 | 2 265.6 |
| fulfilment | 1 | 51.6 | 51.6 | 0.0 | 0.0 | 0.0 |
| fulfilment | 8 | 332.4 | 41.5 | 7.7 | 580.4 | 1 109.6 |

(The README has the full table with all columns.) More robots do more work up to 8, at a falling return
per robot and a rising share of time spent waiting for each other. Past 8 the next section applies.

## Path planning

`warehousesim_planning_bench`, one thread, median of 11, zero allocations on every warm search. A\* against
a plain breadth-first search on the same grids (all answers cross-checked):

| scenario | size | expanded | A\* (us) | breadth-first (us) |
|---|---:|---:|---:|---:|
| open | 64 | 127 | 2.5 | 20.5 |
| open | 512 | 1 023 | 29.3 | 1 275.5 |
| scattered obstacles | 256 | 3 926 | 148.0 | 424.1 |
| maze | 256 | 24 788 | 343.9 | 223.8 |
| maze | 512 | 46 506 | 729.4 | 1 537.5 |
| goal unreachable | 256 | 32 768 | 3 652.7 | 166.9 |
| goal unreachable | 512 | 131 072 | 16 298.3 | 653.1 |
| goal sealed in | 512 | 0 | 0.01 | 1 269.3 |

A\* wins wherever the heuristic helps (open halls, about 40x on 512 x 512) and is about level on mazes. It
is 20 to 25x slower than breadth-first when the goal is unreachable and the whole region has to be exhausted; the
simulator avoids that case by checking a sealed goal first and by bounding searches with a work limit.

## Scenario campaign

`scripts/campaign.py build/release/apps/warehousesim_cli`: 10 layouts (the four built-in, the six
`examples/`) x fleets of 1, 4, 16 and 64 robots x a task every 1, 5 and 25 ticks x seeds 1 to 3, each run
2 000 ticks with `--audit`, so every step of every run is checked against the invariants of
`src/simulation/audit.hpp` (about 38 seconds in all).

**Safety.** 360 runs, 27 of them refused because the layout had no room for the fleet (the small maps hold
fewer than 64 robots), 333 run: **666 333 audited states, 0 violations** of collision safety, task
ownership, event ordering or numerical sanity. Along the way the progress monitor detected 66 669 rings of
robots waiting for each other (56 790 of them in the 64-robot runs, none with one robot); the engine breaks
a ring with a sidestep, a priority boost or by giving up a task, and how many were broken each way was not
tabulated.

**What the first campaign found.** Before this release the campaign had 5 runs, of 333, that broke the
rule "no robot moves more than a cell in a tick": a blocked robot could push aside a blocker that had
already taken its own step in the same tick, so the blocker moved twice (1.005 to 1.25 cells of
distance in a tick, and a logical cell two steps away). All five were crowded maps with 16 to 64 robots,
all with seed 2. In the three reports read in full there was no overlap, swap or doubly held cell (the other
two were seen only as their last line). Fixed in `resolveBlock`, with a
regression test (`EndToEndTest.ARobotThatHasMovedIsNeverPushedAsideInTheSameTick`, red before the fix, green
after), and the engine revision is now 2. The committed baseline, the fleet benchmark above and every
pinned checksum did not change, so only runs that hit the case differ.

**Throughput.** Tasks completed in 2 000 ticks, mean of 3 seeds, with the number that ended *failed* in
brackets when there were any. The ten layouts, in this order: `demo`, `compact`, `fulfilment`, `large`,
`small-depot`, `fulfilment-aisles`, `cross-dock`, `choke-point`, `single-lane`, `charging-hub`. "-" means
the layout has no room for that fleet.

Heavy load, a task every tick (2 000 tasks offered; no fleet comes near):

| layout | 1 robot | 4 robots | 16 robots | 64 robots |
| --- | ---: | ---: | ---: | ---: |
| demo | 41 | 147 | 339 | 85 (105.7 failed) |
| compact | 40 | 118 | 105 (4.3 failed) | - |
| fulfilment | 30 | 103 | 272 | 41 (15.3 failed) |
| large | 7 | 44 (0.3) | 172 (0.3) | 602 (5.0 failed) |
| small-depot | 39 | 103 | 130 | - |
| fulfilment-aisles | 26 | 96 | 273 | 78 (31.3 failed) |
| cross-dock | 33 | 120 | 333 | 68 (67.7 failed) |
| choke-point | 30 | 103 | 202 (0.3) | 34 (90.3 failed) |
| single-lane | 28 | 86 | 66 (5.0 failed) | - |
| charging-hub | 29 | 99 | 266 | 28 (16.7 failed) |

Light load, a task every 25 ticks (80 tasks offered, so 78 or more done is "everything"):

| layout | 1 robot | 4 robots | 16 robots | 64 robots |
| --- | ---: | ---: | ---: | ---: |
| demo | 42 | 79 | 52 (23.7 failed) | 10 (62.7 failed) |
| compact | 40 | 78 | 40 (25.0 failed) | - |
| fulfilment | 29 | 78 | 71 (6.0 failed) | 54 (21.3 failed) |
| large | 9 | 40 | 75 (0.3) | 72 (2.7 failed) |
| small-depot | 38 | 77 | 37 (35.3 failed) | - |
| fulfilment-aisles | 26 | 78 | 59 (16.0 failed) | 41 (31.3 failed) |
| cross-dock | 33 | 78 | 76 (2.0 failed) | 67 (9.7 failed) |
| choke-point | 30 | 78 | 60 (14.7 failed) | 7 (53.0 failed) |
| single-lane | 28 | 76 | 57 (15.3 failed) | - |
| charging-hub | 29 | 78 | 50 (23.7 failed) | 6 (59.0 failed) |

(The medium load, a task every 5 ticks, is in `scripts/campaign.py`'s output; it sits between the two.)

**What the throughput says, and does not.**

- **A fleet bigger than the work can use does worse than a smaller one.** At the light load, 16 robots
  completed fewer tasks than 4 in 9 of the 10 layouts, and a quarter or more of the offered tasks *failed*
  in several (`compact`, `small-depot`, `charging-hub`, `demo`). At 64 robots on the small halls the
  throughput collapses (`demo` at heavy load: 85 done, against 339 with 16 robots). These are
  progress failures and not safety failures: the same runs were audited clean on every step.
- **Likely cause, not proven.** Robots that have finished stay where they are, and nothing sends them to a
  parking area. In the one run inspected (`fulfilment`, 16 robots, a task every 25 ticks, seed 1, tick
  1 500) twelve of the sixteen robots stood idle within a few cells of the east end of the dropoff row, and
  two robots heading there were stuck for 136 and 142 ticks, near the 150 at which a task is given up. That fits the explanation, and it fits the
  pattern above (the damage grows with fleet size and shrinks with hall size). No experiment isolated it, and a
  parking behaviour has not been tried, so it is not claimed as the fix.
- **Not a measure of the model.** The tables are one model, 2 000 ticks, 3 seeds. Differences of a few tasks
  between neighbouring cells are within seed-to-seed spread (the demo hall varies by about +-3 tasks per 500
  ticks) and mean nothing.
- **Why it was not called out earlier.** The fleet benchmark and the scenario tests used up to 8 robots. The
  effect may already show in a small form in the README's light-load table (`demo`: 142.8 tasks an hour with 4
  robots, 129.6 with 8, though a difference that small can be seed noise), and the campaign is the first thing
  to run 16 and 64 robots on the small halls.

## Reproducing

```sh
cmake --preset release && cmake --build --preset release -j2
build/release/bench/warehousesim_regression_bench           # the gate; exact columns must match the baseline
build/release/bench/warehousesim_scaling_bench
build/release/bench/warehousesim_fleet_bench
build/release/bench/warehousesim_planning_bench
python3 scripts/campaign.py build/release/apps/warehousesim_cli
```

On another platform the exact columns are the first thing to compare: if the state checksums differ
there, the arithmetic is not the same, and that is a finding to report (the README, "Recording and replay",
says where it could come from). The wall-clock columns will differ, and should.
