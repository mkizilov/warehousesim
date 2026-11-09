# Verification status

What WarehouseSim does, and how far each part has been checked. It is meant to be read before trusting a claim
in the README, and to be kept true: change it in the same commit as the change that makes a row wrong.

**Status words.**

| Status | Meaning |
|---|---|
| **Verified** | Implemented, and checked by automated tests or tools that were run, on the platform named below, with the result recorded here. It does not mean proved, and it does not mean it works elsewhere. |
| **Partial** | Implemented and verified for what exists, but an original requirement is only partly met or a known limit applies. The limit is stated. |
| **Implemented, unverified** | The code or configuration exists and builds, but it has never been run for real, or only far enough to start. |
| **Not implemented** | Asked for or reasonable to expect, and absent. |

Anything not listed here has not been checked.

## What was run

All checks were run on a **fresh clone of commit `070ca45`** (so a clean checkout, with nothing left over from
earlier builds), on one platform: **macOS 27.2 on Apple M4 (arm64)**, Apple clang 21.0.0 with libc++, CMake
3.31.6, Ninja 1.11.1, clang-format 21.1.8 and clang-tidy 21.1.6 (the pinned versions in `.venv`), with real
GoogleTest 1.17.0, nlohmann_json 3.12.0 and SDL3 3.2.0 fetched by CMake. At most two workers were used. The code
in the commit that adds this document is the same as in `070ca45`; only documents changed.

| Check | Result |
|---|---|
| `clang-format --dry-run --Werror` over 211 files in `src apps tests bench testing` | 0 violations |
| `cmake --preset release`, then `cmake --build --preset release -j2` | configures and builds. Project code compiles with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror` and no warnings. The build log has 41 compiler warnings, 40 in SDL3's Objective-C sources and 1 in a GoogleTest header, none in project code |
| `ctest --preset release -j2` | **1 538 of 1 538** pass (about 10 s) |
| `ctest --preset asan -j2` (AddressSanitizer, UBSan, `float-divide-by-zero`, `float-cast-overflow`, no recovery) | **1 533 of 1 533** pass in 249 s, 0 reports. The five allocation-counting tests are not built under sanitizers, hence 1 533 |
| `ctest --preset tsan -j2` (ThreadSanitizer) | **1 533 of 1 533** pass in 647 s, 0 ThreadSanitizer reports (`-fsanitize=thread` confirmed in the build) |
| `cmake --preset tidy`, then build (clang-tidy on every project target, warnings are errors) | builds with exit 0; clang-tidy ran in 67 compile steps (`--warnings-as-errors=*`), **0 findings** in project code. The installed tool was shown to report a deliberately planted null dereference |
| `warehousesim_regression_bench` against `bench/regression_baseline.txt` | 5 of 5 scenarios identical (done, failed, search work, state checksum, allocations) |
| `scripts/campaign.py` (10 layouts x fleets 1, 4, 16, 64 x 3 loads x 3 seeds, 2 000 ticks, every run `--audit`) | 333 runs audited, **666 333 states, 0 violations**; 27 further runs refused because the layout has no room for the fleet. The totals (32 214 tasks done, 3 507 failed, 12 883 given up, 66 669 rings detected) are the same as in the earlier run of the same tree |
| `warehousesim_fleet_bench` against the README's two fleet tables | every row of both tables (12 and 8 rows) has the same figures |
| `cmake --build --preset package`; `cmake --install build/release --prefix DIR` | the archive builds; the install puts 16 files in `DIR` and `DIR/bin/warehousesim_cli --version` runs |
| the commands in the user guide and the README (18 of them: CLI runs, `--metrics`, `--metrics-csv`, `--record`, `--replay` with and without `--ticks`/`--no-verify`, `--threads 2`, bad layout, missing map, conflicting options, GUI and editor with `SDL_VIDEODRIVER=dummy`, `campaign.py --quick`, install) | all ran, with the exit statuses the documents give (0, and 2 for a bad layout or conflicting options, 1 for a missing map) |
| the GUI and the editor, 5 to 30 frames with SDL's dummy video driver | start, run and exit with status 0 |

Earlier, on the same platform and on the working tree that became `070ca45`: the packaged archive unpacked in
another directory runs (CLI with `--audit`, GUI on the dummy driver) and links no library outside the system
frameworks; the three sanitizer suites and clang-tidy were also clean there.

**Measured while preparing the roadmap** (release build, this machine, not quiet: its load average was about 4):
at a load the fleet serves, the engine step stays at 6.0 us from 2 x 10^4 to 10^6 ticks and peak memory goes from
3.3 to 8.2 MB; with a snapshot and an audit every step the cost grew from 4.5 us (2 x 10^4 ticks) to 10.9 us
(3.2 x 10^5 ticks), because both scan every task ever made; in a saturated run (`demo`, 4 robots, a task every 5
ticks) memory grows to 240 MB at 10^6 ticks, since the open backlog is state.

## What the tests are

1 538 tests: 1 527 GoogleTest cases in 66 test files and 10 executables, linked so that the dependency rules of
AGENTS.md are enforced by the build (`warehousesim_world_tests` against `core` and `world` only, and so on), and
11 script-level tests (the CLI on every layout and on a map file, the six example maps each audited, run twice,
run on threads, recorded and replayed, and four benchmark smoke runs). They include randomised ones with fixed seeds (random halls, random command mixes, random input to
the controllers), reference implementations (a breadth-first search for A\*, an independent greedy dispatcher),
pinned texts and checksums, and the invariant auditor on whole runs. Mutation checks (breaking the code on
purpose to see the tests fail) were run for each module when it was written and are reported in the commit
messages; **the mutation runner and the mutant lists were scratch files and are not in the repository**, so they
cannot be rerun from here (roadmap step 12).

## Status by area

| Area | Status | Evidence, and what is missing |
|---|---|---|
| Grid model, layouts, occupancy (`world`) | **Verified** | unit and model-based tests; `checkLayout`; four built-in layouts |
| Path planning, A\* (`planning`) | **Verified** | against a breadth-first reference on about 14 000 random queries, admissibility checks, zero warm allocations, benchmark |
| Robots: state machine, motion, battery (`robots`) | **Verified** | transition tables, step-length invariance tests, audit of speed and braking. Energy and time scale are stand-ins, not calibrated |
| Tasks and dispatch (`tasks`) | **Verified** | exclusive claims, an independent reference greedy. Greedy only: not globally optimal, no preemption (roadmap 14) |
| Collision safety (world holds, reservations) | **Verified** | 666 333 audited campaign states and all end-to-end tests: no overlap, swap or doubly held cell. A sample, not a proof |
| Collision avoidance and cooperative planning (`traffic`) | **Partial** | safe on every checked run; not complete and not optimal. Fleets much bigger than the work can use do worse than smaller ones (16 robots completed fewer tasks than 4 in 9 of 10 layouts at a light load); one-cell corridors with three or more robots can deadlock (`...StaySafe` tests). Roadmap 1 and 2 |
| Deadlock detection and recovery | **Partial** | rings are detected and mostly broken; 111 of 333 campaign runs had failed tasks (a task is given up after 150 stuck ticks) |
| Simulated metrics, dashboard text, CSV (`metrics`) | **Verified** | hand-checked tests, zero-denominator rules, CSV columns pinned. Every figure is simulated |
| Map file and editing model (`mapio`, `editor`) | **Verified** | pinned text, strict refusals with JSON Pointers, round trips, random halls, undo and redo. Not fuzzed (roadmap 8) |
| Map editor program | **Implemented, unverified** | its logic is tested without a window; the program ran 5 to 30 frames on SDL's dummy driver. Real keys, mouse, text and drawing have not been seen (roadmap 6). No file-name entry, no rectangle or fill tool |
| Fleet interface logic (`view`: controller, session, scenes) | **Partial** | tested with a real runner and no window, including determinism of what the interface does. Cannot cancel a task, take a robot out of service or edit the layout; keeps a copy of the layout for judging clicks that goes stale after `PlaceObstacle` (roadmap 3) |
| SDL3 window and drawing (`rendering`, `apps/gui`) | **Implemented, unverified** | compiles and links against real SDL3 3.2.0 (static) and ran 5 to 30 frames on the dummy driver. The mapping from real input, the drawing, the text (SDL's debug font) and resizing have not been checked by eye or by a test |
| Recording and replay | **Partial** | verified on this platform: record, replay, divergence reports, a recording kept as a fixture, replays on 0 and 2 planning threads. The CLI cannot give commands, a replay cannot be watched in a window, a recording starts at tick 0 with the default planner and dispatcher, and replaying has no time budget (roadmap 4 and 8). Checksums on other platforms: unverified |
| Planning threads (`--threads`) | **Verified** | the state and checksum are the same for every thread count; TSan-clean. The gain is small (2 to 5 % of a whole step) |
| Command line (`warehousesim_cli`) | **Verified** | the documented commands, exit statuses, equal output for equal seeds. `--audit` checks every step |
| Invariant auditor | **Verified** | each rule fires on a hand-built state that breaks it; real runs are clean; auditing never changes a run |
| Performance regression gate | **Verified** | exact behaviour and allocations against a committed baseline; time only against times saved on one machine. Allocation comparison is off for other standard libraries than libc++ |
| Benchmarks and numbers | **Verified** (one machine) | Apple M4, not quiet. Quote only the exact columns elsewhere; the timings belong to this machine |
| Packaging | **Verified** on macOS arm64 | `.tar.gz` of the programs, examples and documents, unpacked and run; SDL3 linked statically. Other operating systems, signing and notarisation: not done |
| Example maps | **Verified** | six maps, canonical form, each audited, run twice, on two planning threads, recorded and replayed in the test suite |
| clang-format, clang-tidy | **Verified** on macOS | pinned versions; tidy against Linux headers has not been run |
| CI workflows (`ci.yml`, `release.yml`) | **Implemented, unverified** | the YAML parses; no hosted runner has run them. Experimental jobs (GCC 13, MSVC, clang-tidy on Linux) are marked as such |
| Linux, GCC, Windows, MSVC builds | **Implemented, unverified** | nothing outside macOS arm64 with Apple clang has been built |
| Cross-platform agreement of checksums and recordings | **Implemented, unverified** | the pinned checksum test and the fixture recording exist to find out; `-ffp-contract=off` is set, MSVC's mode was not examined |
| Leak detection | **Implemented, unverified** | ownership is by RAII (no raw `new`/`delete`); LeakSanitizer does not exist with Apple's clang, so leaks were not tool-checked |
| Parser security (map file, recording) | **Partial** | strict, bounded in size and depth, never throw or crash on the inputs tried, one random-damage test (recordings only); no fuzzing |
| File output security | **Partial** | all-or-nothing by rename; the temporary file name is predictable and not created exclusively (roadmap 9) |
| Supply chain | **Partial** | dependencies and tools pinned by tag and version, not by hash; actions by tag (roadmap 10) |
| Docker | **Not implemented** | not used; any Docker-based check would be unverified here |
| `LICENSE` file | **Not implemented** | not in the repository at this commit |

## Known defects and limits that are not fixed

- Fleets bigger than the work needs make less progress than smaller ones; the likely cause (idle robots left near
  the dropoffs) was seen in one run and is not proven. Documents: [benchmarks](benchmarks.md#scenario-campaign),
  [architecture](architecture.md#limitations).
- One-cell corridors with three or more robots can deadlock (safely).
- `FleetController` judges clicks on the layout it was built with.
- Replaying an untrusted recording has no time budget: a file may declare 10^9 ticks and 4 096 robots.

## Manual checklist (real window)

Not yet run. To be ticked on each operating system, with the date, the commit and the person; the
automated parts of roadmap step 6 are meant to replace as much of it as can be replaced.

Start `warehousesim_gui --layout fulfilment --robots 16 --spawn 3` and check:

- [ ] A window opens at a sensible size with the warehouse, the robot list and the panel visible and readable.
- [ ] Robots move smoothly (no jumps) at 1x, 4x and 16x (`]`) and at 0.5x (`[`); Space pauses and resumes.
- [ ] Wheel zooms about the pointer; dragging and the arrow keys or WASD pan; `0` fits the whole map.
- [ ] Resizing the window re-fits the map until it has been panned or zoomed; a window under 720 x 400 hides the
      panel without crashing.
- [ ] Clicking a robot or a row selects it; Tab and Shift+Tab cycle; the inspector shows its state, battery and
      route; `P` and `O` toggle the route and the held cells.
- [ ] `N`, a pickup click and a dropoff click add a task (`1` to `4` set its priority); the task appears in the
      active list and is delivered; `1` to `4` on a selected waiting task change its priority.
- [ ] `R` restarts; `M` shows and hides the dashboard, which says it is simulated; `E` writes the CSV files
      (start with `--metrics-csv BASE`).
- [ ] Esc cancels task entry, then clears the selection, then quits; closing the window quits cleanly.
- [ ] On a high-density display the text and the squares are the right size.

Start `warehousesim_editor --file my-hall.json --layout compact` and check:

- [ ] Left drag paints with the selected tool, right drag erases, `1` to `6` change tool; a fast drag leaves no
      gaps; a whole drag is one undo step.
- [ ] Shift+arrow keys resize; shrinking says how many objects were cut off; Ctrl or Cmd with Z and Y undo and
      redo; Ctrl or Cmd with S saves; with O loads; `L` cycles the built-in layouts.
- [ ] Loading over unsaved changes, or quitting with them, needs the key twice and asks first.
- [ ] The saved file opens with `warehousesim_cli --map my-hall.json --audit`.

## Reproducing the checks

```sh
export PATH="$PWD/.venv/bin:$PATH"          # the pinned cmake, ninja, clang-format, clang-tidy
cmake --preset release && cmake --build --preset release -j2 && ctest --preset release -j2
cmake --preset asan    && cmake --build --preset asan -j2    && ctest --preset asan -j2
cmake --preset tsan    && cmake --build --preset tsan -j2    && ctest --preset tsan -j2
cmake --preset tidy    && cmake --build --preset tidy -j2
find src apps tests bench testing \( -name '*.cpp' -o -name '*.hpp' \) | xargs clang-format --dry-run --Werror
build/release/bench/warehousesim_regression_bench
python3 scripts/campaign.py build/release/apps/warehousesim_cli
```

To test that a clean checkout works, run them in a fresh `git clone` of the commit.
