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

The local checks ran on fresh clones (a clean checkout, with nothing left over from earlier builds), on one
platform: **macOS 27.2 on Apple M4 (arm64)**, Apple clang 21.0.0 with libc++, CMake 3.31.6, Ninja 1.11.1,
clang-format 21.1.8 and clang-tidy 21.1.6 (the pinned versions in `.venv`), with real GoogleTest 1.17.0,
nlohmann_json 3.12.0 and SDL3 3.2.0 fetched by CMake. At most two workers were used, and every test ran under
the 900 s limit of the test presets. Rows marked † were last run on commit `bc58d15`; the formatting, Debug,
Release, AddressSanitizer and regression gate rows on `ca4803e`; the rest (clang-tidy, the campaign, the
archive) on `77518c8`. From `77518c8` to `ca4803e` only tests, `.gitattributes` and documents changed (nothing
under `src`, `apps`, `bench`, `cmake` or `.github`), so those rows stand. From `bc58d15` to `77518c8` the
changes were the types of some literals, a renamed enumerator, three string copies, an `if constexpr` (the GCC
and MSVC fixes below), the macOS runner label, the license file, the leak script and documents: no behaviour,
which the unchanged results back up (the same 1 538 tests, the same regression baseline, the same campaign
totals).

Hosted runners are a separate platform and are covered under Hosted CI below.

| Check | Result |
|---|---|
| `clang-format --dry-run --Werror` over 217 files in `src apps tests bench testing` | 0 violations |
| `cmake --preset default` (Debug, GUI, real SDL3), build, `ctest --preset default -j2` | **1 538 of 1 538** pass in 75 s; project code compiles with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror` and no warnings |
| `cmake --preset release` (`-O3`, link-time optimisation), build, `ctest --preset release -j2` | **1 538 of 1 538** pass in 9 s; no warnings in project code (the SDL3 and GoogleTest sources produce a few of their own) |
| `ctest --preset asan -j2` (AddressSanitizer, UBSan, `float-divide-by-zero`, `float-cast-overflow`, no recovery) | **1 533 of 1 533** pass in 245 s, 0 reports. The five allocation-counting tests are not built under sanitizers, hence 1 533 |
| `ctest --preset tsan -j2` (ThreadSanitizer) † | **1 533 of 1 533** pass in 617 s, 0 ThreadSanitizer reports. On `ca4803e` the 145 tests of the threaded and end-to-end groups (`-R 'Runner\|Recording\|Replay\|PlanningThreads\|EndToEnd'`, which holds the changed runner test) pass in 228 s, 0 reports |
| `cmake --preset tidy`, then build (clang-tidy on every project target, warnings are errors) | builds with exit 0; clang-tidy ran in 67 compile steps (`--warnings-as-errors=*`), **0 findings** in project code |
| `warehousesim_regression_bench` against `bench/regression_baseline.txt` | 5 of 5 scenarios identical (done, failed, search work, state checksum, allocations) |
| `scripts/campaign.py` (10 layouts x fleets 1, 4, 16, 64 x 3 loads x 3 seeds, 2 000 ticks, every run `--audit`) | 333 runs audited, **666 333 states, 0 violations**; 27 further runs refused because the layout has no room for the fleet. The totals (32 214 tasks done, 3 507 failed, 12 883 given up, 66 669 rings detected) are the same as in every earlier run of this tree |
| `cmake --build --preset package` and unpacking the archive | the archive holds the three programs, the six example maps, the documents and `LICENSE`, and the archive's `LICENSE` is byte-identical to the repository's; the unpacked `warehousesim_cli --version` runs |
| the commands in AGENTS.md's verification block that run a program from `build/default` (9) † | all ran with exit status 0 |
| the GUI and the editor on the **real macOS video driver** (SDL's default: `cocoa`, with the `metal` renderer), 5 frames each † | start, run and exit with status 0 |
| a scratch SDL program linked the way the GUI is, on the same machine † | default driver `cocoa` and renderer `metal` draw a colour and read the same colour back (10, 200, 30); the `dummy` driver (software renderer) reads it back exactly too; the `offscreen` driver cannot create a window here (it needs OpenGL) |
| `scripts/leaks_macos.sh build/release`: macOS `leaks --atExit` over 18 programs (the CLI in six configurations, the GUI and the editor on the dummy driver, and all ten test executables), each bounded to 900 s † | **0 leaks in all 18**. The tool was shown to report a deliberately planted 4 KB leak. The GUI and the editor were also clean on the real Cocoa driver (30 frames each) |
| LeakSanitizer | not available with Apple's clang, so `leaks(1)` was used (above); on Linux it ran in the hosted `asan` job |
| a scan of the tracked files, and of the contents and messages of all earlier commits, for credentials, caches, binaries and generated files † | none found; no credential-like string in any revision. Early versions of `AGENTS.md` (up to `070ca45`) contain a local filesystem path, which the current version no longer has; history was not rewritten |

### Hosted CI

`ci.yml` has run three times on hosted runners: run 42566965565 on commit `315b9b8`, run 42576051990 on
`69afde0` and run 42582839103 on `f03538d`. Job results, from the GitHub API:

| Job | Run 1 (`315b9b8`) | Run 2 (`69afde0`) | Run 3 (`f03538d`) |
|---|---|---|---|
| `format` | passed | passed | passed |
| `clang-tidy` | passed | passed | passed |
| `build-test`, Ubuntu 24.04 (x86-64), clang, Debug + GUI | passed: build and all tests; the scenario campaign, example audits, record and replay, metrics export, and the GUI and editor smoke runs on the dummy driver | passed | passed |
| `build-test`, Ubuntu 24.04, clang, Release + GUI | passed: the same, plus the planning, fleet and scaling benchmarks, the regression gate and the packaged program | passed | passed |
| `sanitizers`, `asan` (run with `ASAN_OPTIONS=detect_leaks=1`) and `tsan` | passed, the whole suite each | passed | passed |
| `build-test`, macOS, clang, Debug + GUI | cancelled before any step: the `macos-14` image had been retired | passed on `macos-26`: build, all tests, campaign, example audits, record and replay, metrics export, GUI and editor smoke runs | passed |
| `build-test`, macOS, clang, Release + GUI | cancelled, as above | passed: the same, plus the benchmarks, the regression gate, the macOS leak check (a `continue-on-error` step that reported success; its log was not read) and the packaged program | passed |
| `build-test`, Ubuntu, GCC 13, headless (experimental) | failed to compile: `-Werror=sign-conversion`, a `uint64_t` counter increased by an `int` ternary (fixed in `1ff3bad`, `53725a2`) | failed to compile: `-Werror=shadow` (the enumerator `EventSource::Command` against the `Command` alias, `recording.hpp:53`) and `-Werror=dangling-reference` (`recording_file.cpp:514`, `574`, `589`). Fixed in `03c73ee` and `c30a216` | passed: configure, build, all 1 538 tests, and the steps that run on it: the scenario campaign, the large hall on 1 and 2 planning threads and its replay, example audits, record and replay, metrics export |
| `build-test`, Windows, MSVC, headless (experimental) | failed to compile: C4244 from `std::fill` on a `vector<char>` given an `int` (fixed in `1ff3bad`) | failed to compile: C4127, a constant conditional, at `bench/regression_bench.cpp:306` (fixed in `77518c8`) | built; the **Test step ran 1 538 tests and 5 failed**: three allocation tests, the example-map test and one runner recording test (below). Fixed in `b874197`, `73749a5` and `dc9dfd9`; **not run since** |

Both required macOS jobs, the Ubuntu jobs and the GCC 13 job pass the pinned-checksum tests, the recording
fixture and the rest of the suite, so the state checksums agree between macOS arm64 (local and hosted), Ubuntu
x86-64 with clang and GCC 13. The five Windows failures are none of those: the checksum tests and the fixture
replay were among the tests that passed there too, so MSVC's Debug build agrees as well (read from the reported
failures; the job log was not read).

The steps after the tests are skipped on the Windows headless job by design (the campaign, the CLI checks and
the smoke runs have `runner.os != 'Windows'` or need a release or GUI preset), so the failing Test step was the
last thing it would have run.

**The five Windows test failures, and why** (the numbers are from the run; the reasons are from the code, the
MSVC STL source and a model, since nothing here can run MSVC):

| Test | Cause | Fix |
|---|---|---|
| `AStarAllocationTest.UnreachableFloodSearchesAllocateNothingWhenWarm`: 2 allocations, expected 0 | the test passed `{}` for the blocked cells inside the measured region; that makes a `PointSet`, which is two `std::vector`s, and in the MSVC Debug STL (`_ITERATOR_DEBUG_LEVEL` 2) each vector made or moved into allocates a small proxy. The search itself allocated nothing | the empty set is made once outside the measured regions |
| `...FindPathAllocatesOnlyTheReturnedPath`: 6 allocations, expected 1 | 2 for the `{}`, plus the proxies of the three vectors made or moved on the way (the one `findPath` fills, the one in the optional it returns, the test's own assignment), plus the path buffer | the empty set is outside, the result is not assigned, and the library's own cost per vector is measured and added to the expectation (`1 + 2 x` for a path, `2 x` for an empty one, `x` for none; `x` is 0 with libc++ and libstdc++, so the numbers there are as before) |
| `...ALargerGridGrowsTheWorkspaceOnceAndThenSettles` | the same `{}` in the measured regions | the same |
| `EndToEndTest.EveryExampleMapIsValidAndRunsCleanOnAFleetAndALoad`: not in the canonical form | git for Windows checks text files out with CRLF (`core.autocrlf`); the writer is canonical and ends lines with LF, and the test compares bytes | `.gitattributes`: `examples/*.json text eol=lf`; the comparison is unchanged |
| `RunnerRecordingTest.TheRecordingOfARunReplays...`: no recording taken midway | the 300-tick run at 8x is 37 ms long, less than the test's own command phase when sleeps and waits have the coarse timer of Windows, so the run was over before the polling loop looked | the run is started with a limit of 150 ticks, so the worker cannot pass the half way mark; the first recording is taken there (certainly a run in progress, at tick 150, with every command applied), then the run is started again for the second half, with the polling kept as an extra |

On macOS the fixed tests pass, the CRLF failure was reproduced (the example maps converted to CRLF: same line,
same message) and cured, and a clone made with `core.autocrlf=true` gets LF example maps byte-identical to the
repository's. The allocation arithmetic was checked against a model of the MSVC STL's proxy allocation that
reproduces the two reported numbers (2 and 6) from the old test and gives the expected values for the new one.
What is **not** checked: the fixed tests were not run on MSVC.

The GCC and MSVC diagnostics of run 2 are quoted from the job logs as they were relayed; the check annotations
of a run carry only the exit status, and the logs need a signed-in GitHub session, so they were not fetched. A
build stops at its first failing target, so the GCC and MSVC jobs may report further diagnostics behind these.

**GCC, locally.** A GCC for macOS arm64 was installed under `.tools/` (conda-forge, GCC 16.2.0; the steps are in
the [developer guide](developer-guide.md#gcc-diagnostics-without-gcc-13)). On `69afde0` it reports exactly the
GCC errors of run 2 and nothing else: the `Command` shadowing in seven library and program translation units
that include `recording.hpp` (the tests are built with a lighter warning set, without `-Wshadow`), and the three
dangling references. After the fixes, the whole headless build compiles with no diagnostic: every library,
program and benchmark with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror`, and the
tests with `-Wall -Wextra -Wpedantic -Werror`. This is GCC 16, not 13: GCC 13's `-Wdangling-reference` is a coarser heuristic, and
the libstdc++ differs. The executables this GCC links do not run correctly on the development machine (a
GoogleTest program with a single test and no project code aborts, and a program printing with `std::cout` prints
nothing), so no test, benchmark or program that it built was run: only compilation is checked.

**MSVC** cannot be run here at all. For C4127 the fix is an `if constexpr`; the variant in which
`kAllocationsComparable` is false (MSVC, or any library but libc++) was type-checked with the project's strict
warning flags on a scratch copy of the file. Other MSVC `/W4 /WX` diagnostics behind it are unknown.

The earlier fixes (`1ff3bad`, `53725a2`) came from the diagnostics and from searching the code for the same
shape: every unsigned counter that receives an `int` ternary now receives an unsigned one, and every
`std::fill` and `assign` was checked against its element type.

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
| Map editor program | **Implemented, unverified** | its logic is tested without a window; the program starts, renders and exits on the real Cocoa/Metal driver (5 frames) and on the dummy driver (30), with no leaks. Nothing was typed or clicked and nobody looked at the picture: real keys, mouse, text and drawing are unchecked (roadmap 6). No file-name entry, no rectangle or fill tool |
| Fleet interface logic (`view`: controller, session, scenes) | **Partial** | tested with a real runner and no window, including determinism of what the interface does. Cannot cancel a task, take a robot out of service or edit the layout; keeps a copy of the layout for judging clicks that goes stale after `PlaceObstacle` (roadmap 3) |
| SDL3 window and drawing (`rendering`, `apps/gui`) | **Implemented, unverified** | compiles and links against real SDL3 3.2.0 (static); starts, renders and exits on the real Cocoa/Metal driver (5 frames) and on the dummy driver (30), with no leaks, and a probe read a drawn colour back exactly from both renderers. The mapping from real input, the look of the drawing, the text (SDL's debug font) and resizing have not been checked by eye or by a test |
| Recording and replay | **Partial** | verified on this platform: record, replay, divergence reports, a recording kept as a fixture, replays on 0 and 2 planning threads. The CLI cannot give commands, a replay cannot be watched in a window, a recording starts at tick 0 with the default planner and dispatcher, and replaying has no time budget (roadmap 4 and 8). Checksums agree between macOS arm64, Ubuntu x86-64 (clang and GCC 13) and, from the reported failures, Windows with MSVC in Debug (the fixture replays on all); the MSVC Release configuration is not run |
| Planning threads (`--threads`) | **Verified** | the state and checksum are the same for every thread count; TSan-clean. The gain is small (2 to 5 % of a whole step) |
| Command line (`warehousesim_cli`) | **Verified** | the documented commands, exit statuses, equal output for equal seeds. `--audit` checks every step |
| Invariant auditor | **Verified** | each rule fires on a state, built directly in the test, that breaks it; real runs are clean; auditing never changes a run |
| Performance regression gate | **Verified** | exact behaviour and allocations against a committed baseline; time only against times saved on one machine. Allocation comparison is off for other standard libraries than libc++. The Release job on Ubuntu passed the gate too (behaviour columns) |
| Benchmarks and numbers | **Verified** (one machine) | Apple M4, not quiet. Quote only the exact columns elsewhere; the timings belong to this machine |
| Packaging | **Verified** on macOS arm64 and Ubuntu | `.tar.gz` of the programs, examples, documents and `LICENSE`, unpacked and run (on Ubuntu by the Release job); SDL3 linked statically. Windows, signing and notarisation: not done |
| Example maps | **Verified** | six maps, canonical form, each audited, run twice, on two planning threads, recorded and replayed in the test suite |
| clang-format, clang-tidy | **Verified** on macOS and Ubuntu | pinned versions; both also passed in the hosted runs |
| CI workflows (`ci.yml`, `release.yml`) | **Partial** | `ci.yml` ran three times on hosted runners; in the second and third runs every required job passed (format, clang-tidy, both Ubuntu clang jobs, both macOS jobs, `asan`, `tsan`) and in the third so did the experimental GCC 13 job (see Hosted CI above). The experimental MSVC job builds and fails five tests; the fixes have not run since. `release.yml` has not run (it runs on a version tag or on demand) |
| Ubuntu (x86-64) with clang | **Verified** | the hosted runs: Debug and Release builds with the GUI, all tests, benchmarks, the campaign and smoke runs, packaging, `asan`, `tsan`, clang-tidy |
| macOS 26 (arm64) on a hosted runner | **Verified** | the second hosted run: Debug and Release builds with the GUI, all tests, benchmarks, the regression gate, the campaign, record and replay, smoke runs, the macOS leak check (step reported success, log not read) and packaging |
| GCC 13 (Linux, headless) | **Verified** | the third hosted run: configure, build, all 1 538 tests, the campaign, the large hall on two planning threads, example audits, record and replay and metrics export (an experimental job) |
| Windows with MSVC (Debug, headless) | **Partial** | the third hosted run built it and ran all 1 538 tests: 1 533 passed. The five failures were test-side (three allocation counts that included the MSVC Debug STL's own proxies, CRLF example maps, one timing assumption in a runner test), are fixed and have not run since. Not run on Windows at all: the Release configuration, packaging as a `.zip`, the GUI and the editor, the scenario campaign and the command line checks of the workflow (which skip Windows), and `release.yml` |
| Cross-platform agreement of checksums and recordings | **Partial** | the pinned checksum test, the recording kept as a fixture, the regression baseline and the rest of the suite pass on macOS arm64 (Apple clang, libc++; locally and on a hosted runner) and on Ubuntu x86-64 (clang, and GCC 13 with libstdc++). On Windows with MSVC (Debug) they were not among the five failures. The regression baseline was not run with GCC or MSVC (the gate runs in the release preset); `-ffp-contract=off` is set, MSVC's floating-point mode was not examined |
| Leak detection | **Verified** on macOS | `scripts/leaks_macos.sh`: 0 leaks in 18 programs (the CLI, the GUI, the editor, all ten test executables); ownership is by RAII (no raw `new`/`delete`). On Linux the hosted `asan` job runs with LeakSanitizer enabled and passed (its log was not read for a LeakSanitizer line); Valgrind: not run (roadmap 11) |
| Parser security (map file, recording) | **Partial** | strict, bounded in size and depth, never throw or crash on the inputs tried, one random-damage test (recordings only); no fuzzing |
| File output security | **Partial** | all-or-nothing by rename; the temporary file name is predictable and not created exclusively (roadmap 9) |
| Supply chain | **Partial** | dependencies and tools pinned by tag and version, not by hash; actions by tag (roadmap 10) |
| Docker | **Not implemented** | not used; any Docker-based check would be unverified here |
| License | **Verified** | `LICENSE` at the root (MIT); the README links to it; the release archive installs it with the documents |

## Known defects and limits that are not fixed

- Fleets bigger than the work needs make less progress than smaller ones; the likely cause (idle robots left near
  the dropoffs) was seen in one run and is not proven. Documents: [benchmarks](benchmarks.md#scenario-campaign),
  [architecture](architecture.md#limitations).
- One-cell corridors with three or more robots can deadlock (safely).
- `FleetController` judges clicks on the layout it was built with.
- Replaying an untrusted recording has no time budget: a file may declare 10^9 ticks and 4 096 robots.

## Manual checklist (real window)

Not yet run by a person. To be ticked on each operating system, with the date, the commit and the person; the
automated parts of roadmap step 6 are meant to replace as much of it as can be replaced. A program has checked
the first item only in the sense that the window opens, renders frames and closes (Cocoa/Metal, 5 frames, exit
status 0); everything that needs input, or a look at the picture, is still open.

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
