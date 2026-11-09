# Developer guide

How to build, test, analyse, benchmark and release WarehouseSim. [architecture.md](architecture.md) says
how it is built; [AGENTS.md](../AGENTS.md) has the rules every change has to keep.

## Toolchain

The pinned tools live in a project-local virtual environment, so nothing is installed globally and CI
uses the same versions:

```sh
python3 -m venv .venv
.venv/bin/pip install cmake==3.31.6 ninja==1.11.1.4 clang-format==21.1.8 clang-tidy==21.1.6
export PATH="$PWD/.venv/bin:$PATH"
```

You also need a C++20 compiler (Apple clang 21 is what the project is developed with). The libraries
nlohmann_json 3.12.0, GoogleTest 1.17.0 and SDL3 3.2.0 are fetched by CMake at those versions (a copy on
the system is used if there is one; a fetched SDL3 is linked statically). To build offline set
`FETCHCONTENT_SOURCE_DIR_<NAME>` or `FETCHCONTENT_FULLY_DISCONNECTED`.

## Presets

All build trees are under `build/` (git-ignored). Use at most two workers: `cmake --build --preset X -j2`.

| Preset | Configuration | Use it for |
|---|---|---|
| `default` | Debug, GUI, tests, warnings as errors | everyday work |
| `headless` | the same without SDL3 | the quickest full build and test (no GUI or editor) |
| `release` | optimised, link-time optimisation, GUI, tests | benchmarks and the package |
| `asan` | headless + AddressSanitizer, UBSan, float-divide-by-zero, float-cast-overflow; no recovery | memory and undefined behaviour |
| `tsan` | headless + ThreadSanitizer | the runner and the thread pool |
| `tidy` | headless + clang-tidy on every project target | static analysis |
| `package` (a build preset of `release`) | builds, then packs programs, examples and docs | the release |

```sh
cmake --preset headless && cmake --build --preset headless -j2 && ctest --preset headless -j2
```

Warnings are errors in every preset (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`
for the libraries), and floating point is compiled with `-ffp-contract=off`: keep both.

## Tests

`ctest` runs every GoogleTest case separately, plus script tests. The suites are split so that the
dependency rules are enforced by the build: a test executable only links what it tests.

| Executable | Links | Covers |
|---|---|---|
| `warehousesim_world_tests` | core, world | grid, occupancy, layouts, checksum, `PointSet`, `ThreadPool` |
| `warehousesim_robot_tests` | robots | state machine, motion, forecasts |
| `warehousesim_planning_tests` | planning | A\*, space-time A\*, batches, against a reference search |
| `warehousesim_planning_alloc_tests` | planning | zero allocations when warm (not under sanitizers) |
| `warehousesim_task_tests` | tasks | queue, dispatcher against an independent greedy, estimators |
| `warehousesim_traffic_tests` | traffic | reservation table, controller, router, progress monitor |
| `warehousesim_mapio_tests`, `_editor_tests`, `_metrics_tests` | their module | file format, editing model, aggregation |
| `warehousesim_tests` | everything but SDL | the engine, scenarios, view logic, recording and replay, **end-to-end** |

The whole-system tests, and what each guards:

| File | What it checks |
|---|---|
| `end_to_end_test.cpp` | every layout × fleet size × task load runs clean and does work; no task, no motion; bigger fleets do more; 100 and 256 robots on `large`; a random mix of every command kind, recorded and replayed on 0 and 2 planning threads; every example map |
| `random_worlds_test.cpp` | random halls: map file round trip, the simulator refuses exactly the unusable ones, A\* and `PathDistance` against a breadth-first search, audited runs that replay exactly |
| `audit_test.cpp` | each invariant of the auditor is noticed when broken (states built directly in the test), real runs are clean, auditing never changes a run |
| `collision_avoidance_test.cpp`, `cooperative_planning_test.cpp`, `movement_test.cpp` | crossing paths, opposing corridor traffic, intersections, simultaneous arrivals; no overlap, no swap, at most a cell a tick |
| `replay_test.cpp`, `recording_test.cpp`, `runner_recording_test.cpp`, `planning_threads_test.cpp` | replay from a recording gives the same state, metrics and checksums; a pinned recording made earlier still replays; thread counts change nothing |
| `cmake/RunHeadlessCli.cmake`, `cmake/RunExample.cmake` | the CLI as a user runs it: exit codes, equal output for equal seeds, refused maps, record and replay, planning threads, every example |

Every test preset gives a test 900 seconds (`execution.timeout` in `CMakePresets.json`; the slowest test takes
about 40 s under ThreadSanitizer), and every CI job has a `timeout-minutes`, so that a hang is reported with the
name of the test instead of waited for. To diagnose one, run that test alone with the filter below, and under
`ctest` with a short `--timeout N`; a threaded test that hangs is first run under ThreadSanitizer.

Useful ways to run a part:

```sh
build/headless/tests/warehousesim_tests --gtest_filter='EndToEndTest.*'
ctest --preset headless -R example_ --output-on-failure
```

### The invariant auditor

`simulation::InvariantAuditor` (`audit.hpp`) is the one place that says what must hold of any run:
collision safety, ownership of tasks, event order and numerical sanity. The tests, the benchmarks and
`warehousesim_cli --audit` all use it. When you add a rule, add it there, with a test in `audit_test.cpp`
that builds a state which breaks it (the auditor takes a plain `Snapshot`, so that is a few lines).

### Randomised tests

Randomness comes only from `core::Rng` with a fixed seed written in the test, so a failure reproduces.
When one fails, the message names the hall, fleet and tick; rerun with `--gtest_filter` and a debugger.

## Sanitizers

```sh
cmake --preset asan && cmake --build --preset asan -j2 && ctest --preset asan -j2
cmake --preset tsan && cmake --build --preset tsan -j2 && ctest --preset tsan -j2
```

`-fno-sanitize-recover=all` is on, so a finding aborts the test instead of printing and carrying on.
LeakSanitizer is not available on macOS; the Linux `asan` CI job runs with it (`ASAN_OPTIONS=detect_leaks=1`).
On macOS use `scripts/leaks_macos.sh build/release`, which runs the command line
program, the GUI, the editor and every test executable under `leaks --atExit` (each bounded by
`LEAKS_TIME_LIMIT`, default 900 s) and fails on any leak. The allocation-counting tests and benchmark parts are
left out under sanitizers because they replace `operator new`.

## Static analysis and formatting

```sh
cmake --preset tidy && cmake --build --preset tidy -j2     # fails on any finding
find src apps tests bench testing \( -name '*.cpp' -o -name '*.hpp' \) | xargs clang-format --dry-run --Werror
```

`.clang-tidy` is curated: every enabled check is one the code is kept clean for. Fix a finding rather
than adding `NOLINT`. After `clang-format -i`, check that long `// ----` banner comments were not wrapped
onto two lines. Do not run clang-format on files you did not change.

## Benchmarks

| Program | Purpose |
|---|---|
| `warehousesim_planning_bench` | A\* against breadth-first search across map sizes; cross-checks the answers |
| `warehousesim_fleet_bench` | layouts × fleet sizes × seeds with the simulated metrics; fails on a collision or an unaccounted task |
| `warehousesim_scaling_bench` | wall-clock cost of a step with and without planning threads; fails if a thread count changes a result |
| `warehousesim_regression_bench` | the performance gate (below) |
| `scripts/campaign.py` | the scenario campaign: layouts × fleets × loads × seeds, every run `--audit`ed (about 40 s; `--quick` for CI). Tables of throughput for reading, a failing exit only for a broken invariant |

Quote timings only from the `release` preset on a quiet machine, with the machine and compiler named.
Results are in [benchmarks.md](benchmarks.md).

### The regression gate

`warehousesim_regression_bench` runs five fixed scenarios and compares them with
`bench/regression_baseline.txt`:

- **behaviour must be identical**: tasks completed and failed, cooperative-search work, state checksum.
  A difference means the simulation now computes something else. That can be intended, but it has to be
  a decision;
- **allocations per step may not grow** by more than 10% (+8); counted by replacing `operator new`, so
  only with libc++ and not under sanitizers;
- **time** is printed but is not in the baseline, because it belongs to a machine. To catch a slowdown
  save the times of a build you trust and compare later builds on the same machine:

```sh
build/release/bench/warehousesim_regression_bench --save-times build/times.txt
# ... change the code, rebuild ...
build/release/bench/warehousesim_regression_bench --times build/times.txt --max-slowdown 1.3
```

`ctest` runs the quick variant (three scenarios) in every preset. To change the baseline on purpose:
build `release`, run `--write-baseline bench/regression_baseline.txt`, and explain in the commit why the
behaviour or the allocations changed. If the behaviour changed, also raise `kEngineRevision` and
regenerate the recording fixture (see AGENTS.md, "Changing what a simulation computes").

## Continuous integration

`.github/workflows/ci.yml` runs on every push:

| Job | What |
|---|---|
| `format` | `clang-format` 21.1.8 over `src apps tests bench testing` |
| `build-test` | Ubuntu and macOS, clang, `default` and `release`: build, `ctest`, benchmarks, the regression gate, the scenario campaign, example audits, record and replay, GUI and editor smoke runs (SDL dummy driver), the package. Experimental (failures reported, not fatal): GCC 13 headless on Ubuntu, MSVC headless on Windows |
| `sanitizers` | `asan` and `tsan` on Ubuntu, the whole suite |
| `clang-tidy` | the `tidy` preset on Ubuntu (experimental) |

Every preset has been run locally on macOS (Apple clang 21, arm64). In the second hosted run (commit
`69afde0`) every required job passed (`format`, `clang-tidy`, the two Ubuntu clang jobs, the two macOS jobs on
`macos-26`, `asan` with leak detection, `tsan`); the experimental GCC 13 and MSVC jobs failed to compile, as in
the first run, at errors that are fixed afterwards and have not run since. A build stops at its first failing
target, so a red GCC or MSVC job may show the next error: treat it as information about the platform, not
necessarily a regression. [verification.md](verification.md) keeps the list of what has passed.

### GCC diagnostics without GCC 13

Neither GCC nor MSVC is part of the macOS toolchain. To see GCC's diagnostics before pushing, a GCC for
macOS arm64 can be installed under `.tools/` (git-ignored) with micromamba and built against the headless
preset; this finds compile errors only:

```sh
mkdir -p .tools/mm && curl -sSL https://micro.mamba.pm/api/micromamba/osx-arm64/latest | tar -xj -C .tools/mm bin/micromamba
MAMBA_ROOT_PREFIX=$PWD/.tools/mm/root .tools/mm/bin/micromamba create --no-rc -y -p .tools/gcc --override-channels \
  -c conda-forge gxx_impl_osx-arm64=16.2.0 gcc_impl_osx-arm64=16.2.0
GCC=$PWD/.tools/gcc/bin/arm64-apple-darwin20.0.0
PATH=$PWD/.tools/gcc/bin:$PATH SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk \
  cmake --preset headless -DCMAKE_C_COMPILER=$GCC-gcc -DCMAKE_CXX_COMPILER=$GCC-g++ -DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
PATH=$PWD/.tools/gcc/bin:$PATH cmake --build --preset headless -j2 -- -k 0
```

conda-forge's GCC for macOS arm64 starts at 15.2, so this is GCC 16.2.0, not 13, and its
`-Wdangling-reference` is not the same heuristic. It does report the `-Wshadow` and `-Wdangling-reference` errors
the GCC 13 job reported. The executables it links do not run correctly on the development machine (a GoogleTest
program of one test aborts, and `std::cout` prints nothing), so this checks that the code compiles with the
project's flags, not that it behaves. The `gtest_discover_tests` step of the test targets then fails at the end
of the build; the library, the programs and the benchmarks build.

## Releasing

1. Update `CHANGELOG.md` and the `VERSION` in the top-level `CMakeLists.txt`.
2. `cmake --preset release && cmake --build --preset release -j2 && ctest --preset release -j2`.
3. Run the regression gate and the benchmarks; update [benchmarks.md](benchmarks.md) if the numbers
   changed.
4. `cmake --build --preset package` gives `build/release/warehousesim-<version>-<system>-<cpu>.tar.gz`.
   Unpack it somewhere else and run `bin/warehousesim_cli --map share/warehousesim/examples/small-depot.json --audit`.
5. Commit, tag `v<version>`. The `Release` workflow rebuilds, tests and packages on Linux and macOS and
   keeps the packages as workflow artifacts; it fails if the tag and the version disagree.

## Adding things

[architecture.md](architecture.md#extension-points) lists where each kind of extension goes. The
checklist that applies to all of them: a test in the executable that matches the module, the rule in
AGENTS.md if the change adds a rule, a README or docs line if a user can see it, and a mutation check of
the new tests (break the code on purpose and see the tests fail; a test that cannot fail proves nothing).
