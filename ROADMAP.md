# Roadmap

What to do next with WarehouseSim 1.0.0, in priority order. The order follows one rule: **requirements that
are only partly met come first, then gaps in verification, correctness and security, then code health, and
new features last.** A feature does not move ahead of a known gap.

Every step states its scope, why it matters (with the evidence in this repository), what it depends on, how to
tell it is done, and an estimate in hours. The estimates are a plan, not a measurement; the assumptions are at
the end, after the total.

What exists today, and how far each part has been checked, is in
[docs/verification.md](docs/verification.md) (implemented, verified, partial, unverified). The
measurements this roadmap cites are in [docs/benchmarks.md](docs/benchmarks.md) and
[docs/audit.md](docs/audit.md).

| Group | Steps | Hours |
|---|---|---:|
| A. Requirements only partly met | 1 to 4 | 94 |
| B. Verification, correctness and security gaps | 5 to 12 | 103 |
| C. Code health that lowers risk | 13 | 16 |
| D. New features | 14 to 20 | 162 |
| **Total** | **20 steps** | **375** |

---

## A. Requirements only partly met

## 1. Make fleets larger than the work needs keep making progress

**Scope.**

- First, an experiment to find the cause. The likely cause is that idle robots stay where they finished and
  crowd the dropoffs (seen in one inspected run, not isolated). Count the ticks robots spend blocked by an
  *idle* robot, and run the same scenarios with finished robots removed from the grid (a bound on what any
  parking policy could gain).
- Then a parking and yielding policy: an idle robot leaves the traffic cells (the cells on shortest routes
  between pickups and dropoffs) for a parking cell after a few idle ticks, and an idle robot in the way yields
  further than one cell. Behind a `SimulationConfig` switch, default on.
- A new engine revision (3), a regenerated regression baseline and recording fixture, and updated README and
  benchmark tables.

**Rationale.** The cooperative-planning requirement was that solvable scenarios make progress. The audited
campaign ([benchmarks](docs/benchmarks.md#scenario-campaign)) shows they often do not: at a task every 25
ticks, 16 robots completed fewer tasks than 4 in 9 of 10 layouts (`demo`: 52 against 79, with 23.7 tasks
failed), 64 robots on `demo` completed 85 tasks against 339 for 16, and 111 of 333 audited runs had failed
tasks. Robots never collide in these runs; they block each other until a task is given up. The campaign is the
first thing that ran fleets of 16 and 64 on the small halls.

**Dependencies.** None.

**Acceptance criteria.**

- The experiment's result is written down. If idle robots are not the main cause, the scope is revised
  before any policy is built.
- With the policy on, in the campaign (`scripts/campaign.py`, seeds 1 to 3, 2 000 ticks): at the light load,
  16 robots complete at least 95 % of the offered tasks with at most 1 % failed in every layout where 4 robots
  do; at the heavy load, 64 robots on `large`, `fulfilment` and `fulfilment-aisles` complete at least 80 % of
  what 16 robots do.
- Throughput with up to 8 robots does not fall by more than the seed-to-seed spread (about 3 tasks per 500
  ticks on `demo`).
- The audit shows 0 violations over the whole campaign; the 5-scenario regression baseline is regenerated
  with the reason written in the commit.
- `kEngineRevision` is 3 and the pinned checksums and the recording fixture are updated on purpose.

**Effort.** 26 h (experiment 6, policy 10, tests and mutation checks 6, baselines and documents 4).

## 2. Resolve one-cell corridors with three or more robots

**Scope.** Detect corridor cells (at most two walkable neighbours, in chains); give a corridor a direction
token or a convoy rule so that robots do not enter against each other; send a robot back to a passing place
or niche when a corridor is contested; keep idle robots out of corridors.

**Rationale.** It is the documented completeness limit of the cooperative planner
([architecture](docs/architecture.md#limitations)). Safety holds, progress does not. Two tests pin the
safe-but-unsolved cases
(`RobotsQueuingAroundAnIdleRobotOnTheirSharedGoalStaySafe`,
`AnIdleRobotPushedIntoADeadEndOntoAnotherRobotsGoalStaysSafe`); `examples/single-lane.json` with 10 robots
ends with 2 failed tasks, and in the campaign `single-lane` completes fewer tasks with 16 robots than with 4
(66 against 86 at the heavy load).

**Dependencies.** Step 1 (shares the parking and yielding rules).

**Acceptance criteria.**

- The two `StaySafe` tests become completion tests (every robot reaches its goal within a stated bound) and
  the limitation text is removed from the README and architecture document on purpose.
- `single-lane` with 10 robots: 0 given-up tasks over seeds 1 to 10 at 1 500 ticks.
- `cross-dock` and `choke-point` do not lose throughput beyond the seed spread; 0 audit violations over the
  campaign; the safety layer (`World` holds) is untouched and `reservations = false` still works.

**Effort.** 24 h (design 6, implementation 10, scenarios and tests 8). Uncertain: plus or minus 40 %.

## 3. Finish the fleet interface

**Scope.** Keys or buttons for the operations the simulation already supports as commands: cancel a task, take
a robot out of or back into service, put up or take down an obstacle at the clicked cell, restart with a new
seed. Make `FleetController` judge clicks on the layout as it is now (it keeps a copy from when it was built).
Hit-testing that prefers the square that is drawn when two robots are in adjacent cells. A usable panel in
windows smaller than 720 x 400.

**Rationale.** The README lists these as limits of the interface. The stale layout copy is a defect found in
the audit ([audit](docs/audit.md#2-concurrency)): after `PlaceObstacle` its click messages can be wrong,
though the engine still refuses what cannot be done.

**Dependencies.** None. The real-window check of the result is part of step 6.

**Acceptance criteria.**

- Each new action has a `FleetController` test, a runner test and a replay test, as AGENTS.md requires of a
  new operator action; none takes a draw from the simulation's random streams.
- A test places a wall by command and then clicks it as a pickup: refused with the right message.
- The random-input property test stays green with the new keys; the README table of keys is updated from the
  code.

**Effort.** 16 h.

## 4. Script and watch recordings

**Scope.**

- A scenario script for the CLI: `--script FILE`, a list of "at tick T, this command", applied through the same
  `apply` as the window's commands and therefore recorded.
- A replay viewer: `warehousesim_gui --replay FILE` with play, pause, single step and speed, and the tick of
  the first divergence marked. Seeking backwards replays from tick 0 (said in the interface).

**Rationale.** The README lists as limits: the CLI cannot give commands, so its recordings have none; a replay
cannot be watched, only run headless; replay takes as long as running.

**Dependencies.** Step 3 (shares the panel code), softly.

**Acceptance criteria.**

- A script round-trips: `--script` then `--record` then `--replay` gives the same final state, checked
  byte for byte, and the recording holds the commands.
- The viewer's glue is tested with a real runner and no window, like `FleetSession`.
- The recording format version is unchanged, or bumped with the old version still read (pinned test).

**Effort.** 28 h (scripts 6, viewer 16, tests and documents 6).

---

## B. Verification, correctness and security gaps

## 5. Get the GCC 13 and MSVC jobs green

**Scope.** Re-run `ci.yml` after the fixes to the GCC 13 and MSVC build errors, and fix whatever those builds
report beyond them (a build stops at its first failing target). Run the tests on both compilers, compare the
state checksums with those of macOS and Ubuntu clang, run `release.yml` once, and decide and document the
supported platforms.

**Rationale.** The second hosted run (commit `69afde0`) passed every required job: Ubuntu 24.04 with clang
(Debug and Release with the GUI, AddressSanitizer with leak detection, ThreadSanitizer, clang-tidy, formatting)
and macOS 26 on arm64 (Debug and Release with the GUI), so the pinned checksum test, the regression baseline and
the recording fixture agree on macOS arm64 and Ubuntu x86-64. The experimental GCC 13 and MSVC jobs have stopped
at their build step in both runs, each time at errors that are fixed afterwards; the latest fixes have not run.
The Windows tests have never run, and `-ffp-contract=off` is a GCC and clang flag: MSVC's floating-point mode has
not been examined.

**Dependencies.** The repository published and a hosted CI available (both are).

**Acceptance criteria.**

- All non-experimental jobs green on the main branch for three runs in a row (they were green in the second run).
- The GCC 13 job green, with `ThePinnedScenarioHasThePinnedChecksums` and the fixture replay passing, or the
  divergence traced to its cause and either fixed or documented with a plan (for example, a per-platform
  checksum). The same for MSVC.
- Windows headless is green, or declared unsupported in the README with the reason.
- `release.yml` has run once and produced its archives.
- The verification document moves each platform from unverified to verified, with the run it comes from.

**Effort.** 17 h (GCC 4, Windows and MSVC 10, the release workflow 1, checksums and documents 2).
Uncertain: plus or minus 40 %.

## 6. Verify the GUI and editor in a real window, and automate what can be

**Scope.** Run the manual checklist in [docs/verification.md](docs/verification.md#manual-checklist-real-window)
on macOS and Linux desktops and record the result. Automate: tests that push synthetic SDL events (keys,
wheel, drag, resize) through `SdlWindow` and assert the `InputEvents` produced; render each scene on SDL's
software renderer and compare with golden frames within a tolerance.

**Rationale.** The GUI and editor have only run with SDL's dummy video driver for a few frames. What they decide
is tested without a display; the mapping from real keys and the mouse to that logic (Shift+Tab, Ctrl or Cmd
with Z, Y, S and O, middle-button drag), the drawing, text, resizing and high-DPI behaviour are not. What has
been checked: both programs start, render and exit on the real macOS driver (`cocoa` with the `metal`
renderer, 5 frames each), and a probe showed that a drawn colour is read back exactly from that renderer and
from the dummy driver's software renderer, so golden frames are feasible without a display.

**Dependencies.** None for the automated part, which uses the dummy driver's software renderer (pixel read-back
confirmed); a desktop session for the manual part.

**Acceptance criteria.**

- At least 25 injected-event tests, one for every key and pointer action in the README tables.
- At least 6 golden-frame tests (fleet view, editor, dashboard, overlays, a small window, a large map) with a
  documented way to update them.
- The manual checklist ticked on two operating systems, with date and commit.

**Effort.** 19 h.

## 7. Share one strict JSON reader between the two file formats

**Scope.** Extract the common reading rules (size and depth limits, unknown and duplicate keys, typed getters
with JSON Pointer paths, a capped problem list) into one internal library used by `mapio` and by the
recording reader.

**Rationale.** Code analysis finds 130 duplicated lines between `mapio/map_file.cpp` and
`simulation/recording_file.cpp` (3 clone groups). They are the two parsers of untrusted input; a strictness fix
made in one and missed in the other is the risk.

**Dependencies.** None.

**Acceptance criteria.**

- Every existing format test passes unchanged, including the pinned texts and the refusals with their
  pointers.
- The duplicate analysis shows 0 clone groups between the two files.
- Breaking a shared rule on purpose is caught by both suites (mutation check). The module table in AGENTS.md is
  updated.

**Effort.** 8 h (behaviour-preserving).

## 8. Fuzz the file parsers and put a budget on untrusted work

**Scope.**

- libFuzzer targets for the map file and the recording file, built as an optional `fuzz` preset with
  AddressSanitizer and UBSan, seeded from `examples/` and the test fixtures, run briefly in CI and for longer
  on a schedule.
- A random-damage test for map files equal to the one recordings already have.
- A budget for replaying and running untrusted recordings: a default limit on `ticks x robots`, an override
  option, and an early refusal of a file that declares more.

**Rationale.** Both parsers are strict and bounded and have never been fuzzed. Only recordings have a random
damage test. A recording may declare up to 10^9 ticks and 4 096 robots and nothing limits the time replaying it
takes (README, recording limits), so a hostile file can hold a CPU for good. Apple's Command Line Tools clang
does not ship the libFuzzer runtime (`libclang_rt.fuzzer_osx.a` is not found), so this needs Linux or a
project-local LLVM.

**Dependencies.** Step 7 (each rule is fuzzed once); a clang with libFuzzer (the Ubuntu CI job, or a
project-local LLVM).

**Acceptance criteria.**

- One CPU-hour of fuzzing per target with no crash, timeout or out-of-memory; any finding is fixed and its
  input added to the corpus as a regression test.
- Replaying a recording that declares 4 096 robots and 10^9 ticks is refused within 100 ms with exit status 1
  and a message that names the limit and the option to raise it.
- The map-file damage test mirrors the recording one.

**Effort.** 16 h (targets and presets 6, budget 4, findings and tests 6).

## 9. Write files safely

**Scope.** One helper for atomic output used by the map, recording and CSV writers: a unique temporary file in
the same directory created exclusively (`O_EXCL`, or `CREATE_NEW` on Windows) with restrictive permissions,
flushed to disk, renamed over the target, and removed on any failure. Decide and document what happens when
the target is a symbolic link.

**Rationale.** All three writers (`map_file.cpp`, `recording_file.cpp`, `metrics/export.cpp`) write to
`<path>.tmp` with a plain `ofstream`. The name is predictable, a pre-placed symbolic link there would be
followed, two processes saving the same file share one temporary file, nothing is flushed to disk before the
rename, and the code exists three times.

**Dependencies.** None; the Windows variant after step 5.

**Acceptance criteria.**

- A pre-planted `<path>.tmp` symbolic link is not followed or overwritten.
- Concurrent saves of one path never leave a torn file.
- A failed rename leaves no temporary file. The existing all-or-nothing behaviour and error messages are
  unchanged. AddressSanitizer and clang-tidy are clean.

**Effort.** 8 h.

## 10. Pin the supply chain and make the environment reproducible

**Scope.** Pin fetched dependencies by full commit hash (keeping the tag in a comment) and check it; pin
GitHub Actions by hash; a hashed lock file for the Python tools; a software bill of materials with each
release; a container recipe.

**Rationale.** `cmake/WarehouseSimDependencies.cmake` fetches by `GIT_TAG` (`v3.12.0`, `v1.17.0`,
`release-3.2.0`), and a tag can be moved. Actions are used by tag (`actions/checkout@v4` and others). The
tools are pinned by version but not by hash. There is no recipe for a clean-machine build; Docker could not
be used where this was developed, so any recipe starts out unverified.

**Dependencies.** Network access; step 5 for the CI parts.

**Acceptance criteria.**

- A configure fails if a fetched revision differs from its pinned hash (shown by changing one character).
- A lint step in CI fails on any action that is not pinned by hash.
- `pip install --require-hashes -r requirements.lock` reproduces the toolchain on macOS and Linux.
- The release workflow attaches an SBOM. The verification document says whether the container recipe has been
  run.

**Effort.** 10 h.

## 11. Detect leaks, soak the engine, and bound long-run cost

**Scope.** Valgrind on the command line program (macOS is covered by `scripts/leaks_macos.sh`, Linux by
LeakSanitizer in the hosted `asan` job); a nightly soak run (10^6 ticks, 256 robots) under AddressSanitizer;
keep open tasks in an index so `snapshot()` costs the same however many tasks were finished;
compact finished tasks to counters plus the last few; an optional cap on open tasks.

**Rationale.** On macOS, `leaks --atExit` reports 0 leaks in 18 programs (the CLI, the GUI, the editor and all
ten test executables; `scripts/leaks_macos.sh`), and the tool reports a leak planted on purpose. On Linux the
hosted `asan` job runs the suite with LeakSanitizer enabled and passed (its log has not been read for a
LeakSanitizer line); Valgrind has not run. Measured for this roadmap
(details in the verification document): the engine's own step
time is flat over 10^6 ticks (6.0 us), memory at a load the fleet serves grows from 3.3 to 8.2 MB between
2 x 10^4 and 10^6 ticks (about 120 bytes per finished task), but with a snapshot and an audit at every step the
cost per step grew from 4.5 us at 2 x 10^4 ticks to 10.9 us at 3.2 x 10^5, because both scan every task ever made;
and in a saturated run (`demo`, 4 robots, a task every 5 ticks) the open backlog grows without limit, to 240 MB
at 10^6 ticks.

**Dependencies.** None for the soak and the index; a Linux machine for Valgrind.

**Acceptance criteria.**

- Valgrind: 0 leaks over the CLI examples; the hosted `asan` log shows LeakSanitizer active.
  (LeakSanitizer itself: 0 leaks over the suite, already passing.)
- Soak: at a load the fleet serves, resident memory at 10^6 ticks is at most 1.1 times that at 10^5.
- Per-step cost with a snapshot each step at 3.2 x 10^5 ticks is within 10 % of that at 2 x 10^4.
- Every pinned checksum is unchanged (the change is behaviour-preserving); the cap on open tasks is off by
  default.

**Effort.** 11 h.

## 12. Make the test suite faster and free of real time, and keep the mutation checks

**Scope.**

- An injectable clock for the runner's pacing so threaded tests need no real time; test labels (`unit`, `e2e`,
  `slow`, `threaded`) and `ctest -L`; a quick tier on every push and the slow tier on a schedule.
- A mutation runner and its mutant lists kept in the repository (`scripts/`): it checksums the sources, applies
  each mutant exactly once, rebuilds with forced timestamps, runs the suite, restores the sources and reports
  survivors. Rebuilt from the descriptions of the earlier runs, covering the modules with the most rules (the
  reservation table, the dispatcher, the file formats, the recorder, the auditor).

**Rationale.** The ThreadSanitizer run of the 1 533 tests took 610 s and 647 s in two runs and AddressSanitizer
249 s, and the example
tests take about 40 s each under ThreadSanitizer. The runner's frame tests rely on real time with generous
bounds (documented in AGENTS.md as an allowed exception). Fast, deterministic tiers get run. AGENTS.md asks for
a mutation check of every new test, and earlier work ran them (the numbers are in the commit messages), but the
runner and the mutant lists were scratch files and are not in the repository, so nobody can rerun them.

**Dependencies.** Step 5.

**Acceptance criteria.**

- 100 consecutive runs of the threaded tests under ThreadSanitizer with 0 flakes (nightly).
- The quick tier under ThreadSanitizer in at most 5 minutes; the slow tier still runs nightly.
- One command runs the mutation checks and reports every mutant caught, or lists the survivors with a reason
  for each; no test is removed or weakened to get there.

**Effort.** 14 h (clock and tiers 10, mutation tooling 4).

---

## C. Code health that lowers risk

## 13. Split the engine and the command line into smaller pieces

**Scope.** Separate the phases of a step (spawn, assign, robot turns, supervise) into private classes behind the
unchanged public interface; a table-driven option parser shared by the three programs; add a cognitive-
complexity check to `.clang-tidy` with a threshold and bring the code under it.

**Rationale.** `simulation.cpp` has 732 lines, `cooperation.cpp` 331 and `runner.cpp` 310. Code analysis finds 78
functions with cyclomatic complexity of 10 or more, the worst one in `apps/cli/main.cpp` at 46. The double-move
defect found in the audit lived in the interaction of two such functions.

**Dependencies.** Steps 1 and 2 first, since they change the same code.

**Acceptance criteria.**

- Behaviour-preserving, shown by: every pinned checksum, the regression gate, the fleet benchmark output
  byte-identical, and the recording fixture.
- No function above complexity 25; `main()` of the CLI at most 60 lines.
- Benchmarks within 3 % of before.

**Effort.** 16 h.

---

## D. New features

## 14. Assign tasks globally and let urgent tasks preempt

**Scope.** A `Dispatcher` that solves the assignment of available robots to pending tasks at once (Hungarian or
auction) within a work limit, and an optional rule that re-assigns a robot still on its way to a pickup when a
more urgent task would be served sooner overall.

**Rationale.** The README states that assignment is greedy in priority order, is not a globally optimal matching,
never preempts a working robot, and uses estimates that ignore other robots.

**Dependencies.** Step 1, so that a gain is not hidden by clogging.

**Acceptance criteria.**

- On the fleet benchmark and the campaign, mean completion time or tasks per hour improves by at least 3 % in at
  least 3 layouts at the medium load, or the result is reported as negative and the dispatcher stays optional.
- Exclusivity is intact (`TaskQueue::tryAssign` is still the only claim); ties are broken deterministically; the
  reference greedy in `dispatcher_test.cpp` is kept in step.

**Effort.** 24 h.

## 15. Charge robots with a plan

**Scope.** First measure what the present policy costs (flat-battery events and waiting for chargers in
`charging-hub` and `compact`). Then, if it costs something: a queue for chargers, charging when idle near a charger,
a charge-to threshold, robots that are full leaving the charger, and energy-aware dispatch (do not assign a task
the robot cannot finish).

**Rationale.** The policy is minimal (README, Robots): an idle robot below its reserve charges where it stands or
goes to the nearest free charger; a flat battery takes the robot out of service. Whether that loses throughput has
not been measured.

**Dependencies.** Step 1.

**Acceptance criteria.**

- The measurement is published first. If a policy follows: no flat-battery events over seeds 1 to 10 at 3 000 ticks
  in `charging-hub` with 12 robots, and throughput not lower than before beyond the seed spread.

**Effort.** 16 h.

## 16. Richer worlds

**Scope.** A different `RobotSpec` for each robot (a mixed fleet); one-way aisles and speed zones as cell
attributes in a version 2 of the map format (version 1 files still read); dynamic obstacles (scripted agents such as
people crossing an aisle); editor tools for the new attributes.

**Rationale.** All robots share one model and all cells are plain; real layouts are neither. The map format already
has a version number and the auditor a place for new rules.

**Dependencies.** Steps 3 and 7.

**Acceptance criteria.**

- Version 1 maps read unchanged (pinned test); version 2 has its own pinned text and refusals.
- The auditor reports a robot driving against a one-way aisle; examples gain a mixed-fleet map.
- The recording format covers the new fields (the compile-time check in `TheFormatCoversEveryFieldOf...` forces
  it).

**Effort.** 32 h.

## 17. Scale the traffic planning with threads

**Scope.** Plan regions of the warehouse in parallel (spatial partitioning with a deterministic merge), or plan
speculatively and resolve conflicts in a fixed order, so that more than the 7 % of a step that is independent today
can use threads.

**Rationale.** Planning threads give 2 to 5 % on a whole step ([benchmarks](docs/benchmarks.md#planning-threads)):
the rest is the traffic controller and the cooperative search, which plan robot after robot against each other's
reservations. At 256 robots a step costs about 0.75 ms.

**Dependencies.** Steps 1, 2 and 13.

**Acceptance criteria.**

- At least 1.5 times faster steps at 256 robots on `large` with 4 planning threads, with the same state checksum
  for every thread count from 0 to 8, clean under ThreadSanitizer. If not reached within the estimate, the
  negative result is documented and the code is not merged.

**Effort.** 40 h. A research step: plus or minus 60 %.

## 18. Package and distribute

**Scope.** A macOS application bundle, a Windows archive or installer, a Linux AppImage or package, with
checksums and signatures; release notes generated from the changelog; the notices that the libraries linked
into the programs require, shipped as separate files in each archive (the programs link SDL3 statically and
include nlohmann_json).

**Rationale.** Today the release is a tarball built with `cpack` on the machine that runs it
(`cmake --build --preset package`); it was unpacked elsewhere and run, but only on macOS.

**Dependencies.** Steps 5 and 10. Code signing and notarisation need credentials that are not in this repository.

**Acceptance criteria.**

- The release workflow produces artifacts for three operating systems and smoke-runs each (`--version`, a CLI
  audit, the GUI on the dummy driver).
- A `SHA256SUMS` file is published with them; the macOS bundle launches on a clean machine (checked manually and
  recorded).
- Each archive contains the project's `LICENSE` and the third-party notice files, checked by listing the archive
  in CI.

**Effort.** 20 h.

## 19. Readable text, text entry and a scalable interface

**Scope.** Real fonts through SDL_ttf with a bundled font under an open licence (reviewed), a layout that scales
with the window and the display, text measurement behind an interface so `view` stays free of SDL. In the editor:
a text field for the file name (SDL text-input events), and rectangle and fill tools.

**Rationale.** Text is the SDL debug font: the panel is 39 characters wide, long messages wrap, the help line in the
editor's bar is longer than a narrow window, and the panel is dropped in windows under 720 x 400. The editor takes
its file name only from `--file`, changes the size one cell per key press, and has only freehand painting (README,
Map editor and map files).

**Dependencies.** Steps 3 and 6 (golden frames).

**Acceptance criteria.**

- Layout tests with fake font metrics; nothing clipped in golden frames at 1280 x 720 and 800 x 600; readable at
  twice the pixel density (checked manually and recorded).
- A file name typed in the editor is used by save and load (a test injects text-input events); a rectangle or a
  fill is one undo step and never leaves the map.

**Effort.** 18 h (text 10, editor 8).

## 20. An API reference and an extension guide

**Scope.** Doxygen for the public headers, built in CI with warnings as errors; a tutorial that writes a custom
dispatcher, planner and command; an example consumer project; install the headers and export the library targets
(only the programs are installed today).

**Rationale.** The extension points are listed in [docs/architecture.md](docs/architecture.md#extension-points), but
there is no generated reference and the libraries are internal.

**Dependencies.** None; it is cheaper after step 13.

**Acceptance criteria.**

- The reference builds with no warnings and covers every public header.
- The example consumer is built in CI from the installed tree.

**Effort.** 12 h.

---

## Total estimate

| Group | Steps | Hours |
|---|---|---:|
| A. Requirements only partly met | 1 (26), 2 (24), 3 (16), 4 (28) | 94 |
| B. Verification, correctness and security | 5 (17), 6 (19), 7 (8), 8 (16), 9 (8), 10 (10), 11 (11), 12 (14) | 103 |
| C. Code health | 13 (16) | 16 |
| D. Features | 14 (24), 15 (16), 16 (32), 17 (40), 18 (20), 19 (18), 20 (12) | 162 |
| **Total** | **20 steps** | **375** |

About 63 working days for one engineer (375 hours at 6 productive hours a day), or about 13 weeks. Groups A to C
alone are 213 hours, about 36 working days. With two engineers working in parallel on independent steps
(for example 1 and 5, then 2 and 6) the calendar time is roughly 7 to 8 weeks.

**Suggested order.** Steps 1, 5 and 6 first and in parallel: they decide whether the product behaves well, runs
elsewhere, and works in a real window. Then 7, 8 and 9, which close the input-handling risks. Step 2 after 1; step 13
after 2; the rest as capacity allows.

## Assumptions

- The estimate is for one engineer who knows this codebase and its rules in AGENTS.md. Hours include the tests,
  the mutation checks, the sanitizer runs, the document updates and the review fixes that the repository's
  quality bar needs; they exclude waiting for a hosted CI queue and for a reviewer.
- Steps marked uncertain (2, 5, 17, and the experiment in 1) carry a range of 40 to 60 % either way. The others
  about 25 %.
- A hosted CI service, a Linux machine or runner, and a desktop session on macOS and Linux are available.
  Windows needs a runner. Docker, code-signing credentials and a Windows machine are not available where this
  was written, so the container recipe in step 10 and the signing in step 18 start out unverified.
- Behaviour changes (steps 1, 2, 14, 15, 16) each raise the engine revision, regenerate the regression
  baseline and the recording fixture, and update the README tables in the same change, as AGENTS.md requires.
- The simulation's figures stay simulated: nothing here turns them into claims about real robots or
  warehouses.

## Not planned

- Multi-floor warehouses, robot arms and picking, and fleet management of real robots. The model is a grid of
  cells with one robot per cell.
- A GPU renderer. The picture is a draw list of rectangles, lines and text and is cheap.
- Calibrating the model against real robots. The time scale and the energy model are stand-ins; that would be a
  different project with real data.
