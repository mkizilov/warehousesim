# Audit

What was examined before version 1.0.0, how, what it found, and what it could not check. The five areas the
release asked for come first; the defects found follow, then what is not covered.

Method, in short: reading the code with targeted searches (ownership, threads, floating point); running the
whole suite under AddressSanitizer + UBSan (with the float checks, no recovery) and ThreadSanitizer;
running clang-tidy over every target; and a new tool, `simulation::InvariantAuditor`, that checks a running
simulation after every step ([`--audit`](user-guide.md#checking-a-run)). The auditor was run over 333
scenario runs (666 333 states) and in the test suite. An audit that finds nothing proves little, so the
rules of the auditor were each tested on hand-built states that break them
(`tests/audit_test.cpp`), and the one regression test of the double-move defect below was shown red against
the unfixed engine before it went green.

## 1. Memory ownership

**Design.** Every object has one owner and is destroyed with it:

| Owner | Owns | How |
|---|---|---|
| `Simulation` | robots, `World` (layout + occupancy), `TaskQueue`, traffic state | members, by value |
| `Simulation` | the planner, the dispatcher, the planning-thread executor | `std::unique_ptr` (interfaces, so they are injected) |
| `PathBatchExecutor` | its planners; a private copy of the layout | `unique_ptr`; `shared_ptr<const Warehouse>` (immutable, read by workers during a batch only) |
| `SimulationRunner` | the `Simulation`, the command queue and log, the metrics collector, the recorder, its worker thread | members; the thread is joined in the destructor |
| `ThreadPool` | its threads | joined in the destructor and in `shutdown()` |
| `SdlWindow` | the SDL window, renderer and library | `unique_ptr` with SDL's destroy functions as deleters |

**Checks.**

- A search of `src/` and `apps/` for `new`, `delete`, `malloc`, `calloc`, `realloc`, `free`, `const_cast`,
  `reinterpret_cast` and C-style casts finds none outside comments. (One hit, a local variable that is
  called `free`, is not a call.) `shared_ptr` is used twice, for the immutable layout copy and for the
  state of one `parallelFor` call; both are read-only or internally synchronised.
- Non-owning raw pointers exist only as borrowed arguments that live for one call (for example the pusher
  of `stepAside`, or a `Task*` looked up in a queue).
- ASan + UBSan: the whole suite (1 531 tests at the first run) clean, including the recorder, replay and
  runner tests that start and stop threads over and over. LeakSanitizer is not available with Apple's
  clang, so **leaks were not checked by tool**; CI runs it on Linux.
- clang-tidy (`bugprone-*`, `performance-*`, `clang-analyzer-*`, `concurrency-*` and others, see
  `.clang-tidy`) over all targets: 21 findings on the first real run and one more in the regression
  benchmark, written after it; all fixed (none suppressed). None was a wrong result. The two that touched ownership: a `std::move` of a trivially copyable value that did
  nothing (removed), and `asArray` in the recording reader, which returned a reference to its argument
  (`bugprone-return-const-ref-from-parameter`: dangling if a caller passed a temporary; no caller did).
  It is now a function that checks and returns nothing.

**Residual risk.** Leaks, as above; and a move-after-use in code the tests do not run.

## 2. Concurrency

**Design.** Exactly two components start threads and no others: `SimulationRunner` (one worker, one mutex,
value-copy snapshots for readers) and `core::ThreadPool` (bounded threads and queue; `shutdown()` accepts
no new task, wakes blocked producers, runs what it accepted and joins). Planning threads answer only pure
questions (a function of the layout and two cells); a worker never touches a `Simulation`; answers are
applied on the stepping thread in the order the questions were asked. One global atomic exists, the version
counter of layouts, which only has to be unique.

**Checks.**

- ThreadSanitizer: the whole suite (1 531 tests) clean, 0 reports. This includes the pool tests, which were
  shown, when they were written, to catch deliberately broken pools (a producer never woken, workers that
  drop tasks, two threads joining one worker).
- The thread count changes no state, checksum or recording: `PlanningThreadsTest`, the replays of the random
  command runs on 0 and on 2 threads, the example tests, the scaling benchmark's self-check and the CLI run
  with `--threads 2` against `--threads 0` and its own replay.
- ASan/UBSan on the same tests.

**Found and fixed.** `SimulationRunner::warehouse()` returned a reference to the layout the runner was built
with, documented as never changing. Since the commands `PlaceObstacle` and `ClearObstacle` exist, that was
false: a drawing built from it would show robots driving through a wall the engine had put up. It now
returns the current layout, copied under the lock. Test:
`RunnerCommandsTest.TheRunnersLayoutFollowsObstaclesPutUpAndTakenDownAndARestartUndoesThem`, which fails
against the old behaviour.

**Not fixed.** `view::FleetController` keeps its own copy of the layout, from when it was built, to judge
clicks. If the layout is edited while a window is open its judgement is out of date. The GUI has no key
that edits the layout, so only code that posts the commands can reach it, and the engine refuses a bad
task anyway (it is the authority); the cost is a misleading message.

**Residual risk.** The tests ran with at most two worker threads and not under load from other
programs; TSan finds races that happen in the run, not all possible ones. The GUI's real event loop (SDL
threads, a real window) was run with the dummy video driver only.

## 3. Event ordering

**Design.** One step is a fixed sequence: a task may be made, tasks are assigned, robots take their turns
(the ones with a temporary priority boost first, then by id), stuck robots are looked after, the clock
advances. Operator commands are applied by the runner's worker between two steps, in the order they were
posted, and recorded with the tick. Tasks are made with consecutive ids and are never made in the future.

**Checks.** The auditor checks, on every state: the tick advances by exactly one; tasks are numbered 1, 2, 3
... in order and created in non-decreasing time; no counter (completed, failed, cancelled, a robot's
deliveries) goes down; the number of robots does not change; every task is accounted for (completed +
failed + cancelled + open = made). Replay checks the same thing against a recording: every command is
accepted or refused as recorded, every task event happens at its tick, and checkpoints of the state match
(`EndToEndTest.AnyMixOfCommandsLeavesARunSafeAndItsRecordingReplaysExactly`: a random mix of every kind of
command, 9 runs of 600 ticks, replayed on 0 and 2 threads).

**Found and fixed.** The double move below was an ordering defect inside a step.

**Residual risk.** The order of robots within a step is by id, with boosts; that is a rule of the model
and not a property that can be checked against anything else.

## 4. Collision safety

**Design.** Two layers, and the lower one is the authority. `World` holds are the truth: a robot only drives
into cells it has claimed, one holder per cell, and a robot cannot stand on a wall or off the map. The
reservation table and cooperative planning above it only decide ahead of time how many cells a robot may
claim; safety does not depend on them (`reservations = false` and `cooperative = false` keep working and the
tests run them).

**Checks.** On every state the auditor checks: no cell is held by two robots; every robot holds the cell it
is on; every held cell is walkable and the held cells of a robot are a chain of neighbours; no two
robots' centres are closer than a cell; no two robots exchange cells in one tick; no robot moves more than a
cell in one tick. Where it ran:

| Run | States audited | Violations |
|---|---:|---:|
| scenario campaign: 10 layouts x 4 fleets (1 to 64) x 3 loads x 3 seeds x 2 000 ticks | 666 333 | 0 (after the fix below) |
| every layout x 1, 4, 16 robots x 3 loads x 2 seeds, 500 ticks | 27 054 | 0 |
| 100 and 256 robots on `large`, 300 ticks | 1 204 | 0 |
| random mix of all commands, 3 layouts x 3 seeds, 600 ticks | 5 409 | 0 |
| random halls, up to 10 robots, 40 runs of 300 ticks | 12 040 | 0 |
| 6 example maps x 2 fleets, 600 ticks | 7 212 | 0 |
| each of the 6 examples through `--audit`, in the test suite | 9 006 | 0 |

(The counts are runs x (ticks + 1), worked out from the run lengths in the tests, since a run observes its
initial state and every step; the first row is what the campaign script printed.) Not one overlap, swap or doubly held cell was found
in any of them. The audited properties are the ones that mean "no collision" in a model where robots occupy
a cell each.

**Found and fixed.** Five campaign runs (of 333) showed robots that moved two cells in one tick: a blocked
robot with a higher id could push aside a blocker that had already taken its own step in that tick
(`resolveBlock`; `recoverDeadlock` already refused such a robot, this path did not). The size of the
excess was 0.005 to 0.25 of a cell on top of the permitted one, and the logical cell changed by two. It was not a
collision in the reports read. Fix: a robot that moved in this step is not pushed aside in it; the engine
revision is now 2. Test: `EndToEndTest.ARobotThatHasMovedIsNeverPushedAsideInTheSameTick` (three of the
five runs, red before, green after). After the fix: the whole campaign, 0 violations; the committed
baseline, the fleet benchmark and every pinned checksum unchanged.

**What the audit does not claim.** Safety is not progress. In 111 of 333 campaign runs at least one task
ended failed, and fleets much bigger than the work can use do worse than smaller ones
(docs/benchmarks.md): the robots never collide, but they can block each other until a task is given up. That
is a limit of the planner, documented in docs/architecture.md and in the README. It is not a safety
violation, and it is the largest finding of the release.

**Residual risk.** The auditor checks what it checks, on the runs that were made: it samples, it does
not prove. Two robots could in principle meet in a state no run has reached.

## 5. Numerical stability

**Design.** Decisions of the simulation use integers (cells, ticks, battery units, ids), except the motion
model (speed, progress along a segment, heading), which is closed-form `double` arithmetic compared with an
epsilon, so that its results do not depend on the length of the step. The state checksum hashes the bit
patterns of those doubles, so the arithmetic has to be exact IEEE: `-ffp-contract=off` in every target, no
fast-math anywhere, and no transcendental libm function (`sin`, `cos`, `pow`, `atan2`, ...) in `src/`
outside `view` (which only draws). A search finds `sqrt`, `fmod`, `floor` and `ceil` in the simulation,
which are correctly rounded, and `sin`, `cos` and `pow` in `view` only.

**Checks.**

- The auditor, on every state: positions, headings and speeds are finite; speed is between 0 and 1 cell a tick;
  a robot's centre is inside the map and no more than a cell from the centre of the cell it is logically in;
  battery between empty and capacity. Over the 666 333 campaign states and the suite: no violation.
- UBSan with `float-divide-by-zero` and `float-cast-overflow` and no recovery, over the whole suite: clean.
  That a finding really is fatal was shown with a scratch program that divides by zero, overflows an `int`,
  casts `1e30` to `int` and overruns a heap buffer: each aborts with exit status 134 under the flags the
  presets use.
- Step-length invariance (`MotionPropertyTest`, `RobotTimestepTest`) and render-rate invariance
  (`FrameRateTest`) are properties tested in the suite.
- Metrics: a figure with a zero denominator is undefined (an empty CSV field, `n/a`), never 0 or NaN; the
  tests cover empty runs.
- Recording and replay: the pinned checksum of a scenario and a recording made earlier replay exactly.

**Residual risk.** Everything above was run on one platform (Apple clang 21, arm64, libc++). Whether the
checksums agree on x86-64, with GCC or with MSVC is **unverified**; the pinned tests are what would say
so. `-ffp-contract=off` is a GCC and clang flag; MSVC's default floating-point mode was not examined, and
the MSVC job is marked experimental for that reason.

## Defects found by this audit and the release checks

| # | Found by | What | Status |
|---|---|---|---|
| 1 | first real CMake configure | the GUI preset could not be configured at all: the editor program and the editor library were both called `warehousesim_editor` | fixed (the program's target is `warehousesim_editor_app`, its file is still `warehousesim_editor`) |
| 2 | scenario campaign + auditor | a robot could move two cells in one tick (above) | fixed, regression test, engine revision 2 |
| 3 | writing the end-to-end tests | `SimulationRunner::warehouse()` did not follow layout commands | fixed, regression test |
| 4 | real clang-tidy | 22 findings (moves with no effect, `reserve`, a dangling-prone helper, a sort the analyser could not follow, widening multiplications, a rounding cast in the new benchmark) | fixed |
| 5 | reading the presets | sanitizers reported and carried on, so a test could pass with the report scrolled away | `-fno-sanitize-recover=all` |
| 6 | reading the packaging | a packaged GUI would need `libSDL3` beside it | SDL3 is linked statically when it is downloaded |
| 7 | scenario campaign | fleets bigger than the work can use do worse than smaller ones; 111 of 333 runs have failed tasks | **not fixed**: documented; likely cause is that idle robots are not parked |
| 8 | this audit | `FleetController` judges clicks on a copy of the layout from when it was built | **not fixed**: documented |

## What was not checked

- Linux and Windows builds, GCC, MSVC, the CI jobs themselves, clang-tidy against Linux headers, and
  `clang-format` from the Ubuntu package (CI installs the pinned pip version instead). Nothing outside macOS
  was run.
- LeakSanitizer (not available with Apple's clang).
- A real window: mouse and keyboard mapping, SDL's rendering, on-screen text. The GUI and the editor ran
  with SDL's dummy video driver for 30 frames, and the test suite covers everything they decide.
- Cross-platform agreement of checksums and recordings.
- Maps beyond 512 x 512 cells and fleets beyond 512 robots (both refused by design), and fleets between 256
  and 512 robots (256 is the largest run in the benchmarks and tests).
- Docker was not involved.
