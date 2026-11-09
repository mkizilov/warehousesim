#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/fixed_timestep.hpp"
#include "warehousesim/metrics/fleet_metrics.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/recording.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::simulation {

// What to draw: the last two simulation states and how far between them the display has got.
// The simulation only moves in whole steps, so a renderer shows the state `alpha` of the way
// from `previous` to `latest` (view::blend does that), which makes motion look smooth at any
// frame rate without the frame rate ever reaching the simulation.
struct FrameState {
  Snapshot previous;  // one step before `latest` (the same as `latest` before the first step)
  Snapshot latest;
  double alpha = 0.0;  // in [0, 1]: 0 shows `previous`, 1 shows `latest`
  // How many times the scenario has been restarted. Ticks, task ids and the command log start over
  // at each restart, so a view that remembers any of them forgets them when this changes.
  std::uint64_t epoch = 0;
};

// Drives a Simulation on a background thread with a fixed timestep and publishes snapshots
// to other threads. One simulation step is owed per `stepDuration` of real time at speed 1.0;
// the step size never depends on how often anyone reads snapshots or draws frames, so a slow
// or stalled renderer cannot change the simulation, only what it shows.
//
// The Simulation is only touched while holding the mutex, so readers never see a
// half-updated state. The destructor stops and joins the worker.
//
// Operator interfaces change the simulation only by posting Commands. The worker applies them one
// at a time, in the order posted, between two steps (also while paused, when the tick does not
// move), and records each with the tick it was applied at, so the whole run can be replayed from
// the scenario and that record. Pause, resume and speed are not commands: they decide when steps
// happen, not what a step does.
class SimulationRunner {
 public:
  // At most this many commands wait to be applied; post() refuses more.
  static constexpr std::size_t kMaxQueuedCommands = 256;

  // `scenario` rebuilds the scenario for ResetScenario and must build the same layout every time;
  // without one, a reset is refused. Throws std::invalid_argument unless stepDuration is positive.
  SimulationRunner(Simulation simulation, std::chrono::milliseconds stepDuration,
                   ScenarioFactory scenario = nullptr);
  // The same, with the metrics measured in the time scale and energy model given.
  SimulationRunner(Simulation simulation, std::chrono::milliseconds stepDuration,
                   ScenarioFactory scenario, metrics::MetricsOptions metricsOptions);
  ~SimulationRunner();

  SimulationRunner(const SimulationRunner&) = delete;
  SimulationRunner& operator=(const SimulationRunner&) = delete;
  SimulationRunner(SimulationRunner&&) = delete;
  SimulationRunner& operator=(SimulationRunner&&) = delete;

  // Starts the worker. It stops stepping once the simulation reaches `untilTick`, if given.
  // Throws std::logic_error if already started.
  void start(std::optional<core::Tick> untilTick = std::nullopt);
  // Idempotent; blocks until the worker has exited.
  void stop();

  void setPaused(bool paused);
  // Playback speed multiplier, clamped to [FixedTimestep::kMinSpeed, kMaxSpeed]. Throws
  // std::invalid_argument for non-finite or non-positive values.
  void setSpeed(double speed);
  [[nodiscard]] double speed() const;

  // Queues a command for the worker; returns at once. False, with nothing queued, if too many
  // are waiting. Commands queued when the runner is stopped (or before it is started) wait until
  // it runs. Safe to call from any thread.
  bool post(Command command);
  // Every command applied since the run started or was last restarted, oldest first, from index
  // `from` on (nothing if `from` is past the end). After an accepted ResetScenario the record
  // starts over with that command as its first entry.
  [[nodiscard]] std::vector<CommandRecord> commandLog(std::size_t from = 0) const;
  // The seed of the run in progress: replaying commandLog() on the scenario built from it gives
  // the state of the run at any tick.
  [[nodiscard]] std::uint64_t seed() const;

  // Starts keeping a recording of the run (see recording.hpp): every command applied and every step
  // taken from now on is written down, with a checksum of the state every `checkpointInterval`
  // ticks, so that recording() can hand over a file that replays the run. Only possible at the very
  // start (tick 0, no task made yet), and for a simulation with the default planner and
  // dispatcher: throws std::invalid_argument otherwise. A restart (ResetScenario) starts the
  // recording over on the new run, as it does the command log; if the new run cannot be recorded,
  // recording stops. Costs a copy of the layout and a few bytes per command and task.
  void startRecording(core::Tick checkpointInterval = kDefaultCheckpointInterval);
  // The recording of the run in progress, as of now, or nothing if none is being kept. A copy, safe
  // to ask from any thread.
  [[nodiscard]] std::optional<Recording> recording() const;

  // Plans on `workers` threads besides the worker (see Simulation::usePlanningThreads), now and
  // after every restart. Changes nothing a run computes. Safe to call at any time from any thread;
  // throws std::invalid_argument for a simulation with a planner of its own.
  void usePlanningThreads(std::size_t workers);
  // What the planning threads have been asked in the run in progress (it starts over with a
  // restart); nothing if there are none.
  [[nodiscard]] std::optional<planning::PathBatchExecutor::Stats> planningBatchStats() const;

  // The performance figures of the run in progress (since the start, or the last restart): every
  // step the worker has taken, none missed. A copy, safe to read from any thread. Measuring only
  // reads the simulation; a runner's state is the same with or without anyone asking.
  [[nodiscard]] metrics::FleetMetrics metrics() const;

  [[nodiscard]] Snapshot snapshot() const;
  // The state to draw right now. While stepping, alpha grows with real time since the last step
  // (scaled by the playback speed); while paused or finished it stays where it was, so the picture
  // holds still, and it carries on from there when playback resumes.
  [[nodiscard]] FrameState frame() const;
  // Blocks until a snapshot at or beyond `tick` is published; false on timeout.
  [[nodiscard]] bool waitForTick(core::Tick tick, std::chrono::milliseconds timeout) const;

  // The layout never changes after construction, so this is safe to read from any thread.
  [[nodiscard]] const world::Warehouse& warehouse() const noexcept { return warehouse_; }

 private:
  void loop();
  [[nodiscard]] bool shouldStep() const;  // requires mutex_ held
  // Applies the queued commands at the current tick and publishes the result. True if the
  // scenario was restarted. Requires mutex_ held.
  bool applyQueued();
  [[nodiscard]] CommandOutcome execute(const Command& command);  // requires mutex_ held
  [[nodiscard]] CommandOutcome restart(const ResetScenario& command);

  Simulation simulation_;
  const world::Warehouse warehouse_;
  const ScenarioFactory scenario_;
  const metrics::MetricsOptions metricsOptions_;
  MetricsCollector collector_;

  mutable std::mutex mutex_;
  std::condition_variable wake_;
  mutable std::condition_variable published_;
  core::FixedTimestep stepper_;
  Snapshot latest_;
  Snapshot previous_;
  std::chrono::steady_clock::time_point lastAdvance_;  // when the stepper last took elapsed time
  double frozenAlpha_ = 0.0;                           // where the display stands while not ticking
  bool ticking_ = false;                               // the worker is currently stepping on time
  std::optional<core::Tick> untilTick_;
  std::uint64_t controlEpoch_ = 0;     // bumped by setPaused/setSpeed/post to cut a sleep short
  std::deque<Command> commands_;       // posted, not yet applied
  std::vector<CommandRecord> log_;     // applied during the current run
  std::optional<RunJournal> journal_;  // set while a recording is being kept
  core::Tick checkpointInterval_ = kDefaultCheckpointInterval;
  std::uint64_t restarts_ = 0;
  std::size_t planningWorkers_ = 0;
  bool stopRequested_ = false;
  bool paused_ = false;
  std::thread worker_;
};

}  // namespace warehousesim::simulation
