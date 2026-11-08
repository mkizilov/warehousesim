#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/fixed_timestep.hpp"
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
};

// Drives a Simulation on a background thread with a fixed timestep and publishes snapshots
// to other threads. One simulation step is owed per `stepDuration` of real time at speed 1.0;
// the step size never depends on how often anyone reads snapshots or draws frames, so a slow
// or stalled renderer cannot change the simulation, only what it shows.
//
// The Simulation is only touched while holding the mutex, so readers never see a
// half-updated state. The destructor stops and joins the worker.
class SimulationRunner {
 public:
  // Throws std::invalid_argument unless stepDuration is positive.
  SimulationRunner(Simulation simulation, std::chrono::milliseconds stepDuration);
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

  Simulation simulation_;
  const world::Warehouse warehouse_;

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
  std::uint64_t controlEpoch_ = 0;  // bumped by setPaused/setSpeed to cut a sleep short
  bool stopRequested_ = false;
  bool paused_ = false;
  std::thread worker_;
};

}  // namespace warehousesim::simulation
