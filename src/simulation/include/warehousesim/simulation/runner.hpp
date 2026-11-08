#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::simulation {

// Drives a Simulation on a background thread at a fixed wall-clock rate and publishes
// snapshots to other threads. The Simulation is only touched while holding the mutex,
// so readers (e.g. the renderer) never see a half-updated state. The destructor stops
// and joins the worker.
class SimulationRunner {
 public:
  SimulationRunner(Simulation simulation, std::chrono::milliseconds tickInterval);
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

  [[nodiscard]] Snapshot snapshot() const;
  // Blocks until a snapshot at or beyond `tick` is published; false on timeout.
  [[nodiscard]] bool waitForTick(core::Tick tick, std::chrono::milliseconds timeout) const;

  // The layout never changes after construction, so this is safe to read from any thread.
  [[nodiscard]] const world::Warehouse& warehouse() const noexcept { return warehouse_; }

 private:
  void loop();
  [[nodiscard]] bool shouldStep() const;  // requires mutex_ held

  Simulation simulation_;
  const world::Warehouse warehouse_;
  const std::chrono::milliseconds interval_;

  mutable std::mutex mutex_;
  std::condition_variable wake_;
  mutable std::condition_variable published_;
  Snapshot latest_;
  std::optional<core::Tick> untilTick_;
  bool stopRequested_ = false;
  bool paused_ = false;
  std::thread worker_;
};

}  // namespace warehousesim::simulation
