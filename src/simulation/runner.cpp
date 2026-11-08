#include "warehousesim/simulation/runner.hpp"

#include <stdexcept>
#include <utility>

namespace warehousesim::simulation {

SimulationRunner::SimulationRunner(Simulation simulation, std::chrono::milliseconds tickInterval)
    : simulation_(std::move(simulation)),
      warehouse_(simulation_.warehouse()),
      interval_(tickInterval),
      latest_(simulation_.snapshot()) {}

SimulationRunner::~SimulationRunner() { stop(); }

void SimulationRunner::start(std::optional<core::Tick> untilTick) {
  const std::scoped_lock lock(mutex_);
  if (worker_.joinable()) {
    throw std::logic_error("runner already started");
  }
  stopRequested_ = false;
  untilTick_ = untilTick;
  worker_ = std::thread([this] { loop(); });
}

void SimulationRunner::stop() {
  std::thread worker;
  {
    const std::scoped_lock lock(mutex_);
    stopRequested_ = true;
    worker = std::move(worker_);
  }
  wake_.notify_all();
  if (worker.joinable()) {
    worker.join();
  }
}

void SimulationRunner::setPaused(bool paused) {
  {
    const std::scoped_lock lock(mutex_);
    paused_ = paused;
  }
  wake_.notify_all();
}

Snapshot SimulationRunner::snapshot() const {
  const std::scoped_lock lock(mutex_);
  return latest_;
}

bool SimulationRunner::waitForTick(core::Tick tick, std::chrono::milliseconds timeout) const {
  std::unique_lock lock(mutex_);
  return published_.wait_for(lock, timeout, [&] { return latest_.tick >= tick; });
}

bool SimulationRunner::shouldStep() const {
  return !paused_ && (!untilTick_ || simulation_.tick() < *untilTick_);
}

void SimulationRunner::loop() {
  std::unique_lock lock(mutex_);
  while (true) {
    wake_.wait(lock, [this] { return stopRequested_ || shouldStep(); });
    if (stopRequested_) {
      return;
    }
    // Sleep one interval, waking early if stopped or paused in the meantime.
    const bool interrupted =
        wake_.wait_for(lock, interval_, [this] { return stopRequested_ || paused_; });
    if (interrupted) {
      continue;
    }
    simulation_.step();
    latest_ = simulation_.snapshot();
    published_.notify_all();
  }
}

}  // namespace warehousesim::simulation
