#include "warehousesim/simulation/runner.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace warehousesim::simulation {

namespace {

using SteadyClock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

// Never spin: even if a step is already due, give other threads a moment on the mutex.
constexpr Seconds kMinSleep{0.0002};

}  // namespace

SimulationRunner::SimulationRunner(Simulation simulation, std::chrono::milliseconds stepDuration)
    : simulation_(std::move(simulation)),
      warehouse_(simulation_.warehouse()),
      stepper_(std::chrono::duration_cast<Seconds>(stepDuration).count()),
      latest_(simulation_.snapshot()),
      previous_(latest_) {}

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
    if (paused && !paused_ && ticking_) {
      // Freeze the picture exactly where it is now.
      frozenAlpha_ =
          stepper_.alphaAfter(std::max(0.0, Seconds(SteadyClock::now() - lastAdvance_).count()));
    }
    paused_ = paused;
    ++controlEpoch_;
  }
  wake_.notify_all();
}

void SimulationRunner::setSpeed(double speed) {
  {
    const std::scoped_lock lock(mutex_);
    stepper_.setSpeed(speed);
    ++controlEpoch_;
  }
  wake_.notify_all();
}

double SimulationRunner::speed() const {
  const std::scoped_lock lock(mutex_);
  return stepper_.speed();
}

Snapshot SimulationRunner::snapshot() const {
  const std::scoped_lock lock(mutex_);
  return latest_;
}

FrameState SimulationRunner::frame() const {
  const std::scoped_lock lock(mutex_);
  double alpha = frozenAlpha_;
  if (ticking_ && !paused_) {
    alpha = stepper_.alphaAfter(std::max(0.0, Seconds(SteadyClock::now() - lastAdvance_).count()));
  }
  return {previous_, latest_, alpha};
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
    if (!shouldStep()) {
      if (ticking_ && !paused_) {
        frozenAlpha_ = 1.0;  // out of steps to take: the display settles on the final state
      }
      ticking_ = false;
      wake_.wait(lock, [this] { return stopRequested_ || shouldStep(); });
    }
    if (stopRequested_) {
      return;
    }
    if (!ticking_) {
      // Time spent paused (or finished) must not be replayed as a burst of steps, but the
      // display carries on from where it stood.
      stepper_.reset(frozenAlpha_);
      lastAdvance_ = SteadyClock::now();
      ticking_ = true;
    }

    // Sleep until the next step is due, waking early if stopped, paused or re-speeded.
    const std::uint64_t epoch = controlEpoch_;
    const Seconds sleep = std::max(kMinSleep, Seconds(stepper_.secondsUntilNextStep()));
    wake_.wait_for(lock, std::chrono::duration_cast<SteadyClock::duration>(sleep),
                   [&] { return stopRequested_ || controlEpoch_ != epoch; });
    if (stopRequested_) {
      return;
    }

    const SteadyClock::time_point now = SteadyClock::now();
    const Seconds elapsed = now - lastAdvance_;
    if (paused_) {
      continue;  // setPaused already froze the display
    }
    lastAdvance_ = now;
    std::uint32_t due = stepper_.advance(elapsed.count());
    bool stepped = false;
    Snapshot beforeLastStep;
    while (due > 0 && shouldStep()) {
      beforeLastStep = simulation_.snapshot();
      simulation_.step();
      stepped = true;
      --due;
    }
    if (stepped) {
      previous_ = std::move(beforeLastStep);
      latest_ = simulation_.snapshot();
      published_.notify_all();
    }
  }
}

}  // namespace warehousesim::simulation
