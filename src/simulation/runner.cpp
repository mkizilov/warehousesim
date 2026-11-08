#include "warehousesim/simulation/runner.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace warehousesim::simulation {

namespace {

using SteadyClock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

// Never spin: even if a step is already due, give other threads a moment on the mutex.
constexpr Seconds kMinSleep{0.0002};

}  // namespace

SimulationRunner::SimulationRunner(Simulation simulation, std::chrono::milliseconds stepDuration,
                                   ScenarioFactory scenario)
    : SimulationRunner(std::move(simulation), stepDuration, std::move(scenario),
                       metrics::MetricsOptions()) {}

SimulationRunner::SimulationRunner(Simulation simulation, std::chrono::milliseconds stepDuration,
                                   ScenarioFactory scenario, metrics::MetricsOptions metricsOptions)
    : simulation_(std::move(simulation)),
      warehouse_(simulation_.warehouse()),
      scenario_(std::move(scenario)),
      metricsOptions_(metricsOptions),
      collector_(simulation_, metricsOptions),
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

bool SimulationRunner::post(Command command) {
  {
    const std::scoped_lock lock(mutex_);
    if (commands_.size() >= kMaxQueuedCommands) {
      return false;
    }
    commands_.push_back(std::move(command));
    ++controlEpoch_;
  }
  wake_.notify_all();
  return true;
}

std::vector<CommandRecord> SimulationRunner::commandLog(std::size_t from) const {
  const std::scoped_lock lock(mutex_);
  if (from >= log_.size()) {
    return {};
  }
  return {log_.begin() + static_cast<std::ptrdiff_t>(from), log_.end()};
}

std::uint64_t SimulationRunner::seed() const {
  const std::scoped_lock lock(mutex_);
  return simulation_.config().seed;
}

void SimulationRunner::startRecording(core::Tick checkpointInterval) {
  const std::scoped_lock lock(mutex_);
  journal_.emplace(simulation_, checkpointInterval);
  checkpointInterval_ = checkpointInterval;
}

std::optional<Recording> SimulationRunner::recording() const {
  const std::scoped_lock lock(mutex_);
  if (!journal_) {
    return std::nullopt;
  }
  return journal_->recording(simulation_);
}

metrics::FleetMetrics SimulationRunner::metrics() const {
  const std::scoped_lock lock(mutex_);
  return collector_.report();
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
  return {previous_, latest_, alpha, restarts_};
}

bool SimulationRunner::waitForTick(core::Tick tick, std::chrono::milliseconds timeout) const {
  std::unique_lock lock(mutex_);
  return published_.wait_for(lock, timeout, [&] { return latest_.tick >= tick; });
}

bool SimulationRunner::shouldStep() const {
  return !paused_ && (!untilTick_ || simulation_.tick() < *untilTick_);
}

CommandOutcome SimulationRunner::restart(const ResetScenario& command) {
  if (!scenario_) {
    return {false, "this run cannot be restarted: there is no scenario to rebuild", std::nullopt};
  }
  const std::uint64_t seed = command.seed.value_or(simulation_.config().seed);
  try {
    Simulation fresh = scenario_(seed);
    if (fresh.warehouse().toRows() != warehouse_.toRows()) {
      return {false, "the scenario would build another layout; not restarted", std::nullopt};
    }
    simulation_ = std::move(fresh);
    collector_ = MetricsCollector(simulation_, metricsOptions_);
  } catch (const std::exception& error) {
    return {false, std::string("could not restart: ") + error.what(), std::nullopt};
  }
  ++restarts_;
  return {true, "scenario restarted (seed " + std::to_string(seed) + ")", std::nullopt};
}

CommandOutcome SimulationRunner::execute(const Command& command) {
  if (const auto* reset = std::get_if<ResetScenario>(&command)) {
    return restart(*reset);
  }
  return simulation::apply(simulation_, command);  // qualified: std::apply would be found too
}

bool SimulationRunner::applyQueued() {
  if (commands_.empty()) {
    return false;
  }
  bool restarted = false;
  while (!commands_.empty()) {
    Command command = std::move(commands_.front());
    commands_.pop_front();
    CommandOutcome outcome = execute(command);
    if (std::holds_alternative<ResetScenario>(command)) {
      if (outcome.accepted) {
        log_.clear();
        restarted = true;
        if (journal_) {
          try {
            journal_.emplace(simulation_, checkpointInterval_);
          } catch (const std::exception&) {
            journal_.reset();  // the new run is not one that can be recorded
          }
        }
      }
    } else if (journal_) {
      journal_->commandApplied(simulation_, command, outcome.accepted);
    }
    log_.push_back({simulation_.tick(), std::move(command), std::move(outcome)});
  }
  latest_ = simulation_.snapshot();
  if (restarted) {
    // A new run: nothing of the old one is left to interpolate from, and the display starts at
    // the start of a step again.
    previous_ = latest_;
    frozenAlpha_ = 0.0;
    ticking_ = false;
  }
  published_.notify_all();
  return restarted;
}

void SimulationRunner::loop() {
  std::unique_lock lock(mutex_);
  while (true) {
    if (stopRequested_) {
      return;
    }
    applyQueued();
    if (!shouldStep()) {
      if (ticking_ && !paused_) {
        frozenAlpha_ = 1.0;  // out of steps to take: the display settles on the final state
      }
      ticking_ = false;
      wake_.wait(lock, [this] { return stopRequested_ || shouldStep() || !commands_.empty(); });
      continue;
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
    if (applyQueued()) {
      continue;  // restarted: begin again from the new simulation
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
      collector_.observe(simulation_);
      if (journal_) {
        journal_->stepTaken(simulation_);
      }
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
