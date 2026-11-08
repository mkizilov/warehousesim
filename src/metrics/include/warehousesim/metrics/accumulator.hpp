#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "warehousesim/metrics/fleet_metrics.hpp"
#include "warehousesim/metrics/observation.hpp"

namespace warehousesim::metrics {

// Turns a series of observations into a FleetMetrics. It is told the state at the start and then
// the state after every tick, in order; it keeps counts only, never the observations, so memory use
// does not grow with the length of the run (apart from the bounded samples).
//
// Each tick's activity is the one the robot was in at the end of that tick. The interval is the
// ticks after the start observation: a run that is observed from tick 0 covers every tick.
class MetricsAccumulator {
 public:
  // Throws std::invalid_argument for invalid options.
  MetricsAccumulator(MetricsOptions options, const TickObservation& start);

  // Adds the tick after the last one seen. Throws std::invalid_argument, changing nothing, if the
  // tick is not exactly the next, if the robots are not those of the start (same ids, same order),
  // or if a task ended before it was created: a metric on a gap or a mismatch would be a guess.
  void add(const TickObservation& observation);

  [[nodiscard]] core::Tick ticksObserved() const noexcept { return metrics_.ticks; }
  [[nodiscard]] core::Tick lastTick() const noexcept { return lastTick_; }
  // The totals so far. Can be called at any time, as often as wanted.
  [[nodiscard]] const FleetMetrics& report() const noexcept { return metrics_; }

 private:
  struct Window {
    core::Tick ticks = 0;
    std::uint64_t completed = 0;
    std::uint64_t busy = 0;
    std::uint64_t service = 0;
    std::uint64_t waiting = 0;
  };

  void validate(const TickObservation& observation) const;
  void countCollisions(const TickObservation& observation);
  void closeWindow(core::Tick tick, std::uint64_t tasksOpen);

  FleetMetrics metrics_;
  core::Tick lastTick_ = 0;
  SystemCounters baseline_;                     // the counters at the start
  SystemCounters last_;                         // ... and at the last observation
  std::vector<std::uint64_t> baseCells_;        // by robot, at the start
  std::vector<std::uint64_t> baseDeliveries_;   // by robot, at the start
  std::vector<core::Point> previousPositions_;  // by robot, as of the last observation
  std::vector<bool> inConflict_;                // by robot: held back by a robot, last observation
  Window window_;
};

}  // namespace warehousesim::metrics
