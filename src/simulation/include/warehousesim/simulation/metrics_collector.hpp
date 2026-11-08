#pragma once

#include <cstddef>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/metrics/accumulator.hpp"
#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::simulation {

// Measures a Simulation from the outside: after every step it reads the simulation's public state
// and hands it to a metrics::MetricsAccumulator. It only ever reads (the simulation is taken by
// const reference), so a run measured and a run not measured are the same run, step for step.
//
// Construct it on the simulation as it is at the start of the interval to measure (tick 0 for a
// whole run), then call observe() after every step, with no step missed: a gap throws.
//
// How the simulation maps onto the metrics' vocabulary:
//   Navigating, given its next cell       -> Moving
//   Navigating, refused its next cell     -> Waiting (cause: the reservation table's conflict, or a
//                                            robot standing in the cell)
//   Loading, Unloading                    -> Handling
//   Charging / Idle / Unavailable         -> Charging / Idle / Unavailable
// A robot that is turning or accelerating is Moving: it has not been refused anything.
class MetricsCollector {
 public:
  MetricsCollector(const Simulation& simulation, metrics::MetricsOptions options = {});

  // Records the step that has just been run. Throws std::invalid_argument, changing nothing, unless
  // the simulation is exactly one tick past the last observation.
  void observe(const Simulation& simulation);

  [[nodiscard]] const metrics::FleetMetrics& report() const noexcept {
    return accumulator_.report();
  }

 private:
  [[nodiscard]] metrics::TickObservation read(const Simulation& simulation);

  metrics::MetricsAccumulator accumulator_;
  std::size_t knownTasks_ = 0;           // tasks seen so far (ids 1..knownTasks_)
  std::vector<core::TaskId> openTasks_;  // of those, the ones not finished at the last observation
};

// The drain of the simulation's own robots as the metrics' EnergyModel (see robots::RobotSpec: idle
// drain per tick, cost per cell entered, handling drain per tick). The collector does not use it by
// default: the estimate uses MetricsOptions::energy, one stated model for every layout, so that a
// hall whose robots have no battery model (the demo hall) is not reported as using no energy. Pass
// MetricsOptions{.., energy = energyModelOf(sim)} to measure with the simulation's own.
[[nodiscard]] metrics::EnergyModel energyModelOf(const Simulation& simulation);

}  // namespace warehousesim::simulation
