#include "warehousesim/metrics/fleet_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace warehousesim::metrics {

namespace {

constexpr double kSecondsPerHour = 3600.0;

std::optional<double> ratio(std::uint64_t numerator, std::uint64_t denominator) noexcept {
  if (denominator == 0) {
    return std::nullopt;
  }
  return static_cast<double>(numerator) / static_cast<double>(denominator);
}

}  // namespace

void MetricsOptions::validate() const {
  if (!std::isfinite(secondsPerTick) || secondsPerTick <= 0.0) {
    throw std::invalid_argument("secondsPerTick must be finite and positive");
  }
}

std::uint64_t TimeBudget::total() const noexcept {
  std::uint64_t sum = 0;
  for (const std::uint64_t value : ticks) {
    sum += value;
  }
  return sum;
}

double FleetMetrics::simulatedSeconds() const noexcept {
  return static_cast<double>(ticks) * options.secondsPerTick;
}

double FleetMetrics::simulatedHours() const noexcept {
  return simulatedSeconds() / kSecondsPerHour;
}

std::uint64_t FleetMetrics::robotTicks() const noexcept {
  std::uint64_t sum = 0;
  for (const RobotMetrics& robot : robots) {
    sum += robot.budget.total();
  }
  return sum;
}

std::uint64_t FleetMetrics::ticksIn(Activity activity) const noexcept {
  std::uint64_t sum = 0;
  for (const RobotMetrics& robot : robots) {
    sum += robot.budget.in(activity);
  }
  return sum;
}

std::optional<double> FleetMetrics::fraction(Activity activity) const noexcept {
  return ratio(ticksIn(activity), robotTicks());
}

std::optional<double> FleetMetrics::utilization() const noexcept {
  const std::uint64_t busy =
      ticksIn(Activity::Moving) + ticksIn(Activity::Waiting) + ticksIn(Activity::Handling);
  return ratio(busy, robotTicks() - ticksIn(Activity::Unavailable));
}

std::optional<double> FleetMetrics::tasksPerHour() const noexcept {
  if (ticks == 0) {
    return std::nullopt;
  }
  return static_cast<double>(tasksCompleted) / simulatedHours();
}

std::optional<double> FleetMetrics::tasksPerRobotHour() const noexcept {
  if (ticks == 0 || robots.empty()) {
    return std::nullopt;
  }
  return static_cast<double>(tasksCompleted) /
         (simulatedHours() * static_cast<double>(robots.size()));
}

std::optional<double> FleetMetrics::meanCompletionTicks() const noexcept {
  return ratio(completionTicksTotal, tasksCompleted);
}

std::optional<double> FleetMetrics::meanCompletionSeconds() const noexcept {
  const std::optional<double> mean = meanCompletionTicks();
  if (!mean) {
    return std::nullopt;
  }
  return *mean * options.secondsPerTick;
}

std::uint64_t FleetMetrics::cellsTravelled() const noexcept {
  std::uint64_t sum = 0;
  for (const RobotMetrics& robot : robots) {
    sum += robot.cellsTravelled;
  }
  return sum;
}

std::optional<double> FleetMetrics::cellsPerRobot() const noexcept {
  return ratio(cellsTravelled(), robots.size());
}

std::optional<double> FleetMetrics::cellsPerCompletedTask() const noexcept {
  return ratio(cellsTravelled(), tasksCompleted);
}

std::optional<double> FleetMetrics::conflictsPreventedPerHour() const noexcept {
  if (ticks == 0) {
    return std::nullopt;
  }
  return static_cast<double>(conflictsPrevented) / simulatedHours();
}

std::optional<double> FleetMetrics::replansPerHour() const noexcept {
  if (ticks == 0) {
    return std::nullopt;
  }
  return static_cast<double>(replanEvents()) / simulatedHours();
}

EnergyEstimate FleetMetrics::energyEstimate() const noexcept {
  EnergyEstimate estimate;
  estimate.idle = ticksIn(Activity::Idle) * options.energy.idlePerTick;
  estimate.moving = cellsTravelled() * options.energy.perCell;
  estimate.handling = ticksIn(Activity::Handling) * options.energy.handlingPerTick;
  return estimate;
}

std::optional<double> FleetMetrics::energyPerCompletedTask() const noexcept {
  return ratio(energyEstimate().total(), tasksCompleted);
}

void FleetMetrics::merge(const FleetMetrics& other) {
  if (!(options == other.options)) {
    throw std::invalid_argument("cannot merge metrics taken with different options");
  }
  if (robots.size() != other.robots.size()) {
    throw std::invalid_argument("cannot merge metrics of fleets of different sizes");
  }
  for (std::size_t i = 0; i < robots.size(); ++i) {
    if (robots[i].id != other.robots[i].id) {
      throw std::invalid_argument("cannot merge metrics of different robots");
    }
  }
  for (std::size_t i = 0; i < robots.size(); ++i) {
    for (std::size_t a = 0; a < kActivityCount; ++a) {
      robots[i].budget.ticks[a] += other.robots[i].budget.ticks[a];
    }
    robots[i].cellsTravelled += other.robots[i].cellsTravelled;
    robots[i].deliveries += other.robots[i].deliveries;
    robots[i].conflictsPrevented += other.robots[i].conflictsPrevented;
  }
  ticks += other.ticks;
  tasksSubmitted += other.tasksSubmitted;
  tasksCompleted += other.tasksCompleted;
  tasksFailed += other.tasksFailed;
  tasksCancelled += other.tasksCancelled;
  tasksOpen += other.tasksOpen;
  completionTicksTotal += other.completionTicksTotal;
  completionTicksMax = std::max(completionTicksMax, other.completionTicksMax);
  conflictsPrevented += other.conflictsPrevented;
  conflictDetections += other.conflictDetections;
  collisionsOccurred += other.collisionsOccurred;
  reroutes += other.reroutes;
  repaths += other.repaths;
  obstacleRepairs += other.obstacleRepairs;
  searches += other.searches;
  samples.clear();
}

}  // namespace warehousesim::metrics
