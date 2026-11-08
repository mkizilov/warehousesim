#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/metrics/observation.hpp"

namespace warehousesim::metrics {

// How the simulated numbers are read. Neither is a claim about real hardware: they only say what a
// tick and an energy unit stand for in this model.
//
// The energy model is deliberately crude: units drawn per tick while idle, per cell entered and per
// tick of loading or unloading. standard() is the model of the bundled scenarios' robots (see
// scenarios.cpp, where a test keeps the two equal).
struct EnergyModel {
  std::uint32_t idlePerTick = 0;
  std::uint32_t perCell = 0;
  std::uint32_t handlingPerTick = 0;

  [[nodiscard]] static constexpr EnergyModel standard() noexcept { return {1, 15, 2}; }
  friend bool operator==(const EnergyModel&, const EnergyModel&) = default;
};

struct MetricsOptions {
  // How many simulated seconds one tick stands for. An arbitrary scale for turning ticks into
  // hours; it is not calibrated to any real robot. Must be finite and positive.
  double secondsPerTick = 1.0;
  EnergyModel energy = EnergyModel::standard();
  // A sample of the running totals is taken every this many ticks (0: none), keeping the last
  // maxSamples. Samples are for the dashboard's trend line and for the series CSV.
  core::Tick sampleInterval = 100;
  std::size_t maxSamples = 240;

  // Throws std::invalid_argument for a seconds-per-tick that is not finite and positive.
  void validate() const;
  friend bool operator==(const MetricsOptions&, const MetricsOptions&) = default;
};

// Robot-ticks spent in each Activity.
struct TimeBudget {
  std::array<std::uint64_t, kActivityCount> ticks{};

  [[nodiscard]] std::uint64_t in(Activity activity) const noexcept {
    return ticks[static_cast<std::size_t>(activity)];
  }
  [[nodiscard]] std::uint64_t total() const noexcept;
  friend bool operator==(const TimeBudget&, const TimeBudget&) = default;
};

struct RobotMetrics {
  core::RobotId id;
  TimeBudget budget;
  std::uint64_t cellsTravelled = 0;
  std::uint64_t deliveries = 0;
  std::uint64_t conflictsPrevented = 0;

  friend bool operator==(const RobotMetrics&, const RobotMetrics&) = default;
};

// The totals over one window of ticks, for the trend line.
struct MetricsSample {
  core::Tick tick = 0;                  // the last tick of the window
  core::Tick windowTicks = 0;           // how many ticks the window covers
  std::uint64_t tasksCompleted = 0;     // in the window
  std::uint64_t busyRobotTicks = 0;     // Moving + Waiting + Handling
  std::uint64_t serviceRobotTicks = 0;  // all but Unavailable
  std::uint64_t waitingRobotTicks = 0;
  std::uint64_t tasksOpen = 0;  // at the end of the window

  friend bool operator==(const MetricsSample&, const MetricsSample&) = default;
};

struct EnergyEstimate {
  std::uint64_t idle = 0;
  std::uint64_t moving = 0;
  std::uint64_t handling = 0;

  [[nodiscard]] std::uint64_t total() const noexcept { return idle + moving + handling; }
  friend bool operator==(const EnergyEstimate&, const EnergyEstimate&) = default;
};

// The report: raw counts over the observed interval, and the derived figures, each of which is
// empty when its denominator is zero (no time observed, no robot, no task finished) rather than a
// misleading 0 or NaN. All of it is simulated, a property of the model and the seed; none of it is
// a measurement of a real robot or warehouse.
//
// "Time" here is simulated time: ticks, and hours at options.secondsPerTick seconds per tick.
struct FleetMetrics {
  MetricsOptions options;
  core::Tick ticks = 0;  // ticks observed
  std::vector<RobotMetrics> robots;

  std::uint64_t tasksSubmitted = 0;  // during the interval
  std::uint64_t tasksCompleted = 0;
  std::uint64_t tasksFailed = 0;
  std::uint64_t tasksCancelled = 0;
  std::uint64_t tasksOpen = 0;  // at the end
  // From submission to completion, in ticks, over the tasks that completed.
  std::uint64_t completionTicksTotal = 0;
  std::uint64_t completionTicksMax = 0;

  std::uint64_t conflictsPrevented = 0;  // robots held back from a clash with another robot
  std::uint64_t conflictDetections = 0;  // clashes the reservation table found, step by step
  std::uint64_t collisionsOccurred = 0;  // two robots in one cell, or swapped cells: always 0
  std::uint64_t reroutes = 0;
  std::uint64_t repaths = 0;
  std::uint64_t obstacleRepairs = 0;
  std::uint64_t searches = 0;

  std::vector<MetricsSample> samples;

  // ---- time
  [[nodiscard]] double simulatedSeconds() const noexcept;
  [[nodiscard]] double simulatedHours() const noexcept;

  // ---- fleet time budget
  [[nodiscard]] std::uint64_t robotTicks() const noexcept;                // robots x ticks
  [[nodiscard]] std::uint64_t ticksIn(Activity activity) const noexcept;  // summed over robots
  [[nodiscard]] std::optional<double> fraction(Activity activity) const noexcept;  // of robotTicks
  // (Moving + Waiting + Handling) / (robotTicks - Unavailable): the share of in-service robot time
  // spent occupied by a task or a trip (a charger trip counts as occupied; charging does not).
  [[nodiscard]] std::optional<double> utilization() const noexcept;

  // ---- tasks
  [[nodiscard]] std::optional<double> tasksPerHour() const noexcept;
  [[nodiscard]] std::optional<double> tasksPerRobotHour() const noexcept;
  [[nodiscard]] std::optional<double> meanCompletionTicks() const noexcept;
  [[nodiscard]] std::optional<double> meanCompletionSeconds() const noexcept;

  // ---- travel
  [[nodiscard]] std::uint64_t cellsTravelled() const noexcept;
  [[nodiscard]] std::optional<double> cellsPerRobot() const noexcept;
  // All the driving, empty trips and charger trips included, per task completed.
  [[nodiscard]] std::optional<double> cellsPerCompletedTask() const noexcept;

  // ---- conflicts
  [[nodiscard]] std::optional<double> conflictsPreventedPerHour() const noexcept;

  // ---- replanning
  // Routes replaced while a robot was under way: after a clash, to get round a robot, or after the
  // layout changed.
  [[nodiscard]] std::uint64_t replanEvents() const noexcept {
    return reroutes + repaths + obstacleRepairs;
  }
  [[nodiscard]] std::optional<double> replansPerHour() const noexcept;

  // ---- energy (estimated from the time budget and the distance with options.energy)
  [[nodiscard]] EnergyEstimate energyEstimate() const noexcept;
  [[nodiscard]] std::optional<double> energyPerCompletedTask() const noexcept;

  // Adds the counts of another run of the same fleet, so that the derived figures become the pooled
  // ones over both (total completed over total hours, and so on). Samples are dropped: windows of
  // different runs do not line up. Throws std::invalid_argument unless the other has the same
  // options and the same robots (ids in the same order).
  void merge(const FleetMetrics& other);
};

}  // namespace warehousesim::metrics
