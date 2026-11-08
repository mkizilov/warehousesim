#include "warehousesim/metrics/accumulator.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace warehousesim::metrics {

namespace {

std::uint64_t key(core::Point p) noexcept {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.x)) << 32) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.y));
}

bool heldBackByARobot(const RobotObservation& robot) noexcept {
  return robot.activity == Activity::Waiting &&
         (robot.waitCause == WaitCause::Vertex || robot.waitCause == WaitCause::Swap);
}

bool isBusy(Activity activity) noexcept {
  return activity == Activity::Moving || activity == Activity::Waiting ||
         activity == Activity::Handling;
}

}  // namespace

MetricsAccumulator::MetricsAccumulator(MetricsOptions options, const TickObservation& start)
    : lastTick_(start.tick), baseline_(start.counters), last_(start.counters) {
  options.validate();
  metrics_.options = options;
  metrics_.tasksOpen = start.tasksOpen;
  std::vector<core::RobotId> ids;
  for (const RobotObservation& robot : start.robots) {
    ids.push_back(robot.id);
    metrics_.robots.push_back({robot.id, {}, 0, 0, 0});
    baseCells_.push_back(robot.cellsTravelled);
    baseDeliveries_.push_back(robot.deliveries);
    previousPositions_.push_back(robot.position);
    inConflict_.push_back(heldBackByARobot(robot));  // a wait that began before the interval
  }
  std::sort(ids.begin(), ids.end());
  if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
    throw std::invalid_argument("two robots have the same id");
  }
}

void MetricsAccumulator::validate(const TickObservation& observation) const {
  if (observation.tick != lastTick_ + 1) {
    throw std::invalid_argument("metrics must observe every tick: after tick " +
                                std::to_string(lastTick_) + " came tick " +
                                std::to_string(observation.tick));
  }
  if (observation.robots.size() != metrics_.robots.size()) {
    throw std::invalid_argument("the number of robots changed during the observation");
  }
  for (std::size_t i = 0; i < metrics_.robots.size(); ++i) {
    const RobotObservation& robot = observation.robots[i];
    if (robot.id != metrics_.robots[i].id) {
      throw std::invalid_argument("the robots changed during the observation");
    }
    if (robot.cellsTravelled < baseCells_[i] + metrics_.robots[i].cellsTravelled ||
        robot.deliveries < baseDeliveries_[i] + metrics_.robots[i].deliveries) {
      throw std::invalid_argument("a cumulative count of a robot went down");
    }
  }
  const SystemCounters& c = observation.counters;
  if (c.tasksSubmitted < last_.tasksSubmitted || c.conflictDetections < last_.conflictDetections ||
      c.reroutes < last_.reroutes || c.repaths < last_.repaths ||
      c.obstacleRepairs < last_.obstacleRepairs || c.searches < last_.searches) {
    throw std::invalid_argument("a cumulative counter went down");
  }
  for (const TaskEnding& ending : observation.tasksEnded) {
    if (ending.createdAt > observation.tick) {
      throw std::invalid_argument("a task ended before it was created");
    }
  }
}

// Robots in one cell, and pairs that changed places, since the last observation. Simulated robots
// never do either: this is an audit that does not trust the thing that prevents them.
void MetricsAccumulator::countCollisions(const TickObservation& observation) {
  const std::size_t n = observation.robots.size();
  std::vector<std::uint64_t> cells;
  cells.reserve(n);
  std::unordered_map<std::uint64_t, std::size_t> robotAt;
  robotAt.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    cells.push_back(key(observation.robots[i].position));
    robotAt.emplace(cells.back(), i);
  }
  std::sort(cells.begin(), cells.end());
  for (std::size_t first = 0; first < n;) {
    std::size_t last = first;
    while (last < n && cells[last] == cells[first]) {
      ++last;
    }
    const std::uint64_t sharing = last - first;
    metrics_.collisionsOccurred += sharing * (sharing - 1) / 2;
    first = last;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const core::Point from = previousPositions_[i];
    const core::Point to = observation.robots[i].position;
    if (from == to) {
      continue;
    }
    const auto other = robotAt.find(key(from));
    if (other != robotAt.end() && other->second > i && previousPositions_[other->second] == to) {
      ++metrics_.collisionsOccurred;  // counted once, from the lower index
    }
  }
}

void MetricsAccumulator::closeWindow(core::Tick tick, std::uint64_t tasksOpen) {
  MetricsSample sample;
  sample.tick = tick;
  sample.windowTicks = window_.ticks;
  sample.tasksCompleted = window_.completed;
  sample.busyRobotTicks = window_.busy;
  sample.serviceRobotTicks = window_.service;
  sample.waitingRobotTicks = window_.waiting;
  sample.tasksOpen = tasksOpen;
  metrics_.samples.push_back(sample);
  if (metrics_.samples.size() > metrics_.options.maxSamples) {
    metrics_.samples.erase(
        metrics_.samples.begin(),
        metrics_.samples.end() - static_cast<std::ptrdiff_t>(metrics_.options.maxSamples));
  }
  window_ = {};
}

void MetricsAccumulator::add(const TickObservation& observation) {
  validate(observation);
  countCollisions(observation);

  for (std::size_t i = 0; i < observation.robots.size(); ++i) {
    const RobotObservation& robot = observation.robots[i];
    RobotMetrics& mine = metrics_.robots[i];
    ++mine.budget.ticks[static_cast<std::size_t>(robot.activity)];
    mine.cellsTravelled = robot.cellsTravelled - baseCells_[i];
    mine.deliveries = robot.deliveries - baseDeliveries_[i];
    const bool conflict = heldBackByARobot(robot);
    if (conflict && !inConflict_[i]) {
      ++mine.conflictsPrevented;
      ++metrics_.conflictsPrevented;
    }
    inConflict_[i] = conflict;
    previousPositions_[i] = robot.position;
    if (robot.activity != Activity::Unavailable) {
      ++window_.service;
    }
    window_.busy += isBusy(robot.activity) ? 1 : 0;
    window_.waiting += robot.activity == Activity::Waiting ? 1 : 0;
  }

  for (const TaskEnding& ending : observation.tasksEnded) {
    switch (ending.how) {
      case TaskEnd::Completed: {
        const std::uint64_t took = observation.tick - ending.createdAt;
        ++metrics_.tasksCompleted;
        metrics_.completionTicksTotal += took;
        metrics_.completionTicksMax = std::max(metrics_.completionTicksMax, took);
        ++window_.completed;
        break;
      }
      case TaskEnd::Failed:
        ++metrics_.tasksFailed;
        break;
      case TaskEnd::Cancelled:
        ++metrics_.tasksCancelled;
        break;
    }
  }

  const SystemCounters& c = observation.counters;
  metrics_.tasksSubmitted = c.tasksSubmitted - baseline_.tasksSubmitted;
  metrics_.conflictDetections = c.conflictDetections - baseline_.conflictDetections;
  metrics_.reroutes = c.reroutes - baseline_.reroutes;
  metrics_.repaths = c.repaths - baseline_.repaths;
  metrics_.obstacleRepairs = c.obstacleRepairs - baseline_.obstacleRepairs;
  metrics_.searches = c.searches - baseline_.searches;
  last_ = c;
  metrics_.tasksOpen = observation.tasksOpen;

  ++metrics_.ticks;
  lastTick_ = observation.tick;
  ++window_.ticks;
  if (metrics_.options.sampleInterval > 0 && window_.ticks >= metrics_.options.sampleInterval) {
    closeWindow(observation.tick, observation.tasksOpen);
  }
}

}  // namespace warehousesim::metrics
