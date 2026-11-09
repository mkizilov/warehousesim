#include "warehousesim/simulation/metrics_collector.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace warehousesim::simulation {

namespace {

metrics::Activity activityOf(const Simulation& simulation, const robots::Robot& robot,
                             metrics::WaitCause& cause) {
  cause = metrics::WaitCause::None;
  switch (robot.state()) {
    case robots::RobotState::Idle:
      return metrics::Activity::Idle;
    case robots::RobotState::Loading:
    case robots::RobotState::Unloading:
      return metrics::Activity::Handling;
    case robots::RobotState::Charging:
      return metrics::Activity::Charging;
    case robots::RobotState::Unavailable:
      return metrics::Activity::Unavailable;
    case robots::RobotState::Navigating:
      break;
  }
  const std::optional<RobotWait> held = simulation.heldBack(robot.id());
  if (!held) {
    return metrics::Activity::Moving;
  }
  switch (held->kind) {
    case traffic::ConflictKind::Vertex:
      cause = metrics::WaitCause::Vertex;
      break;
    case traffic::ConflictKind::Swap:
      cause = metrics::WaitCause::Swap;
      break;
    case traffic::ConflictKind::Blocked:
      cause = metrics::WaitCause::Other;
      break;
  }
  return metrics::Activity::Waiting;
}

metrics::TaskEnd endOf(tasks::TaskStatus status) {
  switch (status) {
    case tasks::TaskStatus::Completed:
      return metrics::TaskEnd::Completed;
    case tasks::TaskStatus::Failed:
      return metrics::TaskEnd::Failed;
    default:
      return metrics::TaskEnd::Cancelled;
  }
}

}  // namespace

metrics::EnergyModel energyModelOf(const Simulation& simulation) {
  const robots::RobotSpec& spec = simulation.config().robot;
  return {spec.idleDrainPerTick, spec.moveCostPerCell, spec.handlingDrainPerTick};
}

MetricsCollector::MetricsCollector(const Simulation& simulation, metrics::MetricsOptions options)
    : accumulator_(options, [&simulation] {
        metrics::TickObservation start;
        start.tick = simulation.tick();
        for (const robots::Robot& robot : simulation.robots()) {
          // What each robot is doing now, so that a wait already under way when the interval
          // begins is not counted as a conflict that starts inside it.
          metrics::WaitCause cause = metrics::WaitCause::None;
          const metrics::Activity activity = activityOf(simulation, robot, cause);
          start.robots.push_back({robot.id(), robot.position(), activity, cause,
                                  robot.cellsTravelled(), robot.deliveries()});
        }
        start.counters.tasksSubmitted = simulation.tasks().all().size();
        start.counters.conflictDetections =
            simulation.trafficStats().vertex + simulation.trafficStats().swaps;
        start.counters.reroutes = simulation.planningStats().reroutes;
        start.counters.repaths = simulation.planningStats().repaths;
        start.counters.obstacleRepairs = simulation.planningStats().obstacleRepairs;
        start.counters.searches = simulation.planningStats().searches;
        for (const tasks::Task& task : simulation.tasks().all()) {
          if (task.isOpen()) {
            ++start.tasksOpen;
          }
        }
        return start;
      }()) {
  knownTasks_ = simulation.tasks().all().size();
  for (const tasks::Task& task : simulation.tasks().all()) {
    if (task.isOpen()) {
      openTasks_.push_back(task.id);
    }
  }
}

metrics::TickObservation MetricsCollector::read(const Simulation& simulation) {
  metrics::TickObservation observation;
  observation.tick = simulation.tick();
  observation.robots.reserve(simulation.robots().size());
  for (const robots::Robot& robot : simulation.robots()) {
    metrics::WaitCause cause = metrics::WaitCause::None;
    const metrics::Activity activity = activityOf(simulation, robot, cause);
    observation.robots.push_back({robot.id(), robot.position(), activity, cause,
                                  robot.cellsTravelled(), robot.deliveries()});
  }

  const tasks::TaskQueue& queue = simulation.tasks();
  const std::vector<tasks::Task>& all = queue.all();
  observation.counters.tasksSubmitted = all.size();
  observation.counters.conflictDetections =
      simulation.trafficStats().vertex + simulation.trafficStats().swaps;
  observation.counters.reroutes = simulation.planningStats().reroutes;
  observation.counters.repaths = simulation.planningStats().repaths;
  observation.counters.obstacleRepairs = simulation.planningStats().obstacleRepairs;
  observation.counters.searches = simulation.planningStats().searches;

  // Tasks that appeared since the last step (a task can be made and finished in the same step), and
  // the ones that were open and no longer are.
  std::vector<core::TaskId> stillOpen;
  stillOpen.reserve(openTasks_.size() + 4);
  const auto note = [&](const tasks::Task& task) {
    if (task.isOpen()) {
      stillOpen.push_back(task.id);
    } else {
      observation.tasksEnded.push_back({endOf(task.status), task.createdAt});
    }
  };
  for (const core::TaskId id : openTasks_) {
    note(*queue.find(id));
  }
  for (std::size_t i = knownTasks_; i < all.size(); ++i) {
    note(all[i]);
  }
  observation.tasksOpen = stillOpen.size();
  knownTasks_ = all.size();
  openTasks_ = std::move(stillOpen);
  return observation;
}

void MetricsCollector::observe(const Simulation& simulation) {
  // Checked before anything is read, so that a refused step leaves the collector as it was.
  if (simulation.tick() != accumulator_.lastTick() + 1) {
    throw std::invalid_argument("metrics must observe every step: after tick " +
                                std::to_string(accumulator_.lastTick()) + " came tick " +
                                std::to_string(simulation.tick()));
  }
  accumulator_.add(read(simulation));
}

}  // namespace warehousesim::simulation
