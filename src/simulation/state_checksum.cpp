#include <cstdint>
#include <optional>

#include "warehousesim/core/checksum.hpp"
#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::simulation {

namespace {

using core::Checksum;
using core::Point;

void addPoint(Checksum& sum, Point p) {
  sum.add(p.x);
  sum.add(p.y);
}

void addPoints(Checksum& sum, const auto& points) {
  sum.add(points.size());
  for (const Point p : points) {
    addPoint(sum, p);
  }
}

// A flag for whether there is one, then the value if there is.
template <typename T, typename AddValue>
void addOptional(Checksum& sum, const std::optional<T>& value, AddValue addValue) {
  sum.add(value.has_value());
  if (value) {
    addValue(sum, *value);
  }
}

constexpr auto addPlainId = [](Checksum& sum, const auto& id) { sum.add(id.value); };

void addRobot(Checksum& sum, const robots::Robot& robot) {
  sum.add(robot.id().value);
  addPoint(sum, robot.position());
  sum.add(robot.heading());
  sum.add(robot.headingAngle());
  sum.add(robot.state());
  sum.add(robot.goal());
  sum.add(robot.speed());
  sum.add(robot.segmentProgress());
  sum.add(robot.authority());
  sum.add(robot.motion());
  sum.add(robot.unusedTime());
  sum.add(robot.dwellRemaining());
  sum.add(robot.blockedTicks());
  sum.add(robot.cellsTravelled());
  sum.add(robot.deliveries());
  sum.add(robot.brakingViolations());
  sum.add(robot.battery().level());
  sum.add(robot.battery().capacity());
  addOptional(sum, robot.task(), addPlainId);
  addOptional(sum, robot.destination(), addPoint);
  addPoints(sum, robot.path());
}

void addTask(Checksum& sum, const tasks::Task& task) {
  sum.add(task.id.value);
  addPoint(sum, task.pickup);
  addPoint(sum, task.dropoff);
  sum.add(task.status);
  addOptional(sum, task.assignee, addPlainId);
  sum.add(task.createdAt);
  sum.add(task.priority);
}

void addPlan(Checksum& sum, const traffic::TrafficPlan& plan) {
  sum.add(plan.clearedCells);
  addOptional(sum, plan.conflict, [](Checksum& s, const traffic::Conflict& conflict) {
    s.add(conflict.kind);
    addPoint(s, conflict.cell);
    s.add(conflict.step);
    addOptional(s, conflict.other, addPlainId);
  });
  sum.add(plan.inTheWay);
  sum.add(plan.delaySteps);
}

void addTrafficStats(Checksum& sum, const traffic::TrafficStats& stats) {
  sum.add(stats.blocked);
  sum.add(stats.vertex);
  sum.add(stats.swaps);
  sum.add(stats.inTheWay);
}

void addPlanningStats(Checksum& sum, const PlanningStats& stats) {
  sum.add(stats.searches);
  sum.add(stats.reroutes);
  sum.add(stats.dodges);
  sum.add(stats.partialPlans);
  sum.add(stats.outOfBudget);
  sum.add(stats.deferred);
  sum.add(stats.work);
  sum.add(stats.peakStepWork);
  sum.add(stats.obstacleRepairs);
  sum.add(stats.repaths);
  sum.add(stats.prolongedBlocks);
  sum.add(stats.deadlocks);
  sum.add(stats.deadlocksRecovered);
  sum.add(stats.deadlocksUnresolved);
  sum.add(stats.priorityBoosts);
  sum.add(stats.abandoned);
}

void addFlags(Checksum& sum, const std::vector<bool>& flags) {
  sum.add(flags.size());
  for (const bool flag : flags) {
    sum.add(flag);
  }
}

void addNumbers(Checksum& sum, const auto& numbers) {
  sum.add(numbers.size());
  for (const auto number : numbers) {
    sum.add(number);
  }
}

}  // namespace

// Everything the next step depends on, and everything a snapshot or a view of the traffic shows.
// Not in it: the planner's and the router's scratch memory (rebuilt on demand, and counted in the
// work totals that are), the reservation table (rebuilt from the robots at the start of every
// step), and the configuration (fixed at construction and part of origin()).
std::uint64_t Simulation::stateChecksum() const {
  Checksum sum;
  sum.add(clock_.tick());

  // The layout can change while running (placeObstacle, clearObstacle).
  const std::vector<std::string> rows = world_.layout().toRows();
  sum.add(rows.size());
  for (const std::string& row : rows) {
    sum.add(std::string_view(row));
  }

  sum.add(robots_.size());
  for (const robots::Robot& robot : robots_) {
    addRobot(sum, robot);
    addPoints(sum, world_.holdsOf(robot.id()));
    addOptional(sum, std::optional<std::uint32_t>(progress_.stuckTicks(robot.id())),
                [](Checksum& s, std::uint32_t ticks) { s.add(ticks); });
    addOptional(sum, progress_.waitingFor(robot.id()), addPlainId);
    addOptional(sum, progress_.anchor(robot.id()), addPoint);
  }

  sum.add(tasks_.all().size());
  for (const tasks::Task& task : tasks_.all()) {
    addTask(sum, task);
  }

  addTrafficStats(sum, trafficStats());
  sum.add(replans_);
  addPlanningStats(sum, planningStats_);

  addOptional(sum, generator_, [](Checksum& s, const tasks::TaskGenerator& generator) {
    s.add(generator.rngState());
  });

  // Per robot, from the step just run or carried between steps.
  sum.add(plans_.size());
  for (const traffic::TrafficPlan& plan : plans_) {
    addPlan(sum, plan);
  }
  addFlags(sum, displaced_);
  addFlags(sum, movedThisStep_);
  addFlags(sum, heldBack_);
  addFlags(sum, reservedShare_);
  addNumbers(sum, nextSearch_);
  addNumbers(sum, boostUntil_);
  addNumbers(sum, deferred_);
  addNumbers(sum, workScale_);
  sum.add(ringsLastStep_.size());
  for (const std::vector<core::RobotId>& ring : ringsLastStep_) {
    sum.add(ring.size());
    for (const core::RobotId id : ring) {
      sum.add(id.value);
    }
  }
  sum.add(workPool_);
  sum.add(stepWork_);
  sum.add(plannerBase_);
  sum.add(plannerWork());
  return sum.value();
}

}  // namespace warehousesim::simulation
