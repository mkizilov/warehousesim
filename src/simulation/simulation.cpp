#include "warehousesim/simulation/simulation.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "warehousesim/planning/astar_planner.hpp"

namespace warehousesim::simulation {

using core::Point;
using robots::Robot;
using robots::RobotStatus;

Simulation::Simulation(world::Warehouse warehouse, const std::vector<Point>& robotStarts,
                       SimulationConfig config, std::unique_ptr<planning::PathPlanner> planner,
                       std::unique_ptr<tasks::TaskAssigner> assigner)
    : warehouse_(std::move(warehouse)),
      planner_(planner ? std::move(planner) : std::make_unique<planning::AStarPlanner>()),
      assigner_(assigner ? std::move(assigner) : std::make_unique<tasks::NearestRobotAssigner>()),
      config_(config) {
  Occupied used;
  for (const Point start : robotStarts) {
    if (!warehouse_.isWalkable(start)) {
      throw std::invalid_argument("robot start cell is not walkable");
    }
    if (!used.insert(start).second) {
      throw std::invalid_argument("two robots share a start cell");
    }
    robots_.emplace_back(core::RobotId{static_cast<std::uint32_t>(robots_.size() + 1)}, start);
  }

  if (config_.taskSpawnInterval > 0) {
    auto pickups = warehouse_.shelfAccessPoints();
    auto dropoffs = warehouse_.cellsOfType(world::CellType::Station);
    if (!pickups.empty() && !dropoffs.empty()) {
      generator_.emplace(std::move(pickups), std::move(dropoffs), config_.seed);
    }
  }
}

core::TaskId Simulation::submitTask(Point pickup, Point dropoff) {
  if (!warehouse_.isWalkable(pickup) || !warehouse_.isWalkable(dropoff)) {
    throw std::invalid_argument("task locations must be walkable cells");
  }
  return tasks_.submit(pickup, dropoff, clock_.tick());
}

void Simulation::step() {
  spawnTask();
  assignTasks();
  moveRobots();
  clock_.advance();
}

void Simulation::run(core::Tick ticks) {
  for (core::Tick i = 0; i < ticks; ++i) {
    step();
  }
}

Snapshot Simulation::snapshot() const {
  Snapshot result;
  result.tick = clock_.tick();
  result.robots.reserve(robots_.size());
  for (const Robot& robot : robots_) {
    result.robots.push_back({robot.id(), robot.position(), robot.status(), robot.task()});
  }
  for (const tasks::Task& task : tasks_.all()) {
    if (task.status == tasks::TaskStatus::Pending || task.status == tasks::TaskStatus::Assigned ||
        task.status == tasks::TaskStatus::InProgress) {
      result.activeTasks.push_back(task);
    }
  }
  result.completedTasks = tasks_.count(tasks::TaskStatus::Completed);
  result.failedTasks = tasks_.count(tasks::TaskStatus::Failed);
  return result;
}

void Simulation::spawnTask() {
  if (!generator_ || clock_.tick() % config_.taskSpawnInterval != 0) {
    return;
  }
  const tasks::TaskRequest request = generator_->next();
  tasks_.submit(request.pickup, request.dropoff, clock_.tick());
}

void Simulation::assignTasks() {
  for (const tasks::Assignment& assignment : assigner_->assign(robots_, tasks_.all())) {
    Robot& robot = robots_[assignment.robot.value - 1];
    const Point pickup = tasks_.find(assignment.task)->pickup;
    tasks_.assign(assignment.task, assignment.robot);
    auto path = planner_->findPath(warehouse_, robot.position(), pickup, {});
    if (!path) {
      tasks_.fail(assignment.task);
      continue;
    }
    robot.beginTask(assignment.task);
    robot.setPath(std::move(*path));
  }
}

void Simulation::moveRobots() {
  Occupied occupied;
  for (const Robot& robot : robots_) {
    occupied.insert(robot.position());
  }

  // Robots act in id order; a cell vacated earlier in the tick is available to later robots.
  for (Robot& robot : robots_) {
    resolveArrival(robot);
    const auto next = robot.nextStep();
    if (!next) {
      continue;
    }
    if (occupied.contains(*next)) {
      robot.noteBlocked();
      // When the blocker steps aside the cell is free right now. Take it immediately,
      // otherwise the blocker would step straight back in on its own turn.
      if (robot.blockedTicks() < config_.maxBlockedTicks || !resolveBlock(robot, *next, occupied) ||
          occupied.contains(*next)) {
        continue;
      }
    }
    occupied.erase(robot.position());
    robot.advance();
    occupied.insert(robot.position());
  }
}

void Simulation::resolveArrival(Robot& robot) {
  while (robot.status() != RobotStatus::Idle && !robot.hasPath()) {
    const core::TaskId taskId = *robot.task();
    if (robot.status() == RobotStatus::ToPickup) {
      const Point dropoff = tasks_.find(taskId)->dropoff;
      tasks_.markInProgress(taskId);
      auto path = planner_->findPath(warehouse_, robot.position(), dropoff, {});
      if (!path) {
        failTask(robot);
        return;
      }
      robot.beginDelivery();
      robot.setPath(std::move(*path));
    } else {
      tasks_.complete(taskId);
      robot.finishTask();
    }
  }
}

// Called once `robot` has waited maxBlockedTicks for `cell`. Priority is the robot id
// (lower wins). An idle or lower-priority blocker gives way by stepping aside; a
// higher-priority blocker is routed around. Making only one side yield breaks head-on and
// cyclic standoffs instead of both robots replanning symmetrically forever. Returns true if
// the blocker moved away.
bool Simulation::resolveBlock(Robot& robot, Point cell, Occupied& occupied) {
  Robot& blocker = *std::ranges::find(robots_, cell, &Robot::position);
  const bool blockerYields = blocker.status() == RobotStatus::Idle || blocker.id() > robot.id();
  if (blockerYields && stepAside(blocker, occupied)) {
    robot.clearBlocked();
    return true;
  }
  if (!tryRepath(robot, occupied)) {
    robot.clearBlocked();  // wait a few more ticks before trying again
  }
  return false;
}

bool Simulation::stepAside(Robot& robot, Occupied& occupied) {
  const Point from = robot.position();
  for (const Point dir : core::kCardinalDirections) {
    const Point target = from + dir;
    if (!warehouse_.isWalkable(target) || occupied.contains(target)) {
      continue;
    }
    occupied.erase(from);
    robot.sidestep(target);
    occupied.insert(target);
    if (robot.status() != RobotStatus::Idle && !tryRepath(robot, occupied) &&
        !tryRepath(robot, {})) {
      failTask(robot);
    }
    return true;
  }
  return false;
}

bool Simulation::tryRepath(Robot& robot, const Occupied& blocked) {
  const tasks::Task& task = *tasks_.find(*robot.task());
  const Point goal = robot.status() == RobotStatus::ToPickup ? task.pickup : task.dropoff;
  auto path = planner_->findPath(warehouse_, robot.position(), goal, blocked);
  if (!path) {
    return false;
  }
  robot.setPath(std::move(*path));
  return true;
}

void Simulation::failTask(Robot& robot) {
  tasks_.fail(*robot.task());
  robot.finishTask();
}

}  // namespace warehousesim::simulation
