#include "warehousesim/simulation/simulation.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "warehousesim/planning/astar_planner.hpp"

namespace warehousesim::simulation {

using core::Point;
using robots::NavigationGoal;
using robots::Robot;
using robots::RobotState;

namespace {

// One tick is one unit of simulated time for the motion model (speeds are cells per tick).
constexpr double kStepLength = 1.0;
// Rounds of turn, claim and drive that one step can hold.
constexpr int kMaxMotionRounds = 8;

}  // namespace

Simulation::Simulation(world::Warehouse warehouse, const std::vector<Point>& robotStarts,
                       SimulationConfig config, std::unique_ptr<planning::PathPlanner> planner,
                       std::unique_ptr<tasks::Dispatcher> dispatcher)
    : world_(std::move(warehouse)),
      chargers_(world_.layout().cellsOfType(world::CellType::Charging)),
      planner_(planner ? std::move(planner) : std::make_unique<planning::AStarPlanner>()),
      dispatcher_(dispatcher ? std::move(dispatcher) : std::make_unique<tasks::GreedyDispatcher>()),
      config_(config) {
  robots::RobotSpec spec = config_.robot;
  if (chargers_.empty()) {
    spec.lowBatteryLevel = 0;  // nowhere to recharge, so a reserve would only strand robots
  }
  for (const Point start : robotStarts) {
    const core::RobotId id{static_cast<std::uint32_t>(robots_.size() + 1)};
    switch (world_.addRobot(id, start)) {
      case world::PlaceStatus::Ok:
        break;
      case world::PlaceStatus::CellOccupied:
        throw std::invalid_argument("two robots share a start cell");
      default:
        throw std::invalid_argument("robot start cell is not walkable");
    }
    robots_.emplace_back(id, start, spec);
  }

  if (config_.taskSpawnInterval > 0) {
    // Explicit Pickup cells win; layouts without any fall back to the aisles beside shelves.
    auto pickups = world_.layout().cellsOfType(world::CellType::Pickup);
    if (pickups.empty()) {
      pickups = world_.layout().shelfAccessPoints();
    }
    auto dropoffs = world_.layout().cellsOfType(world::CellType::Dropoff);
    if (!pickups.empty() && !dropoffs.empty()) {
      generator_.emplace(std::move(pickups), std::move(dropoffs), config_.seed,
                         config_.varyTaskPriority);
    }
  }
}

core::TaskId Simulation::submitTask(Point pickup, Point dropoff, tasks::TaskPriority priority) {
  if (!world_.isWalkable(pickup) || !world_.isWalkable(dropoff)) {
    throw std::invalid_argument("task locations must be walkable cells");
  }
  return tasks_.submit(pickup, dropoff, clock_.tick(), priority);
}

bool Simulation::cancelTask(core::TaskId id) {
  if (tasks_.find(id) == nullptr) {
    throw std::out_of_range("unknown task id");
  }
  if (!tasks_.cancel(id)) {
    return false;
  }
  // The task stays recorded against the robot that held it; that robot stops working on it.
  if (const auto holder = tasks_.find(id)->assignee) {
    Robot& robot = robotById(*holder);
    if (robot.task() == id) {
      robot.abortTask();
      syncHolds(robot);
    }
  }
  return true;
}

Robot& Simulation::robotById(core::RobotId id) {
  if (!id.valid() || id.value > robots_.size()) {
    throw std::out_of_range("unknown robot id");
  }
  return robots_[id.value - 1];
}

bool Simulation::disableRobot(core::RobotId id) {
  Robot& robot = robotById(id);
  if (robot.state() == RobotState::Unavailable) {
    return false;
  }
  releaseTask(robot);
  robot.makeUnavailable();
  syncHolds(robot);
  return true;
}

bool Simulation::restoreRobot(core::RobotId id) {
  Robot& robot = robotById(id);
  if (robot.state() != RobotState::Unavailable) {
    return false;
  }
  if (robot.battery().isEmpty()) {
    robot.replaceBattery();
  }
  robot.restore();
  return true;
}

void Simulation::step() {
  spawnTask();
  assignTasks();
  updateRobots();
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
    result.robots.push_back({robot.id(), robot.position(),
                             RobotPose{robot.pose(), robot.headingAngle()}, robot.heading(),
                             robot.state(), robot.goal(), robot.speed(), robot.battery().level(),
                             robot.battery().capacity(), robot.task(), robot.motion(),
                             robot.carriesCargo(), robot.deliveries()});
  }
  for (const tasks::Task& task : tasks_.all()) {
    if (task.isOpen()) {
      result.activeTasks.push_back(task);
    }
  }
  result.completedTasks = tasks_.count(tasks::TaskStatus::Completed);
  result.failedTasks = tasks_.count(tasks::TaskStatus::Failed);
  result.cancelledTasks = tasks_.count(tasks::TaskStatus::Cancelled);
  return result;
}

void Simulation::spawnTask() {
  if (!generator_ || clock_.tick() % config_.taskSpawnInterval != 0) {
    return;
  }
  const tasks::TaskRequest request = generator_->next();
  tasks_.submit(request.pickup, request.dropoff, clock_.tick(), request.priority);
}

// Asks the dispatcher for a plan and applies it. The plan is only a proposal: a task is handed out
// only if the queue still lets this robot claim it, and only to a robot that is still available,
// so a faulty dispatcher still cannot give one task to two robots or two tasks to one robot.
void Simulation::assignTasks() {
  const tasks::DispatchPlan plan = dispatcher_->plan(world_.layout(), robots_, tasks_);

  for (const tasks::Rejection& rejection : plan.rejections) {
    const tasks::Task* task = tasks_.find(rejection.task);
    if (task != nullptr && task->status == tasks::TaskStatus::Pending) {
      tasks_.reject(rejection.task);
    }
  }

  for (const tasks::Assignment& assignment : plan.assignments) {
    Robot& robot = robotById(assignment.robot);
    if (!robot.isAvailable() || !tasks_.tryAssign(assignment.task, assignment.robot)) {
      continue;
    }
    const Point pickup = tasks_.find(assignment.task)->pickup;
    auto path = planner_->findPath(world_.layout(), robot.position(), pickup, {});
    if (!path) {
      tasks_.fail(assignment.task);
      continue;
    }
    robot.beginTask(assignment.task, pickup, std::move(*path));
  }
}

// Robots act in id order; a cell vacated earlier in the tick is available to later robots.
void Simulation::updateRobots() {
  for (Robot& robot : robots_) {
    updateRobot(robot);
  }
}

// One robot's turn: finish whatever its state allows to finish right now, notice a flat
// battery, let a tick of time pass in the current state, then advance along the path.
void Simulation::updateRobot(Robot& robot) {
  settle(robot);
  breakDownIfDepleted(robot);
  robot.elapseTick();
  if (robot.hasPath()) {
    moveAlongPath(robot);
  }
}

// Performs every state change that is already due, in order, until the robot is waiting for
// something to happen (time to pass, a cell to be reached, a task to arrive).
void Simulation::settle(Robot& robot) {
  while (true) {
    switch (robot.state()) {
      case RobotState::Idle:
        // A robot still coasting to a stop does not go anywhere new until it has stopped.
        if (robot.hasPath() || !robot.needsCharge() || !dispatchToCharger(robot)) {
          return;
        }
        break;
      case RobotState::Navigating:
        if (!robot.hasArrived()) {
          return;
        }
        arrive(robot);
        break;
      case RobotState::Loading:
        if (robot.dwellRemaining() > 0) {
          return;
        }
        finishLoading(robot);
        break;
      case RobotState::Unloading:
        if (robot.dwellRemaining() > 0) {
          return;
        }
        tasks_.complete(*robot.task());
        robot.finishUnloading();
        break;
      case RobotState::Charging:
        if (!robot.battery().isFull()) {
          return;
        }
        robot.stopCharging();
        break;
      case RobotState::Unavailable:
        return;
    }
  }
}

void Simulation::arrive(Robot& robot) {
  switch (robot.goal()) {
    case NavigationGoal::Pickup:
      robot.beginLoading();
      break;
    case NavigationGoal::Dropoff:
      robot.beginUnloading();
      break;
    case NavigationGoal::Charger:
      robot.beginCharging();
      break;
    case NavigationGoal::None:
      throw std::logic_error("navigating robot has no goal");
  }
}

void Simulation::finishLoading(Robot& robot) {
  const core::TaskId taskId = *robot.task();
  const Point dropoff = tasks_.find(taskId)->dropoff;
  tasks_.markInProgress(taskId);
  auto path = planner_->findPath(world_.layout(), robot.position(), dropoff, {});
  if (!path) {
    failTask(robot);
    return;
  }
  robot.finishLoading(dropoff, std::move(*path));
}

// An idle robot with a low battery either starts charging where it stands (if that is a
// charging cell) or sets off for the nearest free charger. False if it can do neither.
bool Simulation::dispatchToCharger(Robot& robot) {
  if (world_.layout().at(robot.position()) == world::CellType::Charging) {
    robot.beginCharging();
    return true;
  }
  if (robot.battery().isEmpty()) {
    return false;  // cannot travel; breakDownIfDepleted will take it out of service
  }
  auto trip = nearestFreeCharger(robot);
  if (!trip) {
    return false;
  }
  robot.beginChargerTrip(trip->first, std::move(trip->second));
  return true;
}

// The charger with the shortest path from the robot that no other robot is standing on.
std::optional<std::pair<Point, robots::RobotPath>> Simulation::nearestFreeCharger(
    const Robot& robot) const {
  std::optional<std::pair<Point, robots::RobotPath>> best;
  for (const Point charger : chargers_) {
    if (world_.isOccupied(charger) && charger != robot.position()) {
      continue;
    }
    auto path = planner_->findPath(world_.layout(), robot.position(), charger, {});
    if (path && (!best || path->size() < best->second.size())) {
      best.emplace(charger, std::move(*path));
    }
  }
  return best;
}

// A robot with an empty battery cannot do anything, except sit on a charger and recharge.
void Simulation::breakDownIfDepleted(Robot& robot) {
  if (robot.battery().isEmpty() && robot.state() != RobotState::Unavailable &&
      robot.state() != RobotState::Charging) {
    releaseTask(robot);
    robot.makeUnavailable();
    syncHolds(robot);
  }
}

// One fixed step of one robot's motion. The world only ever sees whole cells: the robot first
// claims the cells it needs ahead of it, drives within them, and the world is told about each
// cell the robot passes. Where the robot really is between two cells is its own business.
void Simulation::moveAlongPath(Robot& robot) {
  double timeLeft = kStepLength;
  // A robot can come to rest at a corner and still have time in the step; it uses it to turn and
  // set off again, as a shorter step would have. A couple of rounds is all a step can hold.
  for (int round = 0; round < kMaxMotionRounds; ++round) {
    if (robot.state() == RobotState::Navigating) {
      // Face the next cell, then see whether it can be had. A robot that is still rotating has
      // not asked for anything yet and holds nothing.
      timeLeft = robot.turnTowardNext(timeLeft);
      if (timeLeft <= 0.0) {
        (void)robot.advance(0.0);
        return;
      }
      claimAhead(robot, timeLeft);
      if (robot.authority() == 0) {
        // Not even the next cell is available.
        bool freed = false;
        if (round == 0) {
          robot.noteBlocked();
          // When the blocker steps aside the cell is free right now. Take it immediately,
          // otherwise the blocker would step straight back in on its own turn.
          if (robot.blockedTicks() >= config_.maxBlockedTicks &&
              resolveBlock(robot, *robot.nextStep())) {
            claimAhead(robot, timeLeft);
            freed = robot.authority() > 0;
          }
        }
        if (!freed) {
          (void)robot.advance(0.0);  // waits where it is
          return;
        }
      }
    }
    const std::uint32_t passed = robot.advance(timeLeft);
    for (std::uint32_t i = 0; i < passed; ++i) {
      if (!world_.advanceRobot(robot.id())) {
        throw std::logic_error("the world and a robot disagree about the cells it holds");
      }
    }
    timeLeft = robot.unusedTime();
    if (timeLeft <= 0.0 || !robot.hasPath()) {
      return;
    }
  }
}

void Simulation::claimAhead(Robot& robot, double timeToDrive) {
  const std::size_t wanted = robot.wantedAuthority(timeToDrive);
  while (robot.authority() < wanted) {
    const Point cell = robot.path()[robot.authority()];
    const world::MoveStatus status = world_.claimCell(robot.id(), cell);
    if (status == world::MoveStatus::CellOccupied) {
      return;
    }
    // Planned steps are adjacent walkable cells, so the world can only reject one for another
    // reason if the robot and the world have drifted apart.
    if (status != world::MoveStatus::Ok) {
      throw std::logic_error("world rejected a planned robot move");
    }
    robot.extendAuthority();
  }
}

void Simulation::syncHolds(Robot& robot) { world_.releaseClaims(robot.id(), robot.authority()); }

// Called once `robot` has waited maxBlockedTicks for `cell`. Priority is the robot id
// (lower wins). An idle or lower-priority blocker gives way by stepping aside, unless it is
// busy in place; otherwise the robot is routed around the blocker. Making only one side yield
// breaks head-on and cyclic standoffs instead of both robots replanning symmetrically forever.
// Returns true if the blocker moved away.
bool Simulation::resolveBlock(Robot& robot, Point cell) {
  const std::optional<core::RobotId> holder = world_.occupantAt(cell);
  if (!holder) {
    return false;
  }
  Robot& blocker = robotById(*holder);
  // A robot that has been granted cells ahead is on its way and will clear this one by itself;
  // pushing it aside or planning around it would only get in the way.
  if (blocker.isCommitted()) {
    return false;
  }
  const bool blockerYields =
      !blocker.isStationary() && (blocker.state() == RobotState::Idle || blocker.id() > robot.id());
  if (blockerYields && stepAside(blocker)) {
    robot.clearBlocked();
    return true;
  }
  if (!tryRepath(robot, occupiedCells())) {
    // A charger taken by a robot that is charging will not be free for a while: pick another.
    if (robot.goal() != NavigationGoal::Charger || !retargetCharger(robot)) {
      robot.clearBlocked();  // wait a few more ticks before trying again
    }
  }
  return false;
}

bool Simulation::stepAside(Robot& robot) {
  const std::vector<Point> options = world_.freeNeighbors(robot.position());
  if (options.empty()) {
    return false;
  }
  if (world_.moveRobot(robot.id(), options.front()) != world::MoveStatus::Ok) {
    throw std::logic_error("world rejected a robot sidestep");
  }
  robot.sidestep(options.front());
  if (robot.state() == RobotState::Navigating && !tryRepath(robot, occupiedCells()) &&
      !tryRepath(robot, {})) {
    abortNavigation(robot);
  }
  return true;
}

planning::BlockedSet Simulation::occupiedCells() const {
  const std::vector<Point> cells = world_.occupancy().occupiedCells();
  return {cells.begin(), cells.end()};
}

bool Simulation::tryRepath(Robot& robot, const planning::BlockedSet& blocked) {
  auto path = planner_->findPath(world_.layout(), robot.position(), *robot.destination(), blocked);
  if (!path) {
    return false;
  }
  robot.setPath(std::move(*path));
  return true;
}

bool Simulation::retargetCharger(Robot& robot) {
  auto trip = nearestFreeCharger(robot);
  if (!trip || trip->first == robot.destination()) {
    return false;
  }
  robot.redirectChargerTrip(trip->first, std::move(trip->second));
  return true;
}

// Hands back whatever the robot was working on because it is about to leave service: a task
// it had not collected yet becomes available to others, one it was carrying is lost.
void Simulation::releaseTask(Robot& robot) {
  if (!robot.task()) {
    return;
  }
  const core::TaskId id = *robot.task();
  switch (tasks_.find(id)->status) {
    case tasks::TaskStatus::Assigned:
      tasks_.release(id);
      break;
    case tasks::TaskStatus::InProgress:
      tasks_.fail(id);
      break;
    default:
      break;
  }
}

void Simulation::failTask(Robot& robot) {
  tasks_.fail(*robot.task());
  robot.abortTask();
  syncHolds(robot);
}

// The robot's destination became unreachable: fail its task (if any) and stand it down.
void Simulation::abortNavigation(Robot& robot) {
  if (robot.task()) {
    failTask(robot);
  } else {
    robot.abortTask();
    syncHolds(robot);
  }
}

}  // namespace warehousesim::simulation
