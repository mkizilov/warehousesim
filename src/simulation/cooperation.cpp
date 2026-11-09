// The parts of Simulation that plan together: cooperative route search under a work limit, keeping
// watch for robots that cannot get anywhere, getting out of deadlocks, and repairing routes when
// the layout changes.
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <utility>

#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::simulation {

using core::Point;
using core::RobotId;
using robots::NavigationGoal;
using robots::Robot;
using robots::RobotState;

// ---- cooperative search under a work limit ------------------------------------------------------

// Each step the searches share `planningWorkPerStep`. Robots that were put off earlier are served
// first: the longest-waiting few get a share of their own (one search's worth each), and everybody
// else draws on what is left, in the order they take their turn.
void Simulation::beginPlanning() {
  plannerBase_ = plannerWork();
  stepWork_ = 0;
  std::fill(reservedShare_.begin(), reservedShare_.end(), false);

  std::vector<std::size_t> putOff;
  for (std::size_t i = 0; i < deferred_.size(); ++i) {
    if (deferred_[i] > 0) {
      putOff.push_back(i);
    }
  }
  std::stable_sort(putOff.begin(), putOff.end(),
                   [this](std::size_t a, std::size_t b) { return deferred_[a] > deferred_[b]; });
  const std::size_t shares =
      std::min(putOff.size(), config_.planningWorkPerStep / config_.planningWorkPerSearch);
  for (std::size_t k = 0; k < shares; ++k) {
    reservedShare_[putOff[k]] = true;
  }
  workPool_ = config_.planningWorkPerStep - shares * config_.planningWorkPerSearch;
}

// The robot has been planned by the table (and has reserved what it forecasts). If the table holds
// it back, it looks for a better way against the reservations of the robots before it, and, if it
// finds one, takes it and reserves that instead.
traffic::TrafficPlan Simulation::cooperate(Robot& robot, const traffic::TrafficPlan& plan) {
  if (!config_.cooperative || !plan.waiting() || robot.state() != RobotState::Navigating ||
      !robot.destination()) {
    return plan;
  }
  const std::size_t i = robot.id().value - 1;
  const core::Tick now = clock_.tick();
  if (now < nextSearch_[i]) {
    return plan;
  }
  // A robot that has been stuck for long may use twice as much work: its trouble may be that the
  // search is bigger than usual.
  const bool escalated = progress_.stuckTicks(robot.id()) >= config_.prolongedBlockTicks;

  const std::size_t wanted =
      config_.planningWorkPerSearch * workScale_[i] * (escalated ? std::size_t{2} : std::size_t{1});
  const std::size_t own = reservedShare_[i] ? config_.planningWorkPerSearch : 0;
  const std::size_t limit = std::min(wanted, own + workPool_);
  if (limit == 0) {
    ++deferred_[i];
    ++planningStats_.deferred;
    return plan;
  }

  // The cells to steer clear of are only needed by a plan that falls short of the destination, so
  // they are made on demand.
  const planning::BlockedSet none;
  const std::function<planning::BlockedSet()> avoidLater = [this, &robot] {
    return stationaryCells(robot.id());
  };
  const traffic::CooperativeRouterOptions options{traffic_.horizon(), limit, config_.maxDetourCells,
                                                  config_.maxDodgeCells};
  const traffic::RouteOutcome outcome = router_.reroute(
      {world_.layout(), traffic_.table(), robot, now, *planner_, none, options, &avoidLater});

  ++planningStats_.searches;
  planningStats_.work += outcome.work;
  stepWork_ += outcome.work;
  workPool_ -= std::min(workPool_, outcome.work > own ? outcome.work - own : 0);
  deferred_[i] = 0;
  nextSearch_[i] = now + config_.replanInterval;

  switch (outcome.kind) {
    case traffic::RouteOutcomeKind::Rerouted:
      robot.setPath(outcome.path);
      workScale_[i] = 1;
      ++replans_;
      ++planningStats_.reroutes;
      planningStats_.dodges += outcome.dodge ? 1 : 0;
      planningStats_.partialPlans += outcome.partial ? 1 : 0;
      return traffic_.replan(robot);
    case traffic::RouteOutcomeKind::OutOfBudget:
      // It needed more than this much: next time it may use twice as much, if the step has it.
      ++planningStats_.outOfBudget;
      workScale_[i] = std::min<std::uint32_t>(workScale_[i] * 2, 8);
      nextSearch_[i] = now + 1;
      break;
    default:
      workScale_[i] = 1;
      break;
  }
  return plan;
}

// Where robots are standing for good (or at least for a while): a plan that goes on beyond the
// window avoids these where it can.
planning::BlockedSet Simulation::stationaryCells(RobotId except) const {
  planning::BlockedSet cells;
  for (const Robot& other : robots_) {
    if (other.id() != except && !other.hasPath()) {
      cells.insert(other.position());
    }
  }
  return cells;
}

// ---- watching for robots that get nowhere -------------------------------------------------------

void Simulation::recordProgress(const Robot& robot, const traffic::TrafficPlan& plan) {
  const bool wants = robot.state() == RobotState::Navigating && robot.hasPath();
  // Who it is waiting for: the robot whose reservation holds it back, else whoever is in the cell
  // it wants next (not itself: a driving robot holds the cells ahead of it).
  std::optional<RobotId> waitsFor;
  if (wants) {
    if (plan.waiting() && !plan.inTheWay && plan.conflict->other) {
      waitsFor = plan.conflict->other;
    } else {
      waitsFor = world_.occupantAt(*robot.nextStep());
    }
    if (waitsFor == robot.id()) {
      waitsFor.reset();
    }
  }
  progress_.record(robot.id(), wants, robot.position(), waitsFor);
}

// After every robot has had its turn: note robots that have been stuck for long, get rings of
// robots that wait for each other out of the way, and let robots that are hopelessly stuck give up.
void Simulation::supervise() {
  planningStats_.peakStepWork =
      std::max(planningStats_.peakStepWork, stepWork_ + (plannerWork() - plannerBase_));

  for (const Robot& robot : robots_) {
    if (progress_.stuckTicks(robot.id()) == config_.prolongedBlockTicks) {
      ++planningStats_.prolongedBlocks;
    }
  }

  // A ring is dealt with once every deadlockTicks it stays: first one robot is moved aside, and if
  // that did not end it, the robots in it take turns at being put first.
  const auto rings = progress_.deadlocks(config_.deadlockTicks);
  for (const auto& ring : rings) {
    const bool known =
        std::find(ringsLastStep_.begin(), ringsLastStep_.end(), ring) != ringsLastStep_.end();
    planningStats_.deadlocks += known ? 0 : 1;
    std::uint32_t stuck = progress_.stuckTicks(ring.front());
    for (const RobotId member : ring) {
      stuck = std::min(stuck, progress_.stuckTicks(member));
    }
    if (stuck % config_.deadlockTicks != 0) {
      continue;
    }
    if (stuck / config_.deadlockTicks == 1) {
      if (recoverDeadlock(ring)) {
        ++planningStats_.deadlocksRecovered;
      } else {
        ++planningStats_.deadlocksUnresolved;
      }
    } else {
      boostOneOf(ring);
    }
  }
  ringsLastStep_ = rings;

  for (Robot& robot : robots_) {
    if (robot.state() == RobotState::Navigating &&
        progress_.stuckTicks(robot.id()) >= config_.abandonTicks) {
      abandon(robot);
    }
  }
}

// Robots in a ring each need the cell of the next, so none can go until one of them gives way. The
// one with the lowest priority (highest id) that has somewhere to go is moved aside; if nobody has
// a free neighbour the ring stays as it is, and is counted as unresolved.
bool Simulation::recoverDeadlock(const std::vector<RobotId>& ring) {
  std::vector<RobotId> byPriority = ring;
  std::sort(byPriority.begin(), byPriority.end(),
            [this](RobotId a, RobotId b) { return outranks(b, a); });
  for (const RobotId id : byPriority) {
    Robot& robot = robotById(id);
    // A sidestep is a whole cell, so only a robot that stood still this step can be given one.
    if (robot.isStationary() || movedThisStep_[id.value - 1] || robot.authority() > 0 ||
        robot.segmentProgress() > 0.0) {
      continue;
    }
    if (stepAside(robot)) {
      return true;
    }
  }
  return false;
}

// The lowest-priority robot of the ring that is not first in the order already is put first for a
// while. It plans before the others, so they see its route and make way for it: what a ring needs
// when the robot with priority is the one in the other's way.
void Simulation::boostOneOf(const std::vector<RobotId>& ring) {
  std::vector<RobotId> byPriority = ring;
  std::sort(byPriority.begin(), byPriority.end(),
            [this](RobotId a, RobotId b) { return outranks(b, a); });
  for (const RobotId id : byPriority) {
    const std::size_t i = id.value - 1;
    if (boostUntil_[i] <= clock_.tick()) {
      boostUntil_[i] = clock_.tick() + config_.priorityBoostTicks;
      nextSearch_.assign(nextSearch_.size(), 0);  // everybody looks again under the new order
      ++planningStats_.priorityBoosts;
      return;
    }
  }
}

// A robot that has been stuck for abandonTicks hands its task back, so that the queue and the
// other robots are not held up by one that cannot get there.
void Simulation::abandon(Robot& robot) {
  ++planningStats_.abandoned;
  releaseTask(robot);
  robot.abortTask();
  syncHolds(robot);
  progress_.reset(robot.id());
}

// ---- the layout changing under the robots -------------------------------------------------------

void Simulation::rebuildTraffic() {
  trafficCarried_ += traffic_.stats();
  traffic_ = traffic::TrafficController(world_.layout(), config_.reservationHorizon);
}

// Every robot whose remaining route now runs through an obstacle gets a new one from the cell it
// has been granted up to. Without a way to its destination its task goes back to the queue (it had
// not collected it) or fails (it had), and the robot stands down.
//
// The new routes are independent questions about the new layout (none depends on what happened to
// another robot), so they are all asked at once, on the planning threads if there are any, and then
// applied in robot order.
void Simulation::repairRoutes() {
  std::vector<Robot*> broken;
  queries_.clear();
  for (Robot& robot : robots_) {
    if (robot.state() != RobotState::Navigating || !robot.hasPath()) {
      continue;
    }
    const bool cut = std::any_of(robot.path().begin(), robot.path().end(),
                                 [this](Point cell) { return !world_.isWalkable(cell); });
    if (!cut) {
      continue;
    }
    ++planningStats_.obstacleRepairs;
    const std::size_t held = robot.authority();
    broken.push_back(&robot);
    queries_.push_back(
        {held == 0 ? robot.position() : robot.path()[held - 1], *robot.destination()});
  }
  if (broken.empty()) {
    return;
  }
  planRoutes(queries_);
  for (std::size_t i = 0; i < broken.size(); ++i) {
    Robot& robot = *broken[i];
    if (answers_[i].found()) {
      const std::size_t held = robot.authority();
      robots::RobotPath path(robot.path().begin(),
                             robot.path().begin() + static_cast<std::ptrdiff_t>(held));
      path.insert(path.end(), answers_[i].path.begin(), answers_[i].path.end());
      robot.setPath(std::move(path));
      nextSearch_[robot.id().value - 1] = clock_.tick();
    } else if (robot.goal() == NavigationGoal::Pickup && robot.task()) {
      releaseTask(robot);
      robot.abortTask();
      syncHolds(robot);
    } else {
      abortNavigation(robot);
    }
  }
}

bool Simulation::placeObstacle(Point cell, world::CellType type) {
  if (!world::isObstacle(type)) {
    throw std::invalid_argument("placeObstacle needs a wall or a shelf");
  }
  if (!world_.layout().inBounds(cell) || world_.layout().at(cell) != world::CellType::Free ||
      world_.isOccupied(cell)) {
    return false;
  }
  if (world_.placeObstacle(cell, type) != world::CellEdit::Ok) {
    return false;
  }
  // Tasks that start or end on the cell can no longer be done.
  std::vector<core::TaskId> doomed;
  for (const tasks::Task& task : tasks_.all()) {
    if (task.isOpen() && (task.pickup == cell || task.dropoff == cell)) {
      doomed.push_back(task.id);
    }
  }
  for (const core::TaskId id : doomed) {
    (void)cancelTask(id);
  }
  rebuildTraffic();
  repairRoutes();
  return true;
}

bool Simulation::clearObstacle(Point cell) {
  if (!world_.layout().inBounds(cell) || !world_.layout().isObstacle(cell)) {
    return false;
  }
  if (world_.clearCell(cell) != world::CellEdit::Ok) {
    return false;
  }
  rebuildTraffic();
  return true;
}

}  // namespace warehousesim::simulation
