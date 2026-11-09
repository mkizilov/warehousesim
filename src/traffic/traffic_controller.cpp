#include "warehousesim/traffic/traffic_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "warehousesim/robots/forecast.hpp"

namespace warehousesim::traffic {

using core::Point;
using core::RobotId;
using robots::Robot;

namespace {

// Turns per-step lists of held cells into requests, in `requests` (cleared first): the edges
// between consecutive held cells, then the cells themselves. Within a step the edges come first
// because that is how a head-on swap is told from a plain clash: two robots exchanging cells clash
// on both, and the first request that fails is the one reported. Cells held for several steps in a
// row become one request. `holdsAt(s)` is the cells held during step `first + s`, as a span;
// `latest` is working storage, kept by the caller so that building requests does not allocate once
// it has grown.
template <class HoldsAt>
void buildRequests(std::vector<Request>& requests,
                   std::vector<std::pair<Point, std::size_t>>& latest, std::size_t steps,
                   Step first, const HoldsAt& holdsAt) {
  requests.clear();
  latest.clear();  // cell -> index of its newest request
  for (std::size_t s = 0; s < steps; ++s) {
    const Step step = first + s;
    const std::span<const Point> cells = holdsAt(s);
    for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
      requests.emplace_back(Traverse{cells[i], cells[i + 1], step});
    }
    for (const Point cell : cells) {
      const auto known = std::find_if(latest.begin(), latest.end(),
                                      [&](const auto& entry) { return entry.first == cell; });
      if (known != latest.end() && std::get<Occupy>(requests[known->second]).steps.to == step) {
        std::get<Occupy>(requests[known->second]).steps.to = step + 1;
      } else if (known != latest.end()) {
        requests.emplace_back(Occupy{cell, {step, step + 1}});
        known->second = requests.size() - 1;
      } else {
        requests.emplace_back(Occupy{cell, {step, step + 1}});
        latest.emplace_back(cell, requests.size() - 1);
      }
    }
  }
}

// Standing still with nothing granted, so that it can hold off for a step and then set out.
bool atRest(const Robot& robot) {
  return robot.authority() == 0 && robot.speed() == 0.0 && robot.segmentProgress() == 0.0;
}

std::optional<std::size_t> indexInRoute(const std::vector<Point>& route, Point cell) {
  const auto it = std::find(route.begin(), route.end(), cell);
  if (it == route.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(it - route.begin());
}

}  // namespace

TrafficController::TrafficController(const world::Warehouse& warehouse, std::size_t horizon)
    : table_(warehouse), routeCount_(table_.size().cellCount(), 0), horizon_(horizon) {
  if (horizon < 2) {
    throw std::invalid_argument("the reservation horizon must be at least two steps");
  }
}

// The cell a robot stands on followed by the cells it has been granted ahead: what it holds, as the
// requests to put on the table for the current step.
const std::vector<Request>& TrafficController::currentHoldsRequests(const Robot& robot) {
  holds_.clear();
  holds_.push_back(robot.position());
  for (std::size_t i = 0; i < robot.authority(); ++i) {
    holds_.push_back(robot.path()[i]);
  }
  buildRequests(factRequests_, latest_, 1, now_,
                [this](std::size_t) { return std::span<const Point>(holds_); });
  return factRequests_;
}

// The working copy of `robot` that forecasts are made with (its route cut after `cutTo` cells, if
// given). Overwritten by the next call; kept so that making it does not allocate.
Robot& TrafficController::walker(const Robot& robot, std::optional<std::size_t> cutTo) {
  if (!walkerSlot_) {
    walkerSlot_.emplace(robot);
  }
  if (cutTo) {
    walkerSlot_->assignRouteCutOf(robot, *cutTo);
  } else {
    *walkerSlot_ = robot;
  }
  return *walkerSlot_;
}

void TrafficController::countRoute(const Point cell, int by) {
  const world::GridSize size = table_.size();
  if (!size.contains(cell)) {
    return;
  }
  std::uint32_t& count = routeCount_[size.indexOf(cell)];
  if (by > 0) {
    if (count++ == 0) {
      routeCells_.push_back(size.indexOf(cell));
    }
  } else {
    --count;
  }
}

bool TrafficController::onSomeRoute(const Point cell) const {
  const world::GridSize size = table_.size();
  return size.contains(cell) && routeCount_[size.indexOf(cell)] > 0;
}

void TrafficController::beginStep(core::Tick now, std::span<const Robot> fleet,
                                  std::span<const RobotId> priority) {
  table_.clear();
  for (const std::size_t index : routeCells_) {
    routeCount_[index] = 0;
  }
  routeCells_.clear();
  lastRoute_.clear();
  now_ = now;
  began_ = true;
  lastPlanned_.reset();

  // Ranks: the position in `priority`, then everybody else by id.
  std::uint32_t largest = 0;
  for (const Robot& robot : fleet) {
    largest = std::max(largest, robot.id().value);
  }
  rank_.assign(static_cast<std::size_t>(largest) + 1, 0);
  std::uint32_t next = 1;
  for (const RobotId id : priority) {
    if (id.value <= largest && rank_[id.value] == 0) {
      rank_[id.value] = next++;
    }
  }
  for (std::uint32_t id = 1; id <= largest; ++id) {
    if (rank_[id] == 0) {
      rank_[id] = next++;
    }
  }

  for (const Robot& robot : fleet) {
    if (table_.reserveAll(robot.id(), currentHoldsRequests(robot))) {
      throw std::logic_error("two robots hold the same cell");
    }
  }
  // Robots with nowhere to go stay where they are for the whole horizon.
  for (const Robot& robot : fleet) {
    if (!robot.hasPath()) {
      (void)table_.reserve(robot.id(), Occupy{robot.position(), {now + 1, now + horizon_}});
    }
  }
}

void TrafficController::relocate(const Robot& robot) {
  if (!began_) {
    throw std::logic_error("beginStep() must be called before relocating a robot");
  }
  const RobotId id = robot.id();
  table_.release(id);
  // If this clashes with a reservation that was made earlier in the step, leave the robot off the
  // table: plan() copes with a robot that holds a cell somebody has reserved.
  const bool placed = !table_.reserveAll(id, currentHoldsRequests(robot));
  if (placed && !robot.hasPath()) {
    (void)table_.reserve(id, Occupy{robot.position(), {now_ + 1, now_ + horizon_}});
  }
}

TrafficPlan TrafficController::plan(const Robot& robot) { return planAndRecord(robot, false); }

TrafficPlan TrafficController::replan(const Robot& robot) { return planAndRecord(robot, true); }

TrafficPlan TrafficController::planAndRecord(const Robot& robot, bool again) {
  if (again) {
    if (!lastPlanned_ || *lastPlanned_ != robot.id()) {
      throw std::logic_error("only the robot planned last can be planned again");
    }
    for (const Point cell : lastRoute_) {
      countRoute(cell, -1);
    }
  }
  TrafficPlan result = planRoute(robot, again);
  // Only now, so that a robot never counts its own route against itself.
  lastRoute_.assign(robot.path().begin(), robot.path().end());
  for (const Point cell : lastRoute_) {
    countRoute(cell, +1);
  }
  return result;
}

std::size_t TrafficController::reserveDelayed(const Robot& robot) {
  robots::forecastHolds(walker(robot), horizon_, forecast_);
  const std::size_t longest = std::min<std::size_t>(horizon_ / 2, 6);
  for (std::size_t delay = 1; delay <= longest; ++delay) {
    // The robot stays where it is for `delay` steps, then does what it was going to.
    const Point here = robot.position();
    buildRequests(delayRequests_, latest_, horizon_, now_, [&](std::size_t s) {
      return s < delay ? std::span<const Point>(&here, 1) : forecast_.holdsAt(s - delay);
    });
    if (!table_.earliestConflict(robot.id(), delayRequests_)) {
      (void)table_.reserveAll(robot.id(), delayRequests_);
      return delay;
    }
  }
  return 0;
}

TrafficPlan TrafficController::planRoute(const Robot& robot, bool replanning) {
  if (!began_) {
    throw std::logic_error("beginStep() must be called before planning");
  }
  if (robot.id().value >= rank_.size()) {
    throw std::logic_error("the robot was not part of this step");
  }
  if (!replanning && lastPlanned_ && !(rank_[robot.id().value] > rank_[lastPlanned_->value])) {
    throw std::logic_error("reservations must be made in order of priority");
  }
  lastPlanned_ = robot.id();
  const RobotId id = robot.id();

  // Start again from the facts: whatever this robot reserved before is replaced.
  table_.release(id);
  TrafficPlan result;
  result.clearedCells = robot.path().size();
  if (const auto clash = table_.reserveAll(id, currentHoldsRequests(robot))) {
    // Another robot reserved a cell this one holds. The facts were consistent when the step began,
    // so this robot must have been moved since (see relocate) onto a cell that a robot planned
    // before it was already counting on. Waiting cannot get it out of the way; it is left to the
    // world's own claims, and nothing is reserved for it.
    ++stats_.inTheWay;
    result.conflict = clash;
    result.inTheWay = true;
    result.clearedCells = robot.authority();
    return result;
  }
  if (!robot.hasPath()) {
    (void)table_.reserve(id, Occupy{robot.position(), {now_ + 1, now_ + horizon_}});
    return result;
  }

  robots::forecastHolds(walker(robot), horizon_, forecast_);
  buildRequests(wholeRequests_, latest_, horizon_, now_,
                [this](std::size_t s) { return forecast_.holdsAt(s); });
  const std::optional<Conflict> first = table_.earliestConflict(id, wholeRequests_);
  if (!first) {
    (void)table_.reserveAll(id, wholeRequests_);
    return result;
  }

  result.conflict = first;
  switch (first->kind) {
    case ConflictKind::Blocked:
      ++stats_.blocked;
      break;
    case ConflictKind::Vertex:
      ++stats_.vertex;
      break;
    case ConflictKind::Swap:
      ++stats_.swaps;
      break;
  }
  // Hold the robot back before the first cell it cannot have, and reserve where it will wait. If
  // waiting there clashes too, back up to before that cell, and so on.
  route_.assign(robot.path().begin(), robot.path().end());
  std::optional<std::size_t> blockedAt = indexInRoute(route_, first->cell);
  while (true) {
    if (!blockedAt || *blockedAt < robot.authority()) {
      ++stats_.inTheWay;
      result.inTheWay = true;
      result.clearedCells = robot.authority();
      return result;
    }
    const std::size_t keep = *blockedAt;  // the cells strictly before the one it cannot have
    // Do not wait on a cell that a robot with priority will drive through, even if that is further
    // ahead than the table looks: a robot waiting in a corridor would be in the way as soon as the
    // other got there. Wait earlier if there is an earlier place.
    const Point waitingCell = keep == 0 ? robot.position() : route_[keep - 1];
    if (keep > robot.authority() && onSomeRoute(waitingCell)) {
      blockedAt = keep - 1;
      continue;
    }
    robots::forecastHolds(walker(robot, keep), horizon_, forecast_);
    buildRequests(cutRequests_, latest_, horizon_, now_,
                  [this](std::size_t s) { return forecast_.holdsAt(s); });
    const std::optional<Conflict> again = table_.earliestConflict(id, cutRequests_);
    if (!again) {
      // A robot at rest that cannot take its next cell this step or the next (somebody is on it, or
      // about to be) reserves what it will do, which is to wait a little and then go, rather than
      // to stand where it is for the whole horizon. The robots after it then see that it means to
      // move off, and can get out of its way. A clash further off is not treated like this: waiting
      // that out would only push it past the end of the horizon.
      if (keep == 0 && atRest(robot) && first->step <= now_ + 1) {
        result.delaySteps = reserveDelayed(robot);
        if (result.delaySteps > 0) {
          result.clearedCells = 0;
          return result;
        }
      }
      (void)table_.reserveAll(id, cutRequests_);
      result.clearedCells = keep;
      return result;
    }
    blockedAt = indexInRoute(route_, again->cell);
    if (blockedAt && *blockedAt >= keep) {
      blockedAt = keep > 0 ? std::optional<std::size_t>(keep - 1) : std::nullopt;  // always back up
    }
  }
}

}  // namespace warehousesim::traffic
