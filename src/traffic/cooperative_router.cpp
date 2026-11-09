#include "warehousesim/traffic/cooperative_router.hpp"

#include <algorithm>
#include <optional>
#include <utility>

#include "warehousesim/robots/forecast.hpp"

namespace warehousesim::traffic {

using core::Point;
using core::Tick;

namespace {

// The shortest way on the empty map, steering clear of `avoid` if that is possible at all.
std::optional<std::vector<Point>> plainRoute(planning::PathPlanner& planner,
                                             const world::Warehouse& layout, Point from, Point to,
                                             const planning::BlockedSet& avoid) {
  if (from == to) {
    return std::vector<Point>{};
  }
  auto path = planner.findPath(layout, from, to, avoid);
  if (!path && !avoid.empty()) {
    path = planner.findPath(layout, from, to, planning::BlockedSet{});
  }
  return path;
}

}  // namespace

bool TableConstraints::cellFree(Point cell, Tick step) const {
  return table_.isFree(self_, cell, step);
}

bool TableConstraints::edgeFree(Point from, Point to, Tick step) const {
  return table_.isTraversable(self_, from, to, step);
}

const char* toString(RouteOutcomeKind kind) noexcept {
  switch (kind) {
    case RouteOutcomeKind::Skipped:
      return "skipped";
    case RouteOutcomeKind::Kept:
      return "kept";
    case RouteOutcomeKind::Rerouted:
      return "rerouted";
    case RouteOutcomeKind::NoPlan:
      return "no-plan";
    case RouteOutcomeKind::TooLong:
      return "too-long";
    case RouteOutcomeKind::OutOfBudget:
      return "out-of-budget";
  }
  return "unknown";
}

RouteFollow followRoute(const planning::SpaceTimeConstraints& free, Point start, Tick startStep,
                        const std::vector<Point>& rest, bool toGoal, std::size_t window,
                        const std::function<std::int32_t(Point)>& distance) {
  RouteFollow result;
  const std::size_t last = rest.size();  // index 0 is `start`, index i is rest[i - 1]
  const auto cellAt = [&](std::size_t index) { return index == 0 ? start : rest[index - 1]; };

  std::vector<char> layer(last + 1, 0);
  std::vector<char> next(last + 1, 0);
  layer[0] = 1;
  for (std::size_t depth = 0;; ++depth) {
    if (toGoal && last > 0 && layer[last] != 0) {
      bool stays = true;
      for (std::size_t later = depth + 1; later <= window && stays; ++later) {
        stays = free.cellFree(rest.back(), startStep + later);
      }
      if (stays) {
        result.feasible = true;
        result.reaches = true;
        result.arrival = depth;
        return result;
      }
    }
    if (depth == window) {
      break;
    }
    std::fill(next.begin(), next.end(), char{0});
    bool any = false;
    const Tick step = startStep + depth;
    for (std::size_t i = 0; i <= last; ++i) {
      if (layer[i] == 0) {
        continue;
      }
      ++result.work;
      const Point here = cellAt(i);
      if (free.cellFree(here, step + 1)) {
        next[i] = 1;
        any = true;
      }
      if (i < last) {
        const Point there = cellAt(i + 1);
        if (free.cellFree(there, step) && free.cellFree(there, step + 1) &&
            free.edgeFree(here, there, step)) {
          next[i + 1] = 1;
          any = true;
        }
      }
    }
    if (!any) {
      return result;  // not feasible: it would have to leave the route to survive
    }
    layer.swap(next);
  }

  result.feasible = true;
  result.remaining = -1;
  for (std::size_t i = 0; i <= last; ++i) {
    if (layer[i] != 0) {
      const std::int32_t d = distance(cellAt(i));
      if (d >= 0 && (result.remaining < 0 || d < result.remaining)) {
        result.remaining = d;
      }
    }
  }
  if (result.remaining < 0) {
    result.remaining = 0;
  }
  return result;
}

RouteOutcome CooperativeRouter::reroute(const Request& request) {
  RouteOutcome out;
  const robots::Robot& robot = request.robot;
  const CooperativeRouterOptions& options = request.options;
  if (robot.state() != robots::RobotState::Navigating || !robot.destination() || !robot.hasPath()) {
    return out;
  }
  const Point destination = *robot.destination();
  const std::vector<Point> route(robot.path().begin(), robot.path().end());
  const std::size_t held = robot.authority();

  // The plan begins where the cells the robot has been granted end, when it gets there.
  Point startCell = robot.position();
  Tick startStep = request.now;
  if (held > 0) {
    startCell = route[held - 1];
    if (!walkerSlot_) {
      walkerSlot_.emplace(robot);
    } else {
      *walkerSlot_ = robot;
    }
    robots::forecastHolds(*walkerSlot_, options.window, forecast_);
    std::size_t arrives = 0;
    while (arrives < forecast_.steps() && forecast_.holdsAt(arrives).front() != startCell) {
      ++arrives;
    }
    if (arrives == forecast_.steps()) {
      return out;
    }
    startStep = request.now + static_cast<Tick>(arrives);
  }
  if (startCell == destination) {
    out.kind = RouteOutcomeKind::Kept;
    return out;
  }

  const TableConstraints free(request.table, robot.id());

  // Everything below needs the distances to the destination; the first time, they cost a fill of
  // the whole map.
  const std::optional<std::size_t> fill =
      search_.prepare(request.layout, destination, options.maxWork);
  if (!fill) {
    out.kind = RouteOutcomeKind::OutOfBudget;
    return out;
  }
  out.work = *fill;

  // First the cheap part: how the current route fares. Within the window a robot cannot get more
  // than `window` cells along it, so only that much of it matters (and costs).
  const std::size_t restSize = route.size() - held;
  const std::size_t considered = std::min(restSize, options.window);
  const std::vector<Point> rest(route.begin() + static_cast<std::ptrdiff_t>(held),
                                route.begin() + static_cast<std::ptrdiff_t>(held + considered));
  const RouteFollow follow =
      followRoute(free, startCell, startStep, rest, considered == restSize, options.window,
                  [&](Point cell) { return search_.distance(request.layout, cell, destination); });
  out.work += follow.work;
  if (options.maxWork != 0 && out.work >= options.maxWork) {
    out.kind = RouteOutcomeKind::OutOfBudget;
    return out;
  }

  // A robot stops at its destination the first time it gets there, so the plan may not pass it.
  const planning::SpaceTimeOptions searchOptions{
      options.window, options.maxWork == 0 ? 0 : options.maxWork - out.work, true};
  const planning::SpaceTimePlan plan =
      search_.search(request.layout, startCell, destination, startStep, free, searchOptions);
  out.work += plan.work;
  if (plan.status == planning::SpaceTimeStatus::BudgetExceeded) {
    out.kind = RouteOutcomeKind::OutOfBudget;
    return out;
  }
  if (plan.status != planning::SpaceTimeStatus::Found &&
      plan.status != planning::SpaceTimeStatus::Partial) {
    out.kind = RouteOutcomeKind::NoPlan;
    return out;
  }

  bool better = false;
  if (!follow.feasible) {
    better = true;
  } else if (plan.status == planning::SpaceTimeStatus::Found) {
    better = !follow.reaches || plan.duration() < follow.arrival;
  } else {
    better = plan.remaining < follow.remaining;
  }
  if (!better) {
    out.kind = RouteOutcomeKind::Kept;
    return out;
  }

  std::vector<Point> path(route.begin(), route.begin() + static_cast<std::ptrdiff_t>(held));
  Point last = startCell;
  for (std::size_t i = 1; i < plan.timeline.size(); ++i) {
    if (plan.timeline[i] != last) {
      last = plan.timeline[i];
      path.push_back(last);
    }
  }
  if (plan.status == planning::SpaceTimeStatus::Partial) {
    planning::BlockedSet avoidNow;
    if (request.avoidLater != nullptr) {
      avoidNow = (*request.avoidLater)();
    }
    auto tail = plainRoute(request.tailPlanner, request.layout, last, destination,
                           request.avoidLater != nullptr ? avoidNow : request.avoid);
    if (!tail) {
      out.kind = RouteOutcomeKind::NoPlan;
      return out;
    }
    path.insert(path.end(), tail->begin(), tail->end());
    out.partial = true;
  }
  if (path == route) {
    out.kind = RouteOutcomeKind::Kept;
    return out;
  }
  out.dodge = !follow.feasible;
  const std::size_t allowance = out.dodge ? options.maxDodgeCells : options.maxDetourCells;
  if (path.size() > route.size() + allowance) {
    out.kind = RouteOutcomeKind::TooLong;
    return out;
  }
  out.kind = RouteOutcomeKind::Rerouted;
  out.waits = plan.waits;
  out.path = std::move(path);
  return out;
}

}  // namespace warehousesim::traffic
