#include "warehousesim/simulation/audit.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>

namespace warehousesim::simulation {

namespace {

constexpr double kEpsilon = 1e-6;
// A robot's centre is never further than this from the centre of the cell it is logically in: it
// leaves the cell's centre for the next one and the logical cell changes at the half-way mark.
constexpr double kMaxPoseOffset = 1.0 + kEpsilon;
// How far apart (in cells) two logical cells can be while the robots in them still overlap.
constexpr std::int32_t kOverlapReach = 3;

std::string cell(core::Point p) {
  return "(" + std::to_string(p.x) + "," + std::to_string(p.y) + ")";
}

std::string robot(std::size_t index) { return "robot " + std::to_string(index + 1); }

std::string task(core::TaskId id) { return "task " + std::to_string(id.value); }

double distance(core::Vec2 a, core::Vec2 b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

core::Vec2 centreOf(core::Point p) {
  return {static_cast<double>(p.x) + 0.5, static_cast<double>(p.y) + 0.5};
}

}  // namespace

InvariantAuditor::InvariantAuditor(world::Warehouse warehouse)
    : warehouse_(std::move(warehouse)), layoutVersion_(warehouse_.version()) {
  holderOf_.assign(warehouse_.size().cellCount(), -1);
  firstInCell_.assign(warehouse_.size().cellCount(), -1);
}

void InvariantAuditor::setLayout(world::Warehouse warehouse) {
  warehouse_ = std::move(warehouse);
  layoutVersion_ = warehouse_.version();
  holderOf_.assign(warehouse_.size().cellCount(), -1);
  firstInCell_.assign(warehouse_.size().cellCount(), -1);
}

void InvariantAuditor::report(std::string what) {
  ++found_;
  if (kept_.size() < kMaxKept) {
    kept_.push_back({tick_, std::move(what)});
  }
}

std::string InvariantAuditor::summary() const {
  if (found_ == 0) {
    return "audited " + std::to_string(observed_) + " states: no violations";
  }
  std::string text = "audited " + std::to_string(observed_) + " states: " + std::to_string(found_) +
                     " violation" + (found_ == 1 ? "" : "s") + ". First";
  const std::size_t shown = std::min<std::size_t>(kept_.size(), 5);
  text += shown == 1 ? ":" : " " + std::to_string(shown) + ":";
  for (std::size_t i = 0; i < shown; ++i) {
    text += "\n  tick " + std::to_string(kept_[i].tick) + ": " + kept_[i].what;
  }
  return text;
}

void InvariantAuditor::observe(const Snapshot& snapshot) {
  tick_ = snapshot.tick;
  ++observed_;
  checkRobots(snapshot);
  checkOverlaps(snapshot);
  checkTasks(snapshot);
  checkAgainstPrevious(snapshot);
  remember(snapshot);
}

void InvariantAuditor::observe(const Simulation& simulation) {
  if (simulation.warehouse().version() != layoutVersion_) {
    setLayout(simulation.warehouse());
  }
  const Snapshot snapshot = simulation.snapshot();
  observe(snapshot);

  const std::vector<robots::Robot>& robots = simulation.robots();
  for (std::size_t i = 0; i < robots.size(); ++i) {
    if (const auto problem = robots[i].inconsistency()) {
      report(robot(i) + " is not consistent: " + *problem);
    }
  }
  const auto& all = simulation.tasks().all();
  const std::size_t accounted = snapshot.completedTasks + snapshot.failedTasks +
                                snapshot.cancelledTasks + snapshot.activeTasks.size();
  if (accounted != all.size()) {
    report(std::to_string(all.size()) + " tasks were made but " + std::to_string(accounted) +
           " are accounted for");
  }
  for (std::size_t i = 0; i < all.size(); ++i) {
    if (all[i].id.value != i + 1) {
      report(task(all[i].id) + " is in place " + std::to_string(i + 1) + " of the queue");
    }
    if (all[i].createdAt > simulation.tick()) {
      report(task(all[i].id) + " was made at tick " + std::to_string(all[i].createdAt) +
             ", in the future");
    }
    if (i > 0 && all[i].createdAt < all[i - 1].createdAt) {
      report(task(all[i].id) + " was made before " + task(all[i - 1].id));
    }
  }
}

void InvariantAuditor::checkRobots(const Snapshot& s) {
  const double width = static_cast<double>(warehouse_.width());
  const double height = static_cast<double>(warehouse_.height());
  const world::GridSize size = warehouse_.size();
  std::vector<std::size_t> touched;

  for (std::size_t i = 0; i < s.robots.size(); ++i) {
    const RobotSnapshot& r = s.robots[i];
    if (r.id.value != i + 1) {
      report(robot(i) + " has id " + std::to_string(r.id.value));
    }
    if (!warehouse_.isWalkable(r.position)) {
      report(robot(i) + " is on " + cell(r.position) + ", which is a wall or off the map");
    }

    const core::Vec2 centre = r.center();
    const double heading = r.headingAngle();
    if (!std::isfinite(centre.x) || !std::isfinite(centre.y) || !std::isfinite(heading) ||
        !std::isfinite(r.speed)) {
      report(robot(i) + " has a position, heading or speed that is not a number");
      continue;
    }
    if (centre.x < -kEpsilon || centre.y < -kEpsilon || centre.x > width + kEpsilon ||
        centre.y > height + kEpsilon) {
      report(robot(i) + " is outside the map at (" + std::to_string(centre.x) + "," +
             std::to_string(centre.y) + ")");
    }
    if (distance(centre, centreOf(r.position)) > kMaxPoseOffset) {
      report(robot(i) + " is more than a cell away from " + cell(r.position) +
             ", the cell it is in");
    }
    if (r.speed < 0.0 || r.speed > 1.0 + kEpsilon) {
      report(robot(i) + " has speed " + std::to_string(r.speed) + ", outside 0 to 1 cells a tick");
    }
    if (r.batteryCapacity == 0 || r.batteryLevel > r.batteryCapacity) {
      report(robot(i) + " has a battery of " + std::to_string(r.batteryLevel) + " in " +
             std::to_string(r.batteryCapacity));
    }
    if (r.carrying && !r.task) {
      report(robot(i) + " carries cargo for no task");
    }

    if (r.holds.empty() || r.holds.front() != r.position) {
      report(robot(i) + " does not hold the cell it is on, " + cell(r.position));
    }
    for (std::size_t h = 0; h < r.holds.size(); ++h) {
      const core::Point held = r.holds[h];
      if (!warehouse_.isWalkable(held)) {
        report(robot(i) + " holds " + cell(held) + ", which is a wall or off the map");
        continue;
      }
      if (h > 0 && core::manhattanDistance(held, r.holds[h - 1]) != 1) {
        report(robot(i) + " holds " + cell(r.holds[h - 1]) + " and " + cell(held) +
               ", which are not next to each other");
      }
      const std::size_t index = size.indexOf(held);
      if (holderOf_[index] != -1) {
        report("cell " + cell(held) + " is held by " +
               robot(static_cast<std::size_t>(holderOf_[index])) + " and by " + robot(i));
      } else {
        holderOf_[index] = static_cast<std::int32_t>(i);
        touched.push_back(index);
      }
    }
  }
  for (const std::size_t index : touched) {
    holderOf_[index] = -1;
  }
}

void InvariantAuditor::checkOverlaps(const Snapshot& s) {
  const world::GridSize size = warehouse_.size();
  nextInCell_.assign(s.robots.size(), -1);
  std::vector<std::size_t> touched;
  for (std::size_t i = 0; i < s.robots.size(); ++i) {
    if (!warehouse_.inBounds(s.robots[i].position)) {
      continue;
    }
    const std::size_t index = size.indexOf(s.robots[i].position);
    nextInCell_[i] = firstInCell_[index];
    firstInCell_[index] = static_cast<std::int32_t>(i);
    touched.push_back(index);
  }

  for (std::size_t i = 0; i < s.robots.size(); ++i) {
    const core::Point at = s.robots[i].position;
    if (!warehouse_.inBounds(at)) {
      continue;
    }
    const core::Vec2 here = s.robots[i].center();
    for (std::int32_t dy = -kOverlapReach; dy <= kOverlapReach; ++dy) {
      for (std::int32_t dx = -kOverlapReach; dx <= kOverlapReach; ++dx) {
        const core::Point near{at.x + dx, at.y + dy};
        if (!warehouse_.inBounds(near)) {
          continue;
        }
        for (std::int32_t j = firstInCell_[size.indexOf(near)]; j != -1;
             j = nextInCell_[static_cast<std::size_t>(j)]) {
          const auto other = static_cast<std::size_t>(j);
          if (other <= i) {
            continue;
          }
          const double apart = distance(here, s.robots[other].center());
          if (apart < 1.0 - kEpsilon) {
            report(robot(i) + " and " + robot(other) + " overlap: their centres are " +
                   std::to_string(apart) + " cells apart");
          }
        }
      }
    }
  }
  for (const std::size_t index : touched) {
    firstInCell_[index] = -1;
  }
}

void InvariantAuditor::checkTasks(const Snapshot& s) {
  std::unordered_map<std::uint32_t, const tasks::Task*> active;  // looked up, never walked
  std::uint32_t highest = 0;
  for (const tasks::Task& t : s.activeTasks) {
    highest = std::max(highest, t.id.value);
    if (!active.emplace(t.id.value, &t).second) {
      report(task(t.id) + " is listed twice");
    }
    if (!t.isOpen()) {
      report(task(t.id) + " is " + toString(t.status) + " but listed as open");
    }
    if (t.createdAt > s.tick) {
      report(task(t.id) + " was made at tick " + std::to_string(t.createdAt) + ", in the future");
    }
    if (t.isAssigned()) {
      if (!t.assignee || t.assignee->value == 0 || t.assignee->value > s.robots.size()) {
        report(task(t.id) + " is " + toString(t.status) + " but belongs to no robot");
      } else if (s.robots[t.assignee->value - 1].task != std::optional<core::TaskId>(t.id)) {
        report(task(t.id) + " belongs to robot " + std::to_string(t.assignee->value) +
               ", which works on something else");
      }
    }
  }
  for (const tasks::Task& t : s.recentTasks) {
    highest = std::max(highest, t.id.value);
  }
  for (std::size_t i = 1; i < s.activeTasks.size(); ++i) {
    if (s.activeTasks[i].id.value <= s.activeTasks[i - 1].id.value) {
      report(task(s.activeTasks[i].id) + " is listed out of order");
    }
  }

  std::unordered_map<std::uint32_t, std::size_t> workedOnBy;
  for (std::size_t i = 0; i < s.robots.size(); ++i) {
    if (!s.robots[i].task) {
      continue;
    }
    const auto found = active.find(s.robots[i].task->value);
    if (found == active.end()) {
      report(robot(i) + " works on " + task(*s.robots[i].task) + ", which is not an open task");
      continue;
    }
    if (!found->second->isAssigned() || found->second->assignee != std::optional(s.robots[i].id)) {
      report(robot(i) + " works on " + task(*s.robots[i].task) + ", which is not its task");
    }
    const auto [where, fresh] = workedOnBy.emplace(s.robots[i].task->value, i);
    if (!fresh) {
      report(task(*s.robots[i].task) + " is worked on by " + robot(where->second) + " and by " +
             robot(i));
    }
  }

  const std::size_t accounted =
      s.completedTasks + s.failedTasks + s.cancelledTasks + s.activeTasks.size();
  if (accounted != highest) {
    report(std::to_string(highest) + " tasks were made but " + std::to_string(accounted) +
           " are accounted for (" + std::to_string(s.completedTasks) + " completed, " +
           std::to_string(s.failedTasks) + " failed, " + std::to_string(s.cancelledTasks) +
           " cancelled, " + std::to_string(s.activeTasks.size()) + " open)");
  }
}

void InvariantAuditor::checkAgainstPrevious(const Snapshot& s) {
  if (!haveBefore_) {
    return;
  }
  if (s.tick != beforeTick_ + 1) {
    report("tick " + std::to_string(s.tick) + " follows tick " + std::to_string(beforeTick_));
  }
  if (s.completedTasks < beforeCompleted_ || s.failedTasks < beforeFailed_ ||
      s.cancelledTasks < beforeCancelled_) {
    report("a count of finished tasks went down");
  }
  if (s.robots.size() != before_.size()) {
    report("the number of robots changed from " + std::to_string(before_.size()) + " to " +
           std::to_string(s.robots.size()));
    return;
  }
  for (std::size_t i = 0; i < s.robots.size(); ++i) {
    const RobotSnapshot& r = s.robots[i];
    const RobotMemory& was = before_[i];
    if (!std::isfinite(r.center().x) || !std::isfinite(r.center().y)) {
      continue;  // already reported
    }
    if (distance(r.center(), was.center) > 1.0 + kEpsilon) {
      report(robot(i) + " jumped " + std::to_string(distance(r.center(), was.center)) +
             " cells in one tick");
    }
    if (core::manhattanDistance(r.position, was.cell) > 1) {
      report(robot(i) + " went from " + cell(was.cell) + " to " + cell(r.position) +
             " in one tick");
    }
    if (r.deliveries < was.deliveries) {
      report(robot(i) + "'s count of deliveries went down");
    }
    for (std::size_t j = i + 1; j < s.robots.size(); ++j) {
      const bool iMoved = r.position != was.cell;
      if (iMoved && r.position == before_[j].cell && s.robots[j].position == was.cell) {
        report(robot(i) + " and " + robot(j) + " swapped cells " + cell(was.cell) + " and " +
               cell(r.position));
      }
    }
  }
}

void InvariantAuditor::remember(const Snapshot& s) {
  haveBefore_ = true;
  beforeTick_ = s.tick;
  beforeCompleted_ = s.completedTasks;
  beforeFailed_ = s.failedTasks;
  beforeCancelled_ = s.cancelledTasks;
  before_.clear();
  before_.reserve(s.robots.size());
  for (const RobotSnapshot& r : s.robots) {
    before_.push_back({r.position, r.center(), r.deliveries});
  }
}

}  // namespace warehousesim::simulation
