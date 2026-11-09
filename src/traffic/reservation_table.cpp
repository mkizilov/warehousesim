#include "warehousesim/traffic/reservation_table.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <stdexcept>
#include <type_traits>

namespace warehousesim::traffic {

using core::Point;
using core::RobotId;

namespace {

// North, east, south, west.
constexpr std::array<Point, 4> kDirections = {{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

// Which of the four neighbours `to` is of `from`; they must be adjacent.
std::uint8_t directionIndex(Point from, Point to) {
  const Point step{to.x - from.x, to.y - from.y};
  for (std::uint8_t i = 0; i < 3; ++i) {
    if (kDirections[i] == step) {
      return i;
    }
  }
  return 3;
}

}  // namespace

const char* toString(ConflictKind kind) noexcept {
  switch (kind) {
    case ConflictKind::Blocked:
      return "blocked";
    case ConflictKind::Vertex:
      return "vertex";
    case ConflictKind::Swap:
      return "swap";
  }
  return "unknown";
}

ReservationTable::ReservationTable(const world::Warehouse& warehouse)
    : size_(warehouse.size()),
      walkable_(size_.cellCount()),
      cells_(size_.cellCount()),
      edges_(size_.cellCount()),
      isTouched_(size_.cellCount(), 0),
      hasEdges_(size_.cellCount(), 0) {
  for (std::size_t i = 0; i < walkable_.size(); ++i) {
    walkable_[i] = warehouse.isWalkable(size_.pointAt(i));
  }
}

bool ReservationTable::isWalkable(Point cell) const noexcept {
  return size_.contains(cell) && walkable_[size_.indexOf(cell)];
}

std::optional<Conflict> ReservationTable::checkOccupy(RobotId robot, const Occupy& request) const {
  if (request.steps.empty()) {
    throw std::invalid_argument("an occupancy reservation needs at least one step");
  }
  if (!isWalkable(request.cell)) {
    return Conflict{ConflictKind::Blocked, request.cell, request.steps.from, std::nullopt};
  }
  std::optional<Conflict> worst;
  for (const Entry& entry : cells_[size_.indexOf(request.cell)]) {
    if (entry.robot == robot || !entry.steps.overlaps(request.steps)) {
      continue;
    }
    const Step step = std::max(entry.steps.from, request.steps.from);
    // Earliest clash first, then the lowest robot id, so that the answer never depends on the
    // order in which the entries happen to be stored.
    if (!worst || step < worst->step || (step == worst->step && entry.robot < *worst->other)) {
      worst = Conflict{ConflictKind::Vertex, request.cell, step, entry.robot};
    }
  }
  return worst;
}

std::optional<Conflict> ReservationTable::checkTraverse(RobotId robot,
                                                        const Traverse& request) const {
  if (core::manhattanDistance(request.from, request.to) != 1) {
    throw std::invalid_argument("a traversal must be between adjacent cells");
  }
  if (!isWalkable(request.to)) {
    return Conflict{ConflictKind::Blocked, request.to, request.step, std::nullopt};
  }
  if (!size_.contains(request.from)) {
    return Conflict{ConflictKind::Blocked, request.from, request.step, std::nullopt};
  }
  // The opposite traversal starts at `to` and comes back to `from`.
  const std::uint8_t back = directionIndex(request.to, request.from);
  std::optional<Conflict> found;
  for (const EdgeEntry& entry : edges_[size_.indexOf(request.to)]) {
    if (entry.direction == back && entry.robot != robot && entry.step == request.step &&
        (!found || entry.robot < *found->other)) {
      found = Conflict{ConflictKind::Swap, request.to, request.step, entry.robot};
    }
  }
  return found;
}

bool ReservationTable::isFree(RobotId robot, Point cell, Step step) const noexcept {
  if (!isWalkable(cell)) {
    return false;
  }
  for (const Entry& entry : cells_[size_.indexOf(cell)]) {
    if (entry.robot != robot && entry.steps.contains(step)) {
      return false;
    }
  }
  return true;
}

bool ReservationTable::isTraversable(RobotId robot, Point from, Point to,
                                     Step step) const noexcept {
  if (!isWalkable(to) || !size_.contains(from)) {
    return false;
  }
  const std::uint8_t back = directionIndex(to, from);
  for (const EdgeEntry& entry : edges_[size_.indexOf(to)]) {
    if (entry.direction == back && entry.robot != robot && entry.step == step) {
      return false;
    }
  }
  return true;
}

std::optional<Conflict> ReservationTable::check(RobotId robot, const Request& request) const {
  if (!robot.valid()) {
    throw std::invalid_argument("reservations need a valid robot id");
  }
  return std::visit(
      [&](const auto& typed) {
        using T = std::decay_t<decltype(typed)>;
        if constexpr (std::is_same_v<T, Occupy>) {
          return checkOccupy(robot, typed);
        } else {
          return checkTraverse(robot, typed);
        }
      },
      request);
}

std::optional<Conflict> ReservationTable::checkAll(RobotId robot,
                                                   std::span<const Request> requests) const {
  for (const Request& request : requests) {
    if (const auto conflict = check(robot, request)) {
      return conflict;
    }
  }
  return std::nullopt;
}

std::optional<Conflict> ReservationTable::earliestConflict(
    RobotId robot, std::span<const Request> requests) const {
  // At the same step: a wall beats a swap beats a plain clash.
  const auto rank = [](ConflictKind kind) {
    return kind == ConflictKind::Blocked ? 0 : (kind == ConflictKind::Swap ? 1 : 2);
  };
  std::optional<Conflict> best;
  for (const Request& request : requests) {
    const std::optional<Conflict> conflict = check(robot, request);
    if (conflict && (!best || conflict->step < best->step ||
                     (conflict->step == best->step && rank(conflict->kind) < rank(best->kind)))) {
      best = conflict;
    }
  }
  return best;
}

void ReservationTable::insert(RobotId robot, const Request& request) {
  if (const auto* occupy = std::get_if<Occupy>(&request)) {
    const std::size_t index = size_.indexOf(occupy->cell);
    std::vector<Entry>& entries = cells_[index];
    if (isTouched_[index] == 0) {
      isTouched_[index] = 1;
      touched_.push_back(index);
    }
    const bool listed = std::any_of(entries.begin(), entries.end(),
                                    [robot](const Entry& e) { return e.robot == robot; });
    // Merge with the robot's own reservations that overlap or touch this one.
    StepRange merged = occupy->steps;
    for (bool again = true; again;) {
      again = false;
      for (auto it = entries.begin(); it != entries.end(); ++it) {
        if (it->robot == robot && it->steps.from <= merged.to && merged.from <= it->steps.to) {
          merged = {std::min(merged.from, it->steps.from), std::max(merged.to, it->steps.to)};
          entries.erase(it);
          again = true;
          break;
        }
      }
    }
    entries.push_back({robot, merged});
    if (!listed) {
      holdingsOf(robot).cells.push_back(index);
    }
    return;
  }
  const auto& traverse = std::get<Traverse>(request);
  const std::size_t from = size_.indexOf(traverse.from);  // check() has seen that it is on the grid
  const std::uint8_t direction = directionIndex(traverse.from, traverse.to);
  std::vector<EdgeEntry>& entries = edges_[from];
  const bool known = std::any_of(entries.begin(), entries.end(), [&](const EdgeEntry& e) {
    return e.robot == robot && e.step == traverse.step && e.direction == direction;
  });
  if (known) {
    return;
  }
  if (hasEdges_[from] == 0) {
    hasEdges_[from] = 1;
    touchedEdgeCells_.push_back(from);
  }
  const bool listed = std::any_of(entries.begin(), entries.end(),
                                  [robot](const EdgeEntry& e) { return e.robot == robot; });
  entries.push_back({robot, traverse.step, direction});
  ++traversals_;
  if (!listed) {
    holdingsOf(robot).edgeCells.push_back(from);
  }
}

ReservationTable::Holdings& ReservationTable::holdingsOf(RobotId robot) {
  return holdings_[robot.value];
}

std::optional<Conflict> ReservationTable::reserve(RobotId robot, const Request& request) {
  if (const auto conflict = check(robot, request)) {
    return conflict;
  }
  insert(robot, request);
  return std::nullopt;
}

std::optional<Conflict> ReservationTable::reserveAll(RobotId robot,
                                                     std::span<const Request> requests) {
  if (const auto conflict = checkAll(robot, requests)) {
    return conflict;
  }
  for (const Request& request : requests) {
    insert(robot, request);
  }
  return std::nullopt;
}

void ReservationTable::release(RobotId robot) {
  const auto found = holdings_.find(robot.value);
  if (found == holdings_.end()) {
    return;
  }
  Holdings& held = found->second;
  for (const std::size_t index : held.cells) {
    std::erase_if(cells_[index], [robot](const Entry& e) { return e.robot == robot; });
  }
  for (const std::size_t index : held.edgeCells) {
    traversals_ -=
        std::erase_if(edges_[index], [robot](const EdgeEntry& e) { return e.robot == robot; });
  }
  held.cells.clear();
  held.edgeCells.clear();
}

void ReservationTable::clear() {
  for (const std::size_t index : touched_) {
    cells_[index].clear();
    isTouched_[index] = 0;
  }
  touched_.clear();
  for (const std::size_t index : touchedEdgeCells_) {
    edges_[index].clear();
    hasEdges_[index] = 0;
  }
  touchedEdgeCells_.clear();
  traversals_ = 0;
  for (auto& [id, held] : holdings_) {
    held.cells.clear();
    held.edgeCells.clear();
  }
}

std::optional<RobotId> ReservationTable::holder(Point cell, Step step) const {
  if (!size_.contains(cell)) {
    return std::nullopt;
  }
  for (const Entry& entry : cells_[size_.indexOf(cell)]) {
    if (entry.steps.contains(step)) {
      return entry.robot;
    }
  }
  return std::nullopt;
}

std::vector<std::size_t> ReservationTable::touchedCells() const {
  std::vector<std::size_t> indexes = touched_;
  std::sort(indexes.begin(), indexes.end());
  indexes.erase(std::unique(indexes.begin(), indexes.end()), indexes.end());
  return indexes;
}

std::vector<std::pair<Point, RobotId>> ReservationTable::occupiedAt(Step step) const {
  std::vector<std::pair<Point, RobotId>> result;
  for (const std::size_t index : touchedCells()) {
    for (const Entry& entry : cells_[index]) {
      if (entry.steps.contains(step)) {
        result.emplace_back(size_.pointAt(index), entry.robot);
      }
    }
  }
  return result;
}

std::vector<Reservation> ReservationTable::reservationsOf(RobotId robot) const {
  std::vector<Reservation> result;
  for (const std::size_t index : touchedCells()) {
    std::vector<Reservation> here;
    for (const Entry& entry : cells_[index]) {
      if (entry.robot == robot) {
        here.push_back({robot, size_.pointAt(index), entry.steps});
      }
    }
    std::sort(here.begin(), here.end(), [](const Reservation& a, const Reservation& b) {
      return a.steps.from < b.steps.from;
    });
    result.insert(result.end(), here.begin(), here.end());
  }
  return result;
}

std::size_t ReservationTable::occupancyCount() const {
  std::size_t count = 0;
  for (const std::size_t index : touchedCells()) {
    count += cells_[index].size();
  }
  return count;
}

std::size_t ReservationTable::traversalCount() const { return traversals_; }

}  // namespace warehousesim::traffic
