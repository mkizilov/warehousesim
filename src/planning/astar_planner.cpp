#include "warehousesim/planning/astar_planner.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace warehousesim::planning {

namespace {

using core::Point;

constexpr std::uint32_t kNoParent = std::numeric_limits<std::uint32_t>::max();

// Orders the open list as a min-heap on (f, h, cell index). std::push_heap/pop_heap build a
// max-heap, so "lower priority" means "greater key".
struct LowerPriority {
  template <typename Entry>
  bool operator()(const Entry& a, const Entry& b) const noexcept {
    return std::tie(a.f, a.h, a.index) > std::tie(b.f, b.h, b.index);
  }
};

}  // namespace

const char* toString(SearchStatus status) noexcept {
  switch (status) {
    case SearchStatus::Found:
      return "found";
    case SearchStatus::InvalidStart:
      return "invalid_start";
    case SearchStatus::InvalidGoal:
      return "invalid_goal";
    case SearchStatus::GoalBlocked:
      return "goal_blocked";
    case SearchStatus::Unreachable:
      return "unreachable";
    case SearchStatus::BudgetExceeded:
      return "budget_exceeded";
  }
  return "unknown";
}

AStarPlanner::AStarPlanner(AStarOptions options) : options_(options) {
  if (options_.directions.empty()) {
    throw std::invalid_argument("A* needs at least one allowed direction");
  }
  // Fixed north, east, south, west order, whatever the set: part of the tie-breaking contract.
  for (const core::Direction direction : core::kAllDirections) {
    if (options_.directions.contains(direction)) {
      offsets_[offsetCount_++] = core::offsetOf(direction);
    }
  }
}

void AStarPlanner::prepare(std::size_t cellCount) {
  if (cellCount >= kNoParent) {
    throw std::length_error("grid is too large for the A* planner");
  }
  if (nodes_.size() < cellCount) {
    nodes_.resize(cellCount);
    blockedStamp_.resize(cellCount);
  }
  if (++generation_ == 0) {  // wrapped after 2^32 searches: forget every old stamp
    std::ranges::fill(blockedStamp_, 0U);
    for (Node& node : nodes_) {
      node.stamp = 0;
    }
    generation_ = 1;
  }
}

std::optional<Path> AStarPlanner::findPath(const world::Warehouse& warehouse, Point start,
                                           Point goal, const BlockedSet& blocked) {
  Path path;
  if (search(warehouse, start, goal, blocked, path).status != SearchStatus::Found) {
    return std::nullopt;
  }
  return std::optional<Path>(std::move(path));
}

SearchResult AStarPlanner::search(const world::Warehouse& warehouse, Point start, Point goal,
                                  const BlockedSet& blocked, Path& path) {
  path.clear();
  if (!warehouse.isWalkable(start)) {
    return {SearchStatus::InvalidStart, 0, 0};
  }
  if (!warehouse.isWalkable(goal)) {
    return {SearchStatus::InvalidGoal, 0, 0};
  }
  if (blocked.contains(goal)) {
    return {SearchStatus::GoalBlocked, 0, 0};
  }
  if (start == goal) {
    return {SearchStatus::Found, 0, 0};
  }

  const world::GridSize size = warehouse.size();
  prepare(size.cellCount());
  for (const Point cell : blocked) {
    if (size.contains(cell)) {
      blockedStamp_[size.indexOf(cell)] = generation_;
    }
  }
  const auto startIndex = static_cast<std::uint32_t>(size.indexOf(start));
  const auto goalIndex = static_cast<std::uint32_t>(size.indexOf(goal));

  // Cheap rejections for the common "sealed in" cases, before any search: no allowed move leaves
  // the start, or none enters the goal. (The start may itself be blocked; that is its own robot.)
  bool canLeave = false;
  bool canEnter = false;
  for (std::size_t i = 0; i < offsetCount_; ++i) {
    const Point out = start + offsets_[i];
    canLeave = canLeave || (warehouse.isWalkable(out) && !isBlocked(size.indexOf(out)));
    const Point in{goal.x - offsets_[i].x, goal.y - offsets_[i].y};
    canEnter =
        canEnter || (warehouse.isWalkable(in) && (in == start || !isBlocked(size.indexOf(in))));
  }
  if (!canLeave || !canEnter) {
    return {SearchStatus::Unreachable, 0, 0};
  }

  open_.clear();
  nodes_[startIndex] = {generation_, 0, kNoParent};
  const std::int32_t startH = core::manhattanDistance(start, goal);
  open_.push_back({startH, startH, startIndex});

  std::size_t expansions = 0;
  while (!open_.empty()) {
    std::ranges::pop_heap(open_, LowerPriority{});
    const OpenEntry entry = open_.back();
    open_.pop_back();
    const Node current = nodes_[entry.index];
    if (entry.f != current.cost + entry.h) {
      continue;  // superseded by a cheaper entry for the same cell
    }
    if (options_.maxExpansions != 0 && expansions >= options_.maxExpansions) {
      return {SearchStatus::BudgetExceeded, 0, expansions};
    }
    ++expansions;

    if (entry.index == goalIndex) {
      const auto length = static_cast<std::size_t>(current.cost);
      path.resize(length);
      std::uint32_t cell = entry.index;
      for (std::size_t i = length; i > 0; --i) {
        path[i - 1] = size.pointAt(cell);
        cell = nodes_[cell].parent;
      }
      return {SearchStatus::Found, current.cost, expansions};
    }

    const Point at = size.pointAt(entry.index);
    const std::int32_t nextCost = current.cost + 1;
    for (std::size_t i = 0; i < offsetCount_; ++i) {
      const Point next = at + offsets_[i];
      if (!warehouse.isWalkable(next)) {
        continue;
      }
      const auto nextIndex = static_cast<std::uint32_t>(size.indexOf(next));
      if (isBlocked(nextIndex)) {
        continue;
      }
      Node& node = nodes_[nextIndex];
      if (node.stamp == generation_ && nextCost >= node.cost) {
        continue;
      }
      node = {generation_, nextCost, entry.index};
      const std::int32_t h = core::manhattanDistance(next, goal);
      open_.push_back({nextCost + h, h, nextIndex});
      std::ranges::push_heap(open_, LowerPriority{});
    }
  }
  return {SearchStatus::Unreachable, 0, expansions};
}

}  // namespace warehousesim::planning
