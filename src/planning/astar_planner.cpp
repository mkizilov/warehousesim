#include "warehousesim/planning/astar_planner.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

namespace warehousesim::planning {

namespace {

using core::Point;

struct Node {
  std::int32_t f;
  std::int32_t h;
  std::int32_t y;
  std::int32_t x;
};

struct LowerPriority {
  bool operator()(const Node& a, const Node& b) const noexcept {
    return std::tie(a.f, a.h, a.y, a.x) > std::tie(b.f, b.h, b.y, b.x);
  }
};

}  // namespace

std::optional<Path> AStarPlanner::findPath(const world::Warehouse& warehouse, Point start,
                                           Point goal, const BlockedSet& blocked) const {
  if (!warehouse.isWalkable(start) || !warehouse.isWalkable(goal) || blocked.contains(goal)) {
    return std::nullopt;
  }
  if (start == goal) {
    return Path{};
  }

  const auto width = static_cast<std::size_t>(warehouse.width());
  const std::size_t cellCount = width * static_cast<std::size_t>(warehouse.height());
  const auto indexOf = [width](Point p) {
    return static_cast<std::size_t>(p.y) * width + static_cast<std::size_t>(p.x);
  };

  std::vector<std::int32_t> cost(cellCount, std::numeric_limits<std::int32_t>::max());
  std::vector<std::size_t> parent(cellCount, cellCount);
  std::priority_queue<Node, std::vector<Node>, LowerPriority> open;

  cost[indexOf(start)] = 0;
  const std::int32_t startH = core::manhattanDistance(start, goal);
  open.push({startH, startH, start.y, start.x});

  while (!open.empty()) {
    const Node node = open.top();
    open.pop();
    const Point current{node.x, node.y};
    const std::int32_t g = cost[indexOf(current)];
    if (node.f != g + node.h) {
      continue;  // superseded by a cheaper entry
    }
    if (current == goal) {
      Path path;
      for (std::size_t i = indexOf(goal); i != indexOf(start); i = parent[i]) {
        const auto row = static_cast<std::int32_t>(i / width);
        const auto col = static_cast<std::int32_t>(i % width);
        path.push_back({col, row});
      }
      std::ranges::reverse(path);
      return path;
    }
    for (const Point dir : core::kCardinalDirections) {
      const Point next = current + dir;
      if (!warehouse.isWalkable(next) || blocked.contains(next)) {
        continue;
      }
      const std::int32_t nextCost = g + 1;
      const std::size_t nextIndex = indexOf(next);
      if (nextCost < cost[nextIndex]) {
        cost[nextIndex] = nextCost;
        parent[nextIndex] = indexOf(current);
        const std::int32_t h = core::manhattanDistance(next, goal);
        open.push({nextCost + h, h, next.y, next.x});
      }
    }
  }
  return std::nullopt;
}

}  // namespace warehousesim::planning
