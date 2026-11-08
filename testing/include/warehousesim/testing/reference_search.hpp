#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/path_planner.hpp"
#include "warehousesim/world/warehouse.hpp"

// Slow, obviously-correct shortest-path code used to check the real planner in tests and as a
// baseline in benchmarks. It deliberately shares nothing with AStarPlanner except the types.
namespace warehousesim::testing {

inline constexpr std::int32_t kUnreachable = -1;

// Breadth-first distances (in steps) from `source` to every cell, row-major; kUnreachable where
// there is no route. Same rules as the planner: moves go one cell in an allowed direction onto a
// walkable cell that is not blocked; `source` itself may be blocked.
[[nodiscard]] inline std::vector<std::int32_t> distancesFrom(const world::Warehouse& warehouse,
                                                             core::Point source,
                                                             const planning::BlockedSet& blocked,
                                                             core::DirectionSet directions) {
  const world::GridSize size = warehouse.size();
  std::vector<std::int32_t> distance(size.cellCount(), kUnreachable);
  if (!warehouse.isWalkable(source)) {
    return distance;
  }
  std::vector<core::Point> queue{source};
  distance[size.indexOf(source)] = 0;
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const core::Point at = queue[head];
    for (const core::Direction direction : core::kAllDirections) {
      if (!directions.contains(direction)) {
        continue;
      }
      const core::Point next = at + core::offsetOf(direction);
      if (!warehouse.isWalkable(next) || blocked.contains(next) ||
          distance[size.indexOf(next)] != kUnreachable) {
        continue;
      }
      distance[size.indexOf(next)] = distance[size.indexOf(at)] + 1;
      queue.push_back(next);
    }
  }
  return distance;
}

// Breadth-first distances from every cell *to* `target`, following the allowed moves (so the
// search runs over reversed edges). Nothing is blocked. Used to check heuristic admissibility.
[[nodiscard]] inline std::vector<std::int32_t> distancesTo(const world::Warehouse& warehouse,
                                                           core::Point target,
                                                           core::DirectionSet directions) {
  const world::GridSize size = warehouse.size();
  std::vector<std::int32_t> distance(size.cellCount(), kUnreachable);
  if (!warehouse.isWalkable(target)) {
    return distance;
  }
  std::vector<core::Point> queue{target};
  distance[size.indexOf(target)] = 0;
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const core::Point at = queue[head];
    for (const core::Direction direction : core::kAllDirections) {
      if (!directions.contains(direction)) {
        continue;
      }
      // A cell that reaches `at` by moving in `direction` sits one step the other way.
      const core::Point offset = core::offsetOf(direction);
      const core::Point previous{at.x - offset.x, at.y - offset.y};
      if (!warehouse.isWalkable(previous) || distance[size.indexOf(previous)] != kUnreachable) {
        continue;
      }
      distance[size.indexOf(previous)] = distance[size.indexOf(at)] + 1;
      queue.push_back(previous);
    }
  }
  return distance;
}

// Length of a shortest route, or nullopt. Mirrors the planner's contract: start and goal must be
// walkable, the goal must not be blocked, a route from a cell to itself has length 0.
[[nodiscard]] inline std::optional<std::int32_t> shortestDistance(
    const world::Warehouse& warehouse, core::Point start, core::Point goal,
    const planning::BlockedSet& blocked,
    core::DirectionSet directions = core::DirectionSet::all()) {
  if (!warehouse.isWalkable(start) || !warehouse.isWalkable(goal) || blocked.contains(goal)) {
    return std::nullopt;
  }
  const std::int32_t d =
      distancesFrom(warehouse, start, blocked, directions)[warehouse.size().indexOf(goal)];
  if (d == kUnreachable) {
    return std::nullopt;
  }
  return d;
}

// Checks that `path` is a legal route from start to goal (excluding start): every step one cell in
// an allowed direction, onto a walkable cell that is not blocked, ending at the goal. Returns a
// description of the first problem, or nullopt if the path is valid.
[[nodiscard]] inline std::optional<std::string> pathProblem(
    const world::Warehouse& warehouse, core::Point start, core::Point goal,
    const planning::Path& path, const planning::BlockedSet& blocked,
    core::DirectionSet directions = core::DirectionSet::all()) {
  core::Point at = start;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const std::optional<core::Direction> direction = core::directionBetween(at, path[i]);
    if (!direction) {
      return "step " + std::to_string(i) + " is not one orthogonal move";
    }
    if (!directions.contains(*direction)) {
      return "step " + std::to_string(i) + " uses a forbidden direction";
    }
    if (!warehouse.isWalkable(path[i])) {
      return "step " + std::to_string(i) + " enters a non-walkable cell";
    }
    if (blocked.contains(path[i])) {
      return "step " + std::to_string(i) + " enters a blocked cell";
    }
    at = path[i];
  }
  if (at != goal) {
    return "path does not end at the goal";
  }
  return std::nullopt;
}

}  // namespace warehousesim::testing
