#pragma once

#include <optional>
#include <unordered_set>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::planning {

using Path = std::vector<core::Point>;
using BlockedSet = std::unordered_set<core::Point, core::PointHash>;

// Finds routes through a warehouse layout. Planners know only the static layout and an optional
// set of temporarily blocked cells (typically cells other robots stand on); they have nothing to
// do with robots, tasks or rendering.
class PathPlanner {
 public:
  PathPlanner() = default;
  PathPlanner(const PathPlanner&) = delete;
  PathPlanner& operator=(const PathPlanner&) = delete;
  virtual ~PathPlanner() = default;

  // Returns the cells to walk through, excluding `start` and including `goal`
  // (empty when start == goal). Returns nullopt when `goal` cannot be reached.
  // `start` is always allowed to be in `blocked` (a robot's own cell); `goal` is not.
  //
  // Not const and not thread-safe: implementations may keep scratch memory between calls so that
  // repeated planning does not allocate. Use one planner per thread.
  [[nodiscard]] virtual std::optional<Path> findPath(const world::Warehouse& warehouse,
                                                     core::Point start, core::Point goal,
                                                     const BlockedSet& blocked) = 0;
};

}  // namespace warehousesim::planning
