#pragma once

#include <optional>
#include <unordered_set>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::planning {

using Path = std::vector<core::Point>;
using BlockedSet = std::unordered_set<core::Point, core::PointHash>;

class PathPlanner {
 public:
  PathPlanner() = default;
  PathPlanner(const PathPlanner&) = delete;
  PathPlanner& operator=(const PathPlanner&) = delete;
  virtual ~PathPlanner() = default;

  // Returns the cells to walk through, excluding `start` and including `goal`
  // (empty when start == goal). Returns nullopt when `goal` cannot be reached.
  // `start` is always allowed to be in `blocked` (a robot's own cell); `goal` is not.
  [[nodiscard]] virtual std::optional<Path> findPath(const world::Warehouse& warehouse,
                                                     core::Point start, core::Point goal,
                                                     const BlockedSet& blocked) const = 0;
};

}  // namespace warehousesim::planning
