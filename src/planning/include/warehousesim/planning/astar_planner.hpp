#pragma once

#include "warehousesim/planning/path_planner.hpp"

namespace warehousesim::planning {

// 4-connected A* with a Manhattan heuristic. Ties are broken by (f, h, y, x) so the
// returned path is identical across runs and platforms.
class AStarPlanner final : public PathPlanner {
 public:
  [[nodiscard]] std::optional<Path> findPath(const world::Warehouse& warehouse, core::Point start,
                                             core::Point goal,
                                             const BlockedSet& blocked) const override;
};

}  // namespace warehousesim::planning
