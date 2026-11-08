#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/world/grid_size.hpp"
#include "warehousesim/world/status.hpp"

namespace warehousesim::world {

// Dynamic robot occupancy: which robot (if any) stands on each cell, at most one per cell
// and one cell per robot. It knows the grid size but nothing about walls or shelves; World
// combines it with a Warehouse layout and enforces the static rules.
//
// Two indexes are kept in lock-step: cell -> robot and robot -> cell. Every mutator either
// updates both or rejects the request and changes nothing.
class Occupancy {
 public:
  explicit Occupancy(GridSize size);

  [[nodiscard]] GridSize size() const noexcept { return size_; }
  [[nodiscard]] std::size_t robotCount() const noexcept { return positions_.size(); }

  // False for out-of-bounds points.
  [[nodiscard]] bool isOccupied(core::Point p) const noexcept;
  [[nodiscard]] std::optional<core::RobotId> occupantAt(core::Point p) const noexcept;
  [[nodiscard]] std::optional<core::Point> positionOf(core::RobotId robot) const;
  // Occupied cells ordered by robot id.
  [[nodiscard]] std::vector<core::Point> occupiedCells() const;

  // Never returns BlockedByObstacle: static obstacles are not this class's concern.
  PlaceStatus place(core::RobotId robot, core::Point p);
  // Teleports a placed robot to any free in-bounds cell (adjacency is World's rule).
  // Never returns NotAdjacent or BlockedByObstacle. Moving onto its own cell is CellOccupied.
  MoveStatus move(core::RobotId robot, core::Point to);
  // Returns false if the robot was not placed.
  bool remove(core::RobotId robot);

 private:
  GridSize size_;
  std::vector<core::RobotId> cells_;  // row-major; the default id means "empty"
  std::map<core::RobotId, core::Point> positions_;
};

}  // namespace warehousesim::world
