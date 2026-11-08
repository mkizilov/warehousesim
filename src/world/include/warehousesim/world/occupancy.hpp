#pragma once

#include <cstddef>
#include <deque>
#include <map>
#include <optional>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/world/grid_size.hpp"
#include "warehousesim/world/status.hpp"

namespace warehousesim::world {

// Dynamic robot occupancy: which robot (if any) holds each cell, at most one per cell. It knows
// the grid size but nothing about walls or shelves; World combines it with a Warehouse layout and
// enforces the static rules.
//
// A robot holds the cell it stands on (its position) and, while it is driving, the cells ahead
// that it has claimed so that nobody else can enter them before it has passed. Claimed cells
// are kept in order behind the position: advance() is the robot passing the first of them.
// With no claims this is exactly "one cell per robot".
//
// Two indexes are kept in lock-step: cell -> robot and robot -> held cells. Every mutator either
// updates both or rejects the request and changes nothing.
class Occupancy {
 public:
  explicit Occupancy(GridSize size);

  [[nodiscard]] GridSize size() const noexcept { return size_; }
  [[nodiscard]] std::size_t robotCount() const noexcept { return holds_.size(); }

  // True if a robot stands on or has claimed the cell; false for out-of-bounds points.
  [[nodiscard]] bool isOccupied(core::Point p) const noexcept;
  [[nodiscard]] std::optional<core::RobotId> occupantAt(core::Point p) const noexcept;
  // The cell the robot stands on.
  [[nodiscard]] std::optional<core::Point> positionOf(core::RobotId robot) const;
  // The position followed by the claimed cells in order; empty for an unknown robot.
  [[nodiscard]] std::vector<core::Point> holdsOf(core::RobotId robot) const;
  // Every held cell, by robot id and then in hold order.
  [[nodiscard]] std::vector<core::Point> occupiedCells() const;

  // Never returns BlockedByObstacle: static obstacles are not this class's concern.
  PlaceStatus place(core::RobotId robot, core::Point p);
  // Teleports a placed robot to any free in-bounds cell (adjacency is World's rule). Never
  // returns NotAdjacent or BlockedByObstacle. Moving onto its own cell is CellOccupied, and a
  // robot that holds claimed cells cannot be moved (HasClaims).
  MoveStatus move(core::RobotId robot, core::Point to);
  // Adds a free in-bounds cell to the end of the robot's holds. Never returns NotAdjacent or
  // BlockedByObstacle; a cell the robot already holds is CellOccupied.
  MoveStatus claim(core::RobotId robot, core::Point cell);
  // The robot passes the first claimed cell: it stands there now and the cell it stood on is
  // free. False (changing nothing) if the robot is unknown or has no claimed cell.
  bool advance(core::RobotId robot);
  // Gives back the newest claimed cells until at most `keep` remain; returns how many were
  // released. The position is never released.
  std::size_t releaseClaims(core::RobotId robot, std::size_t keep);
  // Returns false if the robot was not placed.
  bool remove(core::RobotId robot);

 private:
  GridSize size_;
  std::vector<core::RobotId> cells_;  // row-major; the default id means "empty"
  std::map<core::RobotId, std::deque<core::Point>> holds_;  // front() is the position
};

}  // namespace warehousesim::world
