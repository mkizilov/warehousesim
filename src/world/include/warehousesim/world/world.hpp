#pragma once

#include <cstddef>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/world/cell_type.hpp"
#include "warehousesim/world/occupancy.hpp"
#include "warehousesim/world/status.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::world {

// The warehouse as the simulation sees it: an immutable-by-default static layout plus the
// dynamic occupancy of robots on top of it.
//
// World is the only way to mutate either half, which keeps one invariant true at all times:
// a robot never stands on or claims a wall or shelf, never shares a cell, and appears in the
// occupancy exactly once. The two halves stay independently readable through layout() and
// occupancy().
class World {
 public:
  explicit World(Warehouse layout);

  [[nodiscard]] const Warehouse& layout() const noexcept { return layout_; }
  [[nodiscard]] const Occupancy& occupancy() const noexcept { return occupancy_; }
  [[nodiscard]] GridSize size() const noexcept { return layout_.size(); }

  // Static questions (layout only).
  [[nodiscard]] bool inBounds(core::Point p) const noexcept { return layout_.inBounds(p); }
  [[nodiscard]] bool isWalkable(core::Point p) const noexcept { return layout_.isWalkable(p); }
  [[nodiscard]] bool isObstacle(core::Point p) const noexcept { return layout_.isObstacle(p); }

  // Dynamic questions (robots only).
  [[nodiscard]] bool isOccupied(core::Point p) const noexcept { return occupancy_.isOccupied(p); }
  [[nodiscard]] std::optional<core::RobotId> occupantAt(core::Point p) const noexcept {
    return occupancy_.occupantAt(p);
  }

  // Combined question: a robot could stand here right now (walkable and unoccupied).
  [[nodiscard]] bool isFree(core::Point p) const noexcept {
    return layout_.isWalkable(p) && !occupancy_.isOccupied(p);
  }

  [[nodiscard]] std::vector<core::Point> neighbors(core::Point p) const {
    return layout_.neighbors(p);
  }
  [[nodiscard]] std::vector<core::Point> walkableNeighbors(core::Point p) const {
    return layout_.walkableNeighbors(p);
  }
  // Neighbours that are walkable and currently unoccupied, in north, east, south, west order.
  [[nodiscard]] std::vector<core::Point> freeNeighbors(core::Point p) const;

  // Robots.
  PlaceStatus addRobot(core::RobotId robot, core::Point p);
  bool removeRobot(core::RobotId robot) { return occupancy_.remove(robot); }

  // Whether `robot` may step to `to` right now: exactly one orthogonal step, in bounds,
  // not into a wall or shelf, and not into a cell another robot occupies.
  [[nodiscard]] MoveStatus checkMove(core::RobotId robot, core::Point to) const;
  // Applies the move if checkMove() says Ok; otherwise changes nothing. A robot that holds
  // claimed cells cannot be moved (HasClaims).
  MoveStatus moveRobot(core::RobotId robot, core::Point to);

  // Continuous motion. A driving robot claims the cells ahead of it one by one, so that nobody
  // can enter them before it has passed, and then passes them in order.
  //
  // Whether `robot` may claim `cell` right now: exactly one orthogonal step beyond the last cell
  // it holds, in bounds, not a wall or shelf, and held by nobody.
  [[nodiscard]] MoveStatus checkClaim(core::RobotId robot, core::Point cell) const;
  // Applies the claim if checkClaim() says Ok; otherwise changes nothing.
  MoveStatus claimCell(core::RobotId robot, core::Point cell);
  // The robot passes the first claimed cell and stands on it; false if it has none.
  bool advanceRobot(core::RobotId robot) { return occupancy_.advance(robot); }
  // Gives back the newest claims until at most `keep` remain; returns how many were released.
  std::size_t releaseClaims(core::RobotId robot, std::size_t keep) {
    return occupancy_.releaseClaims(robot, keep);
  }
  // The robot's position followed by its claimed cells.
  [[nodiscard]] std::vector<core::Point> holdsOf(core::RobotId robot) const {
    return occupancy_.holdsOf(robot);
  }

  // Layout edits. Turning a cell into an obstacle is refused while a robot stands on it.
  CellEdit setCell(core::Point p, CellType type);
  // Like setCell, but only accepts Wall or Shelf.
  CellEdit placeObstacle(core::Point p, CellType type);
  CellEdit clearCell(core::Point p) { return setCell(p, CellType::Free); }

 private:
  Warehouse layout_;
  Occupancy occupancy_;
};

}  // namespace warehousesim::world
