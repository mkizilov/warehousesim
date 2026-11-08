#pragma once

#include <cstdint>

namespace warehousesim::world {

// Outcomes of world operations. Rejected operations leave the world unchanged.

enum class PlaceStatus : std::uint8_t {
  Ok,
  InvalidRobot,    // the robot id is the reserved "none" id
  DuplicateRobot,  // the robot is already placed
  OutOfBounds,
  BlockedByObstacle,  // the cell is a wall or shelf
  CellOccupied,       // another robot is there
};

enum class MoveStatus : std::uint8_t {
  Ok,
  UnknownRobot,
  OutOfBounds,
  NotAdjacent,  // the target is not exactly one orthogonal step away
  BlockedByObstacle,
  CellOccupied,
  HasClaims,  // the robot holds cells beyond the one it stands on, so it cannot be moved whole
};

enum class CellEdit : std::uint8_t {
  Ok,
  OutOfBounds,
  OccupiedByRobot,  // the new type is an obstacle but a robot stands on the cell
  NotAnObstacle,    // placeObstacle was given a walkable cell type
};

}  // namespace warehousesim::world
