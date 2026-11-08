#include "warehousesim/world/world.hpp"

#include <utility>

namespace warehousesim::world {

using core::Point;
using core::RobotId;

World::World(Warehouse layout) : layout_(std::move(layout)), occupancy_(layout_.size()) {}

std::vector<Point> World::freeNeighbors(Point p) const {
  std::vector<Point> result = layout_.walkableNeighbors(p);
  std::erase_if(result, [this](Point neighbor) { return occupancy_.isOccupied(neighbor); });
  return result;
}

PlaceStatus World::addRobot(RobotId robot, Point p) {
  // Id problems and out-of-bounds are reported by Occupancy; the layout adds obstacles.
  if (robot.valid() && !occupancy_.positionOf(robot) && layout_.isObstacle(p)) {
    return PlaceStatus::BlockedByObstacle;
  }
  return occupancy_.place(robot, p);
}

MoveStatus World::checkMove(RobotId robot, Point to) const {
  const auto from = occupancy_.positionOf(robot);
  if (!from) {
    return MoveStatus::UnknownRobot;
  }
  if (!layout_.inBounds(to)) {
    return MoveStatus::OutOfBounds;
  }
  if (!core::directionBetween(*from, to)) {
    return MoveStatus::NotAdjacent;
  }
  if (layout_.isObstacle(to)) {
    return MoveStatus::BlockedByObstacle;
  }
  if (occupancy_.isOccupied(to)) {
    return MoveStatus::CellOccupied;
  }
  return MoveStatus::Ok;
}

MoveStatus World::moveRobot(RobotId robot, Point to) {
  const MoveStatus status = checkMove(robot, to);
  if (status != MoveStatus::Ok) {
    return status;
  }
  return occupancy_.move(robot, to);
}

CellEdit World::setCell(Point p, CellType type) {
  if (!layout_.inBounds(p)) {
    return CellEdit::OutOfBounds;
  }
  if (world::isObstacle(type) && occupancy_.isOccupied(p)) {
    return CellEdit::OccupiedByRobot;
  }
  layout_.set(p, type);
  return CellEdit::Ok;
}

CellEdit World::placeObstacle(Point p, CellType type) {
  if (!world::isObstacle(type)) {
    return CellEdit::NotAnObstacle;
  }
  return setCell(p, type);
}

}  // namespace warehousesim::world
