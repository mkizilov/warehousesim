#include "warehousesim/world/occupancy.hpp"

namespace warehousesim::world {

using core::Point;
using core::RobotId;

Occupancy::Occupancy(GridSize size) : size_(size), cells_(size.cellCount()) {}

bool Occupancy::isOccupied(Point p) const noexcept {
  return size_.contains(p) && cells_[size_.indexOf(p)].valid();
}

std::optional<RobotId> Occupancy::occupantAt(Point p) const noexcept {
  if (!isOccupied(p)) {
    return std::nullopt;
  }
  return cells_[size_.indexOf(p)];
}

std::optional<Point> Occupancy::positionOf(RobotId robot) const {
  const auto it = positions_.find(robot);
  if (it == positions_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<Point> Occupancy::occupiedCells() const {
  std::vector<Point> result;
  result.reserve(positions_.size());
  for (const auto& [robot, position] : positions_) {
    result.push_back(position);
  }
  return result;
}

PlaceStatus Occupancy::place(RobotId robot, Point p) {
  if (!robot.valid()) {
    return PlaceStatus::InvalidRobot;
  }
  if (positions_.contains(robot)) {
    return PlaceStatus::DuplicateRobot;
  }
  if (!size_.contains(p)) {
    return PlaceStatus::OutOfBounds;
  }
  RobotId& cell = cells_[size_.indexOf(p)];
  if (cell.valid()) {
    return PlaceStatus::CellOccupied;
  }
  cell = robot;
  positions_.emplace(robot, p);
  return PlaceStatus::Ok;
}

MoveStatus Occupancy::move(RobotId robot, Point to) {
  const auto it = positions_.find(robot);
  if (it == positions_.end()) {
    return MoveStatus::UnknownRobot;
  }
  if (!size_.contains(to)) {
    return MoveStatus::OutOfBounds;
  }
  RobotId& target = cells_[size_.indexOf(to)];
  if (target.valid()) {
    return MoveStatus::CellOccupied;
  }
  cells_[size_.indexOf(it->second)] = RobotId{};
  target = robot;
  it->second = to;
  return MoveStatus::Ok;
}

bool Occupancy::remove(RobotId robot) {
  const auto it = positions_.find(robot);
  if (it == positions_.end()) {
    return false;
  }
  cells_[size_.indexOf(it->second)] = RobotId{};
  positions_.erase(it);
  return true;
}

}  // namespace warehousesim::world
