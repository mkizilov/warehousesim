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
  const auto it = holds_.find(robot);
  if (it == holds_.end()) {
    return std::nullopt;
  }
  return it->second.front();
}

std::vector<Point> Occupancy::holdsOf(RobotId robot) const {
  const auto it = holds_.find(robot);
  if (it == holds_.end()) {
    return {};
  }
  return {it->second.begin(), it->second.end()};
}

std::vector<Point> Occupancy::occupiedCells() const {
  std::vector<Point> result;
  result.reserve(holds_.size());
  for (const auto& [robot, held] : holds_) {
    result.insert(result.end(), held.begin(), held.end());
  }
  return result;
}

PlaceStatus Occupancy::place(RobotId robot, Point p) {
  if (!robot.valid()) {
    return PlaceStatus::InvalidRobot;
  }
  if (holds_.contains(robot)) {
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
  holds_.emplace(robot, std::deque<Point>{p});
  return PlaceStatus::Ok;
}

MoveStatus Occupancy::move(RobotId robot, Point to) {
  const auto it = holds_.find(robot);
  if (it == holds_.end()) {
    return MoveStatus::UnknownRobot;
  }
  if (!size_.contains(to)) {
    return MoveStatus::OutOfBounds;
  }
  if (it->second.size() > 1) {
    return MoveStatus::HasClaims;
  }
  RobotId& target = cells_[size_.indexOf(to)];
  if (target.valid()) {
    return MoveStatus::CellOccupied;
  }
  cells_[size_.indexOf(it->second.front())] = RobotId{};
  target = robot;
  it->second.front() = to;
  return MoveStatus::Ok;
}

MoveStatus Occupancy::claim(RobotId robot, Point cell) {
  const auto it = holds_.find(robot);
  if (it == holds_.end()) {
    return MoveStatus::UnknownRobot;
  }
  if (!size_.contains(cell)) {
    return MoveStatus::OutOfBounds;
  }
  RobotId& target = cells_[size_.indexOf(cell)];
  if (target.valid()) {
    return MoveStatus::CellOccupied;
  }
  target = robot;
  it->second.push_back(cell);
  return MoveStatus::Ok;
}

bool Occupancy::advance(RobotId robot) {
  const auto it = holds_.find(robot);
  if (it == holds_.end() || it->second.size() < 2) {
    return false;
  }
  cells_[size_.indexOf(it->second.front())] = RobotId{};
  it->second.pop_front();
  return true;
}

std::size_t Occupancy::releaseClaims(RobotId robot, std::size_t keep) {
  const auto it = holds_.find(robot);
  if (it == holds_.end()) {
    return 0;
  }
  std::size_t released = 0;
  while (it->second.size() > 1 + keep) {
    cells_[size_.indexOf(it->second.back())] = RobotId{};
    it->second.pop_back();
    ++released;
  }
  return released;
}

bool Occupancy::remove(RobotId robot) {
  const auto it = holds_.find(robot);
  if (it == holds_.end()) {
    return false;
  }
  for (const Point held : it->second) {
    cells_[size_.indexOf(held)] = RobotId{};
  }
  holds_.erase(it);
  return true;
}

}  // namespace warehousesim::world
