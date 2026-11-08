#include "warehousesim/robots/robot.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace warehousesim::robots {

const char* toString(RobotStatus status) noexcept {
  switch (status) {
    case RobotStatus::Idle:
      return "idle";
    case RobotStatus::ToPickup:
      return "to_pickup";
    case RobotStatus::ToDropoff:
      return "to_dropoff";
  }
  return "unknown";
}

std::optional<core::Point> Robot::nextStep() const noexcept {
  if (path_.empty()) {
    return std::nullopt;
  }
  return path_.front();
}

void Robot::beginTask(core::TaskId task) {
  if (status_ != RobotStatus::Idle) {
    throw std::logic_error("robot is already working on a task");
  }
  status_ = RobotStatus::ToPickup;
  task_ = task;
}

void Robot::beginDelivery() {
  if (status_ != RobotStatus::ToPickup) {
    throw std::logic_error("robot is not heading to a pickup");
  }
  status_ = RobotStatus::ToDropoff;
}

void Robot::finishTask() noexcept {
  status_ = RobotStatus::Idle;
  task_.reset();
  path_.clear();
  blockedTicks_ = 0;
}

void Robot::setPath(std::vector<core::Point> path) {
  path_.assign(path.begin(), path.end());
  blockedTicks_ = 0;
}

void Robot::advance() {
  if (path_.empty()) {
    throw std::logic_error("robot has no path to follow");
  }
  position_ = path_.front();
  path_.pop_front();
  blockedTicks_ = 0;
  ++distanceTravelled_;
}

void Robot::sidestep(core::Point adjacent) {
  if (core::manhattanDistance(position_, adjacent) != 1) {
    throw std::invalid_argument("sidestep target must be adjacent");
  }
  position_ = adjacent;
  path_.clear();
  blockedTicks_ = 0;
  ++distanceTravelled_;
}

}  // namespace warehousesim::robots
