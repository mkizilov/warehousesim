#include "warehousesim/robots/robot.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace warehousesim::robots {

namespace {

using core::Direction;
using core::Point;

// Absorbs floating-point error when summing fractional speeds (e.g. 3 x 1/3 must reach 1).
constexpr double kEpsilon = 1e-9;
// Progress is below 1 before each tick of travel adds at most 1, so it can never reach this.
constexpr double kMaxProgress = 2.0;

Battery makeBattery(const RobotSpec& spec) {
  spec.validate();
  return Battery(spec.batteryCapacity, spec.initialCharge.value_or(spec.batteryCapacity));
}

}  // namespace

Robot::Robot(core::RobotId id, Point position, RobotSpec spec, Direction heading)
    : id_(id),
      position_(position),
      heading_(heading),
      spec_(std::move(spec)),
      battery_(makeBattery(spec_)) {
  if (!id.valid()) {
    throw std::invalid_argument("robot id must be valid");
  }
}

Velocity Robot::velocity() const noexcept {
  const Point direction = core::offsetOf(heading_);
  return {static_cast<double>(direction.x) * speed_, static_cast<double>(direction.y) * speed_};
}

std::optional<Point> Robot::nextStep() const noexcept {
  if (path_.empty()) {
    return std::nullopt;
  }
  return path_.front();
}

bool Robot::hasArrived() const noexcept {
  return state_ == RobotState::Navigating && destination_ && position_ == *destination_;
}

std::optional<std::string> Robot::inconsistency() const {
  const bool navigating = state_ == RobotState::Navigating;
  if (!id_.valid()) {
    return "robot id is invalid";
  }
  if (navigating != (goal_ != NavigationGoal::None)) {
    return "navigation goal must be set exactly while navigating";
  }
  if (navigating != destination_.has_value()) {
    return "destination must be set exactly while navigating";
  }
  const bool holdsTask =
      (navigating && (goal_ == NavigationGoal::Pickup || goal_ == NavigationGoal::Dropoff)) ||
      state_ == RobotState::Loading || state_ == RobotState::Unloading;
  if (holdsTask != task_.has_value()) {
    return "task must be held exactly while heading to a pickup/dropoff, loading or unloading";
  }
  if (!path_.empty() && !navigating) {
    return "only a navigating robot may have a path";
  }
  if (!path_.empty() && path_.back() != *destination_) {
    return "path does not end at the destination";
  }
  Point previous = position_;
  for (const Point step : path_) {
    if (core::manhattanDistance(previous, step) != 1) {
      return "path steps must be orthogonally adjacent";
    }
    previous = step;
  }
  if (!navigating && (speed_ != 0.0 || progress_ != 0.0)) {
    return "speed and travel progress must be zero unless navigating";
  }
  if (speed_ < 0.0 || speed_ > spec_.maxSpeed || progress_ < 0.0 || progress_ > kMaxProgress) {
    return "speed or travel progress out of range";
  }
  const bool dwelling = state_ == RobotState::Loading || state_ == RobotState::Unloading;
  if (dwellRemaining_ != 0 && !dwelling) {
    return "dwell time must be zero unless loading or unloading";
  }
  if (dwellRemaining_ > std::max(spec_.loadTicks, spec_.unloadTicks)) {
    return "dwell time exceeds the spec";
  }
  if (battery_.level() > battery_.capacity()) {
    return "battery level exceeds capacity";
  }
  return std::nullopt;
}

void Robot::require(std::initializer_list<RobotState> allowedFrom, RobotState next,
                    const char* operation) const {
  const bool fromAllowed = std::ranges::find(allowedFrom, state_) != allowedFrom.end();
  if (!fromAllowed || !isValidTransition(state_, next)) {
    throw InvalidTransition(std::string(operation) + ": cannot go from " + toString(state_) +
                            " to " + toString(next));
  }
}

void Robot::requireBattery(const char* operation) const {
  if (battery_.isEmpty()) {
    throw InvalidTransition(std::string(operation) + ": battery is empty");
  }
}

void Robot::validateRoute(Point destination, const RobotPath& path, const char* operation) const {
  if (path.empty()) {
    if (position_ != destination) {
      throw std::invalid_argument(std::string(operation) +
                                  ": empty path but the robot is not at the destination");
    }
    return;
  }
  if (path.back() != destination) {
    throw std::invalid_argument(std::string(operation) + ": path does not end at the destination");
  }
  Point previous = position_;
  for (const Point step : path) {
    if (core::manhattanDistance(previous, step) != 1) {
      throw std::invalid_argument(std::string(operation) +
                                  ": path steps must be orthogonally adjacent");
    }
    previous = step;
  }
}

void Robot::enter(RobotState next, NavigationGoal goal) {
  state_ = next;
  goal_ = goal;
  speed_ = 0.0;
  progress_ = 0.0;
  dwellRemaining_ = 0;
  blockedTicks_ = 0;
}

void Robot::clearRoute() noexcept {
  path_.clear();
  destination_.reset();
}

void Robot::startNavigation(NavigationGoal goal, Point destination, RobotPath path) {
  destination_ = destination;
  path_.assign(path.begin(), path.end());
  enter(RobotState::Navigating, goal);
}

void Robot::beginTask(core::TaskId task, Point destination, RobotPath path) {
  require({RobotState::Idle, RobotState::Charging}, RobotState::Navigating, "beginTask");
  if (!task.valid()) {
    throw std::invalid_argument("beginTask: task id must be valid");
  }
  requireBattery("beginTask");
  validateRoute(destination, path, "beginTask");
  task_ = task;
  startNavigation(NavigationGoal::Pickup, destination, std::move(path));
}

void Robot::beginChargerTrip(Point destination, RobotPath path) {
  require({RobotState::Idle}, RobotState::Navigating, "beginChargerTrip");
  requireBattery("beginChargerTrip");
  validateRoute(destination, path, "beginChargerTrip");
  startNavigation(NavigationGoal::Charger, destination, std::move(path));
}

void Robot::beginLoading() {
  require({RobotState::Navigating}, RobotState::Loading, "beginLoading");
  if (goal_ != NavigationGoal::Pickup || !hasArrived()) {
    throw InvalidTransition("beginLoading: robot has not arrived at its pickup");
  }
  clearRoute();
  enter(RobotState::Loading, NavigationGoal::None);
  dwellRemaining_ = spec_.loadTicks;
}

void Robot::finishLoading(Point destination, RobotPath path) {
  require({RobotState::Loading}, RobotState::Navigating, "finishLoading");
  if (dwellRemaining_ != 0) {
    throw InvalidTransition("finishLoading: loading is not finished");
  }
  validateRoute(destination, path, "finishLoading");
  startNavigation(NavigationGoal::Dropoff, destination, std::move(path));
}

void Robot::beginUnloading() {
  require({RobotState::Navigating}, RobotState::Unloading, "beginUnloading");
  if (goal_ != NavigationGoal::Dropoff || !hasArrived()) {
    throw InvalidTransition("beginUnloading: robot has not arrived at its dropoff");
  }
  clearRoute();
  enter(RobotState::Unloading, NavigationGoal::None);
  dwellRemaining_ = spec_.unloadTicks;
}

void Robot::finishUnloading() {
  require({RobotState::Unloading}, RobotState::Idle, "finishUnloading");
  if (dwellRemaining_ != 0) {
    throw InvalidTransition("finishUnloading: unloading is not finished");
  }
  task_.reset();
  enter(RobotState::Idle, NavigationGoal::None);
}

void Robot::redirectChargerTrip(Point destination, RobotPath path) {
  if (state_ != RobotState::Navigating || goal_ != NavigationGoal::Charger) {
    throw InvalidTransition("redirectChargerTrip: robot is not heading to a charger");
  }
  validateRoute(destination, path, "redirectChargerTrip");
  destination_ = destination;
  path_.assign(path.begin(), path.end());
  blockedTicks_ = 0;
}

void Robot::beginCharging() {
  require({RobotState::Idle, RobotState::Navigating}, RobotState::Charging, "beginCharging");
  if (state_ == RobotState::Navigating && (goal_ != NavigationGoal::Charger || !hasArrived())) {
    throw InvalidTransition("beginCharging: robot has not arrived at a charger");
  }
  clearRoute();
  enter(RobotState::Charging, NavigationGoal::None);
}

void Robot::stopCharging() {
  require({RobotState::Charging}, RobotState::Idle, "stopCharging");
  enter(RobotState::Idle, NavigationGoal::None);
}

void Robot::abortTask() {
  require({RobotState::Navigating, RobotState::Loading, RobotState::Unloading}, RobotState::Idle,
          "abortTask");
  task_.reset();
  clearRoute();
  enter(RobotState::Idle, NavigationGoal::None);
}

void Robot::makeUnavailable() {
  require({RobotState::Idle, RobotState::Navigating, RobotState::Loading, RobotState::Unloading,
           RobotState::Charging},
          RobotState::Unavailable, "makeUnavailable");
  task_.reset();
  clearRoute();
  enter(RobotState::Unavailable, NavigationGoal::None);
}

void Robot::restore() {
  require({RobotState::Unavailable}, RobotState::Idle, "restore");
  requireBattery("restore");
  enter(RobotState::Idle, NavigationGoal::None);
}

void Robot::replaceBattery() {
  if (state_ != RobotState::Unavailable) {
    throw InvalidTransition(
        "replaceBattery: only an unavailable robot can have its battery swapped");
  }
  battery_.fill();
}

void Robot::elapseTick() {
  switch (state_) {
    case RobotState::Idle:
      battery_.consume(spec_.idleDrainPerTick);
      break;
    case RobotState::Loading:
    case RobotState::Unloading:
      battery_.consume(spec_.handlingDrainPerTick);
      if (dwellRemaining_ > 0) {
        --dwellRemaining_;
      }
      break;
    case RobotState::Charging:
      battery_.charge(spec_.chargePerTick);
      break;
    case RobotState::Navigating:
    case RobotState::Unavailable:
      break;
  }
}

bool Robot::travel() {
  if (state_ != RobotState::Navigating || path_.empty()) {
    throw InvalidTransition("travel: robot is not navigating along a path");
  }
  heading_ = *core::directionBetween(position_, path_.front());
  speed_ = spec_.maxSpeed;
  // A robot that has already covered the whole cell (and is waiting for it to clear) does not
  // keep banking distance, otherwise being blocked would make it faster once released.
  if (progress_ < 1.0 - kEpsilon) {
    progress_ += spec_.maxSpeed;
  }
  return progress_ >= 1.0 - kEpsilon;
}

void Robot::halt() {
  speed_ = 0.0;
  progress_ = std::min(progress_, 1.0);
}

void Robot::commitStep() {
  if (state_ != RobotState::Navigating || path_.empty() || progress_ < 1.0 - kEpsilon) {
    throw InvalidTransition("commitStep: robot has not covered a whole cell");
  }
  position_ = path_.front();
  path_.pop_front();
  progress_ = std::max(0.0, progress_ - 1.0);
  ++cellsTravelled_;
  blockedTicks_ = 0;
  battery_.consume(spec_.moveCostPerCell);
  if (path_.empty()) {
    speed_ = 0.0;
    progress_ = 0.0;
  }
}

void Robot::setPath(RobotPath path) {
  if (state_ != RobotState::Navigating) {
    throw InvalidTransition("setPath: robot is not navigating");
  }
  validateRoute(*destination_, path, "setPath");
  path_.assign(path.begin(), path.end());
  blockedTicks_ = 0;
}

void Robot::sidestep(Point adjacent) {
  if (isStationary()) {
    throw InvalidTransition("sidestep: a stationary robot cannot be moved aside");
  }
  const std::optional<Direction> direction = core::directionBetween(position_, adjacent);
  if (!direction) {
    throw std::invalid_argument("sidestep target must be adjacent");
  }
  position_ = adjacent;
  heading_ = *direction;
  path_.clear();
  speed_ = 0.0;
  progress_ = 0.0;
  blockedTicks_ = 0;
  ++cellsTravelled_;
  battery_.consume(spec_.moveCostPerCell);
}

}  // namespace warehousesim::robots
