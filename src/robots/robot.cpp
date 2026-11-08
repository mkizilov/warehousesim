#include "warehousesim/robots/robot.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include "warehousesim/core/angle.hpp"

namespace warehousesim::robots {

namespace {

using core::Direction;
using core::Point;

// Absorbs floating-point error when summing fractional distances (e.g. 3 x 1/3 must reach 1).
constexpr double kEpsilon = 1e-9;

Battery makeBattery(const RobotSpec& spec) {
  spec.validate();
  return Battery(spec.batteryCapacity, spec.initialCharge.value_or(spec.batteryCapacity));
}

}  // namespace

Robot::Robot(core::RobotId id, Point position, RobotSpec spec, Direction heading)
    : id_(id),
      position_(position),
      heading_(heading),
      headingAngle_(core::angleOf(heading)),
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

core::Vec2 Robot::pose() const noexcept {
  core::Vec2 center{static_cast<double>(position_.x) + 0.5, static_cast<double>(position_.y) + 0.5};
  if (progress_ > 0.0 && !path_.empty()) {
    center.x += static_cast<double>(path_.front().x - position_.x) * progress_;
    center.y += static_cast<double>(path_.front().y - position_.y) * progress_;
  }
  return center;
}

bool Robot::carriesCargo() const noexcept {
  return state_ == RobotState::Unloading ||
         (state_ == RobotState::Navigating && goal_ == NavigationGoal::Dropoff);
}

void Robot::setHeadingAngle(double angle) noexcept {
  headingAngle_ = angle;
  heading_ = core::nearestDirection(angle);
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
  const bool coasting = !navigating && !path_.empty();
  if (coasting && state_ != RobotState::Idle && state_ != RobotState::Unavailable) {
    return "only a navigating, idle or unavailable robot may have a path";
  }
  if (coasting && authority_ != path_.size()) {
    return "a coasting robot must hold all of the path it is stopping along";
  }
  if (navigating && !path_.empty() && path_.back() != *destination_) {
    return "path does not end at the destination";
  }
  Point previous = position_;
  for (const Point step : path_) {
    if (core::manhattanDistance(previous, step) != 1) {
      return "path steps must be orthogonally adjacent";
    }
    previous = step;
  }
  if (!navigating && !coasting && (speed_ != 0.0 || progress_ != 0.0 || authority_ != 0)) {
    return "speed, travel progress and authority must be zero unless the robot has a path";
  }
  if (authority_ > path_.size()) {
    return "authority exceeds the path";
  }
  if (speed_ < 0.0 || speed_ > spec_.maxSpeed + kEpsilon || progress_ < 0.0 || progress_ >= 1.0) {
    return "speed or travel progress out of range";
  }
  if (progress_ > 0.0 && authority_ == 0) {
    return "a robot between two cells must hold the cell it is driving into";
  }
  if (speed_ > 0.0 && path_.empty()) {
    return "a robot without a path cannot be moving";
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

void Robot::requireAtRest(const char* operation) const {
  if (!path_.empty()) {
    throw InvalidTransition(std::string(operation) + ": the robot is still moving");
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
  authority_ = 0;
  motion_ = MotionPhase::Stopped;
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
  requireAtRest("beginTask");
  requireBattery("beginTask");
  validateRoute(destination, path, "beginTask");
  task_ = task;
  startNavigation(NavigationGoal::Pickup, destination, std::move(path));
}

void Robot::beginChargerTrip(Point destination, RobotPath path) {
  require({RobotState::Idle}, RobotState::Navigating, "beginChargerTrip");
  requireAtRest("beginChargerTrip");
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
  ++deliveries_;
  enter(RobotState::Idle, NavigationGoal::None);
}

void Robot::redirectChargerTrip(Point destination, RobotPath path) {
  if (state_ != RobotState::Navigating || goal_ != NavigationGoal::Charger) {
    throw InvalidTransition("redirectChargerTrip: robot is not heading to a charger");
  }
  validateRoute(destination, path, "redirectChargerTrip");
  requireGrantedPrefix(path, "redirectChargerTrip");
  destination_ = destination;
  path_.assign(path.begin(), path.end());
  blockedTicks_ = 0;
}

void Robot::beginCharging() {
  require({RobotState::Idle, RobotState::Navigating}, RobotState::Charging, "beginCharging");
  if (state_ == RobotState::Navigating && (goal_ != NavigationGoal::Charger || !hasArrived())) {
    throw InvalidTransition("beginCharging: robot has not arrived at a charger");
  }
  requireAtRest("beginCharging");
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
  windDown();
  state_ = RobotState::Idle;
  goal_ = NavigationGoal::None;
  dwellRemaining_ = 0;
  blockedTicks_ = 0;
}

void Robot::makeUnavailable() {
  require({RobotState::Idle, RobotState::Navigating, RobotState::Loading, RobotState::Unloading,
           RobotState::Charging},
          RobotState::Unavailable, "makeUnavailable");
  task_.reset();
  windDown();
  state_ = RobotState::Unavailable;
  goal_ = NavigationGoal::None;
  dwellRemaining_ = 0;
  blockedTicks_ = 0;
}

void Robot::restore() {
  require({RobotState::Unavailable}, RobotState::Idle, "restore");
  requireBattery("restore");
  // A robot that was still stopping when it was taken out of service keeps doing so.
  state_ = RobotState::Idle;
  goal_ = NavigationGoal::None;
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

std::size_t Robot::mandatoryStopIndex(std::size_t scanLimit) const {
  const std::size_t limit = std::min(scanLimit, path_.size());
  for (std::size_t i = 0; i < limit; ++i) {
    if (i + 1 == path_.size()) {
      return i;  // the destination
    }
    if (!spec_.turnRate) {
      continue;  // turns instantly, so a corner is no reason to stop
    }
    const Point before = i == 0 ? position_ : path_[i - 1];
    if (core::directionBetween(before, path_[i]) !=
        core::directionBetween(path_[i], path_[i + 1])) {
      return i;
    }
  }
  return limit;
}

std::size_t Robot::wantedAuthority(double dt) const {
  if (path_.empty()) {
    return 0;
  }
  if (state_ != RobotState::Navigating) {
    return authority_;  // coasting to a stop: it only uses what it already holds
  }
  const MotionLimits limits = spec_.motionLimits();
  // The fastest the robot can be going during this step, how far that carries it, and how far
  // it then needs to stop. Everything up to there must already be its own.
  const double topSpeed = limits.acceleration
                              ? std::min(limits.maxSpeed, speed_ + *limits.acceleration * dt)
                              : limits.maxSpeed;
  const double reach = progress_ + topSpeed * dt + limits.brakingDistance(topSpeed);
  auto cells = static_cast<std::size_t>(std::ceil(std::max(reach - kEpsilon, 0.0)));
  cells = std::clamp<std::size_t>(cells, 1, path_.size());
  constexpr std::size_t kScan = 4;  // reach < 3 cells for any valid spec
  cells = std::min(cells, mandatoryStopIndex(kScan) + 1);
  return std::max(cells, authority_);
}

void Robot::extendAuthority() {
  if (authority_ >= path_.size()) {
    throw InvalidTransition("extendAuthority: there is no further path cell to grant");
  }
  ++authority_;
}

double Robot::turnTowardNext(double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("turnTowardNext: dt must be finite and positive");
  }
  if (path_.empty()) {
    throw InvalidTransition("turnTowardNext: the robot has no path to follow");
  }
  // A robot that turns at a finite rate does so on the spot, so it must be standing still; the
  // time the turn takes comes out of the time left for driving.
  const double target = core::angleOf(*core::directionBetween(position_, path_.front()));
  const double remaining = core::shortestRotation(headingAngle_, target);
  if (std::fabs(remaining) <= core::kAngleEpsilon) {
    return dt;
  }
  turnedThisStep_ = true;
  if (!spec_.turnRate) {
    setHeadingAngle(target);
    return dt;
  }
  if (speed_ > kEpsilon || progress_ > 0.0) {
    throw std::logic_error("turnTowardNext: a moving robot cannot turn on the spot");
  }
  const double reachable = *spec_.turnRate * dt;
  if (std::fabs(remaining) <= reachable + kEpsilon) {
    setHeadingAngle(target);
    return std::max(0.0, dt - std::fabs(remaining) / *spec_.turnRate);
  }
  setHeadingAngle(core::wrapAngle(headingAngle_ + std::copysign(reachable, remaining)));
  return 0.0;
}

std::uint32_t Robot::advance(double dt) {
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("advance: dt must be finite and non-negative");
  }
  if (path_.empty()) {
    throw InvalidTransition("advance: the robot has no path to follow");
  }
  const double speedBefore = speed_;
  const double budget = dt;
  const bool turned = turnedThisStep_;
  turnedThisStep_ = false;
  unusedTime_ = 0.0;

  std::uint32_t passed = 0;
  MotionStep step;
  if (budget > kEpsilon && authority_ > 0) {
    const std::size_t mandatory = mandatoryStopIndex(authority_);
    const bool stopIsMandatory = mandatory < authority_;
    const std::size_t stopIndex = std::min(mandatory, authority_ - 1);
    MotionLimits limits = spec_.motionLimits();
    if (state_ != RobotState::Navigating && speed_ > kEpsilon) {
      // Withdrawn from its job mid-move: from here it only ever slows down, holding the speed it
      // has until it has to brake for the waypoint it kept.
      limits.maxSpeed = std::min(limits.maxSpeed, speed_);
    }
    step = advanceAlongRun(limits, speed_, static_cast<double>(stopIndex + 1) - progress_, budget);
    if (step.exceededDeceleration) {
      ++brakingViolations_;
    }

    double progress = progress_ + step.distance;
    if (step.arrived) {
      passed = static_cast<std::uint32_t>(stopIndex + 1);
      progress = 0.0;
    } else {
      while (passed < stopIndex && progress >= 1.0 - kEpsilon) {
        ++passed;
        progress -= 1.0;
      }
      progress = std::min(progress, 1.0 - kEpsilon);
    }
    for (std::uint32_t i = 0; i < passed; ++i) {
      position_ = path_.front();
      path_.pop_front();
      --authority_;
      ++cellsTravelled_;
      blockedTicks_ = 0;
      battery_.consume(spec_.moveCostPerCell);
    }
    progress_ = path_.empty() ? 0.0 : progress;
    if (step.arrived && !path_.empty() && (stopIsMandatory || spec_.deceleration)) {
      unusedTime_ = std::max(0.0, budget - step.timeUsed);
      if (unusedTime_ <= kEpsilon) {
        unusedTime_ = 0.0;
      }
    }

    if (path_.empty()) {
      speed_ = 0.0;
    } else if (step.arrived) {
      // Running out of granted cells exactly as the step ends is not a stop for a robot that
      // brakes instantly: it was still driving, the next step simply needs more authority.
      const bool stillDriving =
          !stopIsMandatory && !spec_.deceleration && step.timeUsed >= budget - kEpsilon;
      speed_ = stillDriving ? step.speedOnArrival : 0.0;
    } else {
      speed_ = step.speed;
    }
    // A robot that turns instantly and has already driven into the next segment must face along
    // it, or its heading and pose would disagree. One that stops exactly on a waypoint turns when
    // it next moves.
    if (passed > 0 && !path_.empty() && !spec_.turnRate && progress_ > 0.0) {
      setHeadingAngle(core::angleOf(*core::directionBetween(position_, path_.front())));
    }
  } else {
    // A robot that brakes at a finite rate is never left without the room to stop in.
    if (speed_ > kEpsilon && spec_.deceleration) {
      ++brakingViolations_;
    }
    speed_ = 0.0;
  }

  if (turned && speed_ <= kEpsilon && step.distance <= kEpsilon) {
    motion_ = MotionPhase::Turning;
  } else if (speed_ > speedBefore + kEpsilon) {
    motion_ = MotionPhase::Accelerating;
  } else if (speed_ < speedBefore - kEpsilon) {
    motion_ = MotionPhase::Braking;
  } else {
    motion_ = speed_ > kEpsilon ? MotionPhase::Cruising : MotionPhase::Stopped;
  }
  return passed;
}

void Robot::requireGrantedPrefix(const RobotPath& path, const char* operation) const {
  for (std::size_t i = 0; i < authority_; ++i) {
    if (i >= path.size() || path[i] != path_[i]) {
      throw std::invalid_argument(std::string(operation) +
                                  ": the cells the robot has been granted cannot be changed");
    }
  }
}

void Robot::setPath(RobotPath path) {
  if (state_ != RobotState::Navigating) {
    throw InvalidTransition("setPath: robot is not navigating");
  }
  validateRoute(*destination_, path, "setPath");
  requireGrantedPrefix(path, "setPath");
  path_.assign(path.begin(), path.end());
  blockedTicks_ = 0;
}

Robot Robot::withRouteCutTo(std::size_t pathCells) const {
  Robot copy = *this;
  const std::size_t keep = std::max(pathCells, authority_);
  if (keep < copy.path_.size()) {
    copy.path_.resize(keep);
    if (copy.destination_) {
      copy.destination_ = keep == 0 ? copy.position_ : copy.path_.back();
    }
  }
  return copy;
}

void Robot::windDown() {
  const bool moving = authority_ > 0 && (speed_ > kEpsilon || progress_ > kEpsilon);
  if (!moving) {
    clearRoute();
    speed_ = 0.0;
    progress_ = 0.0;
    authority_ = 0;
    motion_ = MotionPhase::Stopped;
    return;
  }
  // Brake along the cells already granted and stop at the first one that is far enough away.
  const double brake = spec_.motionLimits().brakingDistance(speed_);
  std::size_t stopAt = authority_ - 1;
  for (std::size_t i = 0; i < authority_; ++i) {
    if (static_cast<double>(i + 1) - progress_ >= brake - kEpsilon) {
      stopAt = i;
      break;
    }
  }
  path_.resize(stopAt + 1);
  authority_ = stopAt + 1;
  destination_.reset();
}

void Robot::sidestep(Point adjacent) {
  if (isStationary()) {
    throw InvalidTransition("sidestep: a stationary robot cannot be moved aside");
  }
  if (authority_ > 0 || progress_ > 0.0) {
    throw InvalidTransition("sidestep: a robot with cells granted ahead cannot be moved aside");
  }
  const std::optional<Direction> direction = core::directionBetween(position_, adjacent);
  if (!direction) {
    throw std::invalid_argument("sidestep target must be adjacent");
  }
  position_ = adjacent;
  setHeadingAngle(core::angleOf(*direction));
  path_.clear();
  speed_ = 0.0;
  progress_ = 0.0;
  motion_ = MotionPhase::Stopped;
  blockedTicks_ = 0;
  ++cellsTravelled_;
  battery_.consume(spec_.moveCostPerCell);
}

}  // namespace warehousesim::robots
