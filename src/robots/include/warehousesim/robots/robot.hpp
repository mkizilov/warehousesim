#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/vec2.hpp"
#include "warehousesim/robots/battery.hpp"
#include "warehousesim/robots/motion.hpp"
#include "warehousesim/robots/robot_spec.hpp"
#include "warehousesim/robots/robot_state.hpp"

namespace warehousesim::robots {

// Velocity in cells per tick: +x is east, +y is south.
struct Velocity {
  double x = 0.0;
  double y = 0.0;
  friend bool operator==(const Velocity&, const Velocity&) = default;
};

using RobotPath = std::vector<core::Point>;

// A simulated robot: identity, pose, motion, battery, task, path and an explicit state machine.
//
// A Robot is plain state plus rules. It never looks at the warehouse, other robots, planners or
// task lists (the Simulation does that and tells the robot what happened), and it knows nothing
// about drawing. Every state change goes through a named operation below; an operation that is
// not allowed from the current state, or whose preconditions do not hold, throws
// InvalidTransition (or std::invalid_argument for a malformed argument) and changes nothing.
//
// Logical and continuous position. position() is the grid cell the robot last passed through;
// that is what occupancy, planning and tasks use. pose() is where its centre really is, which is
// between two cells while it drives from one to the next. The two are kept separate on purpose:
// the world only ever sees whole cells, the picture only ever sees the pose.
//
// Motion is fixed-step. The simulation grants the robot the cells ahead of it that it needs to
// be able to stop in (authority); drive() then moves it along them within its speed,
// acceleration, deceleration and turn-rate limits, and never beyond them. A robot that cannot
// be granted its next cell stays where it is.
//
// Invariants (checked by inconsistency(), and held after every operation):
//   * Navigating <=> goal() != None; the robot has a destination exactly when navigating.
//   * A task is held exactly while navigating to a pickup/dropoff, loading or unloading.
//   * A path exists only while Navigating, or while an idle/unavailable robot is still coasting
//     to a stop after its task was taken away mid-move. Speed, motion progress, authority and
//     dwell time are zero whenever they do not apply; the battery stays within [0, capacity].
//   * progress is in [0, 1) and non-zero only with authority; authority never exceeds the path.
//   * The robot id never changes.
class Robot {
 public:
  // Throws std::invalid_argument for an invalid id or spec.
  Robot(core::RobotId id, core::Point position, RobotSpec spec = {},
        core::Direction heading = core::Direction::North);

  // ---- identity and pose ----
  [[nodiscard]] core::RobotId id() const noexcept { return id_; }
  [[nodiscard]] core::Point position() const noexcept { return position_; }
  // The cardinal direction closest to headingAngle().
  [[nodiscard]] core::Direction heading() const noexcept { return heading_; }
  [[nodiscard]] const RobotSpec& spec() const noexcept { return spec_; }

  // ---- motion ----
  [[nodiscard]] double maxSpeed() const noexcept { return spec_.maxSpeed; }
  // Cells per tick right now (0 when stopped).
  [[nodiscard]] double speed() const noexcept { return speed_; }
  [[nodiscard]] Velocity velocity() const noexcept;
  [[nodiscard]] std::uint64_t cellsTravelled() const noexcept { return cellsTravelled_; }
  // Where the robot's centre is, in world space (cell (x, y) has its centre at (x + 0.5, y + 0.5)).
  [[nodiscard]] core::Vec2 pose() const noexcept;
  // Continuous heading in radians: east 0, south pi/2, west pi, north -pi/2.
  [[nodiscard]] double headingAngle() const noexcept { return headingAngle_; }
  // How far along the way from position() to the next path cell the robot is, in [0, 1).
  [[nodiscard]] double segmentProgress() const noexcept { return progress_; }
  // What the last drive() did: stopped, turning on the spot, speeding up, cruising or braking.
  [[nodiscard]] MotionPhase motion() const noexcept { return motion_; }
  // How many leading path cells the robot has been granted and may therefore drive into.
  [[nodiscard]] std::size_t authority() const noexcept { return authority_; }
  // It has been granted cells ahead, so it cannot be redirected or pushed aside.
  [[nodiscard]] bool isCommitted() const noexcept { return authority_ > 0; }
  // Times a drive() had to brake harder than the spec allows. A simulation that grants enough
  // authority keeps this at zero; it exists so that tests can prove it.
  [[nodiscard]] std::uint64_t brakingViolations() const noexcept { return brakingViolations_; }

  // ---- cargo ----
  // Between finishing loading and finishing unloading the robot carries its task's cargo.
  [[nodiscard]] bool carriesCargo() const noexcept;
  // Cargo handed over at a dropoff, over the robot's lifetime.
  [[nodiscard]] std::uint64_t deliveries() const noexcept { return deliveries_; }

  // ---- battery ----
  [[nodiscard]] const Battery& battery() const noexcept { return battery_; }
  // Below the spec's low-battery level: the robot wants to recharge.
  [[nodiscard]] bool needsCharge() const noexcept {
    return battery_.level() < spec_.lowBatteryLevel;
  }

  // ---- task and path ----
  [[nodiscard]] std::optional<core::TaskId> task() const noexcept { return task_; }
  [[nodiscard]] std::optional<core::Point> destination() const noexcept { return destination_; }
  [[nodiscard]] const std::deque<core::Point>& path() const noexcept { return path_; }
  [[nodiscard]] bool hasPath() const noexcept { return !path_.empty(); }
  [[nodiscard]] std::optional<core::Point> nextStep() const noexcept;
  [[nodiscard]] std::uint32_t blockedTicks() const noexcept { return blockedTicks_; }

  // ---- state machine ----
  [[nodiscard]] RobotState state() const noexcept { return state_; }
  [[nodiscard]] NavigationGoal goal() const noexcept { return goal_; }
  // Ticks left in Loading/Unloading before the phase may be finished.
  [[nodiscard]] std::uint32_t dwellRemaining() const noexcept { return dwellRemaining_; }
  [[nodiscard]] bool canTransitionTo(RobotState next) const noexcept {
    return isValidTransition(state_, next);
  }
  [[nodiscard]] bool isStationary() const noexcept { return robots::isStationary(state_); }
  // Idle, at rest, holding some charge and not wanting a recharge: may be given a task.
  [[nodiscard]] bool isAvailable() const noexcept {
    return state_ == RobotState::Idle && path_.empty() && !needsCharge() && !battery_.isEmpty();
  }
  // Navigating and standing on its destination.
  [[nodiscard]] bool hasArrived() const noexcept;

  // Returns a description of the first broken invariant, or nullopt if the robot is consistent.
  [[nodiscard]] std::optional<std::string> inconsistency() const;

  // ---- transitions ----
  // Idle|Charging -> Navigating(Pickup). `path` leads from the current cell to `destination`
  // (excluding the current cell; empty if already there). Requires a non-empty battery.
  void beginTask(core::TaskId task, core::Point destination, RobotPath path);
  // Idle -> Navigating(Charger). Requires a non-empty battery.
  void beginChargerTrip(core::Point destination, RobotPath path);
  // Navigating(Pickup), arrived -> Loading.
  void beginLoading();
  // Loading, dwell finished -> Navigating(Dropoff).
  void finishLoading(core::Point destination, RobotPath path);
  // Navigating(Dropoff), arrived -> Unloading.
  void beginUnloading();
  // Unloading, dwell finished -> Idle; the task is released from the robot.
  void finishUnloading();
  // Navigating(Charger) -> Navigating(Charger) towards a different charger.
  void redirectChargerTrip(core::Point destination, RobotPath path);
  // Idle, or Navigating(Charger) once arrived -> Charging.
  void beginCharging();
  // Charging -> Idle.
  void stopCharging();
  // Navigating|Loading|Unloading -> Idle, dropping the task and path.
  void abortTask();
  // Any state but Unavailable -> Unavailable, dropping the task and path.
  void makeUnavailable();
  // Unavailable -> Idle. Requires a non-empty battery.
  void restore();
  // Only while Unavailable: swaps in a fully charged battery.
  void replaceBattery();

  // ---- per-tick updates, called by the simulation ----
  // Time passes in the current state: Idle drains, Loading/Unloading drain and count down,
  // Charging recharges. Navigating and Unavailable robots are unaffected.
  void elapseTick();
  // How many leading path cells the robot would like to be granted for a step of `dt` ticks:
  // enough to cover the distance it can drive plus the distance it needs to stop afterwards, but
  // not past a point where it has to stop anyway (a corner it must turn at, or the destination).
  // Never less than the authority it already has; 0 without a path, and for a coasting robot
  // just what it already holds.
  [[nodiscard]] std::size_t wantedAuthority(double dt) const;
  // Grants the next path cell (the caller has claimed it in the world). Throws InvalidTransition
  // if the whole path is already granted.
  void extendAuthority();
  // A fixed step of motion along the path has two parts, in this order. The simulation does
  // something in between: with the robot facing the right way and knowing how much of the step
  // is left, it claims the cells ahead that the robot wants.
  //
  // Turns towards the next path cell for up to `dt` ticks: on the spot at turnRate, or instantly
  // if the robot has no turn-rate limit. Returns the part of `dt` left for driving (0 if the
  // turn is not finished). A moving robot cannot turn, and the corners a robot turns at are
  // places it stops. Throws InvalidTransition without a path.
  [[nodiscard]] double turnTowardNext(double dt);
  // Drives for `dt` ticks within the granted authority, under the speed, acceleration and
  // deceleration limits, braking to rest at the end of its authority, at a corner it has to turn
  // at, or at the destination. Passing a path cell makes it the robot's position and charges
  // moveCostPerCell. Returns how many cells were passed (at most one for maxSpeed * dt <= 1).
  // With no authority, or no time left, it stays put. Throws InvalidTransition without a path.
  [[nodiscard]] std::uint32_t advance(double dt);
  // Time left over from the last advance(): it came to rest at a corner or at the end of its
  // authority with path still ahead before the step was up. A caller that wants the motion to
  // be the same whatever the step length carries this into another turnTowardNext()/advance()
  // round, exactly as a shorter step would have. Zero otherwise.
  [[nodiscard]] double unusedTime() const noexcept { return unusedTime_; }
  // turnTowardNext(dt) followed by advance() with whatever time that left.
  [[nodiscard]] std::uint32_t drive(double dt = 1.0) { return advance(turnTowardNext(dt)); }
  // Replaces the remaining path while Navigating; it must end at the destination and keep the
  // cells the robot has already been granted.
  void setPath(RobotPath path);
  // Forced one-cell move to an adjacent cell to make room for another robot. Only a robot that
  // is at rest with no authority and not in a stationary state can be moved. The old path is
  // dropped; a navigating robot must be given a new one.
  void sidestep(core::Point adjacent);

  void noteBlocked() noexcept { ++blockedTicks_; }
  void clearBlocked() noexcept { blockedTicks_ = 0; }

 private:
  // Throws InvalidTransition unless the robot is in one of `allowedFrom` and the state table
  // permits the move to `next`.
  void require(std::initializer_list<RobotState> allowedFrom, RobotState next,
               const char* operation) const;
  void requireAtRest(const char* operation) const;
  // The first authority() cells of `path` must be the ones already granted.
  void requireGrantedPrefix(const RobotPath& path, const char* operation) const;
  void requireBattery(const char* operation) const;
  void validateRoute(core::Point destination, const RobotPath& path, const char* operation) const;
  void startNavigation(NavigationGoal goal, core::Point destination, RobotPath path);
  void enter(RobotState next, NavigationGoal goal);
  void clearRoute() noexcept;
  // Gives up the route. At rest this is immediate; while moving the robot keeps the part of its
  // path it needs to come to a stop and coasts along it.
  void windDown();
  void setHeadingAngle(double angle) noexcept;
  // Index of the first path cell, within the first `scanLimit`, at which the robot has to stop:
  // the end of the path, or a corner if it cannot turn while moving. `scanLimit` if none.
  [[nodiscard]] std::size_t mandatoryStopIndex(std::size_t scanLimit) const;

  core::RobotId id_;
  core::Point position_;
  core::Direction heading_;
  double headingAngle_;
  RobotSpec spec_;
  Battery battery_;

  RobotState state_ = RobotState::Idle;
  NavigationGoal goal_ = NavigationGoal::None;
  std::optional<core::TaskId> task_;
  std::optional<core::Point> destination_;
  std::deque<core::Point> path_;

  double speed_ = 0.0;
  double progress_ = 0.0;  // fraction of the way from position_ to path_.front()
  std::size_t authority_ = 0;
  MotionPhase motion_ = MotionPhase::Stopped;
  bool turnedThisStep_ = false;  // set by turnTowardNext(), consumed by advance()
  double unusedTime_ = 0.0;
  std::uint32_t dwellRemaining_ = 0;
  std::uint32_t blockedTicks_ = 0;
  std::uint64_t cellsTravelled_ = 0;
  std::uint64_t deliveries_ = 0;
  std::uint64_t brakingViolations_ = 0;
};

}  // namespace warehousesim::robots
