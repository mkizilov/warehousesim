#pragma once

#include <cstdint>
#include <deque>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/battery.hpp"
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
// Invariants (checked by inconsistency(), and held after every operation):
//   * Navigating <=> goal() != None; the robot has a destination exactly when navigating.
//   * A task is held exactly while navigating to a pickup/dropoff, loading or unloading.
//   * A path exists only while Navigating; speed, motion progress and dwell time are zero
//     whenever they do not apply; the battery level stays within [0, capacity].
//   * The robot id never changes.
class Robot {
 public:
  // Throws std::invalid_argument for an invalid id or spec.
  Robot(core::RobotId id, core::Point position, RobotSpec spec = {},
        core::Direction heading = core::Direction::North);

  // ---- identity and pose ----
  [[nodiscard]] core::RobotId id() const noexcept { return id_; }
  [[nodiscard]] core::Point position() const noexcept { return position_; }
  [[nodiscard]] core::Direction heading() const noexcept { return heading_; }
  [[nodiscard]] const RobotSpec& spec() const noexcept { return spec_; }

  // ---- motion ----
  [[nodiscard]] double maxSpeed() const noexcept { return spec_.maxSpeed; }
  // Cells per tick right now: maxSpeed while travelling, 0 otherwise.
  [[nodiscard]] double speed() const noexcept { return speed_; }
  [[nodiscard]] Velocity velocity() const noexcept;
  [[nodiscard]] std::uint64_t cellsTravelled() const noexcept { return cellsTravelled_; }

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
  // Idle and not wanting a recharge: may be given a task.
  [[nodiscard]] bool isAvailable() const noexcept {
    return state_ == RobotState::Idle && !needsCharge();
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
  // While Navigating with a path: faces the next cell and adds one tick of travel at max speed.
  // Returns true once a whole cell has been covered and the robot may enter the next cell.
  [[nodiscard]] bool travel();
  // The next cell is not available this tick: stop and wait.
  void halt();
  // Enters the next path cell (after travel() returned true): moves, consumes energy, and
  // arrives (speed 0) if that was the last step.
  void commitStep();
  // Replaces the remaining path while Navigating; it must end at the destination.
  void setPath(RobotPath path);
  // Forced one-cell move to an adjacent cell to make room for another robot (not allowed in a
  // stationary state). The old path is dropped; a navigating robot must be given a new one.
  void sidestep(core::Point adjacent);

  void noteBlocked() noexcept { ++blockedTicks_; }
  void clearBlocked() noexcept { blockedTicks_ = 0; }

 private:
  // Throws InvalidTransition unless the robot is in one of `allowedFrom` and the state table
  // permits the move to `next`.
  void require(std::initializer_list<RobotState> allowedFrom, RobotState next,
               const char* operation) const;
  void requireBattery(const char* operation) const;
  void validateRoute(core::Point destination, const RobotPath& path, const char* operation) const;
  void startNavigation(NavigationGoal goal, core::Point destination, RobotPath path);
  void enter(RobotState next, NavigationGoal goal);
  void clearRoute() noexcept;

  core::RobotId id_;
  core::Point position_;
  core::Direction heading_;
  RobotSpec spec_;
  Battery battery_;

  RobotState state_ = RobotState::Idle;
  NavigationGoal goal_ = NavigationGoal::None;
  std::optional<core::TaskId> task_;
  std::optional<core::Point> destination_;
  std::deque<core::Point> path_;

  double speed_ = 0.0;
  double progress_ = 0.0;  // fraction of the next cell already covered
  std::uint32_t dwellRemaining_ = 0;
  std::uint32_t blockedTicks_ = 0;
  std::uint64_t cellsTravelled_ = 0;
};

}  // namespace warehousesim::robots
