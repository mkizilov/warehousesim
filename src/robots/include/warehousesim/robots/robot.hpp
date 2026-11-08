#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"

namespace warehousesim::robots {

enum class RobotStatus : std::uint8_t {
  Idle,
  ToPickup,
  ToDropoff,
};

[[nodiscard]] const char* toString(RobotStatus status) noexcept;

// A robot owns its own state and follows a path one cell per advance(). It knows
// nothing about the warehouse, planners, or tasks beyond the id it is working on.
class Robot {
 public:
  Robot(core::RobotId id, core::Point position) noexcept : id_(id), position_(position) {}

  [[nodiscard]] core::RobotId id() const noexcept { return id_; }
  [[nodiscard]] core::Point position() const noexcept { return position_; }
  [[nodiscard]] RobotStatus status() const noexcept { return status_; }
  [[nodiscard]] std::optional<core::TaskId> task() const noexcept { return task_; }
  [[nodiscard]] bool hasPath() const noexcept { return !path_.empty(); }
  [[nodiscard]] std::optional<core::Point> nextStep() const noexcept;
  [[nodiscard]] std::uint32_t blockedTicks() const noexcept { return blockedTicks_; }
  [[nodiscard]] std::uint64_t distanceTravelled() const noexcept { return distanceTravelled_; }

  // Idle -> ToPickup. Throws std::logic_error unless the robot is idle.
  void beginTask(core::TaskId task);
  // ToPickup -> ToDropoff. Throws std::logic_error unless heading to a pickup.
  void beginDelivery();
  // Any state -> Idle, dropping the task and remaining path.
  void finishTask() noexcept;

  // Replaces the remaining path and resets the blocked counter.
  void setPath(std::vector<core::Point> path);
  // Moves onto the next path cell. Throws std::logic_error when there is no path.
  void advance();

  void noteBlocked() noexcept { ++blockedTicks_; }
  void clearBlocked() noexcept { blockedTicks_ = 0; }

  // Moves to an orthogonally adjacent cell to make room for another robot. The old path
  // no longer applies, so it is dropped and the caller must set a new one if the robot is
  // still working. Throws std::invalid_argument if the cell is not adjacent.
  void sidestep(core::Point adjacent);

 private:
  core::RobotId id_;
  core::Point position_;
  RobotStatus status_ = RobotStatus::Idle;
  std::optional<core::TaskId> task_;
  std::deque<core::Point> path_;
  std::uint32_t blockedTicks_ = 0;
  std::uint64_t distanceTravelled_ = 0;
};

}  // namespace warehousesim::robots
