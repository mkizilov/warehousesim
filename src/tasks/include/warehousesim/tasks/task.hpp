#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"

namespace warehousesim::tasks {

// How urgently a task should be served: the dispatcher always considers higher priorities first.
enum class TaskPriority : std::uint8_t {
  Low,
  Normal,
  High,
  Urgent,
};

[[nodiscard]] const char* toString(TaskPriority priority) noexcept;

// Lifecycle:
//
//   Pending -> Assigned -> InProgress -> Completed
//
// Assigned -> Pending when a robot gives the task back before collecting it. Failed is reachable
// from Pending (the dispatcher found it impossible), Assigned and InProgress. Cancelled is
// reachable from every open state. Completed, Failed and Cancelled are final.
enum class TaskStatus : std::uint8_t {
  Pending,     // waiting in the queue for a robot
  Assigned,    // exactly one robot has claimed it and is heading to the pickup (or loading)
  InProgress,  // picked up, being delivered
  Completed,   // delivered
  Failed,      // could not be done (unreachable, or the carrying robot was lost)
  Cancelled,   // withdrawn by the requester
};

inline constexpr std::size_t kTaskStatusCount = 6;

[[nodiscard]] const char* toString(TaskStatus status) noexcept;

struct Task {
  core::TaskId id;
  core::Point pickup;
  core::Point dropoff;
  TaskStatus status = TaskStatus::Pending;
  std::optional<core::RobotId> assignee;  // the robot that holds it, or held it last
  core::Tick createdAt = 0;
  TaskPriority priority = TaskPriority::Normal;

  // Still to be done: not yet completed, failed or cancelled.
  [[nodiscard]] bool isOpen() const noexcept {
    return status == TaskStatus::Pending || status == TaskStatus::Assigned ||
           status == TaskStatus::InProgress;
  }
  // A robot currently holds it.
  [[nodiscard]] bool isAssigned() const noexcept {
    return status == TaskStatus::Assigned || status == TaskStatus::InProgress;
  }
  [[nodiscard]] bool isFinished() const noexcept { return !isOpen(); }

  bool operator==(const Task&) const = default;
};

}  // namespace warehousesim::tasks
