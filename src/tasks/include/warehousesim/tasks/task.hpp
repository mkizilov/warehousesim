#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"

namespace warehousesim::tasks {

enum class TaskStatus : std::uint8_t {
  Pending,
  Assigned,
  InProgress,  // picked up, being delivered
  Completed,
  Failed,  // a required location was unreachable
};

[[nodiscard]] const char* toString(TaskStatus status) noexcept;

struct Task {
  core::TaskId id;
  core::Point pickup;
  core::Point dropoff;
  TaskStatus status = TaskStatus::Pending;
  std::optional<core::RobotId> assignee;
  core::Tick createdAt = 0;

  bool operator==(const Task&) const = default;
};

// Owns every task and enforces the lifecycle
// Pending -> Assigned -> InProgress -> Completed, with Failed reachable from the middle two and
// Assigned -> Pending when a robot gives a task back before collecting it.
// Invalid transitions throw std::logic_error; unknown ids throw std::out_of_range.
class TaskBoard {
 public:
  core::TaskId submit(core::Point pickup, core::Point dropoff, core::Tick now);

  [[nodiscard]] const Task* find(core::TaskId id) const noexcept;
  [[nodiscard]] const std::vector<Task>& all() const noexcept { return tasks_; }
  [[nodiscard]] std::size_t count(TaskStatus status) const noexcept;

  void assign(core::TaskId id, core::RobotId robot);
  // Assigned -> Pending: the robot can no longer do it, so it goes back up for assignment.
  void release(core::TaskId id);
  void markInProgress(core::TaskId id);
  void complete(core::TaskId id);
  void fail(core::TaskId id);

 private:
  Task& mutableTask(core::TaskId id);
  void transition(core::TaskId id, TaskStatus expected, TaskStatus next);

  std::vector<Task> tasks_;  // task with id N lives at index N - 1
};

}  // namespace warehousesim::tasks
