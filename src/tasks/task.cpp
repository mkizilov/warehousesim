#include "warehousesim/tasks/task.hpp"

#include <algorithm>
#include <stdexcept>

namespace warehousesim::tasks {

const char* toString(TaskStatus status) noexcept {
  switch (status) {
    case TaskStatus::Pending:
      return "pending";
    case TaskStatus::Assigned:
      return "assigned";
    case TaskStatus::InProgress:
      return "in_progress";
    case TaskStatus::Completed:
      return "completed";
    case TaskStatus::Failed:
      return "failed";
  }
  return "unknown";
}

core::TaskId TaskBoard::submit(core::Point pickup, core::Point dropoff, core::Tick now) {
  const core::TaskId id{static_cast<std::uint32_t>(tasks_.size() + 1)};
  tasks_.push_back(Task{id, pickup, dropoff, TaskStatus::Pending, std::nullopt, now});
  return id;
}

const Task* TaskBoard::find(core::TaskId id) const noexcept {
  if (!id.valid() || id.value > tasks_.size()) {
    return nullptr;
  }
  return &tasks_[id.value - 1];
}

std::size_t TaskBoard::count(TaskStatus status) const noexcept {
  return static_cast<std::size_t>(std::ranges::count(tasks_, status, &Task::status));
}

void TaskBoard::assign(core::TaskId id, core::RobotId robot) {
  transition(id, TaskStatus::Pending, TaskStatus::Assigned);
  mutableTask(id).assignee = robot;
}

void TaskBoard::release(core::TaskId id) {
  transition(id, TaskStatus::Assigned, TaskStatus::Pending);
  mutableTask(id).assignee.reset();
}

void TaskBoard::markInProgress(core::TaskId id) {
  transition(id, TaskStatus::Assigned, TaskStatus::InProgress);
}

void TaskBoard::complete(core::TaskId id) {
  transition(id, TaskStatus::InProgress, TaskStatus::Completed);
}

void TaskBoard::fail(core::TaskId id) {
  Task& task = mutableTask(id);
  if (task.status != TaskStatus::Assigned && task.status != TaskStatus::InProgress) {
    throw std::logic_error("only assigned or in-progress tasks can fail");
  }
  task.status = TaskStatus::Failed;
}

Task& TaskBoard::mutableTask(core::TaskId id) {
  if (!id.valid() || id.value > tasks_.size()) {
    throw std::out_of_range("unknown task id");
  }
  return tasks_[id.value - 1];
}

void TaskBoard::transition(core::TaskId id, TaskStatus expected, TaskStatus next) {
  Task& task = mutableTask(id);
  if (task.status != expected) {
    throw std::logic_error("invalid task status transition");
  }
  task.status = next;
}

}  // namespace warehousesim::tasks
