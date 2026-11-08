#include "warehousesim/tasks/task_queue.hpp"

#include <stdexcept>
#include <string>

namespace warehousesim::tasks {

core::TaskId TaskQueue::submit(core::Point pickup, core::Point dropoff, core::Tick now,
                               TaskPriority priority) {
  const core::TaskId id{static_cast<std::uint32_t>(tasks_.size() + 1)};
  Task& task = tasks_.emplace_back();
  task.id = id;
  task.pickup = pickup;
  task.dropoff = dropoff;
  task.createdAt = now;
  task.priority = priority;
  ++counts_[static_cast<std::size_t>(TaskStatus::Pending)];
  pending_.insert({priority, now, id});
  return id;
}

const Task* TaskQueue::find(core::TaskId id) const noexcept {
  if (!id.valid() || id.value > tasks_.size()) {
    return nullptr;
  }
  return &tasks_[id.value - 1];
}

Task& TaskQueue::mutableTask(core::TaskId id) {
  if (!id.valid() || id.value > tasks_.size()) {
    throw std::out_of_range("unknown task id");
  }
  return tasks_[id.value - 1];
}

void TaskQueue::expect(const Task& task, TaskStatus expected) const {
  if (task.status != expected) {
    throw std::logic_error(std::string("invalid task status transition from ") +
                           toString(task.status));
  }
}

void TaskQueue::setStatus(Task& task, TaskStatus next) {
  --counts_[static_cast<std::size_t>(task.status)];
  ++counts_[static_cast<std::size_t>(next)];
  if (task.status == TaskStatus::Pending) {
    pending_.erase({task.priority, task.createdAt, task.id});
  }
  if (next == TaskStatus::Pending) {
    pending_.insert({task.priority, task.createdAt, task.id});
  }
  task.status = next;
}

bool TaskQueue::tryAssign(core::TaskId id, core::RobotId robot) {
  Task& task = mutableTask(id);
  if (task.status != TaskStatus::Pending) {
    return false;
  }
  setStatus(task, TaskStatus::Assigned);
  task.assignee = robot;
  return true;
}

void TaskQueue::assign(core::TaskId id, core::RobotId robot) {
  if (!tryAssign(id, robot)) {
    throw std::logic_error(std::string("task cannot be assigned: it is ") +
                           toString(mutableTask(id).status));
  }
}

void TaskQueue::release(core::TaskId id) {
  Task& task = mutableTask(id);
  expect(task, TaskStatus::Assigned);
  setStatus(task, TaskStatus::Pending);
  task.assignee.reset();
}

void TaskQueue::markInProgress(core::TaskId id) {
  Task& task = mutableTask(id);
  expect(task, TaskStatus::Assigned);
  setStatus(task, TaskStatus::InProgress);
}

void TaskQueue::complete(core::TaskId id) {
  Task& task = mutableTask(id);
  expect(task, TaskStatus::InProgress);
  setStatus(task, TaskStatus::Completed);
}

void TaskQueue::fail(core::TaskId id) {
  Task& task = mutableTask(id);
  if (!task.isAssigned()) {
    throw std::logic_error("only assigned or in-progress tasks can fail");
  }
  setStatus(task, TaskStatus::Failed);
}

void TaskQueue::reject(core::TaskId id) {
  Task& task = mutableTask(id);
  expect(task, TaskStatus::Pending);
  setStatus(task, TaskStatus::Failed);
}

bool TaskQueue::cancel(core::TaskId id) {
  Task& task = mutableTask(id);
  if (task.isFinished()) {
    return false;
  }
  setStatus(task, TaskStatus::Cancelled);
  return true;
}

bool TaskQueue::setPriority(core::TaskId id, TaskPriority priority) {
  Task& task = mutableTask(id);
  if (task.status != TaskStatus::Pending) {
    return false;
  }
  if (task.priority != priority) {
    pending_.erase({task.priority, task.createdAt, task.id});
    task.priority = priority;
    pending_.insert({priority, task.createdAt, task.id});
  }
  return true;
}

}  // namespace warehousesim::tasks
