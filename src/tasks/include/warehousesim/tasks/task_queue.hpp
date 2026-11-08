#pragma once

#include <array>
#include <cstddef>
#include <set>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/tasks/task.hpp"

namespace warehousesim::tasks {

// A pending task's place in the dispatch order.
struct PendingKey {
  TaskPriority priority;
  core::Tick createdAt;
  core::TaskId id;
};

// Highest priority first, then oldest, then lowest id: a total order, so the queue's
// order never depends on insertion history or on how the container is implemented.
struct PendingOrder {
  bool operator()(const PendingKey& a, const PendingKey& b) const noexcept {
    if (a.priority != b.priority) {
      return a.priority > b.priority;
    }
    if (a.createdAt != b.createdAt) {
      return a.createdAt < b.createdAt;
    }
    return a.id < b.id;
  }
};

using PendingSet = std::set<PendingKey, PendingOrder>;

// The central task queue: owns every task ever submitted, enforces the lifecycle documented on
// TaskStatus, and keeps the waiting tasks in dispatch order.
//
// A task is exclusive: it is held by at most one robot at a time. tryAssign() is the only way to
// claim one, and it succeeds for exactly one caller, so two robots can never be given the same task
// even if a dispatcher proposes it twice.
//
// Unknown or default ids throw std::out_of_range. Transitions that the lifecycle does not allow
// throw std::logic_error, except tryAssign() and cancel(), which report "not possible" by returning
// false because losing such a race is an expected outcome rather than a bug.
class TaskQueue {
 public:
  core::TaskId submit(core::Point pickup, core::Point dropoff, core::Tick now,
                      TaskPriority priority = TaskPriority::Normal);

  [[nodiscard]] const Task* find(core::TaskId id) const noexcept;
  [[nodiscard]] const std::vector<Task>& all() const noexcept { return tasks_; }
  [[nodiscard]] std::size_t count(TaskStatus status) const noexcept {
    return counts_[static_cast<std::size_t>(status)];
  }
  // Tasks waiting for a robot, in dispatch order.
  [[nodiscard]] const PendingSet& pending() const noexcept { return pending_; }

  // Pending -> Assigned to `robot`. Returns false (changing nothing) if the task is not pending,
  // for instance because another robot already claimed it or it was cancelled.
  [[nodiscard]] bool tryAssign(core::TaskId id, core::RobotId robot);
  // Like tryAssign, but a task that cannot be claimed is a logic error.
  void assign(core::TaskId id, core::RobotId robot);
  // Assigned -> Pending: the robot can no longer do it, so it goes back up for assignment.
  void release(core::TaskId id);
  void markInProgress(core::TaskId id);
  void complete(core::TaskId id);
  // Assigned or InProgress -> Failed.
  void fail(core::TaskId id);
  // Pending -> Failed: the dispatcher determined the task cannot be done at all.
  void reject(core::TaskId id);
  // Any open state -> Cancelled. Returns false if the task had already finished. Telling a robot
  // that holds the task to stop is the caller's job (Simulation::cancelTask does it).
  bool cancel(core::TaskId id);
  // Changes the priority of a task that is still Pending, which moves it to its new place in the
  // dispatch order (its age and id do not change). Returns false, changing nothing, for a task in
  // any other state: priority only orders tasks that wait, so for a task a robot has claimed there
  // is nothing left for it to decide. Setting the priority it already has succeeds.
  bool setPriority(core::TaskId id, TaskPriority priority);

 private:
  Task& mutableTask(core::TaskId id);
  void expect(const Task& task, TaskStatus expected) const;
  void setStatus(Task& task, TaskStatus next);

  std::vector<Task> tasks_;  // task with id N lives at index N - 1
  PendingSet pending_;
  std::array<std::size_t, kTaskStatusCount> counts_{};
};

}  // namespace warehousesim::tasks
