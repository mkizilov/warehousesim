#pragma once

#include <span>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/tasks/task.hpp"

namespace warehousesim::tasks {

struct Assignment {
  core::RobotId robot;
  core::TaskId task;

  bool operator==(const Assignment&) const = default;
};

class TaskAssigner {
 public:
  TaskAssigner() = default;
  TaskAssigner(const TaskAssigner&) = delete;
  TaskAssigner& operator=(const TaskAssigner&) = delete;
  virtual ~TaskAssigner() = default;

  // Pure decision: pairs idle robots with pending tasks. Each robot and task appears
  // at most once in the result. Does not mutate robots or tasks.
  [[nodiscard]] virtual std::vector<Assignment> assign(std::span<const robots::Robot> robots,
                                                       std::span<const Task> tasks) const = 0;
};

// Walks pending tasks in id order and gives each the closest idle robot (Manhattan
// distance to the pickup), breaking ties by lowest robot id.
class NearestRobotAssigner final : public TaskAssigner {
 public:
  [[nodiscard]] std::vector<Assignment> assign(std::span<const robots::Robot> robots,
                                               std::span<const Task> tasks) const override;
};

}  // namespace warehousesim::tasks
