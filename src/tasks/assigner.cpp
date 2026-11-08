#include "warehousesim/tasks/assigner.hpp"

namespace warehousesim::tasks {

std::vector<Assignment> NearestRobotAssigner::assign(std::span<const robots::Robot> robots,
                                                     std::span<const Task> tasks) const {
  std::vector<const robots::Robot*> available;
  for (const robots::Robot& robot : robots) {
    if (robot.isAvailable()) {
      available.push_back(&robot);
    }
  }

  std::vector<Assignment> assignments;
  for (const Task& task : tasks) {
    if (available.empty()) {
      break;
    }
    if (task.status != TaskStatus::Pending) {
      continue;
    }
    auto best = available.begin();
    std::int32_t bestDistance = core::manhattanDistance((*best)->position(), task.pickup);
    for (auto it = available.begin() + 1; it != available.end(); ++it) {
      const std::int32_t distance = core::manhattanDistance((*it)->position(), task.pickup);
      if (distance < bestDistance || (distance == bestDistance && (*it)->id() < (*best)->id())) {
        best = it;
        bestDistance = distance;
      }
    }
    assignments.push_back({(*best)->id(), task.id});
    available.erase(best);
  }
  return assignments;
}

}  // namespace warehousesim::tasks
