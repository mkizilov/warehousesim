#include "warehousesim/tasks/dispatcher.hpp"

#include <algorithm>
#include <utility>

namespace warehousesim::tasks {

const char* toString(RejectionReason reason) noexcept {
  switch (reason) {
    case RejectionReason::DropoffUnreachable:
      return "dropoff_unreachable";
    case RejectionReason::PickupUnreachable:
      return "pickup_unreachable";
  }
  return "unknown";
}

GreedyDispatcher::GreedyDispatcher(std::unique_ptr<DistanceEstimator> estimator)
    : estimator_(estimator ? std::move(estimator) : std::make_unique<PathDistance>()) {}

DispatchPlan GreedyDispatcher::plan(const world::Warehouse& warehouse,
                                    std::span<const robots::Robot> fleet, const TaskQueue& queue) {
  DispatchPlan result;
  std::vector<const robots::Robot*> available;
  for (const robots::Robot& robot : fleet) {
    if (robot.isAvailable()) {
      available.push_back(&robot);
    }
  }
  if (available.empty()) {
    return result;
  }

  for (const PendingKey& key : queue.pending()) {
    if (available.empty()) {
      break;
    }
    const Task& task = *queue.find(key.id);

    if (!estimator_->distance(warehouse, task.pickup, task.dropoff)) {
      result.rejections.push_back({task.id, RejectionReason::DropoffUnreachable});
      continue;
    }

    auto best = available.end();
    std::int32_t bestDistance = 0;
    for (auto it = available.begin(); it != available.end(); ++it) {
      const auto distance = estimator_->distance(warehouse, (*it)->position(), task.pickup);
      if (distance && (best == available.end() || *distance < bestDistance ||
                       (*distance == bestDistance && (*it)->id() < (*best)->id()))) {
        best = it;
        bestDistance = *distance;
      }
    }
    if (best != available.end()) {
      result.assignments.push_back({(*best)->id(), task.id, bestDistance});
      available.erase(best);
      continue;
    }

    // No available robot can get there. If no robot at all could, waiting is pointless.
    const bool anyoneCouldReach = std::ranges::any_of(fleet, [&](const robots::Robot& robot) {
      return estimator_->distance(warehouse, robot.position(), task.pickup).has_value();
    });
    if (!anyoneCouldReach) {
      result.rejections.push_back({task.id, RejectionReason::PickupUnreachable});
    }
  }
  return result;
}

}  // namespace warehousesim::tasks
