#include "warehousesim/tasks/task_generator.hpp"

#include <stdexcept>
#include <utility>

namespace warehousesim::tasks {

TaskGenerator::TaskGenerator(std::vector<core::Point> pickups, std::vector<core::Point> dropoffs,
                             std::uint64_t seed, bool varyPriority)
    : pickups_(std::move(pickups)),
      dropoffs_(std::move(dropoffs)),
      rng_(seed),
      varyPriority_(varyPriority) {
  if (pickups_.empty() || dropoffs_.empty()) {
    throw std::invalid_argument("task generator needs at least one pickup and one dropoff");
  }
}

TaskRequest TaskGenerator::next() {
  const core::Point pickup = pickups_[rng_.below(pickups_.size())];
  const core::Point dropoff = dropoffs_[rng_.below(dropoffs_.size())];
  TaskPriority priority = TaskPriority::Normal;
  if (varyPriority_) {
    const std::uint64_t roll = rng_.below(100);
    priority = roll < 15   ? TaskPriority::Low
               : roll < 75 ? TaskPriority::Normal
               : roll < 95 ? TaskPriority::High
                           : TaskPriority::Urgent;
  }
  return {pickup, dropoff, priority};
}

}  // namespace warehousesim::tasks
