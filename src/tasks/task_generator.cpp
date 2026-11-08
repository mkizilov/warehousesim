#include "warehousesim/tasks/task_generator.hpp"

#include <stdexcept>
#include <utility>

namespace warehousesim::tasks {

TaskGenerator::TaskGenerator(std::vector<core::Point> pickups, std::vector<core::Point> dropoffs,
                             std::uint64_t seed)
    : pickups_(std::move(pickups)), dropoffs_(std::move(dropoffs)), rng_(seed) {
  if (pickups_.empty() || dropoffs_.empty()) {
    throw std::invalid_argument("task generator needs at least one pickup and one dropoff");
  }
}

TaskRequest TaskGenerator::next() {
  const core::Point pickup = pickups_[rng_.below(pickups_.size())];
  const core::Point dropoff = dropoffs_[rng_.below(dropoffs_.size())];
  return {pickup, dropoff};
}

}  // namespace warehousesim::tasks
