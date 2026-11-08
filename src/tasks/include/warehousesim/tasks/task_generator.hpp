#pragma once

#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/tasks/task.hpp"

namespace warehousesim::tasks {

struct TaskRequest {
  core::Point pickup;
  core::Point dropoff;
  TaskPriority priority = TaskPriority::Normal;
};

// Produces a reproducible stream of pickup/dropoff pairs from a seed. With `varyPriority` each
// request also gets a random priority (15% low, 60% normal, 20% high, 5% urgent); without it every
// request is Normal and the random stream is exactly the pickup/dropoff one, as before priorities
// existed.
class TaskGenerator {
 public:
  // Throws std::invalid_argument when either location list is empty.
  TaskGenerator(std::vector<core::Point> pickups, std::vector<core::Point> dropoffs,
                std::uint64_t seed, bool varyPriority = false);

  [[nodiscard]] TaskRequest next();

  // Where the random stream stands (see core::Rng::state): equal states mean equal tasks from here.
  [[nodiscard]] std::uint64_t rngState() const noexcept { return rng_.state(); }

 private:
  std::vector<core::Point> pickups_;
  std::vector<core::Point> dropoffs_;
  core::Rng rng_;
  bool varyPriority_;
};

}  // namespace warehousesim::tasks
