#pragma once

#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"

namespace warehousesim::tasks {

struct TaskRequest {
  core::Point pickup;
  core::Point dropoff;
};

// Produces a reproducible stream of pickup/dropoff pairs from a seed.
class TaskGenerator {
 public:
  // Throws std::invalid_argument when either location list is empty.
  TaskGenerator(std::vector<core::Point> pickups, std::vector<core::Point> dropoffs,
                std::uint64_t seed);

  [[nodiscard]] TaskRequest next();

 private:
  std::vector<core::Point> pickups_;
  std::vector<core::Point> dropoffs_;
  core::Rng rng_;
};

}  // namespace warehousesim::tasks
