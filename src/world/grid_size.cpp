#include "warehousesim/world/grid_size.hpp"

#include <stdexcept>

namespace warehousesim::world {

GridSize::GridSize(std::int32_t width, std::int32_t height) : width_(width), height_(height) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
}

}  // namespace warehousesim::world
