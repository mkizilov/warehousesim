#pragma once

#include <cstddef>
#include <cstdint>

#include "warehousesim/core/point.hpp"

namespace warehousesim::world {

// Validated grid dimensions: both sides are at least 1.
class GridSize {
 public:
  // Throws std::invalid_argument unless width > 0 and height > 0.
  GridSize(std::int32_t width, std::int32_t height);

  [[nodiscard]] std::int32_t width() const noexcept { return width_; }
  [[nodiscard]] std::int32_t height() const noexcept { return height_; }
  [[nodiscard]] std::size_t cellCount() const noexcept {
    return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  }

  [[nodiscard]] bool contains(core::Point p) const noexcept {
    return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_;
  }

  // Row-major index of a contained point. Precondition: contains(p).
  [[nodiscard]] std::size_t indexOf(core::Point p) const noexcept {
    return static_cast<std::size_t>(p.y) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(p.x);
  }

  // Inverse of indexOf. Precondition: index < cellCount().
  [[nodiscard]] core::Point pointAt(std::size_t index) const noexcept {
    const auto width = static_cast<std::size_t>(width_);
    return {static_cast<std::int32_t>(index % width), static_cast<std::int32_t>(index / width)};
  }

  friend bool operator==(const GridSize&, const GridSize&) = default;

 private:
  std::int32_t width_;
  std::int32_t height_;
};

}  // namespace warehousesim::world
