#include "warehousesim/rendering/viewport.hpp"

#include <algorithm>
#include <cmath>

namespace warehousesim::rendering {

Viewport Viewport::fit(std::int32_t windowWidth, std::int32_t windowHeight, std::int32_t gridWidth,
                       std::int32_t gridHeight) noexcept {
  if (windowWidth <= 0 || windowHeight <= 0 || gridWidth <= 0 || gridHeight <= 0) {
    return {0.0F, 0.0F, 0.0F};
  }
  const float cell =
      std::floor(std::min(static_cast<float>(windowWidth) / static_cast<float>(gridWidth),
                          static_cast<float>(windowHeight) / static_cast<float>(gridHeight)));
  const float originX =
      (static_cast<float>(windowWidth) - cell * static_cast<float>(gridWidth)) / 2.0F;
  const float originY =
      (static_cast<float>(windowHeight) - cell * static_cast<float>(gridHeight)) / 2.0F;
  return {cell, originX, originY};
}

Rect Viewport::cellRect(core::Point cell) const noexcept {
  return {originX_ + cellSize_ * static_cast<float>(cell.x),
          originY_ + cellSize_ * static_cast<float>(cell.y), cellSize_, cellSize_};
}

Rect Viewport::insetCellRect(core::Point cell, float margin) const noexcept {
  const Rect full = cellRect(cell);
  const float inset = cellSize_ * margin;
  return {full.x + inset, full.y + inset, full.width - 2.0F * inset, full.height - 2.0F * inset};
}

}  // namespace warehousesim::rendering
