#pragma once

#include <cstdint>

#include "warehousesim/core/point.hpp"

namespace warehousesim::rendering {

struct Rect {
  float x = 0.0F;
  float y = 0.0F;
  float width = 0.0F;
  float height = 0.0F;
};

// Maps grid cells to window pixels using square cells, centred in the window.
// Pure math with no SDL dependency, so it can be unit-tested.
class Viewport {
 public:
  // cellSize is 0 when the window or grid has no area.
  [[nodiscard]] static Viewport fit(std::int32_t windowWidth, std::int32_t windowHeight,
                                    std::int32_t gridWidth, std::int32_t gridHeight) noexcept;

  [[nodiscard]] float cellSize() const noexcept { return cellSize_; }
  [[nodiscard]] Rect cellRect(core::Point cell) const noexcept;
  // Rectangle of the cell shrunk by `margin` (fraction of the cell, per side).
  [[nodiscard]] Rect insetCellRect(core::Point cell, float margin) const noexcept;

 private:
  Viewport(float cellSize, float originX, float originY) noexcept
      : cellSize_(cellSize), originX_(originX), originY_(originY) {}

  float cellSize_;
  float originX_;
  float originY_;
};

}  // namespace warehousesim::rendering
