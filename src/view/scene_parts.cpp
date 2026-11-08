#include "warehousesim/view/scene_parts.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include "warehousesim/view/scene.hpp"

namespace warehousesim::view {

namespace {

using core::Point;

constexpr Color kGridLine{26, 28, 35};
constexpr Color kLetter{15, 15, 15};

int digitCount(std::int32_t value) { return static_cast<int>(std::to_string(value).size()); }

}  // namespace

ScreenRect inset(const ScreenRect& rect, double fraction) {
  const double margin = rect.width * fraction;
  return {rect.x + margin, rect.y + margin, rect.width - 2.0 * margin, rect.height - 2.0 * margin};
}

bool contains(const CellRange& range, Point p) {
  return p.x >= range.left && p.x < range.right && p.y >= range.top && p.y < range.bottom;
}

Text centeredText(const ScreenRect& rect, std::string text, Color color) {
  const double width = kGlyphSize * static_cast<double>(text.size());
  return {{rect.x + (rect.width - width) / 2.0, rect.y + (rect.height - kGlyphSize) / 2.0},
          std::move(text),
          color};
}

void addCells(DrawList& list, const world::Warehouse& warehouse, const Camera& camera,
              const CellRange& range) {
  // One rectangle for all the open floor in view, then only the cells that differ from it.
  const ScreenRect first = camera.cellRect({range.left, range.top});
  const ScreenRect last = camera.cellRect({range.right - 1, range.bottom - 1});
  list.commands.emplace_back(
      FillRect{{first.x, first.y, last.right() - first.x, last.bottom() - first.y}, kFloor});

  const bool letters = camera.zoom() >= kMinZoomForLetters;
  for (std::int32_t y = range.top; y < range.bottom; ++y) {
    for (std::int32_t x = range.left; x < range.right; ++x) {
      const world::CellType type = warehouse.at({x, y});
      if (type == world::CellType::Free) {
        continue;
      }
      const ScreenRect rect = camera.cellRect({x, y});
      list.commands.emplace_back(FillRect{rect, cellColor(type)});
      const bool special = type == world::CellType::Pickup || type == world::CellType::Dropoff ||
                           type == world::CellType::Charging;
      if (letters && special) {
        list.commands.emplace_back(
            centeredText(rect, std::string(1, world::toChar(type)), kLetter));
      }
    }
  }
}

void addGridLines(DrawList& list, const Camera& camera, const CellRange& range) {
  if (camera.zoom() < kMinZoomForGridLines) {
    return;
  }
  const ScreenPoint start =
      camera.worldToScreen({static_cast<double>(range.left), static_cast<double>(range.top)});
  const ScreenPoint end =
      camera.worldToScreen({static_cast<double>(range.right), static_cast<double>(range.bottom)});
  for (std::int32_t x = range.left; x <= range.right; ++x) {
    const double sx = camera.worldToScreen({static_cast<double>(x), 0.0}).x;
    list.commands.emplace_back(Line{{sx, start.y}, {sx, end.y}, kGridLine});
  }
  for (std::int32_t y = range.top; y <= range.bottom; ++y) {
    const double sy = camera.worldToScreen({0.0, static_cast<double>(y)}).y;
    list.commands.emplace_back(Line{{start.x, sy}, {end.x, sy}, kGridLine});
  }
}

void addOutline(DrawList& list, const ScreenRect& cell, Color color) {
  const ScreenRect r = inset(cell, 0.06);
  const ScreenPoint topLeft{r.x, r.y};
  const ScreenPoint topRight{r.right(), r.y};
  const ScreenPoint bottomRight{r.right(), r.bottom()};
  const ScreenPoint bottomLeft{r.x, r.bottom()};
  list.commands.emplace_back(Line{topLeft, topRight, color});
  list.commands.emplace_back(Line{topRight, bottomRight, color});
  list.commands.emplace_back(Line{bottomRight, bottomLeft, color});
  list.commands.emplace_back(Line{bottomLeft, topLeft, color});
}

double leftRulerWidth(world::GridSize grid) {
  return digitCount(grid.height() - 1) * kGlyphSize + 6.0;
}

void addRulers(DrawList& list, const Camera& camera, const CellRange& range, double bottomMargin) {
  const ScreenSize screen = camera.screen();
  const world::GridSize grid = camera.grid();
  const double leftWidth = leftRulerWidth(grid);
  const double width = static_cast<double>(screen.width);
  const double height = static_cast<double>(screen.height);

  list.commands.emplace_back(FillRect{{0.0, 0.0, width, kRulerThickness}, kPanel});
  list.commands.emplace_back(FillRect{{0.0, 0.0, leftWidth, height}, kPanel});

  const int columnStride =
      labelStride(camera.zoom(), digitCount(grid.width() - 1) * kGlyphSize + 8.0);
  for (std::int32_t x = range.left; x < range.right; ++x) {
    if (x % columnStride != 0) {
      continue;
    }
    std::string label = std::to_string(x);
    const double labelWidth = kGlyphSize * static_cast<double>(label.size());
    const double left = camera.worldToScreen({x + 0.5, 0.0}).x - labelWidth / 2.0;
    if (left >= leftWidth && left + labelWidth <= width) {
      list.commands.emplace_back(Text{{left, 3.0}, std::move(label), kPanelText});
    }
  }

  const int rowStride = labelStride(camera.zoom(), kGlyphSize + 6.0);
  for (std::int32_t y = range.top; y < range.bottom; ++y) {
    if (y % rowStride != 0) {
      continue;
    }
    const double top = camera.worldToScreen({0.0, y + 0.5}).y - kGlyphSize / 2.0;
    if (top >= kRulerThickness && top + kGlyphSize <= height - bottomMargin) {
      list.commands.emplace_back(Text{{3.0, top}, std::to_string(y), kPanelText});
    }
  }
}

}  // namespace warehousesim::view
