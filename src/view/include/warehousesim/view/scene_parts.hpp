#pragma once

#include <string>

#include "warehousesim/core/point.hpp"
#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/world/grid_size.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::view {

// The pieces of a frame that the simulation view (buildScene) and the map editor view
// (buildEditorScene) draw the same way: the grid, its rulers, and the panel colours.
inline constexpr Color kBackground{18, 20, 26};
inline constexpr Color kFloor{44, 48, 58};
inline constexpr Color kPanel{30, 33, 41};
inline constexpr Color kPanelText{200, 205, 215};

// The rectangle `fraction` of its width in from every side.
[[nodiscard]] ScreenRect inset(const ScreenRect& rect, double fraction);
[[nodiscard]] bool contains(const CellRange& range, core::Point p);
// A text command centred on a rectangle.
[[nodiscard]] Text centeredText(const ScreenRect& rect, std::string text, Color color);

// The cells in `range`: one rectangle for all the open floor, then the cells that differ from it
// (with their letter when zoomed in far enough).
void addCells(DrawList& list, const world::Warehouse& warehouse, const Camera& camera,
              const CellRange& range);
void addGridLines(DrawList& list, const Camera& camera, const CellRange& range);
// A thin outline just inside a cell's rectangle.
void addOutline(DrawList& list, const ScreenRect& cell, Color color);

// How wide the strip of row numbers along the left edge is for a grid of this height.
[[nodiscard]] double leftRulerWidth(world::GridSize grid);
// Column numbers along the top edge and row numbers along the left edge of the camera's screen.
// Row labels stop `bottomMargin` pixels above its bottom edge (room for a status bar).
void addRulers(DrawList& list, const Camera& camera, const CellRange& range, double bottomMargin);

}  // namespace warehousesim::view
