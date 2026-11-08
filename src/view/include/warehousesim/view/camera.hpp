#pragma once

#include <cstdint>
#include <optional>

#include "warehousesim/core/point.hpp"
#include "warehousesim/world/grid_size.hpp"

namespace warehousesim::view {

// Distinct types for the two coordinate spaces so they cannot be mixed up by accident.
//
// World space is continuous and measured in grid cells: cell (cx, cy) covers
// [cx, cx + 1) x [cy, cy + 1), so (0, 0) is the top-left corner of the warehouse and +y
// points south. Screen space is measured in render pixels with the origin at the top-left of
// the window and +y pointing down.
struct WorldPoint {
  double x = 0.0;
  double y = 0.0;
  friend bool operator==(const WorldPoint&, const WorldPoint&) = default;
};

struct ScreenPoint {
  double x = 0.0;
  double y = 0.0;
  friend bool operator==(const ScreenPoint&, const ScreenPoint&) = default;
};

struct ScreenSize {
  int width = 0;
  int height = 0;
  friend bool operator==(const ScreenSize&, const ScreenSize&) = default;
};

struct ScreenRect {
  double x = 0.0;
  double y = 0.0;
  double width = 0.0;
  double height = 0.0;
  [[nodiscard]] double right() const noexcept { return x + width; }
  [[nodiscard]] double bottom() const noexcept { return y + height; }
  friend bool operator==(const ScreenRect&, const ScreenRect&) = default;
};

// A block of grid cells: columns [left, right) and rows [top, bottom).
struct CellRange {
  std::int32_t left = 0;
  std::int32_t top = 0;
  std::int32_t right = 0;
  std::int32_t bottom = 0;
  [[nodiscard]] bool empty() const noexcept { return left >= right || top >= bottom; }
  friend bool operator==(const CellRange&, const CellRange&) = default;
};

// A 2D camera over the warehouse grid: it maps world space to screen space and back, and
// supports panning, zooming about a point, and window resizing. It is pure arithmetic with no
// SDL or simulation dependency.
//
//   screen = (world - center) * zoom + screenCentre
//
// `zoom` is pixels per cell. The camera centre is kept inside the warehouse rectangle so the
// grid can never be panned out of reach, and zoom is kept within [kMinZoom, kMaxZoom].
// A new camera is "fitted": the whole grid is visible, centred, and resizing the window
// re-fits it. Any manual pan or zoom ends the fitted state until fit() is called again.
class Camera {
 public:
  static constexpr double kMinZoom = 2.0;
  static constexpr double kMaxZoom = 256.0;
  static constexpr double kZoomPerStep = 1.15;  // zoom factor of one wheel notch
  static constexpr double kFitMarginPixels = 24.0;

  Camera(ScreenSize screen, world::GridSize grid);

  [[nodiscard]] ScreenSize screen() const noexcept { return screen_; }
  [[nodiscard]] world::GridSize grid() const noexcept { return grid_; }
  [[nodiscard]] double zoom() const noexcept { return zoom_; }
  [[nodiscard]] WorldPoint center() const noexcept { return center_; }
  [[nodiscard]] bool fitted() const noexcept { return fitted_; }

  // Coordinate transforms. screenToWorld(worldToScreen(p)) == p up to rounding.
  [[nodiscard]] ScreenPoint worldToScreen(WorldPoint p) const noexcept;
  [[nodiscard]] WorldPoint screenToWorld(ScreenPoint p) const noexcept;

  // The pixel rectangle of a cell. Adjacent cells share edges exactly. Works for cells outside
  // the grid too.
  [[nodiscard]] ScreenRect cellRect(core::Point cell) const noexcept;
  // The grid cell under a screen position, or nullopt if that position is outside the grid.
  [[nodiscard]] std::optional<core::Point> cellAt(ScreenPoint p) const noexcept;
  // The grid cells overlapping the window, clamped to the grid (empty if none).
  [[nodiscard]] CellRange visibleCells() const noexcept;

  // Changes the window size. A fitted camera re-fits; otherwise zoom and centre are kept.
  // Negative sizes are treated as zero (e.g. a minimised window).
  void resize(ScreenSize screen);
  // Looks at another grid (the map was resized or replaced). A fitted camera re-fits; otherwise
  // zoom and centre are kept, the centre moved back inside the new grid if it fell outside.
  void setGrid(world::GridSize grid);
  // Shows the whole grid, centred, with kFitMarginPixels around it.
  void fit();
  // Moves the content by (dx, dy) pixels on screen, as when dragging the map with the mouse.
  void panByPixels(double dx, double dy);
  void centerOn(WorldPoint p);
  // Sets pixels per cell (clamped) while keeping the window centre fixed. Throws
  // std::invalid_argument for non-finite or non-positive values.
  void setZoom(double pixelsPerCell);
  // Multiplies the zoom by `factor` (> 0, finite, else std::invalid_argument) so that the world
  // point under `anchor` stays under `anchor` (unless the centre limit intervenes).
  void zoomBy(double factor, ScreenPoint anchor);
  // zoomBy(kZoomPerStep ^ steps, anchor); positive steps zoom in.
  void zoomBySteps(double steps, ScreenPoint anchor);

 private:
  [[nodiscard]] static double clampZoom(double zoom) noexcept;
  [[nodiscard]] ScreenPoint screenCenter() const noexcept;
  void constrainCenter() noexcept;

  ScreenSize screen_;
  world::GridSize grid_;
  double zoom_ = 1.0;
  WorldPoint center_;
  bool fitted_ = true;
};

}  // namespace warehousesim::view
