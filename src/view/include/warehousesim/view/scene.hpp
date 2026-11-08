#pragma once

#include <array>
#include <optional>
#include <string>

#include "warehousesim/robots/robot.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::view {

// Playback state shown in the on-screen display.
struct ViewStatus {
  bool paused = false;
  double speed = 1.0;
  std::optional<ScreenPoint> pointer;  // pointer position, if it is inside the window
};

// Thickness of the coordinate rulers along the top and left edges and of the status bar at the
// bottom; scene content is clipped visually by these strips.
inline constexpr double kRulerThickness = kGlyphSize + 6.0;
inline constexpr double kStatusBarHeight = 2.0 * kGlyphSize + 8.0;
// Below these zooms (pixels per cell) the grid lines, cell letters and robot ids are omitted
// because they would only add clutter.
inline constexpr double kMinZoomForGridLines = 6.0;
inline constexpr double kMinZoomForLetters = 16.0;
inline constexpr double kMinZoomForRobotIds = 20.0;

// Fill colours for cell types and robot states.
[[nodiscard]] Color cellColor(world::CellType type);
[[nodiscard]] Color robotColor(robots::RobotStatus status);

// Smallest label spacing, in cells, from {1, 2, 5, 10, 20, 50, ...} that puts neighbouring
// coordinate labels at least `minSpacingPixels` apart at this zoom. Returns 1 if the zoom is
// not positive and finite.
[[nodiscard]] int labelStride(double pixelsPerCell, double minSpacingPixels);

// The two status-bar lines: simulation and view state, then the key bindings.
[[nodiscard]] std::array<std::string, 2> statusLines(const world::Warehouse& warehouse,
                                                     const simulation::Snapshot& snapshot,
                                                     const Camera& camera,
                                                     const ViewStatus& status);

// Everything visible in one frame, back to front: the grid (walls, shelves, pickup, dropoff
// and charging cells, grid lines), task markers, robot placeholders, coordinate rulers and the
// status bar. Only cells inside the window are emitted.
[[nodiscard]] DrawList buildScene(const world::Warehouse& warehouse,
                                  const simulation::Snapshot& snapshot, const Camera& camera,
                                  const ViewStatus& status);

}  // namespace warehousesim::view
