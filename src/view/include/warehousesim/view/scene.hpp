#pragma once

#include <array>
#include <optional>
#include <string>

#include "warehousesim/robots/robot_state.hpp"
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
  // Overlays: each robot's planned route (and a cross on the cell it is held back from), and the
  // cells robots hold, outlined in the robot's colour.
  bool showPaths = false;
  bool showOccupied = false;
};

// Thickness of the coordinate rulers along the top and left edges and of the status bar at the
// bottom; scene content is clipped visually by these strips.
inline constexpr double kRulerThickness = kGlyphSize + 6.0;
inline constexpr double kStatusBarHeight = 2.0 * kGlyphSize + 8.0;
// Below these zooms (pixels per cell) the grid lines, heading markers, cell letters, robot ids
// and battery gauges are omitted
// because they would only add clutter.
inline constexpr double kMinZoomForGridLines = 6.0;
inline constexpr double kMinZoomForHeading = 10.0;
inline constexpr double kMinZoomForLetters = 16.0;
inline constexpr double kMinZoomForRobotIds = 20.0;

// Fill colours for cell types and robot states (a navigating robot's colour depends on where
// it is heading).
[[nodiscard]] Color cellColor(world::CellType type);
[[nodiscard]] Color robotColor(robots::RobotState state, robots::NavigationGoal goal);

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
// and charging cells, grid lines), task markers, the route and occupied-cell overlays if they are
// on, robot placeholders, coordinate rulers and the status bar. Only cells inside the window are
// emitted.
[[nodiscard]] DrawList buildScene(const world::Warehouse& warehouse,
                                  const simulation::Snapshot& snapshot, const Camera& camera,
                                  const ViewStatus& status);

}  // namespace warehousesim::view
