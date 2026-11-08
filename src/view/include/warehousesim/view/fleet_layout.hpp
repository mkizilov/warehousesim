#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/draw_list.hpp"

namespace warehousesim::view {

// Where things go in the fleet window: the map on the left and a panel on the right with the robot
// list, an inspector for the selected robot, the task lists, a prompt, a legend and the key help.
// The scene draws from this layout and the controller finds what a click landed on with it, so the
// two cannot disagree about where a row is. It is pure arithmetic on the window size and on how
// many robots and tasks there are to list.
inline constexpr std::size_t kPanelColumns = 39;  // characters of the 8-pixel debug font per line
inline constexpr double kPanelWidth = 320.0;      // those and a little padding on each side
// Below this window width or height the panel is left out and the map takes the whole window.
inline constexpr double kPanelMinWindowWidth = 720.0;
inline constexpr double kPanelMinWindowHeight = 400.0;
inline constexpr double kPanelLineHeight = kGlyphSize + 2.0;
inline constexpr double kPanelPadding = 4.0;
inline constexpr std::size_t kMaxRobotRows = 8;
inline constexpr std::size_t kInspectorLines = 10;
inline constexpr std::size_t kPromptLines = 2;
inline constexpr std::size_t kLegendLines = 2;
inline constexpr std::size_t kHelpLines = 8;

struct FleetCounts {
  std::size_t robots = 0;
  std::size_t activeTasks = 0;
  std::size_t finishedTasks = 0;
};

struct FleetLayout {
  ScreenSize canvas;  // the part of the window the camera looks at
  ScreenRect panel;   // zero width when the window is too narrow for a panel

  // The top of each block of the panel, as a distance from the top of the window. A header is one
  // line; its rows follow it.
  double titleY = 0.0;  // two lines: the title and the playback state
  double robotsY = 0.0;
  double inspectorY = 0.0;  // then kInspectorLines lines
  double activeY = 0.0;
  double finishedY = 0.0;
  double promptY = 0.0;  // kPromptLines lines
  double legendY = 0.0;  // kLegendLines lines
  double helpY = 0.0;    // kHelpLines lines, down to the bottom of the window

  // One rectangle per row that fits, top to bottom. Row i of robotRows is robot i of the snapshot;
  // row i of activeRows is the i-th active task and row i of finishedRows the i-th finished one, as
  // the lists are ordered by taskLists().
  std::vector<ScreenRect> robotRows;
  std::vector<ScreenRect> activeRows;
  std::vector<ScreenRect> finishedRows;

  [[nodiscard]] bool hasPanel() const noexcept { return panel.width > 0.0; }
};

[[nodiscard]] FleetLayout computeLayout(ScreenSize window, FleetCounts counts);

// What a click on the panel landed on.
struct PanelHit {
  enum class Kind : std::uint8_t { Robot, ActiveTask, FinishedTask };
  Kind kind = Kind::Robot;
  std::size_t index = 0;  // the row

  bool operator==(const PanelHit&) const = default;
};

[[nodiscard]] std::optional<PanelHit> hitTest(const FleetLayout& layout, ScreenPoint at);

}  // namespace warehousesim::view
