#include "warehousesim/view/fleet_layout.hpp"

#include <algorithm>
#include <cmath>

namespace warehousesim::view {

namespace {

constexpr double kLine = kPanelLineHeight;

ScreenRect rowAt(const ScreenRect& panel, double y) { return {panel.x, y, panel.width, kLine}; }

bool contains(const ScreenRect& rect, ScreenPoint p) {
  return p.x >= rect.x && p.x < rect.right() && p.y >= rect.y && p.y < rect.bottom();
}

}  // namespace

FleetLayout computeLayout(ScreenSize window, FleetCounts counts) {
  FleetLayout layout;
  const int width = std::max(window.width, 0);
  const int height = std::max(window.height, 0);
  const bool panel = static_cast<double>(width) >= kPanelMinWindowWidth &&
                     static_cast<double>(height) >= kPanelMinWindowHeight;
  layout.canvas = {panel ? width - static_cast<int>(kPanelWidth) : width, height};
  if (!panel) {
    return layout;
  }
  layout.panel = {static_cast<double>(layout.canvas.width), 0.0, kPanelWidth,
                  static_cast<double>(height)};

  // From the top: title and state, the robots, the inspector, then the task lists take whatever
  // height is left over between that and the prompt, legend and key help pinned to the bottom.
  layout.titleY = kPanelPadding;
  layout.robotsY = layout.titleY + 3.0 * kLine;
  const std::size_t robotRows = std::min(counts.robots, kMaxRobotRows);
  for (std::size_t i = 0; i < robotRows; ++i) {
    layout.robotRows.push_back(
        rowAt(layout.panel, layout.robotsY + kLine * static_cast<double>(i + 1)));
  }
  layout.inspectorY = layout.robotsY + kLine * static_cast<double>(robotRows + 2);
  layout.activeY = layout.inspectorY + kLine * static_cast<double>(kInspectorLines + 2);

  layout.helpY =
      static_cast<double>(height) - kPanelPadding - kLine * static_cast<double>(kHelpLines);
  layout.legendY = layout.helpY - kLine * static_cast<double>(kLegendLines);
  layout.promptY = layout.legendY - kLine * static_cast<double>(kPromptLines);

  // The room for rows: between the active header and the prompt, less the finished header.
  const double room = layout.promptY - (layout.activeY + kLine) - kLine;
  const auto available = static_cast<std::size_t>(std::max(0.0, std::floor(room / kLine)));
  const std::size_t finishedWanted = std::min(counts.finishedTasks, available / 3);
  const std::size_t active = std::min(counts.activeTasks, available - finishedWanted);
  const std::size_t finished = std::min(counts.finishedTasks, available - active);

  for (std::size_t i = 0; i < active; ++i) {
    layout.activeRows.push_back(
        rowAt(layout.panel, layout.activeY + kLine * static_cast<double>(i + 1)));
  }
  layout.finishedY = layout.activeY + kLine * static_cast<double>(active + 1);
  for (std::size_t i = 0; i < finished; ++i) {
    layout.finishedRows.push_back(
        rowAt(layout.panel, layout.finishedY + kLine * static_cast<double>(i + 1)));
  }
  return layout;
}

std::optional<PanelHit> hitTest(const FleetLayout& layout, ScreenPoint at) {
  if (!std::isfinite(at.x) || !std::isfinite(at.y)) {
    return std::nullopt;
  }
  const auto find = [at](const std::vector<ScreenRect>& rows) -> std::optional<std::size_t> {
    for (std::size_t i = 0; i < rows.size(); ++i) {
      if (contains(rows[i], at)) {
        return i;
      }
    }
    return std::nullopt;
  };
  if (const auto row = find(layout.robotRows)) {
    return PanelHit{PanelHit::Kind::Robot, *row};
  }
  if (const auto row = find(layout.activeRows)) {
    return PanelHit{PanelHit::Kind::ActiveTask, *row};
  }
  if (const auto row = find(layout.finishedRows)) {
    return PanelHit{PanelHit::Kind::FinishedTask, *row};
  }
  return std::nullopt;
}

}  // namespace warehousesim::view
