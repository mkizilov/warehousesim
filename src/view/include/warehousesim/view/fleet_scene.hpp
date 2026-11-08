#pragma once

#include <array>
#include <optional>
#include <string>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/metrics/fleet_metrics.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/view/fleet_controller.hpp"
#include "warehousesim/view/fleet_layout.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::view {

// The words of the fleet panel, as functions of the state so that they can be tested. Every line is
// at most kPanelColumns characters.

// "R2 navigating>dropoff   82%": the robot list row.
[[nodiscard]] std::string robotRowText(const simulation::RobotSnapshot& robot);
// "#14 urgent (2,3)>(8,9) R2 carrying": a task list row.
[[nodiscard]] std::string taskRowText(const tasks::Task& task);
// Everything about the selected robot, or a hint when none is selected.
[[nodiscard]] std::array<std::string, kInspectorLines> inspectorLines(
    const simulation::Snapshot& snapshot, std::optional<core::RobotId> selected);
// The key help at the bottom of the panel.
[[nodiscard]] std::array<std::string, kHelpLines> helpLines();
// The two lines of the prompt: the task being made, else the last message, else a hint.
[[nodiscard]] std::array<std::string, kPromptLines> promptLines(const FleetController& controller);

// Everything visible in one frame of the fleet window, back to front: the map with its overlays
// (buildScene, with the selection emphasised and a ring round every destination), what a click in
// task entry would choose, the performance dashboard if there are `metrics` to show, and the panel.
// `drawn` is the snapshot to draw (usually blended).
[[nodiscard]] DrawList buildFleetScene(const world::Warehouse& warehouse,
                                       const simulation::Snapshot& drawn,
                                       const FleetController& controller,
                                       const metrics::FleetMetrics* metrics = nullptr);

// The performance dashboard over the top left of the map: metrics::dashboardLines as text, and
// under it one bar for each sampling window of the run so far (the latest that fit), as high as the
// number of tasks completed in the window. Its width and height in pixels are what it needs; if the
// map area is too small for that it shows a single line saying so instead.
inline constexpr double kMetricsPadding = 6.0;
inline constexpr double kMetricsChartHeight = 36.0;
inline constexpr double kMetricsBarWidth = 3.0;
inline constexpr double kMetricsBarStep = 4.0;
[[nodiscard]] ScreenRect metricsOverlayRect(const metrics::FleetMetrics& metrics,
                                            const FleetController& controller);

}  // namespace warehousesim::view
