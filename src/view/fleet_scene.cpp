#include "warehousesim/view/fleet_scene.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/robots/robot_state.hpp"
#include "warehousesim/view/scene.hpp"
#include "warehousesim/view/scene_parts.hpp"

namespace warehousesim::view {

namespace {

using simulation::RobotSnapshot;
using tasks::Task;
using tasks::TaskStatus;

constexpr Color kHeader{150, 175, 230};
constexpr Color kSelectedRow{58, 70, 98};
constexpr Color kError{255, 120, 100};
constexpr Color kWaitingTask{240, 210, 60};
constexpr Color kClaimedTask{90, 200, 230};
constexpr Color kDoneTask{110, 200, 120};
constexpr Color kFailedTask{230, 90, 90};
constexpr Color kCancelledTask{140, 140, 150};
constexpr Color kDivider{60, 66, 82};
constexpr Color kWhite{255, 255, 255};
constexpr Color kPendingPickup{110, 230, 130};

std::string clip(std::string text) {
  if (text.size() > kPanelColumns) {
    text.resize(kPanelColumns);
  }
  return text;
}

std::string cell(core::Point p) {
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
}
std::string tightCell(core::Point p) {
  return "(" + std::to_string(p.x) + "," + std::to_string(p.y) + ")";
}

std::string percent(const RobotSnapshot& robot) {
  const auto capacity = robot.batteryCapacity == 0 ? 1 : robot.batteryCapacity;
  return std::to_string((robot.batteryLevel * 100ULL + capacity / 2) / capacity) + "%";
}

Color taskColor(const Task& task) {
  switch (task.status) {
    case TaskStatus::Pending:
      return kWaitingTask;
    case TaskStatus::Assigned:
    case TaskStatus::InProgress:
      return kClaimedTask;
    case TaskStatus::Completed:
      return kDoneTask;
    case TaskStatus::Failed:
      return kFailedTask;
    case TaskStatus::Cancelled:
      return kCancelledTask;
  }
  return kCancelledTask;
}

// Greedy word wrap into at most `lines` lines of kPanelColumns; what does not fit is cut off.
std::vector<std::string> wrap(const std::string& text, std::size_t lines) {
  std::vector<std::string> result;
  std::istringstream words(text);
  std::string word;
  std::string line;
  while (words >> word) {
    while (word.size() > kPanelColumns) {  // a word longer than a line is broken up
      if (!line.empty()) {
        result.push_back(std::move(line));
        line.clear();
      }
      result.push_back(word.substr(0, kPanelColumns));
      word.erase(0, kPanelColumns);
    }
    if (!line.empty() && line.size() + 1 + word.size() > kPanelColumns) {
      result.push_back(std::move(line));
      line.clear();
    }
    line += (line.empty() ? "" : " ") + word;
  }
  if (!line.empty()) {
    result.push_back(std::move(line));
  }
  result.resize(std::min(result.size(), lines));
  return result;
}

void put(DrawList& list, double x, double y, std::string text, Color color) {
  list.commands.emplace_back(Text{{x, y}, std::move(text), color});
}

void addRobotRows(DrawList& list, const FleetLayout& layout, const simulation::Snapshot& drawn,
                  std::optional<core::RobotId> selected, double textX) {
  for (std::size_t i = 0; i < layout.robotRows.size() && i < drawn.robots.size(); ++i) {
    const ScreenRect& row = layout.robotRows[i];
    const RobotSnapshot& robot = drawn.robots[i];
    if (robot.id == selected) {
      list.commands.emplace_back(FillRect{row, kSelectedRow});
    }
    list.commands.emplace_back(FillRect{{textX, row.y + 1.0, kGlyphSize, kGlyphSize},
                                        robotColor(robot.state, robot.goal)});
    put(list, textX + kGlyphSize + 4.0, row.y + 1.0, robotRowText(robot), kPanelText);
  }
}

void addTaskRows(DrawList& list, const std::vector<ScreenRect>& rows,
                 const std::vector<Task>& tasks, std::optional<core::TaskId> selected,
                 double textX) {
  for (std::size_t i = 0; i < rows.size() && i < tasks.size(); ++i) {
    if (tasks[i].id == selected) {
      list.commands.emplace_back(FillRect{rows[i], kSelectedRow});
    }
    put(list, textX, rows[i].y + 1.0, taskRowText(tasks[i]), taskColor(tasks[i]));
  }
}

// What a click in task entry would choose: the cell under the pointer, and the pickup chosen so
// far.
void addEntryOverlay(DrawList& list, const FleetController& controller) {
  if (controller.entry() == TaskEntry::None) {
    return;
  }
  const Camera& camera = controller.camera();
  const CellRange range = camera.visibleCells();
  if (const auto pickup = controller.pickup(); pickup && contains(range, *pickup)) {
    addDiamond(list, inset(camera.cellRect(*pickup), 0.02), kPendingPickup);
    addDiamond(list, inset(camera.cellRect(*pickup), 0.12), kPendingPickup);
  }
  if (const auto hovered = controller.hoveredCell()) {
    addOutline(list, camera.cellRect(*hovered), kWhite);
  }
}

// ---- the performance dashboard ------------------------------------------------------------------

constexpr Color kMetricsBacking{12, 14, 20};
constexpr Color kMetricsBar{110, 200, 120};
constexpr Color kMetricsAxis{90, 96, 112};

ScreenPoint metricsOrigin(const FleetController& controller) {
  return {leftRulerWidth(controller.camera().grid()) + 8.0, kRulerThickness + 8.0};
}

// What the overlay is: the dashboard text, then a chart line and the bars.
struct MetricsBox {
  std::vector<std::string> lines;
  double width = 0.0;
  double height = 0.0;
};

MetricsBox metricsBox(const metrics::FleetMetrics& metrics) {
  MetricsBox box;
  box.lines = metrics::dashboardLines(metrics);
  box.width = static_cast<double>(metrics::kDashboardColumns) * kGlyphSize + 2.0 * kMetricsPadding;
  box.height = static_cast<double>(box.lines.size() + 1) * kPanelLineHeight + kMetricsChartHeight +
               2.0 * kMetricsPadding;
  return box;
}

void addMetricsOverlay(DrawList& list, const FleetController& controller,
                       const metrics::FleetMetrics& metrics) {
  const MetricsBox box = metricsBox(metrics);
  const ScreenPoint origin = metricsOrigin(controller);
  const ScreenSize canvas = controller.layout().canvas;
  const double bottom = static_cast<double>(canvas.height) - kStatusBarHeight;
  if (origin.x + box.width > static_cast<double>(canvas.width) || origin.y + box.height > bottom) {
    constexpr std::string_view kNotice = "metrics: window too small (M closes)";
    const double width = static_cast<double>(kNotice.size()) * kGlyphSize + 2.0 * kMetricsPadding;
    if (origin.x + width <= static_cast<double>(canvas.width) &&
        origin.y + kPanelLineHeight + 2.0 * kMetricsPadding <= bottom) {
      list.commands.emplace_back(FillRect{
          {origin.x, origin.y, width, kPanelLineHeight + 2.0 * kMetricsPadding}, kMetricsBacking});
      put(list, origin.x + kMetricsPadding, origin.y + kMetricsPadding, std::string(kNotice),
          kPanelText);
    }
    return;
  }
  list.commands.emplace_back(
      FillRect{{origin.x, origin.y, box.width, box.height}, kMetricsBacking});
  double y = origin.y + kMetricsPadding;
  const double x = origin.x + kMetricsPadding;
  for (std::size_t i = 0; i < box.lines.size(); ++i) {
    const bool title = i == 0;
    const bool disclaimer = i + 1 == box.lines.size();
    put(list, x, y, box.lines[i], title ? kHeader : (disclaimer ? kError : kPanelText));
    y += kPanelLineHeight;
  }

  // One bar per sampling window: tasks completed in it.
  const auto& samples = metrics.samples;
  put(list, x, y,
      samples.empty() ? std::string("tasks done per window: no window complete yet")
                      : "tasks done per " + std::to_string(samples.back().windowTicks) +
                            "-tick window (latest " + std::to_string(samples.size()) + ")",
      kPanelText);
  y += kPanelLineHeight;
  const double baseline = y + kMetricsChartHeight;
  list.commands.emplace_back(
      Line{{x, baseline}, {x + box.width - 2.0 * kMetricsPadding, baseline}, kMetricsAxis});
  std::uint64_t most = 1;
  for (const metrics::MetricsSample& sample : samples) {
    most = std::max(most, sample.tasksCompleted);
  }
  const auto fit = static_cast<std::size_t>((box.width - 2.0 * kMetricsPadding) / kMetricsBarStep);
  const std::size_t first = samples.size() > fit ? samples.size() - fit : 0;
  for (std::size_t i = first; i < samples.size(); ++i) {
    const double height = kMetricsChartHeight * static_cast<double>(samples[i].tasksCompleted) /
                          static_cast<double>(most);
    if (height > 0.0) {
      list.commands.emplace_back(FillRect{{x + static_cast<double>(i - first) * kMetricsBarStep,
                                           baseline - height, kMetricsBarWidth, height},
                                          kMetricsBar});
    }
  }
}

void addLegend(DrawList& list, const FleetLayout& layout) {
  const double x = layout.panel.x + kPanelPadding;
  const double firstY = layout.legendY;
  const double secondY = layout.legendY + kPanelLineHeight;
  const auto square = [&](double at, double y, Color color, const char* label) {
    list.commands.emplace_back(FillRect{{at, y + 1.0, kGlyphSize, kGlyphSize}, color});
    put(list, at + kGlyphSize + 4.0, y + 1.0, label, kPanelText);
  };
  const auto diamond = [&](double at, double y, Color color, const char* label) {
    addDiamond(list, {at, y + 1.0, kGlyphSize, kGlyphSize}, color);
    put(list, at + kGlyphSize + 4.0, y + 1.0, label, kPanelText);
  };
  square(x, firstY, robotColor(robots::RobotState::Idle, robots::NavigationGoal::None), "robot");
  square(x + 96.0, firstY, cellColor(world::CellType::Wall), "wall");
  square(x + 176.0, firstY, cellColor(world::CellType::Shelf), "shelf");
  diamond(x, secondY, kPickupRing, "pickup");
  diamond(x + 96.0, secondY, kDropoffRing, "dropoff");
  list.commands.emplace_back(
      Line{{x + 176.0, secondY + 5.0},
           {x + 176.0 + kGlyphSize, secondY + 5.0},
           robotColor(robots::RobotState::Navigating, robots::NavigationGoal::Dropoff)});
  put(list, x + 176.0 + kGlyphSize + 4.0, secondY + 1.0, "route", kPanelText);
}

void addPanel(DrawList& list, const simulation::Snapshot& drawn,
              const FleetController& controller) {
  const FleetLayout& layout = controller.layout();
  if (!layout.hasPanel()) {
    return;
  }
  const double x = layout.panel.x + kPanelPadding;
  list.commands.emplace_back(FillRect{layout.panel, kPanel});
  list.commands.emplace_back(
      Line{{layout.panel.x, 0.0}, {layout.panel.x, layout.panel.bottom()}, kDivider});

  // Title and playback state.
  std::ostringstream state;
  state << "tick " << drawn.tick << "  speed " << std::setprecision(3) << controller.speed()
        << "x  " << (controller.paused() ? "PAUSED" : "RUNNING");
  put(list, x, layout.titleY, "FLEET", kHeader);
  put(list, x, layout.titleY + kPanelLineHeight, clip(state.str()),
      controller.paused() ? kError : kPanelText);

  put(list, x, layout.robotsY, clip("ROBOTS (" + std::to_string(drawn.robots.size()) + ")"),
      kHeader);
  addRobotRows(list, layout, drawn, controller.selectedRobot(), x);

  put(list, x, layout.inspectorY, "SELECTED ROBOT", kHeader);
  const auto inspector = inspectorLines(drawn, controller.selectedRobot());
  for (std::size_t i = 0; i < inspector.size(); ++i) {
    put(list, x, layout.inspectorY + kPanelLineHeight * static_cast<double>(i + 1), inspector[i],
        kPanelText);
  }

  const TaskLists& tasks = controller.tasks();
  std::string active = "ACTIVE TASKS (" + std::to_string(tasks.active.size());
  if (tasks.active.size() > layout.activeRows.size()) {
    active += ", " + std::to_string(tasks.active.size() - layout.activeRows.size()) + " not shown";
  }
  put(list, x, layout.activeY, clip(active + ")"), kHeader);
  addTaskRows(list, layout.activeRows, tasks.active, controller.selectedTask(), x);

  put(list, x, layout.finishedY,
      clip("FINISHED  done " + std::to_string(drawn.completedTasks) + "  failed " +
           std::to_string(drawn.failedTasks) + "  cancelled " +
           std::to_string(drawn.cancelledTasks)),
      kHeader);
  addTaskRows(list, layout.finishedRows, tasks.finished, controller.selectedTask(), x);

  const auto prompt = promptLines(controller);
  for (std::size_t i = 0; i < prompt.size(); ++i) {
    put(list, x, layout.promptY + kPanelLineHeight * static_cast<double>(i), prompt[i],
        controller.messageIsError() ? kError : kPanelText);
  }

  addLegend(list, layout);

  const std::array<std::string, kHelpLines> help = helpLines();
  for (std::size_t i = 0; i < help.size(); ++i) {
    put(list, x, layout.helpY + kPanelLineHeight * static_cast<double>(i), help[i], kPanelText);
  }
}

}  // namespace

std::array<std::string, kHelpLines> helpLines() {
  return {"Click a robot or row, Tab, T: select",  "N, pickup, dropoff clicks: new task",
          "1-4: priority of new / selected task",  "Space pause   [ ] speed   R restart",
          "P routes   O held cells   0 fit view",  "M metrics dashboard   E export CSV",
          "Wheel +/- zoom; right drag, WASD: pan", "Esc: cancel, clear selection, quit"};
}

std::string robotRowText(const RobotSnapshot& robot) {
  std::ostringstream out;
  out << 'R' << std::left << std::setw(3) << robot.id.value << std::setw(11)
      << robots::toString(robot.state) << std::setw(9)
      << (robot.goal == robots::NavigationGoal::None
              ? ""
              : std::string(">") + robots::toString(robot.goal))
      << std::right << std::setw(4) << percent(robot);
  return clip(out.str());
}

std::string taskRowText(const Task& task) {
  std::ostringstream out;
  out << '#' << std::left << std::setw(4) << task.id.value << std::setw(7)
      << tasks::toString(task.priority) << tightCell(task.pickup) << '>' << tightCell(task.dropoff)
      << ' ';
  const std::string robot = task.assignee ? "R" + std::to_string(task.assignee->value) + " " : "";
  switch (task.status) {
    case TaskStatus::Pending:
      out << "waiting";
      break;
    case TaskStatus::Assigned:
      out << robot << "fetching";
      break;
    case TaskStatus::InProgress:
      out << robot << "carrying";
      break;
    case TaskStatus::Completed:
      out << "done";
      break;
    case TaskStatus::Failed:
      out << "failed";
      break;
    case TaskStatus::Cancelled:
      out << "cancelled";
      break;
  }
  return clip(out.str());
}

std::array<std::string, kInspectorLines> inspectorLines(const simulation::Snapshot& snapshot,
                                                        std::optional<core::RobotId> selected) {
  std::array<std::string, kInspectorLines> lines;
  const auto found =
      std::find_if(snapshot.robots.begin(), snapshot.robots.end(),
                   [&selected](const RobotSnapshot& robot) { return robot.id == selected; });
  if (found == snapshot.robots.end()) {
    lines[0] = "click a robot on the map or in";
    lines[1] = "the list, or press Tab";
    return lines;
  }
  const RobotSnapshot& robot = *found;
  std::ostringstream speed;
  speed << "speed " << std::fixed << std::setprecision(2) << robot.speed << " cells/tick  "
        << robots::toString(robot.motion);

  lines[0] = "robot " + std::to_string(robot.id.value) + "  " + robots::toString(robot.state) +
             (robot.goal == robots::NavigationGoal::None
                  ? std::string()
                  : std::string(" > ") + robots::toString(robot.goal));
  lines[1] = "cell " + cell(robot.position) + "  facing " + core::toString(robot.heading);
  lines[2] = speed.str();
  lines[3] = "battery " + std::to_string(robot.batteryLevel) + "/" +
             std::to_string(robot.batteryCapacity) + " (" + percent(robot) + ")";

  if (robot.task) {
    const auto task = std::find_if(snapshot.activeTasks.begin(), snapshot.activeTasks.end(),
                                   [&robot](const Task& t) { return t.id == *robot.task; });
    lines[4] = "task #" + std::to_string(robot.task->value);
    if (task != snapshot.activeTasks.end()) {
      lines[4] += std::string("  ") + tasks::toString(task->priority);
      lines[5] = "from " + cell(task->pickup) + " to " + cell(task->dropoff);
    }
  } else {
    lines[4] = "no task";
  }
  lines[6] = std::string(robot.carrying ? "carrying cargo" : "no cargo") + "  deliveries " +
             std::to_string(robot.deliveries);
  if (robot.path.empty()) {
    lines[7] = "route: none";
  } else {
    lines[7] =
        "route: " + std::to_string(robot.path.size()) + " cells to " + cell(robot.path.back());
    std::string next = "next:";
    for (std::size_t i = 0; i < robot.path.size() && i < 3; ++i) {
      next += " " + tightCell(robot.path[i]);
    }
    lines[8] = next;
  }
  if (robot.waiting) {
    lines[9] = "held back at " + cell(robot.waiting->cell);
    if (robot.waiting->other) {
      lines[9] += " by R" + std::to_string(robot.waiting->other->value);
    }
  } else if (robot.stuckTicks > 0) {
    lines[9] = "not moving for " + std::to_string(robot.stuckTicks) + " ticks";
  } else {
    lines[9] = "not held back";
  }
  for (std::string& line : lines) {
    line = clip(std::move(line));
  }
  return lines;
}

std::array<std::string, kPromptLines> promptLines(const FleetController& controller) {
  std::array<std::string, kPromptLines> lines;
  const std::string priority = tasks::toString(controller.newTaskPriority());
  switch (controller.entry()) {
    case TaskEntry::Pickup:
      lines[0] = "NEW TASK (" + priority + "): click a pickup";
      lines[1] = "1-4 priority   Esc cancel";
      return lines;
    case TaskEntry::Dropoff:
      lines[0] = "NEW TASK (" + priority + "): pickup " +
                 (controller.pickup() ? cell(*controller.pickup()) : "");
      lines[1] = "click a dropoff   Esc cancel";
      return lines;
    case TaskEntry::None:
      break;
  }
  if (controller.message().empty()) {
    lines[0] = "N new task   1-4 priority";
    lines[1] = "R restart    Tab robots, T tasks";
    return lines;
  }
  const std::vector<std::string> wrapped = wrap(controller.message(), kPromptLines);
  for (std::size_t i = 0; i < wrapped.size(); ++i) {
    lines[i] = wrapped[i];
  }
  return lines;
}

ScreenRect metricsOverlayRect(const metrics::FleetMetrics& metrics,
                              const FleetController& controller) {
  const MetricsBox box = metricsBox(metrics);
  const ScreenPoint origin = metricsOrigin(controller);
  return {origin.x, origin.y, box.width, box.height};
}

DrawList buildFleetScene(const world::Warehouse& warehouse, const simulation::Snapshot& drawn,
                         const FleetController& controller, const metrics::FleetMetrics* metrics) {
  ViewStatus status{controller.paused(),
                    controller.speed(),
                    controller.pointer(),
                    controller.showPaths(),
                    controller.showOccupied(),
                    controller.selectedRobot(),
                    controller.selectedTask(),
                    true,
                    "Click select  N new task  1-4 priority  Space pause  [ ] speed  R restart  "
                    "P routes  Esc cancel"};
  DrawList list = buildScene(warehouse, drawn, controller.camera(), status);
  addEntryOverlay(list, controller);
  if (metrics != nullptr && controller.showMetrics()) {
    addMetricsOverlay(list, controller, *metrics);
  }
  addPanel(list, drawn, controller);
  return list;
}

}  // namespace warehousesim::view
