#include "warehousesim/view/scene.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/vec2.hpp"
#include "warehousesim/view/scene_parts.hpp"

namespace warehousesim::view {

namespace {

using core::Point;

constexpr Color kWall{100, 100, 110};
constexpr Color kShelf{140, 100, 60};
constexpr Color kPickup{170, 140, 40};
constexpr Color kDropoff{60, 160, 90};
constexpr Color kCharging{130, 90, 190};
constexpr Color kPendingTask{240, 210, 60};
constexpr Color kActiveTask{90, 200, 230};
constexpr Color kRobotId{255, 255, 255};
constexpr Color kHeadingMarker{250, 250, 250};
constexpr Color kCargo{235, 215, 160};
constexpr Color kGaugeBackground{20, 20, 24};
constexpr Color kPausedText{255, 120, 100};
constexpr Color kHeldBack{255, 70, 70};

Color batteryColor(double fraction) {
  if (fraction < 0.25) {
    return {220, 60, 60};
  }
  return fraction < 0.5 ? Color{230, 190, 60} : Color{80, 200, 100};
}

void addTaskMarkers(DrawList& list, const simulation::Snapshot& snapshot, const Camera& camera,
                    const CellRange& range) {
  for (const tasks::Task& task : snapshot.activeTasks) {
    if (task.status != tasks::TaskStatus::InProgress && contains(range, task.pickup)) {
      const Color color = task.status == tasks::TaskStatus::Pending ? kPendingTask : kActiveTask;
      list.commands.emplace_back(FillRect{inset(camera.cellRect(task.pickup), 0.3), color});
    }
    if (contains(range, task.dropoff)) {
      list.commands.emplace_back(FillRect{inset(camera.cellRect(task.dropoff), 0.4), kActiveTask});
    }
  }
}

// The cells each robot holds (its own and those it has claimed ahead), outlined in its colour.
void addOccupiedCells(DrawList& list, const simulation::Snapshot& snapshot, const Camera& camera,
                      const CellRange& range) {
  for (const simulation::RobotSnapshot& robot : snapshot.robots) {
    const Color color = robotColor(robot.state, robot.goal);
    for (const Point cell : robot.holds) {
      if (contains(range, cell)) {
        addOutline(list, camera.cellRect(cell), color);
      }
    }
  }
}

// Each robot's remaining route as a line from where it is through the centre of every cell to the
// goal, with a marker on the goal; and a cross on the cell a held-back robot may not enter yet.
void addPaths(DrawList& list, const simulation::Snapshot& snapshot, const Camera& camera,
              const CellRange& range) {
  const auto centre = [&camera](Point cell) {
    return camera.worldToScreen(
        {static_cast<double>(cell.x) + 0.5, static_cast<double>(cell.y) + 0.5});
  };
  for (const simulation::RobotSnapshot& robot : snapshot.robots) {
    if (robot.path.empty()) {
      continue;
    }
    const Color color = robotColor(robot.state, robot.goal);
    const core::Vec2 start = robot.center();
    ScreenPoint from = camera.worldToScreen({start.x, start.y});
    Point previous = robot.position;
    for (const Point cell : robot.path) {
      const ScreenPoint to = centre(cell);
      if (contains(range, previous) || contains(range, cell)) {
        list.commands.emplace_back(Line{from, to, color});
      }
      from = to;
      previous = cell;
    }
    if (contains(range, robot.path.back())) {
      list.commands.emplace_back(FillRect{inset(camera.cellRect(robot.path.back()), 0.4), color});
    }
  }
  for (const simulation::RobotSnapshot& robot : snapshot.robots) {
    if (robot.waiting && contains(range, robot.waiting->cell)) {
      const ScreenRect r = inset(camera.cellRect(robot.waiting->cell), 0.2);
      list.commands.emplace_back(Line{{r.x, r.y}, {r.right(), r.bottom()}, kHeldBack});
      list.commands.emplace_back(Line{{r.right(), r.y}, {r.x, r.bottom()}, kHeldBack});
    }
  }
}

// Where the heading nub goes on a robot's body. A robot facing a cardinal direction has it on the
// middle of that edge; one part-way through a turn has it on the circle that passes through those
// four places.
ScreenRect headingMarker(const ScreenRect& body, double angle) {
  const double nub = body.width * 0.25;
  const double centerX = body.x + (body.width - nub) / 2.0;
  const double centerY = body.y + (body.height - nub) / 2.0;
  ScreenRect marker{centerX, centerY, nub, nub};
  const core::Direction nearest = core::nearestDirection(angle);
  if (std::fabs(core::shortestRotation(angle, core::angleOf(nearest))) <= core::kAngleEpsilon) {
    switch (nearest) {
      case core::Direction::North:
        marker.y = body.y;
        break;
      case core::Direction::East:
        marker.x = body.right() - nub;
        break;
      case core::Direction::South:
        marker.y = body.bottom() - nub;
        break;
      case core::Direction::West:
        marker.x = body.x;
        break;
    }
    return marker;
  }
  marker.x += std::cos(angle) * (body.width - nub) / 2.0;
  marker.y += std::sin(angle) * (body.height - nub) / 2.0;
  return marker;
}

// A coloured square per robot (colour = state) at its continuous pose, a nub on the side it is
// facing, a box on top while it carries cargo and, when there is room, its id and a battery
// gauge. Purely a picture of the snapshot; no behaviour lives here.
void addRobots(DrawList& list, const simulation::Snapshot& snapshot, const Camera& camera,
               const CellRange& range) {
  for (const simulation::RobotSnapshot& robot : snapshot.robots) {
    // The pose can be outside the logical cell (a robot between two cells, or an interpolated
    // one), so a robot is on screen if either its cell or the cell under its centre is.
    const core::Vec2 center = robot.center();
    const Point visualCell{static_cast<std::int32_t>(std::floor(center.x)),
                           static_cast<std::int32_t>(std::floor(center.y))};
    if (!contains(range, robot.position) && !contains(range, visualCell)) {
      continue;
    }
    // The body is the usual square in the logical cell, moved by how far the pose is from that
    // cell's centre. At rest that offset is exactly zero.
    ScreenRect rect = inset(camera.cellRect(robot.position), 0.15);
    rect.x += (center.x - (static_cast<double>(robot.position.x) + 0.5)) * camera.zoom();
    rect.y += (center.y - (static_cast<double>(robot.position.y) + 0.5)) * camera.zoom();
    list.commands.emplace_back(FillRect{rect, robotColor(robot.state, robot.goal)});

    if (robot.carrying) {
      list.commands.emplace_back(FillRect{inset(rect, 0.3), kCargo});
    }

    if (camera.zoom() >= kMinZoomForHeading) {
      list.commands.emplace_back(
          FillRect{headingMarker(rect, robot.headingAngle()), kHeadingMarker});
    }

    if (camera.zoom() >= kMinZoomForRobotIds) {
      list.commands.emplace_back(centeredText(rect, std::to_string(robot.id.value), kRobotId));
      const double barHeight = std::max(2.0, rect.height * 0.12);
      const double fraction =
          static_cast<double>(robot.batteryLevel) / static_cast<double>(robot.batteryCapacity);
      const ScreenRect gauge{rect.x, rect.bottom() - barHeight, rect.width, barHeight};
      list.commands.emplace_back(FillRect{gauge, kGaugeBackground});
      list.commands.emplace_back(FillRect{{gauge.x, gauge.y, gauge.width * fraction, gauge.height},
                                          batteryColor(fraction)});
    }
  }
}

void addStatusBar(DrawList& list, const std::array<std::string, 2>& lines, bool paused,
                  ScreenSize screen) {
  const double top = static_cast<double>(screen.height) - kStatusBarHeight;
  list.commands.emplace_back(
      FillRect{{0.0, top, static_cast<double>(screen.width), kStatusBarHeight}, kPanel});
  list.commands.emplace_back(Text{{4.0, top + 3.0}, lines[0], paused ? kPausedText : kPanelText});
  list.commands.emplace_back(Text{{4.0, top + 3.0 + kGlyphSize + 2.0}, lines[1], kPanelText});
}

}  // namespace

Color cellColor(world::CellType type) {
  switch (type) {
    case world::CellType::Free:
      return kFloor;
    case world::CellType::Wall:
      return kWall;
    case world::CellType::Shelf:
      return kShelf;
    case world::CellType::Pickup:
      return kPickup;
    case world::CellType::Dropoff:
      return kDropoff;
    case world::CellType::Charging:
      return kCharging;
  }
  return kFloor;
}

Color robotColor(robots::RobotState state, robots::NavigationGoal goal) {
  switch (state) {
    case robots::RobotState::Idle:
      return {80, 140, 240};
    case robots::RobotState::Navigating:
      switch (goal) {
        case robots::NavigationGoal::Dropoff:
          return {220, 70, 100};
        case robots::NavigationGoal::Charger:
          return {170, 210, 70};
        case robots::NavigationGoal::Pickup:
        case robots::NavigationGoal::None:
          return {240, 150, 40};
      }
      return {240, 150, 40};
    case robots::RobotState::Loading:
      return {230, 190, 70};
    case robots::RobotState::Unloading:
      return {200, 110, 220};
    case robots::RobotState::Charging:
      return {70, 200, 140};
    case robots::RobotState::Unavailable:
      return {110, 110, 115};
  }
  return kFloor;
}

int labelStride(double pixelsPerCell, double minSpacingPixels) {
  if (!std::isfinite(pixelsPerCell) || pixelsPerCell <= 0.0) {
    return 1;
  }
  // 1, 2, 5, 10, 20, 50, ... (capped well before int overflow).
  int decade = 1;
  while (decade < 100000000) {
    for (const int multiple : {1, 2, 5}) {
      const int stride = decade * multiple;
      if (stride * pixelsPerCell >= minSpacingPixels) {
        return stride;
      }
    }
    decade *= 10;
  }
  return decade;
}

std::array<std::string, 2> statusLines(const world::Warehouse& warehouse,
                                       const simulation::Snapshot& snapshot, const Camera& camera,
                                       const ViewStatus& status) {
  std::ostringstream first;
  if (status.paused) {
    first << "PAUSED  ";
  }
  first << "tick " << snapshot.tick << "  speed " << std::setprecision(3) << status.speed
        << "x  zoom " << std::fixed << std::setprecision(0) << camera.zoom() << "  robots "
        << snapshot.robots.size() << "  tasks " << snapshot.activeTasks.size() << " active "
        << snapshot.completedTasks << " done " << snapshot.failedTasks << " failed";

  first << "  cell ";
  const auto cell = status.pointer ? camera.cellAt(*status.pointer) : std::nullopt;
  if (cell) {
    first << '(' << cell->x << ',' << cell->y << ") " << world::toString(warehouse.at(*cell));
    for (const simulation::RobotSnapshot& robot : snapshot.robots) {
      if (robot.position == *cell) {
        first << " robot " << robot.id.value << ' ' << robots::toString(robot.state);
        if (robot.goal != robots::NavigationGoal::None) {
          first << "->" << robots::toString(robot.goal);
        }
        first << ' '
              << (robot.batteryLevel * 100ULL + robot.batteryCapacity / 2) / robot.batteryCapacity
              << '%';
      }
    }
  } else {
    first << '-';
  }

  return {first.str(),
          "Space pause  Wheel/+/- zoom  Drag/arrows/WASD pan  [ ] speed  0 fit view  "
          "P paths  O occupied  Esc quit"};
}

DrawList buildScene(const world::Warehouse& warehouse, const simulation::Snapshot& snapshot,
                    const Camera& camera, const ViewStatus& status) {
  DrawList list{kBackground, {}};
  const ScreenSize screen = camera.screen();
  if (screen.width <= 0 || screen.height <= 0) {
    return list;
  }

  const CellRange range = camera.visibleCells();
  if (!range.empty()) {
    addCells(list, warehouse, camera, range);
    addGridLines(list, camera, range);
    addTaskMarkers(list, snapshot, camera, range);
    if (status.showOccupied) {
      addOccupiedCells(list, snapshot, camera, range);
    }
    if (status.showPaths) {
      addPaths(list, snapshot, camera, range);
    }
    addRobots(list, snapshot, camera, range);
  }
  addRulers(list, camera, range, kStatusBarHeight);
  addStatusBar(list, statusLines(warehouse, snapshot, camera, status), status.paused, screen);
  return list;
}

}  // namespace warehousesim::view
