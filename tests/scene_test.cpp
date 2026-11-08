#include "warehousesim/view/scene.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/world/layouts.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Direction;
using warehousesim::core::Point;
using warehousesim::robots::NavigationGoal;
using warehousesim::robots::RobotState;
using warehousesim::simulation::RobotPose;
using warehousesim::simulation::RobotSnapshot;
using warehousesim::simulation::Snapshot;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskStatus;
using warehousesim::view::Camera;
using warehousesim::view::Color;
using warehousesim::view::DrawList;
using warehousesim::view::FillRect;
using warehousesim::view::Line;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenRect;
using warehousesim::view::ScreenSize;
using warehousesim::view::Text;
using warehousesim::view::ViewStatus;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

constexpr double kEps = 1e-9;

Point P(int x, int y) { return Point{x, y}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// 3x2 grid:  # . S
//            P D C
Warehouse smallWarehouse() { return layout({"#.S", "PDC"}); }

// A 300x200 window at 40 px/cell with the 3x2 grid centred: cell (0,0) is at x 90, y 60, and the
// grid spans x 90..210, y 60..140. The rulers occupy the top and left 14 px, the status bar the
// bottom 24 px.
Camera smallCamera(double zoom = 40.0) {
  Camera camera(ScreenSize{300, 200}, warehousesim::world::GridSize(3, 2));
  camera.setZoom(zoom);
  camera.centerOn({1.5, 1.0});
  return camera;
}

std::vector<FillRect> fills(const DrawList& list) {
  std::vector<FillRect> result;
  for (const auto& command : list.commands) {
    if (const auto* fill = std::get_if<FillRect>(&command)) {
      result.push_back(*fill);
    }
  }
  return result;
}

std::vector<Line> lines(const DrawList& list) {
  std::vector<Line> result;
  for (const auto& command : list.commands) {
    if (const auto* line = std::get_if<Line>(&command)) {
      result.push_back(*line);
    }
  }
  return result;
}

std::vector<Text> texts(const DrawList& list) {
  std::vector<Text> result;
  for (const auto& command : list.commands) {
    if (const auto* text = std::get_if<Text>(&command)) {
      result.push_back(*text);
    }
  }
  return result;
}

std::optional<std::size_t> indexOfFill(const DrawList& list, const ScreenRect& rect) {
  for (std::size_t i = 0; i < list.commands.size(); ++i) {
    if (const auto* fill = std::get_if<FillRect>(&list.commands[i]); fill && fill->rect == rect) {
      return i;
    }
  }
  return std::nullopt;
}

ScreenRect inset(const ScreenRect& r, double fraction) {
  const double margin = r.width * fraction;
  return {r.x + margin, r.y + margin, r.width - 2.0 * margin, r.height - 2.0 * margin};
}

bool isNumber(const std::string& s) {
  return !s.empty() && s.find_first_not_of("0123456789") == std::string::npos;
}

DrawList sceneOf(const Warehouse& w, const Snapshot& snapshot, const Camera& camera,
                 const ViewStatus& status = {}) {
  return warehousesim::view::buildScene(w, snapshot, camera, status);
}

// ---- grid, walls, shelves, pickup/dropoff/charging --------------------------------------

TEST(SceneTest, AnEmptyWindowProducesNoDrawing) {
  const Camera camera(ScreenSize{0, 0}, warehousesim::world::GridSize(3, 2));
  EXPECT_TRUE(sceneOf(smallWarehouse(), Snapshot{}, camera).commands.empty());
}

TEST(SceneTest, FloorIsOneRectangleCoveringTheVisibleGrid) {
  const Warehouse w = smallWarehouse();
  const Camera camera = smallCamera();
  const DrawList list = sceneOf(w, Snapshot{}, camera);
  const auto* floor = std::get_if<FillRect>(&list.commands.front());
  ASSERT_TRUE(floor != nullptr);
  EXPECT_NEAR(floor->rect.x, 90.0, kEps);
  EXPECT_NEAR(floor->rect.y, 60.0, kEps);
  EXPECT_NEAR(floor->rect.width, 120.0, kEps);
  EXPECT_NEAR(floor->rect.height, 80.0, kEps);
  EXPECT_EQ(floor->color, warehousesim::view::cellColor(CellType::Free));
}

TEST(SceneTest, EveryNonFreeCellGetsItsOwnRectangleInItsOwnColour) {
  const Warehouse w = smallWarehouse();
  const Camera camera = smallCamera();
  const DrawList list = sceneOf(w, Snapshot{}, camera);
  const std::vector<std::pair<Point, CellType>> expected = {{P(0, 0), CellType::Wall},
                                                            {P(2, 0), CellType::Shelf},
                                                            {P(0, 1), CellType::Pickup},
                                                            {P(1, 1), CellType::Dropoff},
                                                            {P(2, 1), CellType::Charging}};
  for (const auto& [cell, type] : expected) {
    const auto index = indexOfFill(list, camera.cellRect(cell));
    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(std::get<FillRect>(list.commands[*index]).color, warehousesim::view::cellColor(type));
  }
  // The Free cell (1,0) has no rectangle of its own: the floor rectangle shows through.
  EXPECT_FALSE(indexOfFill(list, camera.cellRect(P(1, 0))).has_value());
}

TEST(SceneTest, AllCellTypesHaveDistinctColours) {
  std::set<std::string> seen;
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    const Color c = warehousesim::view::cellColor(type);
    seen.insert(std::to_string(c.r) + "," + std::to_string(c.g) + "," + std::to_string(c.b));
  }
  EXPECT_EQ(seen.size(), 6U);
}

TEST(SceneTest, SpecialCellsAreLabelledWithTheirLetterWhenZoomedIn) {
  const Warehouse w = smallWarehouse();
  const Camera camera = smallCamera(40.0);
  const DrawList list = sceneOf(w, Snapshot{}, camera);
  std::set<std::string> letters;
  for (const Text& text : texts(list)) {
    if (!isNumber(text.text) && text.text.size() == 1) {
      letters.insert(text.text);
    }
  }
  EXPECT_EQ(letters, (std::set<std::string>{"P", "D", "C"}));

  // The 'D' sits inside the dropoff cell.
  const ScreenRect dropoff = camera.cellRect(P(1, 1));
  for (const Text& text : texts(list)) {
    if (text.text == "D") {
      EXPECT_TRUE(text.position.x >= dropoff.x && text.position.x + 8.0 <= dropoff.right());
      EXPECT_TRUE(text.position.y >= dropoff.y && text.position.y + 8.0 <= dropoff.bottom());
    }
  }
}

TEST(SceneTest, LettersAreOmittedWhenZoomedOut) {
  const DrawList list = sceneOf(smallWarehouse(), Snapshot{}, smallCamera(10.0));
  for (const Text& text : texts(list)) {
    EXPECT_TRUE(isNumber(text.text) || text.text.size() > 1);  // only coordinates and status
  }
}

TEST(SceneTest, GridLinesFrameEveryVisibleCell) {
  const Camera camera = smallCamera(40.0);
  const DrawList list = sceneOf(smallWarehouse(), Snapshot{}, camera);
  const auto gridLines = lines(list);
  EXPECT_EQ(gridLines.size(), 7U);  // 4 vertical + 3 horizontal
  std::size_t vertical = 0;
  for (const Line& line : gridLines) {
    if (line.from.x == line.to.x) {
      ++vertical;
      EXPECT_NEAR(line.from.y, 60.0, kEps);
      EXPECT_NEAR(line.to.y, 140.0, kEps);
    } else {
      EXPECT_NEAR(line.from.x, 90.0, kEps);
      EXPECT_NEAR(line.to.x, 210.0, kEps);
    }
  }
  EXPECT_EQ(vertical, 4U);
}

TEST(SceneTest, GridLinesAreOmittedWhenFarZoomedOut) {
  EXPECT_TRUE(lines(sceneOf(smallWarehouse(), Snapshot{}, smallCamera(4.0))).empty());
}

TEST(SceneTest, CellsOutsideTheWindowAreNotEmitted) {
  Camera camera(ScreenSize{100, 100}, warehousesim::world::GridSize(3, 2));
  camera.setZoom(40.0);
  camera.centerOn({0.5, 0.5});  // sees x -0.75..1.75 -> columns 0 and 1 only
  const DrawList list = sceneOf(smallWarehouse(), Snapshot{}, camera);
  EXPECT_TRUE(indexOfFill(list, camera.cellRect(P(0, 0))).has_value());   // wall
  EXPECT_TRUE(indexOfFill(list, camera.cellRect(P(1, 1))).has_value());   // dropoff
  EXPECT_FALSE(indexOfFill(list, camera.cellRect(P(2, 0))).has_value());  // shelf, off screen
  EXPECT_FALSE(indexOfFill(list, camera.cellRect(P(2, 1))).has_value());  // charging, off screen
}

// ---- robot placeholders and tasks -------------------------------------------------------

// What a robot is doing, as the snapshot reports it.
struct Look {
  RobotState state;
  NavigationGoal goal;
};
constexpr Look kIdle{RobotState::Idle, NavigationGoal::None};
constexpr Look kToPickup{RobotState::Navigating, NavigationGoal::Pickup};
constexpr Look kToDropoff{RobotState::Navigating, NavigationGoal::Dropoff};

Snapshot snapshotWithRobot(Point at, Look look, std::uint32_t id = 7) {
  Snapshot snapshot;
  RobotSnapshot robot;
  robot.id = warehousesim::core::RobotId{id};
  robot.position = at;
  robot.state = look.state;
  robot.goal = look.goal;
  robot.batteryLevel = 4000;
  robot.batteryCapacity = 10000;
  snapshot.robots.push_back(robot);
  return snapshot;
}

TEST(SceneTest, RobotsAreColouredSquaresInsideTheirCell) {
  const Camera camera = smallCamera();
  const DrawList list = sceneOf(smallWarehouse(), snapshotWithRobot(P(1, 0), kToPickup), camera);
  const ScreenRect expected = inset(camera.cellRect(P(1, 0)), 0.15);
  const auto index = indexOfFill(list, expected);
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(std::get<FillRect>(list.commands[*index]).color,
            warehousesim::view::robotColor(kToPickup.state, kToPickup.goal));
  EXPECT_TRUE(expected.x > camera.cellRect(P(1, 0)).x);
  EXPECT_TRUE(expected.right() < camera.cellRect(P(1, 0)).right());
}

TEST(SceneTest, EveryRobotStateAndGoalHasItsOwnColour) {
  using warehousesim::view::robotColor;
  std::set<std::string> seen;
  const auto key = [](Color c) {
    return std::to_string(c.r) + "," + std::to_string(c.g) + "," + std::to_string(c.b);
  };
  seen.insert(key(robotColor(RobotState::Idle, NavigationGoal::None)));
  seen.insert(key(robotColor(RobotState::Loading, NavigationGoal::None)));
  seen.insert(key(robotColor(RobotState::Unloading, NavigationGoal::None)));
  seen.insert(key(robotColor(RobotState::Charging, NavigationGoal::None)));
  seen.insert(key(robotColor(RobotState::Unavailable, NavigationGoal::None)));
  seen.insert(key(robotColor(RobotState::Navigating, NavigationGoal::Pickup)));
  seen.insert(key(robotColor(RobotState::Navigating, NavigationGoal::Dropoff)));
  seen.insert(key(robotColor(RobotState::Navigating, NavigationGoal::Charger)));
  EXPECT_EQ(seen.size(), 8U);
}

TEST(SceneTest, ARobotShowsWhichWayItIsFacing) {
  const Camera camera = smallCamera(40.0);
  const ScreenRect body = inset(camera.cellRect(P(1, 0)), 0.15);
  struct Case {
    warehousesim::core::Direction heading;
    double nubX;
    double nubY;
  };
  const double nub = body.width * 0.25;
  const double middleX = body.x + (body.width - nub) / 2.0;
  const double middleY = body.y + (body.height - nub) / 2.0;
  const std::vector<Case> cases = {
      {warehousesim::core::Direction::North, middleX, body.y},
      {warehousesim::core::Direction::East, body.right() - nub, middleY},
      {warehousesim::core::Direction::South, middleX, body.bottom() - nub},
      {warehousesim::core::Direction::West, body.x, middleY},
  };
  for (const Case& c : cases) {
    Snapshot snapshot = snapshotWithRobot(P(1, 0), kIdle);
    snapshot.robots[0].heading = c.heading;
    const DrawList list = sceneOf(smallWarehouse(), snapshot, camera);
    EXPECT_TRUE(indexOfFill(list, ScreenRect{c.nubX, c.nubY, nub, nub}).has_value());
  }
}

TEST(SceneTest, HeadingMarkerIsOmittedWhenFarZoomedOut) {
  const Camera camera = smallCamera(8.0);
  const DrawList list = sceneOf(smallWarehouse(), snapshotWithRobot(P(1, 0), kIdle), camera);
  const ScreenRect body = inset(camera.cellRect(P(1, 0)), 0.15);
  const double nub = body.width * 0.25;
  EXPECT_FALSE(indexOfFill(list, ScreenRect{body.x + (body.width - nub) / 2.0, body.y, nub, nub})
                   .has_value());
}

TEST(SceneTest, BatteryGaugeShowsTheChargeFractionWhenThereIsRoom) {
  const Camera camera = smallCamera(40.0);
  const DrawList list = sceneOf(smallWarehouse(), snapshotWithRobot(P(1, 0), kIdle), camera);
  const ScreenRect body = inset(camera.cellRect(P(1, 0)), 0.15);
  const double barHeight = std::max(2.0, body.height * 0.12);
  const double barY = body.bottom() - barHeight;
  // 4000 / 10000 = 40% full.
  EXPECT_TRUE(indexOfFill(list, ScreenRect{body.x, barY, body.width, barHeight}).has_value());
  EXPECT_TRUE(indexOfFill(list, ScreenRect{body.x, barY, body.width * 0.4, barHeight}).has_value());

  const Camera far = smallCamera(10.0);
  const DrawList small = sceneOf(smallWarehouse(), snapshotWithRobot(P(1, 0), kIdle), far);
  const ScreenRect smallBody = inset(far.cellRect(P(1, 0)), 0.15);
  EXPECT_FALSE(
      indexOfFill(small, ScreenRect{smallBody.x, smallBody.bottom() - 2.0, smallBody.width, 2.0})
          .has_value());
}

TEST(SceneTest, RobotIdIsPrintedOnlyWhenThereIsRoom) {
  const Warehouse w = smallWarehouse();
  const Snapshot snapshot = snapshotWithRobot(P(1, 0), kIdle, 7);
  auto hasId = [&](double zoom) {
    for (const Text& text : texts(sceneOf(w, snapshot, smallCamera(zoom)))) {
      if (text.text == "7") {
        return true;
      }
    }
    return false;
  };
  EXPECT_TRUE(hasId(40.0));
  EXPECT_FALSE(hasId(10.0));
  // The square itself is still drawn when zoomed out.
  const Camera far = smallCamera(10.0);
  EXPECT_TRUE(
      indexOfFill(sceneOf(w, snapshot, far), inset(far.cellRect(P(1, 0)), 0.15)).has_value());
}

TEST(SceneTest, RobotsOutsideTheWindowAreSkipped) {
  Camera camera(ScreenSize{100, 100}, warehousesim::world::GridSize(3, 2));
  camera.setZoom(40.0);
  camera.centerOn({0.5, 0.5});
  const DrawList list = sceneOf(smallWarehouse(), snapshotWithRobot(P(2, 1), kIdle), camera);
  EXPECT_FALSE(indexOfFill(list, inset(camera.cellRect(P(2, 1)), 0.15)).has_value());
}

TEST(SceneTest, TaskMarkersShowPickupUntilCollectedAndDropoffUntilDelivered) {
  const Camera camera = smallCamera();
  const Warehouse w = smallWarehouse();
  Snapshot snapshot;
  Task task;
  task.id = warehousesim::core::TaskId{1};
  task.pickup = P(0, 1);
  task.dropoff = P(1, 1);

  task.status = TaskStatus::Pending;
  snapshot.activeTasks = {task};
  DrawList list = sceneOf(w, snapshot, camera);
  EXPECT_TRUE(indexOfFill(list, inset(camera.cellRect(P(0, 1)), 0.3)).has_value());
  EXPECT_TRUE(indexOfFill(list, inset(camera.cellRect(P(1, 1)), 0.4)).has_value());

  task.status = TaskStatus::InProgress;  // already picked up: only the dropoff is marked
  snapshot.activeTasks = {task};
  list = sceneOf(w, snapshot, camera);
  EXPECT_FALSE(indexOfFill(list, inset(camera.cellRect(P(0, 1)), 0.3)).has_value());
  EXPECT_TRUE(indexOfFill(list, inset(camera.cellRect(P(1, 1)), 0.4)).has_value());
}

TEST(SceneTest, LayersAreOrderedGridThenRobotsThenRulersThenStatusBar) {
  const Camera camera = smallCamera();
  const DrawList list = sceneOf(smallWarehouse(), snapshotWithRobot(P(1, 0), kIdle), camera);
  const auto wall = indexOfFill(list, camera.cellRect(P(0, 0)));
  const auto robot = indexOfFill(list, inset(camera.cellRect(P(1, 0)), 0.15));
  const auto topRuler = indexOfFill(list, ScreenRect{0.0, 0.0, 300.0, 14.0});
  const auto statusBar = indexOfFill(list, ScreenRect{0.0, 176.0, 300.0, 24.0});
  ASSERT_TRUE(wall && robot && topRuler && statusBar);
  EXPECT_TRUE(*wall < *robot);
  EXPECT_TRUE(*robot < *topRuler);
  EXPECT_TRUE(*topRuler < *statusBar);
}

// ---- coordinate references --------------------------------------------------------------

TEST(SceneTest, ColumnAndRowIndicesLabelTheRulers) {
  const Camera camera = smallCamera();
  const DrawList list = sceneOf(smallWarehouse(), Snapshot{}, camera);
  EXPECT_TRUE(
      indexOfFill(list, ScreenRect{0.0, 0.0, 14.0, 200.0}).has_value());  // left ruler strip

  std::vector<Text> columns;
  std::vector<Text> rows;
  for (const Text& text : texts(list)) {
    if (!isNumber(text.text)) {
      continue;
    }
    (text.position.y == 3.0 ? columns : rows).push_back(text);
  }
  ASSERT_EQ(columns.size(), 3U);
  ASSERT_EQ(rows.size(), 2U);
  for (std::size_t x = 0; x < 3; ++x) {
    EXPECT_EQ(columns[x].text, std::to_string(x));
    // Centred over the cell: cell centre minus half the glyph.
    const ScreenRect cell = camera.cellRect(P(static_cast<int>(x), 0));
    EXPECT_NEAR(columns[x].position.x, cell.x + cell.width / 2.0 - 4.0, kEps);
  }
  for (std::size_t y = 0; y < 2; ++y) {
    EXPECT_EQ(rows[y].text, std::to_string(y));
    EXPECT_NEAR(rows[y].position.x, 3.0, kEps);
    const ScreenRect cell = camera.cellRect(P(0, static_cast<int>(y)));
    EXPECT_NEAR(rows[y].position.y, cell.y + cell.height / 2.0 - 4.0, kEps);
  }
}

TEST(SceneTest, LabelStrideGrowsAsTheZoomShrinks) {
  using warehousesim::view::labelStride;
  EXPECT_EQ(labelStride(40.0, 16.0), 1);
  EXPECT_EQ(labelStride(16.0, 16.0), 1);  // exactly enough room
  EXPECT_EQ(labelStride(10.0, 16.0), 2);
  EXPECT_EQ(labelStride(5.0, 16.0), 5);
  EXPECT_EQ(labelStride(2.0, 16.0), 10);
  EXPECT_EQ(labelStride(0.5, 16.0), 50);
  EXPECT_EQ(labelStride(0.1, 16.0), 200);
  int previous = labelStride(256.0, 24.0);
  for (double zoom = 256.0; zoom >= 2.0; zoom *= 0.9) {
    const int stride = labelStride(zoom, 24.0);
    EXPECT_TRUE(stride >= previous);  // never denser when zooming out
    EXPECT_TRUE(stride * zoom >= 24.0);
    previous = stride;
  }
}

TEST(SceneTest, LabelStrideSurvivesNonsenseZoom) {
  using warehousesim::view::labelStride;
  EXPECT_EQ(labelStride(0.0, 16.0), 1);
  EXPECT_EQ(labelStride(-3.0, 16.0), 1);
  EXPECT_EQ(labelStride(std::nan(""), 16.0), 1);
  EXPECT_TRUE(labelStride(1e-12, 16.0) > 0);  // capped, no overflow or endless loop
}

TEST(SceneTest, ColumnLabelsNeverOverlapAtAnyZoom) {
  const Warehouse w(120, 6);
  for (double zoom : {2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 40.0}) {
    Camera camera(ScreenSize{900, 300}, warehousesim::world::GridSize(120, 6));
    camera.setZoom(zoom);
    camera.centerOn({60.0, 3.0});
    std::vector<Text> labels;
    for (const Text& text : texts(sceneOf(w, Snapshot{}, camera))) {
      if (isNumber(text.text) && text.position.y == 3.0) {
        labels.push_back(text);
      }
    }
    ASSERT_FALSE(labels.empty());
    for (std::size_t i = 1; i < labels.size(); ++i) {
      const double previousEnd =
          labels[i - 1].position.x + 8.0 * static_cast<double>(labels[i - 1].text.size());
      EXPECT_TRUE(labels[i].position.x > previousEnd);
    }
  }
}

TEST(SceneTest, RulerLabelsStayClearOfTheRulerStripsAndStatusBar) {
  const Warehouse w(60, 40);
  Camera camera(ScreenSize{400, 300}, warehousesim::world::GridSize(60, 40));
  camera.setZoom(14.0);
  camera.centerOn({10.0, 12.0});
  for (const Text& text : texts(sceneOf(w, Snapshot{}, camera))) {
    if (!isNumber(text.text)) {
      continue;
    }
    const double right = text.position.x + 8.0 * static_cast<double>(text.text.size());
    if (text.position.y == 3.0) {                              // column label
      EXPECT_TRUE(text.position.x >= 8.0 * 2.0 + 6.0 - kEps);  // right of the 2-digit row strip
      EXPECT_TRUE(right <= 400.0);
    } else {  // row label
      EXPECT_TRUE(text.position.y >= 14.0 - kEps);
      EXPECT_TRUE(text.position.y + 8.0 <= 300.0 - 24.0 + kEps);
    }
  }
}

// ---- status bar -------------------------------------------------------------------------

TEST(SceneTest, StatusLinesReportTickSpeedZoomAndCounts) {
  const Warehouse w = smallWarehouse();
  Snapshot snapshot = snapshotWithRobot(P(1, 0), kIdle, 3);
  snapshot.tick = 42;
  snapshot.completedTasks = 5;
  snapshot.failedTasks = 1;
  ViewStatus status;
  status.speed = 2.0;
  const auto lines = warehousesim::view::statusLines(w, snapshot, smallCamera(), status);
  EXPECT_TRUE(lines[0].find("tick 42") != std::string::npos);
  EXPECT_TRUE(lines[0].find("speed 2x") != std::string::npos);
  EXPECT_TRUE(lines[0].find("zoom 40") != std::string::npos);
  EXPECT_TRUE(lines[0].find("robots 1") != std::string::npos);
  EXPECT_TRUE(lines[0].find("5 done") != std::string::npos);
  EXPECT_TRUE(lines[0].find("1 failed") != std::string::npos);
  EXPECT_TRUE(lines[0].find("PAUSED") == std::string::npos);
  EXPECT_TRUE(lines[1].find("Space") != std::string::npos);
}

TEST(SceneTest, StatusLinesFormatFractionalSpeedsAndPause) {
  ViewStatus status;
  status.paused = true;
  status.speed = 0.25;
  const auto lines =
      warehousesim::view::statusLines(smallWarehouse(), Snapshot{}, smallCamera(), status);
  EXPECT_TRUE(lines[0].rfind("PAUSED", 0) == 0);
  EXPECT_TRUE(lines[0].find("speed 0.25x") != std::string::npos);
}

TEST(SceneTest, StatusLinesDescribeTheCellUnderThePointer) {
  const Camera camera = smallCamera();
  const Warehouse w = smallWarehouse();
  const Snapshot snapshot = snapshotWithRobot(P(1, 1), kIdle, 9);
  ViewStatus status;

  const ScreenRect shelf = camera.cellRect(P(2, 0));
  status.pointer = ScreenPoint{shelf.x + 5.0, shelf.y + 5.0};
  EXPECT_TRUE(warehousesim::view::statusLines(w, snapshot, camera, status)[0].find(
                  "cell (2,0) shelf") != std::string::npos);

  const ScreenRect dropoff = camera.cellRect(P(1, 1));
  status.pointer = ScreenPoint{dropoff.x + 5.0, dropoff.y + 5.0};
  const std::string onRobot = warehousesim::view::statusLines(w, snapshot, camera, status)[0];
  EXPECT_TRUE(onRobot.find("cell (1,1) dropoff") != std::string::npos);
  EXPECT_TRUE(onRobot.find("robot 9 idle 40%") != std::string::npos);

  const Snapshot navigating = snapshotWithRobot(P(1, 1), kToPickup, 4);
  EXPECT_TRUE(warehousesim::view::statusLines(w, navigating, camera, status)[0].find(
                  "robot 4 navigating->pickup 40%") != std::string::npos);

  status.pointer = ScreenPoint{5.0, 5.0};  // outside the grid
  EXPECT_TRUE(warehousesim::view::statusLines(w, snapshot, camera, status)[0].find("cell -") !=
              std::string::npos);
  status.pointer.reset();
  EXPECT_TRUE(warehousesim::view::statusLines(w, snapshot, camera, status)[0].find("cell -") !=
              std::string::npos);
}

TEST(SceneTest, StatusBarSitsAtTheBottomWithBothLines) {
  const DrawList list = sceneOf(smallWarehouse(), Snapshot{}, smallCamera());
  EXPECT_TRUE(indexOfFill(list, ScreenRect{0.0, 176.0, 300.0, 24.0}).has_value());
  const std::vector<Text> all = texts(list);
  ASSERT_TRUE(all.size() >= 2U);
  const Text& help = all.back();
  const Text& info = all[all.size() - 2];
  EXPECT_NEAR(info.position.x, 4.0, kEps);
  EXPECT_NEAR(info.position.y, 179.0, kEps);
  EXPECT_NEAR(help.position.y, 189.0, kEps);
  EXPECT_TRUE(help.position.y + 8.0 <= 200.0);
}

TEST(SceneTest, PausedTextUsesAWarningColour) {
  ViewStatus running;
  ViewStatus paused;
  paused.paused = true;
  const Camera camera = smallCamera();
  const auto info = [&](const ViewStatus& s) {
    const auto all = texts(sceneOf(smallWarehouse(), Snapshot{}, camera, s));
    return all[all.size() - 2];
  };
  EXPECT_NE(info(running).color, info(paused).color);
}

// ---- general properties -----------------------------------------------------------------

TEST(SceneTest, SameInputsGiveIdenticalDrawLists) {
  const Warehouse w = smallWarehouse();
  const Camera camera = smallCamera();
  const Snapshot snapshot = snapshotWithRobot(P(1, 0), kToDropoff);
  const DrawList a = sceneOf(w, snapshot, camera);
  const DrawList b = sceneOf(w, snapshot, camera);
  EXPECT_TRUE(a.commands == b.commands);
  EXPECT_EQ(a.background, b.background);
}

TEST(SceneTest, ExtremeZoomsProduceOnlyFiniteCoordinates) {
  const Warehouse w = warehousesim::world::makeFulfilmentWarehouse();
  for (double zoom : {Camera::kMinZoom, 7.0, Camera::kMaxZoom}) {
    Camera camera(ScreenSize{960, 600}, w.size());
    camera.setZoom(zoom);
    const DrawList list = sceneOf(w, Snapshot{}, camera);
    for (const FillRect& f : fills(list)) {
      EXPECT_TRUE(std::isfinite(f.rect.x) && std::isfinite(f.rect.y));
      EXPECT_TRUE(std::isfinite(f.rect.width) && std::isfinite(f.rect.height));
    }
    for (const Text& t : texts(list)) {
      EXPECT_TRUE(std::isfinite(t.position.x) && std::isfinite(t.position.y));
    }
  }
}

// ---- robots at a continuous pose, carrying cargo, part-way through a turn -----------------------

bool hasFillNear(const DrawList& list, const ScreenRect& rect) {
  for (const FillRect& f : fills(list)) {
    if (std::fabs(f.rect.x - rect.x) < 1e-9 && std::fabs(f.rect.y - rect.y) < 1e-9 &&
        std::fabs(f.rect.width - rect.width) < 1e-9 &&
        std::fabs(f.rect.height - rect.height) < 1e-9) {
      return true;
    }
  }
  return false;
}

TEST(SceneTest, ARobotCarryingCargoShowsABoxOnItsBodyAndAnEmptyOneDoesNot) {
  const Camera camera = smallCamera(40.0);
  const ScreenRect body = inset(camera.cellRect(P(1, 0)), 0.15);
  Snapshot loaded = snapshotWithRobot(P(1, 0), kToDropoff);
  loaded.robots[0].carrying = true;
  EXPECT_TRUE(indexOfFill(sceneOf(smallWarehouse(), loaded, camera), inset(body, 0.3)).has_value());
  const Snapshot empty = snapshotWithRobot(P(1, 0), kToPickup);
  EXPECT_FALSE(indexOfFill(sceneOf(smallWarehouse(), empty, camera), inset(body, 0.3)).has_value());
}

TEST(SceneTest, ARobotBetweenTwoCellsIsDrawnBetweenThemWhileItsLogicalCellStaysPut) {
  const Camera camera = smallCamera(40.0);
  const ScreenRect resting = inset(camera.cellRect(P(0, 0)), 0.15);
  Snapshot snapshot = snapshotWithRobot(P(0, 0), kToPickup);
  // Half way from the middle of cell (0, 0) to the middle of cell (1, 0).
  snapshot.robots[0].pose = RobotPose{{1.0, 0.5}, 0.0};
  const DrawList list = sceneOf(smallWarehouse(), snapshot, camera);
  ScreenRect shifted = resting;
  shifted.x += 0.5 * camera.zoom();
  EXPECT_TRUE(indexOfFill(list, shifted).has_value());
  EXPECT_FALSE(indexOfFill(list, resting).has_value());
}

TEST(SceneTest, ARobotWithNoPoseOrAPoseAtItsCellCentreIsDrawnExactlyInTheCell) {
  const Camera camera = smallCamera(40.0);
  const ScreenRect expected = inset(camera.cellRect(P(2, 1)), 0.15);
  Snapshot plain = snapshotWithRobot(P(2, 1), kIdle);
  EXPECT_TRUE(indexOfFill(sceneOf(smallWarehouse(), plain, camera), expected).has_value());
  Snapshot posed = snapshotWithRobot(P(2, 1), kIdle);
  posed.robots[0].pose = RobotPose{{2.5, 1.5}, warehousesim::core::angleOf(Direction::North)};
  EXPECT_TRUE(indexOfFill(sceneOf(smallWarehouse(), posed, camera), expected).has_value());
}

TEST(SceneTest, TheHeadingMarkerFollowsTheContinuousHeadingNotTheCardinalOne) {
  const Camera camera = smallCamera(40.0);
  const ScreenRect body = inset(camera.cellRect(P(1, 0)), 0.15);
  const double nub = body.width * 0.25;

  // At a cardinal angle the marker is exactly where the discrete heading would put it, whatever
  // the (stale) discrete heading says.
  Snapshot east = snapshotWithRobot(P(1, 0), kIdle);
  east.robots[0].heading = Direction::North;
  east.robots[0].pose = RobotPose{{1.5, 0.5}, warehousesim::core::angleOf(Direction::East)};
  EXPECT_TRUE(
      indexOfFill(sceneOf(smallWarehouse(), east, camera),
                  ScreenRect{body.right() - nub, body.y + (body.height - nub) / 2.0, nub, nub})
          .has_value());

  // Half way through a turn from north to east the marker sits on the corner between them.
  Snapshot turning = snapshotWithRobot(P(1, 0), kIdle);
  const double angle = -warehousesim::core::kPi / 4.0;
  turning.robots[0].pose = RobotPose{{1.5, 0.5}, angle};
  const DrawList list = sceneOf(smallWarehouse(), turning, camera);
  const double reach = (body.width - nub) / 2.0;
  const ScreenRect corner{body.x + reach + std::cos(angle) * reach,
                          body.y + reach + std::sin(angle) * reach, nub, nub};
  EXPECT_TRUE(hasFillNear(list, corner));
  EXPECT_LT(corner.x + nub / 2.0 - (body.x + body.width / 2.0), reach + 1e-9);
  EXPECT_GT(corner.x, body.x + reach);  // to the east of centre...
  EXPECT_LT(corner.y, body.y + reach);  // ...and to the north of it
}

TEST(SceneTest, ARobotIsOnScreenIfEitherItsCellOrTheCellUnderItsCentreIs) {
  Camera camera(ScreenSize{300, 200}, warehousesim::world::GridSize(3, 2));
  camera.setZoom(100.0);
  camera.centerOn({2.5, 1.0});  // columns 1 and 2 are on screen, column 0 is not
  const auto robotColour = warehousesim::view::robotColor(kToPickup.state, kToPickup.goal);
  const auto robotFills = [&](const DrawList& list) {
    int count = 0;
    for (const FillRect& f : fills(list)) {
      count += f.color == robotColour ? 1 : 0;
    }
    return count;
  };

  Snapshot offscreen = snapshotWithRobot(P(0, 0), kToPickup);  // resting off screen: not drawn
  EXPECT_EQ(robotFills(sceneOf(smallWarehouse(), offscreen, camera)), 0);

  Snapshot driving = snapshotWithRobot(P(0, 0), kToPickup);  // still in cell 0, but its centre has
  driving.robots[0].pose = RobotPose{{1.2, 0.5}, 0.0};       // crossed into cell 1, on screen
  EXPECT_EQ(robotFills(sceneOf(smallWarehouse(), driving, camera)), 1);
}

// Robots are drawn at their continuous pose, which is between two cells while they drive.
TEST(SceneTest, LiveSimulationSnapshotShowsEveryRobotAtItsPose) {
  auto simulation = warehousesim::simulation::makeDemoSimulation(1);
  const Camera camera(ScreenSize{960, 600}, simulation.warehouse().size());
  const auto near = [](double a, double b) { return std::fabs(a - b) < 1e-6; };
  int robotsBetweenCells = 0;
  for (int tick = 0; tick < 80; ++tick) {
    simulation.step();
    const Snapshot snapshot = simulation.snapshot();
    const DrawList list = sceneOf(simulation.warehouse(), snapshot, camera);
    for (const RobotSnapshot& robot : snapshot.robots) {
      ASSERT_TRUE(robot.pose.has_value());
      const warehousesim::view::ScreenPoint centre =
          camera.worldToScreen({robot.pose->center.x, robot.pose->center.y});
      const double side = camera.zoom() * 0.7;
      bool drawn = false;
      for (const FillRect& f : fills(list)) {
        drawn =
            drawn || (f.color == warehousesim::view::robotColor(robot.state, robot.goal) &&
                      near(f.rect.x + f.rect.width / 2.0, centre.x) &&
                      near(f.rect.y + f.rect.height / 2.0, centre.y) && near(f.rect.width, side));
      }
      EXPECT_TRUE(drawn) << "robot " << robot.id.value << " at tick " << snapshot.tick;
      robotsBetweenCells +=
          robot.center().x != robot.position.x + 0.5 || robot.center().y != robot.position.y + 0.5;
    }
  }
  EXPECT_GT(robotsBetweenCells, 20);  // the run really did catch robots in mid-move
}

}  // namespace
