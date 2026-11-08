#include "warehousesim/view/fleet_scene.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "warehousesim/core/rng.hpp"
#include "warehousesim/view/scene.hpp"
#include "warehousesim/view/scene_parts.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::NavigationGoal;
using warehousesim::robots::RobotState;
using warehousesim::simulation::RobotSnapshot;
using warehousesim::simulation::RobotWait;
using warehousesim::simulation::Snapshot;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskPriority;
using warehousesim::tasks::TaskStatus;
using warehousesim::view::buildFleetScene;
using warehousesim::view::buildScene;
using warehousesim::view::Camera;
using warehousesim::view::Color;
using warehousesim::view::DrawList;
using warehousesim::view::FillRect;
using warehousesim::view::FleetController;
using warehousesim::view::FleetKey;
using warehousesim::view::InputEvents;
using warehousesim::view::kGlyphSize;
using warehousesim::view::kPanelColumns;
using warehousesim::view::Line;
using warehousesim::view::PointerAction;
using warehousesim::view::PointerButton;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenRect;
using warehousesim::view::ScreenSize;
using warehousesim::view::Text;
using warehousesim::view::ViewStatus;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

constexpr double kEps = 1e-6;
constexpr ScreenSize kWindow{1280, 720};

Point P(int x, int y) { return Point{x, y}; }
bool near(double a, double b) { return std::fabs(a - b) < kEps; }

Warehouse hall() {
  return *Warehouse::fromRows(
      {"..........", "..#.......", "..S.......", "..........", "..........", ".........."});
}

RobotSnapshot robot(std::uint32_t id, Point at, RobotState state = RobotState::Idle) {
  RobotSnapshot r;
  r.id = RobotId{id};
  r.position = at;
  r.state = state;
  r.batteryLevel = 7000;
  r.batteryCapacity = 10000;
  return r;
}

Task task(std::uint32_t id, TaskStatus status, Point pickup = P(0, 0), Point dropoff = P(9, 5),
          TaskPriority priority = TaskPriority::Normal) {
  Task t;
  t.id = TaskId{id};
  t.pickup = pickup;
  t.dropoff = dropoff;
  t.status = status;
  t.priority = priority;
  return t;
}

Snapshot world() {
  Snapshot s;
  s.tick = 120;
  s.robots = {robot(1, P(0, 3)), robot(2, P(4, 3), RobotState::Navigating),
              robot(3, P(8, 3), RobotState::Charging)};
  s.robots[1].goal = NavigationGoal::Dropoff;
  s.robots[1].task = TaskId{2};
  s.robots[1].path = {P(5, 3), P(6, 3), P(7, 3)};
  s.activeTasks = {task(1, TaskStatus::Pending, P(0, 0), P(9, 5), TaskPriority::Low),
                   task(2, TaskStatus::InProgress, P(1, 0), P(7, 3), TaskPriority::High),
                   task(3, TaskStatus::Pending, P(6, 0), P(9, 0), TaskPriority::Urgent)};
  s.recentTasks = {task(5, TaskStatus::Failed), task(4, TaskStatus::Completed)};
  s.completedTasks = 9;
  s.failedTasks = 2;
  s.cancelledTasks = 1;
  return s;
}

struct Rig {
  FleetController controller{kWindow, hall()};
  Snapshot snapshot = world();

  Rig() { (void)frame({}); }
  void frame(const InputEvents& input) { (void)controller.apply(input, 0.016, snapshot, 0); }
  void key(FleetKey k) {
    InputEvents input;
    input.fleetKeys.push_back(k);
    frame(input);
  }
  ScreenPoint cellCentre(Point cell) const {
    const ScreenRect r = controller.camera().cellRect(cell);
    return {r.x + r.width / 2.0, r.y + r.height / 2.0};
  }
  void clickAt(ScreenPoint at) {
    InputEvents input;
    input.pointerActions = {{PointerAction::Kind::Press, PointerButton::Primary, at},
                            {PointerAction::Kind::Release, PointerButton::Primary, at}};
    frame(input);
  }
  DrawList scene() const { return buildFleetScene(hall(), snapshot, controller); }
};

std::vector<const Text*> texts(const DrawList& list) {
  std::vector<const Text*> found;
  for (const auto& command : list.commands) {
    if (const auto* text = std::get_if<Text>(&command)) {
      found.push_back(text);
    }
  }
  return found;
}

const Text* findText(const DrawList& list, const std::string& part) {
  for (const Text* text : texts(list)) {
    if (text->text.find(part) != std::string::npos) {
      return text;
    }
  }
  return nullptr;
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

using warehousesim::view::inset;

bool hasFill(const DrawList& list, const ScreenRect& rect,
             std::optional<Color> color = std::nullopt) {
  return std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
    const auto* fill = std::get_if<FillRect>(&command);
    return fill != nullptr && near(fill->rect.x, rect.x) && near(fill->rect.y, rect.y) &&
           near(fill->rect.width, rect.width) && near(fill->rect.height, rect.height) &&
           (!color || fill->color == *color);
  });
}

// A diamond is four lines joining the middles of the sides of the rectangle.
std::size_t diamonds(const DrawList& list, const ScreenRect& r,
                     std::optional<Color> color = std::nullopt) {
  const ScreenPoint top{r.x + r.width / 2.0, r.y};
  const ScreenPoint right{r.right(), r.y + r.height / 2.0};
  const ScreenPoint bottom{r.x + r.width / 2.0, r.bottom()};
  const ScreenPoint left{r.x, r.y + r.height / 2.0};
  const auto has = [&](ScreenPoint a, ScreenPoint b) {
    return std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
      const auto* line = std::get_if<Line>(&command);
      return line != nullptr && near(line->from.x, a.x) && near(line->from.y, a.y) &&
             near(line->to.x, b.x) && near(line->to.y, b.y) && (!color || line->color == *color);
    });
  };
  return has(top, right) && has(right, bottom) && has(bottom, left) && has(left, top) ? 1U : 0U;
}

// The white outline of a cell, as the entry overlay draws it.
bool hasWhiteOutline(const DrawList& list, const ScreenRect& cell) {
  const double margin = cell.width * 0.06;
  return std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
    const auto* line = std::get_if<Line>(&command);
    return line != nullptr && line->color == Color{255, 255, 255} &&
           near(line->from.x, cell.x + margin) && near(line->from.y, cell.y + margin) &&
           near(line->to.x, cell.right() - margin) && near(line->to.y, cell.y + margin);
  });
}

ViewStatus status(std::optional<RobotId> robotId = std::nullopt,
                  std::optional<TaskId> taskId = std::nullopt, bool rings = false,
                  bool paths = false) {
  ViewStatus s;
  s.showPaths = paths;
  s.selectedRobot = robotId;
  s.selectedTask = taskId;
  s.destinationRings = rings;
  return s;
}

Camera camera() { return Camera(ScreenSize{800, 600}, warehousesim::world::GridSize(10, 6)); }

// ---- the words of the panel ---------------------------------------------------------------------

TEST(FleetTextTest, ARobotRowSaysWhoWhatAndHowCharged) {
  RobotSnapshot r = robot(2, P(4, 3), RobotState::Navigating);
  r.goal = NavigationGoal::Dropoff;
  r.batteryLevel = 8240;
  EXPECT_EQ(warehousesim::view::robotRowText(r), "R2  navigating >dropoff  82%");
  r.batteryLevel = 8250;  // 82.5 rounds up
  EXPECT_EQ(warehousesim::view::robotRowText(r), "R2  navigating >dropoff  83%");
  r.batteryLevel = 8249;
  EXPECT_EQ(warehousesim::view::robotRowText(r), "R2  navigating >dropoff  82%");
  r.batteryLevel = 8240;
  RobotSnapshot idle = robot(12, P(0, 0));
  idle.batteryLevel = 10000;
  EXPECT_EQ(warehousesim::view::robotRowText(idle),
            std::string("R12 idle") + std::string(16, ' ') + "100%");
}

TEST(FleetTextTest, ATaskRowSaysNumberPriorityCellsAndWhoHasIt) {
  Task t = task(14, TaskStatus::Pending, P(2, 3), P(8, 9), TaskPriority::Urgent);
  EXPECT_EQ(warehousesim::view::taskRowText(t), "#14  urgent (2,3)>(8,9) waiting");
  t.status = TaskStatus::Assigned;
  t.assignee = RobotId{2};
  EXPECT_EQ(warehousesim::view::taskRowText(t), "#14  urgent (2,3)>(8,9) R2 fetching");
  t.status = TaskStatus::InProgress;
  EXPECT_EQ(warehousesim::view::taskRowText(t), "#14  urgent (2,3)>(8,9) R2 carrying");
  t.status = TaskStatus::Completed;
  EXPECT_TRUE(contains(warehousesim::view::taskRowText(t), "done"));
  t.status = TaskStatus::Failed;
  EXPECT_TRUE(contains(warehousesim::view::taskRowText(t), "failed"));
  t.status = TaskStatus::Cancelled;
  EXPECT_TRUE(contains(warehousesim::view::taskRowText(t), "cancelled"));
}

TEST(FleetTextTest, NoRowIsLongerThanThePanelHoldsEvenWithHugeNumbers) {
  RobotSnapshot r = robot(4000000000U, P(2000000000, 2000000000), RobotState::Unavailable);
  r.goal = NavigationGoal::Charger;
  r.batteryLevel = 4000000000U;
  r.batteryCapacity = 1;
  EXPECT_LE(warehousesim::view::robotRowText(r).size(), kPanelColumns);
  Task t = task(4000000000U, TaskStatus::InProgress, P(2000000000, 2000000000),
                P(2000000000, 2000000000), TaskPriority::Urgent);
  t.assignee = RobotId{4000000000U};
  EXPECT_LE(warehousesim::view::taskRowText(t).size(), kPanelColumns);
}

TEST(FleetTextTest, ARobotWithAZeroCapacityBatteryDoesNotDivideByZero) {
  RobotSnapshot r = robot(1, P(0, 0));
  r.batteryCapacity = 0;
  r.batteryLevel = 0;
  EXPECT_FALSE(warehousesim::view::robotRowText(r).empty());
  Snapshot s;
  s.robots = {r};
  EXPECT_FALSE(warehousesim::view::inspectorLines(s, RobotId{1})[3].empty());
}

TEST(FleetInspectorTest, WithNothingSelectedItSaysHowToSelect) {
  const auto lines = warehousesim::view::inspectorLines(world(), std::nullopt);
  EXPECT_TRUE(contains(lines[0], "click a robot"));
  for (std::size_t i = 2; i < lines.size(); ++i) {
    EXPECT_TRUE(lines[i].empty());
  }
  EXPECT_EQ(warehousesim::view::inspectorLines(world(), RobotId{99})[0],
            lines[0]);  // gone: same hint
}

TEST(FleetInspectorTest, ARobotOnATaskShowsItsStateCellBatteryTaskAndRoute) {
  Snapshot s = world();
  s.robots[1].heading = warehousesim::core::Direction::East;
  s.robots[1].speed = 0.75;
  s.robots[1].motion = warehousesim::robots::MotionPhase::Cruising;
  s.robots[1].carrying = true;
  s.robots[1].deliveries = 6;
  const auto lines = warehousesim::view::inspectorLines(s, RobotId{2});
  EXPECT_EQ(lines[0], "robot 2  navigating > dropoff");
  EXPECT_TRUE(contains(lines[1], "cell (4, 3)")) << lines[1];
  EXPECT_TRUE(contains(lines[1], "east")) << lines[1];
  EXPECT_TRUE(contains(lines[2], "0.75")) << lines[2];
  EXPECT_TRUE(contains(lines[2], "cruising")) << lines[2];
  EXPECT_EQ(lines[3], "battery 7000/10000 (70%)");
  EXPECT_EQ(lines[4], "task #2  high");
  EXPECT_EQ(lines[5], "from (1, 0) to (7, 3)");
  EXPECT_EQ(lines[6], "carrying cargo  deliveries 6");
  EXPECT_EQ(lines[7], "route: 3 cells to (7, 3)");
  EXPECT_EQ(lines[8], "next: (5,3) (6,3) (7,3)");
  EXPECT_EQ(lines[9], "not held back");
}

TEST(FleetInspectorTest, AnIdleRobotHasNoTaskNoRouteAndNoCargo) {
  const auto lines = warehousesim::view::inspectorLines(world(), RobotId{1});
  EXPECT_EQ(lines[0], "robot 1  idle");
  EXPECT_EQ(lines[4], "no task");
  EXPECT_TRUE(lines[5].empty());
  EXPECT_TRUE(contains(lines[6], "no cargo"));
  EXPECT_EQ(lines[7], "route: none");
  EXPECT_TRUE(lines[8].empty());
}

TEST(FleetInspectorTest, AHeldBackRobotSaysWhereAndByWhomAndAStuckOneForHowLong) {
  Snapshot s = world();
  s.robots[1].waiting = RobotWait{P(5, 3), warehousesim::traffic::ConflictKind::Vertex, RobotId{3}};
  EXPECT_EQ(warehousesim::view::inspectorLines(s, RobotId{2})[9], "held back at (5, 3) by R3");
  s.robots[1].waiting =
      RobotWait{P(5, 3), warehousesim::traffic::ConflictKind::Blocked, std::nullopt};
  EXPECT_EQ(warehousesim::view::inspectorLines(s, RobotId{2})[9], "held back at (5, 3)");
  s.robots[1].waiting.reset();
  s.robots[1].stuckTicks = 17;
  EXPECT_EQ(warehousesim::view::inspectorLines(s, RobotId{2})[9], "not moving for 17 ticks");
}

TEST(FleetInspectorTest, ATaskThatIsNoLongerActiveStillShowsItsNumber) {
  Snapshot s = world();
  s.activeTasks.clear();
  const auto lines = warehousesim::view::inspectorLines(s, RobotId{2});
  EXPECT_EQ(lines[4], "task #2");
  EXPECT_TRUE(lines[5].empty());
}

TEST(FleetInspectorTest, NoLineIsLongerThanThePanelHolds) {
  Snapshot s = world();
  s.robots[1].id = RobotId{4000000000U};
  s.robots[1].position = P(2000000000, 2000000000);
  s.robots[1].speed = 123456.789;
  s.robots[1].batteryLevel = 4000000000ULL;
  s.robots[1].batteryCapacity = 4000000000ULL;
  s.robots[1].deliveries = 18000000000000000000ULL;
  s.robots[1].task = TaskId{4000000000U};
  s.robots[1].waiting = RobotWait{P(2000000000, 2000000000),
                                  warehousesim::traffic::ConflictKind::Swap, RobotId{4000000000U}};
  s.robots[1].path = std::vector<Point>(500, P(2000000000, 2000000000));
  s.activeTasks.push_back(task(4000000000U, TaskStatus::InProgress, P(2000000000, 2000000000),
                               P(2000000000, 2000000000), TaskPriority::Urgent));
  for (const std::string& line : warehousesim::view::inspectorLines(s, RobotId{4000000000U})) {
    EXPECT_LE(line.size(), kPanelColumns) << line;
  }
}

TEST(FleetTextTest, EveryHelpLineFitsThePanelAndTheHelpMentionsEveryWayOfDrivingTheFleet) {
  std::string all;
  for (const std::string& line : warehousesim::view::helpLines()) {
    EXPECT_FALSE(line.empty());
    EXPECT_LE(line.size(), kPanelColumns) << line;
    all += line + "\n";
  }
  for (const char* part : {"select", "Tab", "new task", "priority", "pause", "speed", "restart",
                           "routes", "zoom", "pan", "Esc"}) {
    EXPECT_TRUE(contains(all, part)) << part;
  }
}

TEST(FleetPromptTest, TheLinesFollowTaskEntryThenTheMessageThenAHint) {
  Rig rig;
  auto lines = warehousesim::view::promptLines(rig.controller);
  EXPECT_TRUE(contains(lines[0], "N new task"));

  rig.key(FleetKey::NewTask);
  lines = warehousesim::view::promptLines(rig.controller);
  EXPECT_EQ(lines[0], "NEW TASK (normal): click a pickup");
  rig.key(FleetKey::Priority4);
  EXPECT_EQ(warehousesim::view::promptLines(rig.controller)[0],
            "NEW TASK (urgent): click a pickup");
  rig.clickAt(rig.cellCentre(P(6, 0)));
  lines = warehousesim::view::promptLines(rig.controller);
  EXPECT_EQ(lines[0], "NEW TASK (urgent): pickup (6, 0)");
  EXPECT_TRUE(contains(lines[1], "click a dropoff"));

  rig.key(FleetKey::Cancel);
  lines = warehousesim::view::promptLines(rig.controller);
  EXPECT_TRUE(contains(lines[0], "new task cancelled")) << lines[0];
}

TEST(FleetPromptTest, ALongMessageWrapsAtWordsIntoTwoLinesAndIsCutAfterThat) {
  Rig rig;
  rig.controller.report(
      "task #1 is assigned; only a task that is still waiting can change priority and more words "
      "follow here",
      true);
  const auto lines = warehousesim::view::promptLines(rig.controller);
  EXPECT_LE(lines[0].size(), kPanelColumns);
  EXPECT_LE(lines[1].size(), kPanelColumns);
  EXPECT_EQ(lines[0], "task #1 is assigned; only a task that");
  EXPECT_FALSE(lines[1].empty());
  EXPECT_TRUE(contains(lines[0] + " " + lines[1], "still waiting"));

  // A message that fills a line exactly stays on it.
  rig.controller.report(std::string(19, 'a') + " " + std::string(19, 'b'), false);
  const auto exact = warehousesim::view::promptLines(rig.controller);
  EXPECT_EQ(exact[0].size(), kPanelColumns);
  EXPECT_TRUE(exact[1].empty());

  rig.controller.report(std::string(120, 'x'), true);  // one word longer than a line
  const auto cut = warehousesim::view::promptLines(rig.controller);
  EXPECT_EQ(cut[0].size(), kPanelColumns);
  EXPECT_EQ(cut[1].size(), kPanelColumns);
}

// ---- the whole window ---------------------------------------------------------------------------

TEST(FleetSceneTest, ThePanelListsRobotsTheSelectedRobotActiveAndFinishedTasks) {
  Rig rig;
  const DrawList list = rig.scene();
  for (const char* part : {"FLEET", "tick 120", "RUNNING", "ROBOTS (3)", "SELECTED ROBOT",
                           "ACTIVE TASKS (3)", "FINISHED", "done 9", "failed 2", "cancelled 1"}) {
    EXPECT_TRUE(findText(list, part) != nullptr) << part;
  }
  // One text row for every robot and every task.
  for (const char* row : {"R1  idle", "R2  navigating", "R3  charging", "#3   urgent", "#2   high",
                          "#1   low", "#5   normal", "#4   normal"}) {
    EXPECT_TRUE(findText(list, row) != nullptr) << row;
  }
}

TEST(FleetSceneTest, AMoreThanFullTaskListSaysHowManyAreNotShown) {
  Rig rig;
  rig.snapshot.activeTasks.clear();
  for (std::uint32_t i = 1; i <= 80; ++i) {
    rig.snapshot.activeTasks.push_back(task(i, TaskStatus::Pending));
  }
  rig.frame({});
  const DrawList list = rig.scene();
  EXPECT_TRUE(findText(list, "ACTIVE TASKS (80, ") != nullptr);
  EXPECT_TRUE(findText(list, " not shown)") != nullptr);
}

TEST(FleetSceneTest, PlaybackStateShowsPausedAndTheSpeed) {
  Rig rig;
  InputEvents pause;
  pause.togglePause = true;
  pause.speedSteps = 2;
  rig.frame(pause);
  const DrawList list = rig.scene();
  const Text* state = findText(list, "PAUSED");
  ASSERT_TRUE(state != nullptr);
  EXPECT_TRUE(contains(state->text, "speed 4x")) << state->text;
  const Text* running = findText(rig.scene(), "RUNNING");
  EXPECT_TRUE(running == nullptr);
}

TEST(FleetSceneTest, ASelectedRowIsHighlightedAndTheOthersAreNot) {
  Rig rig;
  rig.clickAt({rig.controller.layout().robotRows[1].x + 30.0,
               rig.controller.layout().robotRows[1].y + 4.0});
  ASSERT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{2}));
  DrawList list = rig.scene();
  const auto& rows = rig.controller.layout().robotRows;
  const auto highlight = [&list](const ScreenRect& row) {
    return std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
      const auto* fill = std::get_if<FillRect>(&command);
      return fill != nullptr && near(fill->rect.y, row.y) && near(fill->rect.x, row.x) &&
             near(fill->rect.width, row.width) && near(fill->rect.height, row.height);
    });
  };
  EXPECT_FALSE(highlight(rows[0]));
  EXPECT_TRUE(highlight(rows[1]));
  EXPECT_FALSE(highlight(rows[2]));

  rig.clickAt({rig.controller.layout().activeRows[0].x + 30.0,
               rig.controller.layout().activeRows[0].y + 4.0});
  list = rig.scene();
  EXPECT_TRUE(highlight(rig.controller.layout().activeRows[0]));
  EXPECT_FALSE(highlight(rig.controller.layout().activeRows[1]));
}

TEST(FleetSceneTest, TaskRowsAreColouredByStatus) {
  Rig rig;
  const DrawList list = rig.scene();
  const Text* waiting = findText(list, "waiting");
  const Text* carrying = findText(list, "carrying");
  const Text* done = findText(list, "(9,5) done");
  const Text* failed = findText(list, "(9,5) failed");
  ASSERT_TRUE(waiting && carrying && done && failed);
  const std::set<std::tuple<int, int, int>> colours{
      {waiting->color.r, waiting->color.g, waiting->color.b},
      {carrying->color.r, carrying->color.g, carrying->color.b},
      {done->color.r, done->color.g, done->color.b},
      {failed->color.r, failed->color.g, failed->color.b}};
  EXPECT_EQ(colours.size(), 4U);
}

TEST(FleetSceneTest, NoTextRunsPastTheRightEdgeOfTheWindowOrOutOfThePanel) {
  Rig rig;
  rig.snapshot.robots[1].path = std::vector<Point>(30, P(5, 3));
  rig.key(FleetKey::NextRobot);
  rig.controller.report(
      "a rather long message with many words that must wrap inside the panel and not run off",
      false);
  for (const ScreenSize size :
       {ScreenSize{1280, 720}, ScreenSize{720, 400}, ScreenSize{1000, 900}}) {
    rig.controller.resize(size);
    rig.frame({});
    const DrawList list = rig.scene();
    for (const Text* text : texts(list)) {
      const double right = text->position.x + kGlyphSize * static_cast<double>(text->text.size());
      if (text->position.x >= rig.controller.layout().panel.x) {
        EXPECT_LE(right, static_cast<double>(size.width)) << text->text;
      }
    }
  }
}

TEST(FleetSceneTest, NoPanelTextInAWindowTooSmallForThePanel) {
  FleetController c(ScreenSize{600, 500}, hall());
  Snapshot s = world();
  (void)c.apply({}, 0.016, s, 0);
  const DrawList list = buildFleetScene(hall(), s, c);
  EXPECT_TRUE(findText(list, "FLEET") == nullptr);
  EXPECT_TRUE(findText(list, "ACTIVE TASKS") == nullptr);
  // But the map and its status bar are there.
  EXPECT_TRUE(findText(list, "tick 120") != nullptr);
}

TEST(FleetSceneTest, ThePanelIsDrawnOverTheMapAndAfterIt) {
  Rig rig;
  const DrawList list = rig.scene();
  const ScreenRect panel = rig.controller.layout().panel;
  std::size_t panelFill = list.commands.size();
  for (std::size_t i = 0; i < list.commands.size(); ++i) {
    const auto* fill = std::get_if<FillRect>(&list.commands[i]);
    if (fill != nullptr && near(fill->rect.x, panel.x) && near(fill->rect.width, panel.width) &&
        near(fill->rect.height, panel.height)) {
      panelFill = i;
    }
  }
  ASSERT_LT(panelFill, list.commands.size());
  const std::size_t robotBody = [&] {
    const ScreenRect body = inset(rig.controller.camera().cellRect(P(4, 3)), 0.15);
    for (std::size_t i = 0; i < list.commands.size(); ++i) {
      const auto* fill = std::get_if<FillRect>(&list.commands[i]);
      if (fill != nullptr && near(fill->rect.x, body.x) && near(fill->rect.width, body.width)) {
        return i;
      }
    }
    return list.commands.size();
  }();
  ASSERT_LT(robotBody, list.commands.size());
  EXPECT_LT(robotBody, panelFill);
}

TEST(FleetSceneTest, ASceneIsAPureFunctionOfTheState) {
  Rig rig;
  rig.clickAt(rig.cellCentre(P(4, 3)));
  EXPECT_TRUE(rig.scene().commands == rig.scene().commands);
}

TEST(FleetSceneTest, TaskEntryMarksThePickupChosenAndTheCellUnderThePointer) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  rig.clickAt(rig.cellCentre(P(6, 5)));
  InputEvents hover;
  hover.pointer = rig.cellCentre(P(7, 5));
  rig.frame(hover);
  const DrawList list = rig.scene();
  const Camera& cam = rig.controller.camera();
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(6, 5)), 0.02)), 1U);
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(6, 5)), 0.12)), 1U);
  EXPECT_TRUE(hasWhiteOutline(list, cam.cellRect(P(7, 5))));  // the cell a click would choose
  EXPECT_FALSE(hasWhiteOutline(list, cam.cellRect(P(6, 5))));
  // And with no entry under way the extra marks are gone, though the pointer has not moved.
  rig.key(FleetKey::Cancel);
  const DrawList after = rig.scene();
  EXPECT_EQ(diamonds(after, inset(cam.cellRect(P(6, 5)), 0.02)), 0U);
  EXPECT_FALSE(hasWhiteOutline(after, cam.cellRect(P(7, 5))));
  EXPECT_TRUE(rig.controller.hoveredCell().has_value());
}

// ---- telling the things apart -------------------------------------------------------------------

TEST(FleetLooksTest,
     RobotsAreSquaresDestinationsAreDiamondsObstaclesAreOtherColoursAndPathsAreLines) {
  const Camera cam = camera();
  Snapshot s = world();
  s.robots[1].path = {P(5, 3), P(6, 3), P(7, 3)};
  s.robots[1].state = RobotState::Navigating;
  const DrawList list = buildScene(hall(), s, cam, status(std::nullopt, std::nullopt, true, true));

  // A robot: a filled square in its colour.
  const ScreenRect body = inset(cam.cellRect(P(4, 3)), 0.15);
  EXPECT_TRUE(hasFill(
      list, body, warehousesim::view::robotColor(RobotState::Navigating, NavigationGoal::Dropoff)));
  // A destination: a diamond of lines, and not a filled square.
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(9, 5)), 0.04), warehousesim::view::kDropoffRing),
            1U);
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(0, 0)), 0.04), warehousesim::view::kPickupRing),
            1U);
  // An obstacle: a filled cell in the wall or shelf colour, with no diamond.
  EXPECT_TRUE(hasFill(list, cam.cellRect(P(2, 1)), warehousesim::view::cellColor(CellType::Wall)));
  EXPECT_TRUE(hasFill(list, cam.cellRect(P(2, 2)), warehousesim::view::cellColor(CellType::Shelf)));
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(2, 1)), 0.04)), 0U);
  // A planned path: a line from cell centre to cell centre in the robot's colour.
  const auto centre = [&cam](Point p) { return cam.worldToScreen({p.x + 0.5, p.y + 0.5}); };
  const ScreenPoint from = centre(P(5, 3));
  const ScreenPoint to = centre(P(6, 3));
  EXPECT_TRUE(std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
    const auto* line = std::get_if<Line>(&command);
    return line != nullptr && near(line->from.x, from.x) && near(line->to.x, to.x) &&
           near(line->from.y, from.y);
  }));
}

TEST(FleetLooksTest, TheColoursOfRobotsObstaclesDestinationsAndPathsAreAllDifferent) {
  std::set<std::tuple<int, int, int>> colours;
  const auto add = [&colours](Color c) { return colours.insert({c.r, c.g, c.b}).second; };
  EXPECT_TRUE(add(warehousesim::view::cellColor(CellType::Free)));
  EXPECT_TRUE(add(warehousesim::view::cellColor(CellType::Wall)));
  EXPECT_TRUE(add(warehousesim::view::cellColor(CellType::Shelf)));
  EXPECT_TRUE(add(warehousesim::view::cellColor(CellType::Pickup)));
  EXPECT_TRUE(add(warehousesim::view::cellColor(CellType::Dropoff)));
  EXPECT_TRUE(add(warehousesim::view::cellColor(CellType::Charging)));
  EXPECT_TRUE(add(warehousesim::view::kPickupRing));
  EXPECT_TRUE(add(warehousesim::view::kDropoffRing));
  // The robot colours are by state; none of them may pass for an obstacle or a floor cell. Path
  // lines use these same colours, so what tells a path from a robot is that it is a thin line.
  for (const RobotState state : {RobotState::Idle, RobotState::Loading, RobotState::Unloading,
                                 RobotState::Charging, RobotState::Unavailable}) {
    EXPECT_TRUE(add(warehousesim::view::robotColor(state, NavigationGoal::None)))
        << static_cast<int>(state);
  }
  for (const NavigationGoal goal :
       {NavigationGoal::Pickup, NavigationGoal::Dropoff, NavigationGoal::Charger}) {
    EXPECT_TRUE(add(warehousesim::view::robotColor(RobotState::Navigating, goal)))
        << static_cast<int>(goal);
  }
}

TEST(FleetLooksTest, NoRobotColourCanPassForAFloorOrAnObstacleOrAStationCell) {
  const auto distance = [](Color a, Color b) {
    return std::abs(a.r - b.r) + std::abs(a.g - b.g) + std::abs(a.b - b.b);
  };
  std::vector<Color> robots;
  for (const RobotState state : {RobotState::Idle, RobotState::Loading, RobotState::Unloading,
                                 RobotState::Charging, RobotState::Unavailable}) {
    robots.push_back(warehousesim::view::robotColor(state, NavigationGoal::None));
  }
  for (const NavigationGoal goal :
       {NavigationGoal::Pickup, NavigationGoal::Dropoff, NavigationGoal::Charger}) {
    robots.push_back(warehousesim::view::robotColor(RobotState::Navigating, goal));
  }
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    for (const Color robotColour : robots) {
      EXPECT_GE(distance(robotColour, warehousesim::view::cellColor(type)), 60)
          << static_cast<int>(type);
    }
  }
}

TEST(FleetLooksTest, DestinationRingsAreOffUnlessAskedFor) {
  const Camera cam = camera();
  const DrawList plain = buildScene(hall(), world(), cam, status());
  EXPECT_EQ(diamonds(plain, inset(cam.cellRect(P(9, 5)), 0.04)), 0U);
  const DrawList rings = buildScene(hall(), world(), cam, status(std::nullopt, std::nullopt, true));
  EXPECT_EQ(diamonds(rings, inset(cam.cellRect(P(9, 5)), 0.04)), 1U);
}

TEST(FleetLooksTest, EveryOpenTaskGetsARingAtItsDropoffAndAtItsPickupUntilCollected) {
  const Camera cam = camera();
  Snapshot s = world();  // task 2 is in progress: picked up, so no ring on its pickup (1, 0)
  const DrawList list = buildScene(hall(), s, cam, status(std::nullopt, std::nullopt, true));
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(1, 0)), 0.04)), 0U);
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(7, 3)), 0.04), warehousesim::view::kDropoffRing),
            1U);
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(6, 0)), 0.04), warehousesim::view::kPickupRing),
            1U);
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(9, 0)), 0.04), warehousesim::view::kDropoffRing),
            1U);
  s.activeTasks.clear();
  EXPECT_EQ(diamonds(buildScene(hall(), s, cam, status(std::nullopt, std::nullopt, true)),
                     inset(cam.cellRect(P(9, 5)), 0.04)),
            0U);
}

TEST(FleetLooksTest, TheSelectedTasksDestinationsAreDoubledInWhite) {
  const Camera cam = camera();
  const Snapshot s = world();
  const DrawList list = buildScene(hall(), s, cam, status(std::nullopt, TaskId{3}, true));
  const Color white{255, 255, 255};
  for (const Point cell : {P(6, 0), P(9, 0)}) {  // task 3: pickup and dropoff
    EXPECT_EQ(diamonds(list, inset(cam.cellRect(cell), 0.02), white), 1U);
    EXPECT_EQ(diamonds(list, inset(cam.cellRect(cell), 0.12), white), 1U);
  }
  // Another task's rings are not.
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(0, 0)), 0.04), warehousesim::view::kPickupRing),
            1U);
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(0, 0)), 0.02), white), 0U);
}

TEST(FleetLooksTest, TheDestinationsOfTheSelectedRobotsTaskAreDoubledToo) {
  const Camera cam = camera();
  const DrawList list = buildScene(hall(), world(), cam, status(RobotId{2}, std::nullopt, true));
  const Color white{255, 255, 255};
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(7, 3)), 0.02), white), 1U);  // task 2's dropoff
  const DrawList other = buildScene(hall(), world(), cam, status(RobotId{1}, std::nullopt, true));
  EXPECT_EQ(diamonds(other, inset(cam.cellRect(P(7, 3)), 0.02), white), 0U);  // robot 1 has no task
}

TEST(FleetLooksTest, TheSelectedRobotIsFramedInWhiteWhateverItsColour) {
  const Camera cam = camera();
  const Color white{255, 255, 255};
  const auto framed = [&](const DrawList& list, Point cell) {
    const ScreenRect r = cam.cellRect(cell);
    const double margin = r.width * 0.06;
    return std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
      const auto* line = std::get_if<Line>(&command);
      return line != nullptr && line->color == white && near(line->from.x, r.x + margin) &&
             near(line->to.x, r.right() - margin) && near(line->from.y, r.y + margin);
    });
  };
  const DrawList none = buildScene(hall(), world(), cam, status());
  const DrawList second = buildScene(hall(), world(), cam, status(RobotId{2}));
  EXPECT_FALSE(framed(none, P(4, 3)));
  EXPECT_TRUE(framed(second, P(4, 3)));
  EXPECT_FALSE(framed(second, P(0, 3)));
  EXPECT_FALSE(framed(second, P(8, 3)));
}

TEST(FleetLooksTest, TheSelectedRobotsRouteIsShownAndThickEvenWhenRoutesAreOff) {
  const Camera cam = camera();
  Snapshot s = world();
  s.robots[0].path = {P(1, 3), P(2, 3)};  // robot 1 has a route as well
  const auto routeLines = [&](const DrawList& list, Point firstCell) {
    const ScreenPoint to = cam.worldToScreen({firstCell.x + 0.5, firstCell.y + 0.5});
    return std::count_if(list.commands.begin(), list.commands.end(), [&](const auto& command) {
      const auto* line = std::get_if<Line>(&command);
      return line != nullptr && (near(line->to.x, to.x) || near(line->to.x, to.x + 1.0)) &&
             (near(line->to.y, to.y) || near(line->to.y, to.y + 1.0));
    });
  };
  const DrawList off = buildScene(hall(), s, cam, status());
  EXPECT_EQ(routeLines(off, P(5, 3)), 0);
  const DrawList selected = buildScene(hall(), s, cam, status(RobotId{2}));
  EXPECT_EQ(routeLines(selected, P(5, 3)), 3);  // the line and its two thickening copies
  EXPECT_EQ(routeLines(selected, P(1, 3)), 0);  // robot 1's route stays hidden
  const DrawList all = buildScene(hall(), s, cam, status(RobotId{2}, std::nullopt, false, true));
  EXPECT_EQ(routeLines(all, P(1, 3)), 1);  // with routes on, the others are plain lines
  EXPECT_EQ(routeLines(all, P(5, 3)), 3);
}

TEST(FleetLooksTest, TheCrossOnACellARobotIsHeldBackFromIsShownForTheSelectedRobotOnly) {
  const Camera cam = camera();
  Snapshot s = world();
  s.robots[1].waiting =
      RobotWait{P(5, 3), warehousesim::traffic::ConflictKind::Vertex, std::nullopt};
  const ScreenRect r = inset(cam.cellRect(P(5, 3)), 0.2);
  const auto crossed = [&](const DrawList& list) {
    return std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
      const auto* line = std::get_if<Line>(&command);
      return line != nullptr && near(line->from.x, r.x) && near(line->from.y, r.y) &&
             near(line->to.x, r.right()) && near(line->to.y, r.bottom());
    });
  };
  EXPECT_FALSE(crossed(buildScene(hall(), s, cam, status())));
  EXPECT_TRUE(crossed(buildScene(hall(), s, cam, status(RobotId{2}))));
  EXPECT_FALSE(crossed(buildScene(hall(), s, cam, status(RobotId{1}))));
}

TEST(FleetLooksTest, ASelectionOutsideTheSnapshotDrawsNothingExtra) {
  const Camera cam = camera();
  const DrawList plain = buildScene(hall(), world(), cam, status());
  const DrawList missing = buildScene(hall(), world(), cam, status(RobotId{99}, TaskId{99}));
  EXPECT_TRUE(plain.commands == missing.commands);
}

TEST(FleetLooksTest, TheKeyHelpOfTheBarCanBeReplacedAndTheDefaultStaysOtherwise) {
  const Camera cam = camera();
  ViewStatus custom = status();
  custom.keyHelp = "my own help";
  EXPECT_TRUE(findText(buildScene(hall(), world(), cam, custom), "my own help") != nullptr);
  EXPECT_TRUE(findText(buildScene(hall(), world(), cam, status()), "Space pause") != nullptr);
}

TEST(FleetLooksTest, ThingsOutsideTheWindowAreNotDrawn) {
  Camera cam = camera();
  cam.setZoom(60.0);
  cam.centerOn({1.0, 1.0});
  Snapshot s = world();
  const DrawList list = buildScene(hall(), s, cam, status(RobotId{3}, TaskId{3}, true));
  EXPECT_EQ(diamonds(list, inset(cam.cellRect(P(9, 5)), 0.04)), 0U);
  for (const auto& command : list.commands) {
    if (const auto* line = std::get_if<Line>(&command)) {
      EXPECT_TRUE(std::isfinite(line->from.x) && std::isfinite(line->to.x));
    }
  }
}

}  // namespace
