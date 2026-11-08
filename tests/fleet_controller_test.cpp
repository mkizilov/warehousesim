#include "warehousesim/view/fleet_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "warehousesim/core/fixed_timestep.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/simulation/commands.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::simulation::Command;
using warehousesim::simulation::CommandRecord;
using warehousesim::simulation::ResetScenario;
using warehousesim::simulation::RobotSnapshot;
using warehousesim::simulation::SetTaskPriority;
using warehousesim::simulation::Snapshot;
using warehousesim::simulation::SubmitTask;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskPriority;
using warehousesim::tasks::TaskStatus;
using warehousesim::view::FleetController;
using warehousesim::view::FleetKey;
using warehousesim::view::FleetOutput;
using warehousesim::view::InputEvents;
using warehousesim::view::PointerAction;
using warehousesim::view::PointerButton;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenRect;
using warehousesim::view::ScreenSize;
using warehousesim::view::TaskEntry;
using warehousesim::world::Warehouse;

using Kind = PointerAction::Kind;

constexpr ScreenSize kWindow{1280, 720};

Point P(int x, int y) { return Point{x, y}; }

// 10 x 6: a wall and a shelf in the third column, open floor otherwise.
Warehouse hall() {
  return *Warehouse::fromRows(
      {"..........", "..#.......", "..S.......", "..........", "..........", ".........."});
}

RobotSnapshot robot(std::uint32_t id, Point at) {
  RobotSnapshot snapshot;
  snapshot.id = RobotId{id};
  snapshot.position = at;
  return snapshot;
}

Task task(std::uint32_t id, TaskStatus status, TaskPriority priority = TaskPriority::Normal,
          std::uint64_t createdAt = 0) {
  Task t;
  t.id = TaskId{id};
  t.pickup = P(0, 0);
  t.dropoff = P(9, 5);
  t.status = status;
  t.priority = priority;
  t.createdAt = createdAt;
  return t;
}

// Three robots in a row on the top line; tasks 1-3 waiting or claimed, 4-5 finished.
Snapshot world() {
  Snapshot snapshot;
  snapshot.robots = {robot(1, P(0, 3)), robot(2, P(4, 3)), robot(3, P(8, 3))};
  snapshot.activeTasks = {task(1, TaskStatus::Pending, TaskPriority::Low, 0),
                          task(2, TaskStatus::Assigned, TaskPriority::High, 1),
                          task(3, TaskStatus::Pending, TaskPriority::Urgent, 5)};
  snapshot.recentTasks = {task(5, TaskStatus::Failed), task(4, TaskStatus::Completed)};
  snapshot.completedTasks = 1;
  snapshot.failedTasks = 1;
  return snapshot;
}

// A controller that has seen one frame of `snapshot`, so that its panel and map are laid out.
struct Rig {
  FleetController controller{kWindow, hall()};
  Snapshot snapshot = world();
  std::uint64_t epoch = 0;

  Rig() { (void)frame({}); }

  FleetOutput frame(const InputEvents& input) {
    return controller.apply(input, 0.016, snapshot, epoch);
  }
  FleetOutput key(FleetKey k) {
    InputEvents input;
    input.fleetKeys.push_back(k);
    return frame(input);
  }
  FleetOutput keys(const std::vector<FleetKey>& ks) {
    InputEvents input;
    input.fleetKeys = ks;
    return frame(input);
  }
  FleetOutput clickAt(ScreenPoint at, PointerButton button = PointerButton::Primary) {
    InputEvents input;
    input.pointerActions = {{Kind::Press, button, at}, {Kind::Release, button, at}};
    return frame(input);
  }
  ScreenPoint cellCentre(Point cell) const {
    const ScreenRect r = controller.camera().cellRect(cell);
    return {r.x + r.width / 2.0, r.y + r.height / 2.0};
  }
  FleetOutput clickCell(Point cell) { return clickAt(cellCentre(cell)); }
  ScreenPoint robotRow(std::size_t i) const {
    const ScreenRect& r = controller.layout().robotRows.at(i);
    return {r.x + 20.0, r.y + r.height / 2.0};
  }
  ScreenPoint activeRow(std::size_t i) const {
    const ScreenRect& r = controller.layout().activeRows.at(i);
    return {r.x + 20.0, r.y + r.height / 2.0};
  }
  ScreenPoint finishedRow(std::size_t i) const {
    const ScreenRect& r = controller.layout().finishedRows.at(i);
    return {r.x + 20.0, r.y + r.height / 2.0};
  }
};

template <class T>
const T* only(const FleetOutput& out) {
  if (out.commands.size() != 1) {
    return nullptr;
  }
  return std::get_if<T>(&out.commands[0]);
}

// ---- the lists ----------------------------------------------------------------------------------

TEST(FleetTaskListsTest, WaitingTasksComeFirstInDispatchOrderThenTheClaimedOnes) {
  Snapshot s = world();
  s.activeTasks = {task(7, TaskStatus::InProgress),
                   task(1, TaskStatus::Pending, TaskPriority::Low, 0),
                   task(4, TaskStatus::Pending, TaskPriority::Urgent, 9),
                   task(3, TaskStatus::Pending, TaskPriority::Urgent, 2),
                   task(2, TaskStatus::Assigned),
                   task(6, TaskStatus::Pending, TaskPriority::Urgent, 2)};
  const auto lists = warehousesim::view::taskLists(s);
  std::vector<std::uint32_t> ids;
  for (const Task& t : lists.active) {
    ids.push_back(t.id.value);
  }
  // Urgent by age then id (3, 6, 4), then low (1), then the claimed ones by id (2, 7).
  EXPECT_EQ(ids, (std::vector<std::uint32_t>{3, 6, 4, 1, 2, 7}));
}

TEST(FleetTaskListsTest, FinishedTasksKeepTheOrderTheyWereGivenInNewestFirst) {
  const auto lists = warehousesim::view::taskLists(world());
  ASSERT_EQ(lists.finished.size(), 2U);
  EXPECT_EQ(lists.finished[0].id.value, 5U);
  EXPECT_EQ(lists.finished[1].id.value, 4U);
}

// ---- selecting a robot --------------------------------------------------------------------------

TEST(FleetControllerTest, NothingIsSelectedAtFirst) {
  Rig rig;
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
  EXPECT_FALSE(rig.controller.selectedTask().has_value());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::None);
  EXPECT_FALSE(rig.controller.wantsQuit());
}

TEST(FleetControllerTest, ClickingARobotOnTheMapSelectsItAndPostsNothing) {
  Rig rig;
  const FleetOutput out = rig.clickCell(P(4, 3));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{2}));
  EXPECT_TRUE(out.commands.empty());
  EXPECT_FALSE(out.paused.has_value());
  EXPECT_FALSE(out.speed.has_value());
}

TEST(FleetControllerTest, ClickingEmptyFloorClearsTheSelectionAndSoDoesClickingAWall) {
  Rig rig;
  rig.clickCell(P(0, 3));
  ASSERT_TRUE(rig.controller.selectedRobot().has_value());
  rig.clickCell(P(6, 0));
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
  rig.clickCell(P(8, 3));
  rig.clickCell(P(2, 1));  // the wall
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
}

TEST(FleetControllerTest, ARobotBetweenCellsIsFoundWhereItIsDrawn) {
  Rig rig;
  rig.snapshot.robots[0].position = P(0, 3);
  rig.snapshot.robots[0].pose =
      warehousesim::simulation::RobotPose{{1.4, 3.5}, 0.0};  // drawn in column 1
  (void)rig.frame({});
  rig.clickCell(P(1, 3));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{1}));
}

TEST(FleetControllerTest, ARobotIsStillFoundByItsLogicalCellIfNothingIsDrawnUnderTheClick) {
  Rig rig;
  rig.snapshot.robots[0].pose = warehousesim::simulation::RobotPose{{1.4, 3.5}, 0.0};
  (void)rig.frame({});
  rig.clickCell(P(0, 3));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{1}));
}

TEST(FleetControllerTest, ClickingARowOfTheRobotListSelectsThatRobot) {
  Rig rig;
  rig.clickAt(rig.robotRow(2));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{3}));
  rig.clickAt(rig.robotRow(0));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{1}));
}

TEST(FleetControllerTest, TabStepsThroughTheRobotsAndWrapsAndShiftTabGoesBackwards) {
  Rig rig;
  std::vector<std::uint32_t> seen;
  for (int i = 0; i < 4; ++i) {
    rig.key(FleetKey::NextRobot);
    seen.push_back(rig.controller.selectedRobot()->value);
  }
  EXPECT_EQ(seen, (std::vector<std::uint32_t>{1, 2, 3, 1}));
  seen.clear();
  for (int i = 0; i < 4; ++i) {
    rig.key(FleetKey::PreviousRobot);
    seen.push_back(rig.controller.selectedRobot()->value);
  }
  EXPECT_EQ(seen, (std::vector<std::uint32_t>{3, 2, 1, 3}));
}

TEST(FleetControllerTest, ShiftTabWithNothingSelectedStartsAtTheLastRobot) {
  Rig rig;
  rig.key(FleetKey::PreviousRobot);
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{3}));
}

TEST(FleetControllerTest, TabWithNoRobotsSelectsNothing) {
  Rig rig;
  rig.snapshot.robots.clear();
  rig.key(FleetKey::NextRobot);
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
  rig.key(FleetKey::PreviousRobot);
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
}

TEST(FleetControllerTest, ASelectedRobotThatIsNoLongerInTheSnapshotIsDeselected) {
  Rig rig;
  rig.key(FleetKey::NextRobot);
  rig.key(FleetKey::NextRobot);
  ASSERT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{2}));
  rig.snapshot.robots.erase(rig.snapshot.robots.begin() + 1);
  (void)rig.frame({});
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
}

// ---- selecting a task ---------------------------------------------------------------------------

TEST(FleetControllerTest, ClickingATaskRowSelectsThatTaskInTheOrderOfTheList) {
  Rig rig;
  // Active rows: 3 (urgent, waiting), 1 (low, waiting), 2 (claimed). Finished rows: 5, 4.
  rig.clickAt(rig.activeRow(0));
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{3}));
  rig.clickAt(rig.activeRow(2));
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{2}));
  rig.clickAt(rig.finishedRow(1));
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{4}));
}

TEST(FleetControllerTest, TStepsThroughActiveThenFinishedTasksAndWraps) {
  Rig rig;
  std::vector<std::uint32_t> seen;
  for (int i = 0; i < 6; ++i) {
    rig.key(FleetKey::NextTask);
    seen.push_back(rig.controller.selectedTask()->value);
  }
  EXPECT_EQ(seen, (std::vector<std::uint32_t>{3, 1, 2, 5, 4, 3}));
  rig.key(FleetKey::PreviousTask);
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{4}));
}

TEST(FleetControllerTest, ASelectedTaskThatLeavesBothListsIsDeselected) {
  Rig rig;
  rig.clickAt(rig.activeRow(1));  // task 1
  ASSERT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{1}));
  rig.snapshot.activeTasks.erase(rig.snapshot.activeTasks.begin());
  (void)rig.frame({});
  EXPECT_FALSE(rig.controller.selectedTask().has_value());
}

TEST(FleetControllerTest, ATaskThatMovesFromActiveToFinishedStaysSelected) {
  Rig rig;
  rig.clickAt(rig.activeRow(2));  // task 2
  ASSERT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{2}));
  Task done = rig.snapshot.activeTasks[1];
  done.status = TaskStatus::Completed;
  rig.snapshot.activeTasks.erase(rig.snapshot.activeTasks.begin() + 1);
  rig.snapshot.recentTasks.insert(rig.snapshot.recentTasks.begin(), done);
  (void)rig.frame({});
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{2}));
}

// ---- adding a task ------------------------------------------------------------------------------

TEST(FleetControllerTest, NThenAPickupThenADropoffPostsOneTaskWithTheChosenCells) {
  Rig rig;
  EXPECT_TRUE(rig.key(FleetKey::NewTask).commands.empty());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Pickup);
  EXPECT_TRUE(rig.clickCell(P(1, 0)).commands.empty());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Dropoff);
  EXPECT_EQ(rig.controller.pickup(), std::optional<Point>(P(1, 0)));

  const FleetOutput out = rig.clickCell(P(8, 5));
  const auto* submit = only<SubmitTask>(out);
  ASSERT_TRUE(submit != nullptr);
  EXPECT_EQ(submit->pickup, P(1, 0));
  EXPECT_EQ(submit->dropoff, P(8, 5));
  EXPECT_EQ(submit->priority, TaskPriority::Normal);
  EXPECT_EQ(rig.controller.entry(), TaskEntry::None);
  EXPECT_FALSE(rig.controller.pickup().has_value());
}

TEST(FleetControllerTest, ThePriorityChosenDuringEntryIsTheOneOfTheTask) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  rig.key(FleetKey::Priority4);
  EXPECT_EQ(rig.controller.newTaskPriority(), TaskPriority::Urgent);
  rig.clickCell(P(1, 0));
  rig.key(FleetKey::Priority1);  // can still be changed between the two clicks
  const FleetOutput out = rig.clickCell(P(8, 5));
  const auto* submit = only<SubmitTask>(out);
  ASSERT_TRUE(submit != nullptr);
  EXPECT_EQ(submit->priority, TaskPriority::Low);
}

TEST(FleetControllerTest, ThePriorityPickedForOneTaskIsKeptForTheNextOneToo) {
  Rig rig;
  rig.keys({FleetKey::NewTask, FleetKey::Priority3});
  rig.clickCell(P(1, 0));
  rig.clickCell(P(8, 5));
  rig.key(FleetKey::NewTask);
  rig.clickCell(P(1, 0));
  const FleetOutput out = rig.clickCell(P(7, 5));
  const auto* second = only<SubmitTask>(out);
  ASSERT_TRUE(second != nullptr);
  EXPECT_EQ(second->priority, TaskPriority::High);
}

TEST(FleetControllerTest, ACellThatIsNotFloorCannotBeAPickupOrADropoffAndEntryContinues) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  EXPECT_TRUE(rig.clickCell(P(2, 1)).commands.empty());  // the wall
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Pickup);
  EXPECT_TRUE(rig.controller.messageIsError());
  EXPECT_TRUE(rig.controller.message().find("wall") != std::string::npos)
      << rig.controller.message();
  rig.clickCell(P(5, 0));
  EXPECT_TRUE(rig.clickCell(P(2, 2)).commands.empty());  // the shelf
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Dropoff);
  EXPECT_TRUE(rig.controller.message().find("shelf") != std::string::npos)
      << rig.controller.message();
}

TEST(FleetControllerTest, TheDropoffMayNotBeThePickup) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  rig.clickCell(P(5, 0));
  const FleetOutput out = rig.clickCell(P(5, 0));
  EXPECT_TRUE(out.commands.empty());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Dropoff);
  EXPECT_TRUE(rig.controller.messageIsError());
}

TEST(FleetControllerTest, ACellWithARobotOnItCanBeAPickup) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  rig.clickCell(P(4, 3));  // robot 2 stands there
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Dropoff);
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());  // and it did not select the robot
}

TEST(FleetControllerTest, EscapeCancelsTaskEntryAtEitherStep) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  rig.key(FleetKey::Cancel);
  EXPECT_EQ(rig.controller.entry(), TaskEntry::None);
  EXPECT_FALSE(rig.controller.wantsQuit());

  rig.key(FleetKey::NewTask);
  rig.clickCell(P(5, 0));
  rig.key(FleetKey::Cancel);
  EXPECT_EQ(rig.controller.entry(), TaskEntry::None);
  EXPECT_FALSE(rig.controller.pickup().has_value());
  EXPECT_FALSE(rig.controller.wantsQuit());
  // And a click now selects again instead of continuing the task.
  EXPECT_TRUE(rig.clickCell(P(4, 3)).commands.empty());
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{2}));
}

TEST(FleetControllerTest, PressingNAgainStartsTheTaskOver) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  rig.clickCell(P(5, 0));
  rig.key(FleetKey::NewTask);
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Pickup);
  EXPECT_FALSE(rig.controller.pickup().has_value());
}

TEST(FleetControllerTest, ClicksOnThePanelDuringEntryChooseNoCell) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  EXPECT_TRUE(rig.clickAt(rig.robotRow(0)).commands.empty());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Pickup);
  EXPECT_FALSE(rig.controller.pickup().has_value());
  EXPECT_EQ(rig.controller.selectedRobot(),
            std::optional<RobotId>(RobotId{1}));  // it selects, as ever
}

// ---- changing a priority ------------------------------------------------------------------------

TEST(FleetControllerTest, APriorityKeyOnASelectedWaitingTaskPostsTheChange) {
  Rig rig;
  rig.clickAt(rig.activeRow(1));  // task 1, low, waiting
  for (const auto& [k, expected] : std::vector<std::pair<FleetKey, TaskPriority>>{
           {FleetKey::Priority2, TaskPriority::Normal},
           {FleetKey::Priority3, TaskPriority::High},
           {FleetKey::Priority4, TaskPriority::Urgent}}) {
    const FleetOutput out = rig.key(k);
    const auto* change = only<SetTaskPriority>(out);
    ASSERT_TRUE(change != nullptr);
    EXPECT_EQ(change->task, TaskId{1});
    EXPECT_EQ(change->priority, expected);
  }
}

TEST(FleetControllerTest, TheSamePriorityPostsNothingAndSaysSo) {
  Rig rig;
  rig.clickAt(rig.activeRow(1));  // low
  const FleetOutput out = rig.key(FleetKey::Priority1);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_TRUE(rig.controller.message().find("already low") != std::string::npos)
      << rig.controller.message();
}

TEST(FleetControllerTest, ATaskARobotHasClaimedOrThatIsFinishedCannotChangePriority) {
  Rig rig;
  rig.clickAt(rig.activeRow(2));  // task 2, assigned
  FleetOutput out = rig.key(FleetKey::Priority4);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_TRUE(rig.controller.messageIsError());
  EXPECT_TRUE(rig.controller.message().find("assigned") != std::string::npos)
      << rig.controller.message();

  rig.clickAt(rig.finishedRow(0));  // task 5, failed
  out = rig.key(FleetKey::Priority4);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_TRUE(rig.controller.message().find("failed") != std::string::npos)
      << rig.controller.message();
}

TEST(FleetControllerTest, APriorityKeyWithNoTaskSelectedPostsNothingAndSaysWhy) {
  Rig rig;
  const FleetOutput out = rig.key(FleetKey::Priority3);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_TRUE(rig.controller.messageIsError());
  EXPECT_TRUE(rig.controller.message().find("select a waiting task") != std::string::npos);
}

TEST(FleetControllerTest, SelectingARobotDoesNotSelectItsTaskSoThePriorityKeysNeedATaskOfTheirOwn) {
  Rig rig;
  rig.clickCell(P(4, 3));
  EXPECT_TRUE(rig.key(FleetKey::Priority4).commands.empty());
}

// ---- other commands -----------------------------------------------------------------------------

TEST(FleetControllerTest, RPostsAResetAndNothingElse) {
  Rig rig;
  const FleetOutput out = rig.key(FleetKey::Reset);
  const auto* reset = only<ResetScenario>(out);
  ASSERT_TRUE(reset != nullptr);
  EXPECT_FALSE(reset->seed.has_value());
}

TEST(FleetControllerTest, PauseAndSpeedAreNotCommandsTheyAreReportedSeparately) {
  Rig rig;
  InputEvents pause;
  pause.togglePause = true;
  FleetOutput out = rig.frame(pause);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_EQ(out.paused, std::optional<bool>(true));
  EXPECT_TRUE(rig.controller.paused());
  out = rig.frame(pause);
  EXPECT_EQ(out.paused, std::optional<bool>(false));

  InputEvents faster;
  faster.speedSteps = 2;
  out = rig.frame(faster);
  EXPECT_TRUE(out.commands.empty());
  ASSERT_TRUE(out.speed.has_value());
  EXPECT_DOUBLE_EQ(*out.speed, 4.0);
  EXPECT_DOUBLE_EQ(rig.controller.speed(), 4.0);
  InputEvents slower;
  slower.speedSteps = -3;
  out = rig.frame(slower);
  EXPECT_DOUBLE_EQ(*out.speed, 0.5);
  EXPECT_FALSE(rig.frame({}).paused.has_value());
  EXPECT_FALSE(rig.frame({}).speed.has_value());
}

TEST(FleetControllerTest, TheSpeedStopsAtItsLimits) {
  Rig rig;
  InputEvents many;
  many.speedSteps = 50;
  EXPECT_DOUBLE_EQ(*rig.frame(many).speed, warehousesim::core::FixedTimestep::kMaxSpeed);
  many.speedSteps = -100;
  EXPECT_DOUBLE_EQ(*rig.frame(many).speed, warehousesim::core::FixedTimestep::kMinSpeed);
}

TEST(FleetControllerTest, TheInitialSpeedIsTheOneGiven) {
  const FleetController c(kWindow, hall(), 4.0);
  EXPECT_DOUBLE_EQ(c.speed(), 4.0);
}

TEST(FleetControllerTest, TheMetricsDashboardStartsOffAndMTogglesItWithoutPostingAnything) {
  Rig rig;
  EXPECT_FALSE(rig.controller.showMetrics());
  FleetOutput out = rig.key(FleetKey::ToggleMetrics);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_FALSE(out.paused.has_value());
  EXPECT_FALSE(out.speed.has_value());
  EXPECT_FALSE(out.exportMetrics);
  EXPECT_TRUE(rig.controller.showMetrics());
  rig.key(FleetKey::ToggleMetrics);
  EXPECT_FALSE(rig.controller.showMetrics());
}

TEST(FleetControllerTest, EAsksForAnExportAndForNothingElse) {
  Rig rig;
  const FleetOutput out = rig.key(FleetKey::ExportMetrics);
  EXPECT_TRUE(out.exportMetrics);
  EXPECT_TRUE(out.commands.empty());  // an export is not a command: it changes nothing
  EXPECT_FALSE(out.paused.has_value());
  EXPECT_FALSE(rig.controller.showMetrics());
  EXPECT_FALSE(rig.frame({}).exportMetrics);  // and it is asked for once, not every frame
}

TEST(FleetControllerTest, TheDashboardSurvivesARestartOfTheScenario) {
  Rig rig;
  rig.key(FleetKey::ToggleMetrics);
  rig.epoch = 1;
  (void)rig.frame({});
  EXPECT_TRUE(rig.controller.showMetrics());
}

TEST(FleetControllerTest, TheOverlaysToggleAndStartOn) {
  Rig rig;
  EXPECT_TRUE(rig.controller.showPaths());
  EXPECT_TRUE(rig.controller.showOccupied());
  InputEvents toggle;
  toggle.togglePaths = true;
  EXPECT_TRUE(rig.frame(toggle).commands.empty());
  EXPECT_FALSE(rig.controller.showPaths());
  EXPECT_TRUE(rig.controller.showOccupied());
}

// ---- escape -------------------------------------------------------------------------------------

TEST(FleetControllerTest, EscapeCancelsEntryThenClearsTheSelectionThenQuits) {
  Rig rig;
  rig.clickCell(P(4, 3));
  rig.clickAt(rig.activeRow(0));
  rig.key(FleetKey::NewTask);
  ASSERT_EQ(rig.controller.entry(), TaskEntry::Pickup);

  rig.key(FleetKey::Cancel);  // entry
  EXPECT_EQ(rig.controller.entry(), TaskEntry::None);
  EXPECT_TRUE(rig.controller.selectedRobot().has_value());
  EXPECT_FALSE(rig.controller.wantsQuit());
  rig.key(FleetKey::Cancel);  // selection
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
  EXPECT_FALSE(rig.controller.selectedTask().has_value());
  EXPECT_FALSE(rig.controller.wantsQuit());
  rig.key(FleetKey::Cancel);  // nothing left: quit
  EXPECT_TRUE(rig.controller.wantsQuit());
}

TEST(FleetControllerTest, ClosingTheWindowQuitsAtOnceWhateverIsGoingOn) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  InputEvents quit;
  quit.quit = true;
  rig.frame(quit);
  EXPECT_TRUE(rig.controller.wantsQuit());
}

// ---- what a click is ----------------------------------------------------------------------------

TEST(FleetControllerTest, APressAndReleaseFarApartIsADragNotAClick) {
  Rig rig;
  InputEvents drag;
  // Pressed on one robot and let go over another: that is not a click on either.
  drag.pointerActions = {{Kind::Press, PointerButton::Primary, rig.cellCentre(P(4, 3))},
                         {Kind::Release, PointerButton::Primary, rig.cellCentre(P(8, 3))}};
  rig.frame(drag);
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::None);

  // And a drag in task entry chooses no cell.
  rig.key(FleetKey::NewTask);
  rig.frame(drag);
  EXPECT_FALSE(rig.controller.pickup().has_value());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Pickup);
}

TEST(FleetControllerTest, ASmallWobbleBetweenPressAndReleaseIsStillAClick) {
  Rig rig;
  const ScreenPoint at = rig.cellCentre(P(4, 3));
  InputEvents click;
  click.pointerActions = {{Kind::Press, PointerButton::Primary, at},
                          {Kind::Release, PointerButton::Primary, {at.x + 3.0, at.y - 2.0}}};
  rig.frame(click);
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{2}));
}

TEST(FleetControllerTest, OnlyTheLeftButtonClicks) {
  Rig rig;
  rig.clickAt(rig.cellCentre(P(4, 3)), PointerButton::Secondary);
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
}

TEST(FleetControllerTest, AReleaseWithoutAPressIsIgnored) {
  Rig rig;
  InputEvents stray;
  stray.pointerActions = {{Kind::Release, PointerButton::Primary, rig.cellCentre(P(4, 3))}};
  rig.frame(stray);
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
}

TEST(FleetControllerTest, APressInOneFrameAndTheReleaseInTheNextIsAClick) {
  Rig rig;
  const ScreenPoint at = rig.cellCentre(P(4, 3));
  InputEvents press;
  press.pointerActions = {{Kind::Press, PointerButton::Primary, at}};
  rig.frame(press);
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
  InputEvents release;
  release.pointerActions = {{Kind::Release, PointerButton::Primary, at}};
  rig.frame(release);
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{2}));
}

TEST(FleetControllerTest, ClicksOnTheRulersTheStatusBarAndOutsideTheWindowSelectNothing) {
  Rig rig;
  rig.clickCell(P(4, 3));
  ASSERT_TRUE(rig.controller.selectedRobot().has_value());
  const std::vector<ScreenPoint> nowhere{{3.0, 3.0},     {400.0, 3.0},   {3.0, 400.0},
                                         {400.0, 715.0}, {-10.0, -10.0}, {5000.0, 5000.0}};
  for (const ScreenPoint at : nowhere) {
    rig.clickAt(at);
    EXPECT_TRUE(rig.controller.selectedRobot().has_value())
        << at.x << "," << at.y;  // not even deselected
  }
}

TEST(FleetControllerTest, NonFiniteInputIsIgnored) {
  Rig rig;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  InputEvents junk;
  junk.pointerActions = {{Kind::Press, PointerButton::Primary, {nan, 5.0}},
                         {Kind::Release, PointerButton::Primary, {nan, 5.0}},
                         {Kind::Press, PointerButton::Primary, {inf, inf}},
                         {Kind::Release, PointerButton::Primary, {inf, inf}}};
  junk.zoomSteps = nan;
  junk.dragX = inf;
  junk.panX = nan;
  junk.pointer = ScreenPoint{nan, nan};
  const FleetOutput out = rig.frame(junk);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_FALSE(rig.controller.selectedRobot().has_value());
  EXPECT_TRUE(std::isfinite(rig.controller.camera().zoom()));
}

TEST(FleetControllerTest, ZoomPanAndHoverMoveTheViewAndPostNothing) {
  Rig rig;
  const double fitted = rig.controller.camera().zoom();
  InputEvents view;
  view.zoomSteps = 4.0;
  view.dragX = 25.0;
  view.panY = 1.0;
  view.pointer = rig.cellCentre(P(4, 3));
  const FleetOutput out = rig.frame(view);
  EXPECT_TRUE(out.commands.empty());
  EXPECT_GT(rig.controller.camera().zoom(), fitted);
  EXPECT_TRUE(rig.controller.pointer().has_value());
}

TEST(FleetControllerTest, ClicksFollowTheCameraAfterZoomingAndPanning) {
  Rig rig;
  InputEvents zoom;
  zoom.zoomSteps = 5.0;
  zoom.zoomAnchor = rig.cellCentre(P(8, 3));
  rig.frame(zoom);
  rig.clickCell(P(8, 3));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{3}));
}

TEST(FleetControllerTest, TheHoveredCellIsOnlyOverThePartOfTheMapThatCanBeClicked) {
  Rig rig;
  InputEvents over;
  over.pointer = rig.cellCentre(P(6, 2));
  rig.frame(over);
  EXPECT_EQ(rig.controller.hoveredCell(), std::optional<Point>(P(6, 2)));
  over.pointer = ScreenPoint{3.0, 3.0};
  rig.frame(over);
  EXPECT_FALSE(rig.controller.hoveredCell().has_value());
  over.pointer = ScreenPoint{1200.0, 300.0};  // in the panel
  rig.frame(over);
  EXPECT_FALSE(rig.controller.hoveredCell().has_value());
}

// ---- the panel and the window -------------------------------------------------------------------

TEST(FleetControllerTest, ClicksInThePanelNeverTouchTheMapEvenWhereTheMapWouldBeUnderneath) {
  Rig rig;
  rig.key(FleetKey::NewTask);
  const FleetOutput out = rig.clickAt({rig.controller.layout().panel.x + 100.0, 300.0});
  EXPECT_TRUE(out.commands.empty());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::Pickup);
}

TEST(FleetControllerTest, WithoutRoomForAPanelTheMapStillTakesClicks) {
  FleetController c(ScreenSize{600, 500}, hall());
  Snapshot snapshot = world();
  (void)c.apply({}, 0.016, snapshot, 0);
  EXPECT_FALSE(c.layout().hasPanel());
  const ScreenRect r = c.camera().cellRect(P(4, 3));
  InputEvents click;
  const ScreenPoint at{r.x + r.width / 2.0, r.y + r.height / 2.0};
  click.pointerActions = {{Kind::Press, PointerButton::Primary, at},
                          {Kind::Release, PointerButton::Primary, at}};
  (void)c.apply(click, 0.016, snapshot, 0);
  EXPECT_EQ(c.selectedRobot(), std::optional<RobotId>(RobotId{2}));
}

TEST(FleetControllerTest, ResizingKeepsTheSelectionAndRefitsTheMap) {
  Rig rig;
  rig.clickCell(P(4, 3));
  rig.controller.resize(ScreenSize{1000, 640});
  (void)rig.frame({});
  EXPECT_EQ(rig.controller.window(), (ScreenSize{1000, 640}));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{2}));
  EXPECT_EQ(rig.controller.camera().screen().width +
                static_cast<int>(rig.controller.layout().panel.width),
            1000);
  EXPECT_TRUE(rig.controller.camera().fitted());
  rig.clickCell(P(8, 3));
  EXPECT_EQ(rig.controller.selectedRobot(), std::optional<RobotId>(RobotId{3}));
}

// ---- the scenario starting over -----------------------------------------------------------------

TEST(FleetControllerTest, WhenTheScenarioIsRestartedTheTaskSelectionAndTaskEntryAreDropped) {
  Rig rig;
  rig.clickAt(rig.activeRow(0));
  rig.key(FleetKey::NewTask);
  rig.clickCell(P(5, 0));
  ASSERT_TRUE(rig.controller.selectedTask().has_value());
  ASSERT_EQ(rig.controller.entry(), TaskEntry::Dropoff);
  rig.snapshot.activeTasks.clear();
  rig.snapshot.recentTasks.clear();
  rig.epoch = 1;
  (void)rig.frame({});
  EXPECT_FALSE(rig.controller.selectedTask().has_value());
  EXPECT_EQ(rig.controller.entry(), TaskEntry::None);
  EXPECT_FALSE(rig.controller.pickup().has_value());
}

TEST(FleetControllerTest, ATaskWithTheSameNumberInTheNewRunIsNotTheSelectedOne) {
  Rig rig;
  rig.clickAt(rig.activeRow(0));  // task 3
  ASSERT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{3}));
  rig.epoch = 1;  // restarted; the new run has a task #3 of its own
  (void)rig.frame({});
  EXPECT_FALSE(rig.controller.selectedTask().has_value());
}

TEST(FleetControllerTest, TheSameEpochChangesNothing) {
  Rig rig;
  rig.clickAt(rig.activeRow(0));
  (void)rig.frame({});
  (void)rig.frame({});
  EXPECT_TRUE(rig.controller.selectedTask().has_value());
}

// ---- what happened to a command -----------------------------------------------------------------

TEST(FleetControllerTest, AnAddedTaskIsSelectedAndItsOutcomeShown) {
  Rig rig;
  CommandRecord record;
  record.tick = 12;
  record.command = SubmitTask{P(0, 0), P(9, 5), TaskPriority::High};
  record.outcome = {true, "task #3 added: (0, 0) to (9, 5), high", TaskId{3}};
  rig.controller.note(record);
  EXPECT_EQ(rig.controller.message(), record.outcome.message);
  EXPECT_FALSE(rig.controller.messageIsError());
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{3}));
  (void)rig.frame({});  // task 3 is in the lists, so the selection stands
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{3}));
}

TEST(FleetControllerTest, ARefusedCommandShowsItsReasonAsAnErrorAndSelectsNothing) {
  Rig rig;
  CommandRecord record;
  record.command = SetTaskPriority{TaskId{1}, TaskPriority::High};
  record.outcome = {false,
                    "task #1 is assigned; only a task that is still waiting can change priority",
                    std::nullopt};
  rig.controller.note(record);
  EXPECT_TRUE(rig.controller.messageIsError());
  EXPECT_TRUE(rig.controller.message().find("assigned") != std::string::npos);
  EXPECT_FALSE(rig.controller.selectedTask().has_value());
}

TEST(FleetControllerTest, APriorityChangeDoesNotMoveTheSelectionToAnotherTask) {
  Rig rig;
  rig.clickAt(rig.activeRow(1));  // task 1
  CommandRecord record;
  record.command = SetTaskPriority{TaskId{1}, TaskPriority::High};
  record.outcome = {true, "task #1 is now high", TaskId{1}};
  rig.controller.note(record);
  EXPECT_EQ(rig.controller.selectedTask(), std::optional<TaskId>(TaskId{1}));
}

TEST(FleetControllerTest, TheCallerCanShowAMessageOfItsOwn) {
  Rig rig;
  rig.controller.report("too many commands waiting; try again", true);
  EXPECT_TRUE(rig.controller.messageIsError());
  EXPECT_EQ(rig.controller.message(), "too many commands waiting; try again");
}

// ---- whatever the input -------------------------------------------------------------------------

TEST(FleetControllerPropertyTest, RandomInputOnlyEverPostsCommandsThatAreValidOnTheirFace) {
  const Warehouse layout = hall();
  for (std::uint64_t seed = 1; seed <= 8; ++seed) {
    warehousesim::core::Rng rng(seed);
    Rig rig;
    for (int step = 0; step < 500; ++step) {
      InputEvents input;
      for (std::uint64_t i = rng.below(3); i > 0; --i) {
        input.fleetKeys.push_back(static_cast<FleetKey>(rng.below(13)));
      }
      for (std::uint64_t i = rng.below(5); i > 0; --i) {
        const ScreenPoint at{static_cast<double>(rng.below(1400)) - 60.0,
                             static_cast<double>(rng.below(800)) - 40.0};
        input.pointerActions.push_back(
            {static_cast<Kind>(rng.below(3)), static_cast<PointerButton>(rng.below(2)), at});
        if (rng.below(2) == 0) {  // very often a real click
          input.pointerActions.push_back({Kind::Press, PointerButton::Primary, at});
          input.pointerActions.push_back({Kind::Release, PointerButton::Primary, at});
        }
      }
      if (rng.below(8) == 0) {
        input.zoomSteps = static_cast<double>(rng.below(7)) - 3.0;
      }
      if (rng.below(10) == 0) {
        input.togglePause = true;
      }
      if (rng.below(10) == 0) {
        input.speedSteps = static_cast<int>(rng.below(5)) - 2;
      }
      if (rng.below(40) == 0) {
        rig.controller.resize(
            ScreenSize{static_cast<int>(rng.below(1500)), static_cast<int>(rng.below(900))});
      }
      if (rng.below(60) == 0) {
        rig.epoch += 1;
      }
      const FleetOutput out = rig.frame(input);
      for (const Command& command : out.commands) {
        if (const auto* submit = std::get_if<SubmitTask>(&command)) {
          ASSERT_TRUE(layout.isWalkable(submit->pickup)) << "seed " << seed << " step " << step;
          ASSERT_TRUE(layout.isWalkable(submit->dropoff));
          ASSERT_FALSE(submit->pickup == submit->dropoff);
        } else if (const auto* change = std::get_if<SetTaskPriority>(&command)) {
          const auto& lists = rig.controller.tasks();
          bool waiting = false;
          for (const Task& t : lists.active) {
            waiting = waiting || (t.id == change->task && t.status == TaskStatus::Pending &&
                                  t.priority != change->priority);
          }
          ASSERT_TRUE(waiting) << "seed " << seed << " step " << step;
        }
      }
      // The selection always refers to something that exists.

      if (const auto robotId = rig.controller.selectedRobot()) {
        bool found = false;
        for (const RobotSnapshot& r : rig.snapshot.robots) {
          found = found || r.id == *robotId;
        }
        ASSERT_TRUE(found);
      }
    }
  }
}

TEST(FleetControllerPropertyTest, InputThatNeverAsksForACommandNeverPostsOne) {
  warehousesim::core::Rng rng(77);
  Rig rig;
  for (int step = 0; step < 400; ++step) {
    InputEvents input;
    // No N, no priority keys, no R: only looking around and selecting.
    for (std::uint64_t i = rng.below(3); i > 0; --i) {
      const FleetKey looking[] = {FleetKey::NextRobot,     FleetKey::PreviousRobot,
                                  FleetKey::NextTask,      FleetKey::PreviousTask,
                                  FleetKey::ToggleMetrics, FleetKey::ExportMetrics};
      input.fleetKeys.push_back(looking[rng.below(6)]);
    }
    const ScreenPoint at{static_cast<double>(rng.below(1400)), static_cast<double>(rng.below(800))};
    input.pointerActions = {{Kind::Press, PointerButton::Primary, at},
                            {Kind::Release, PointerButton::Primary, at}};
    input.zoomSteps = static_cast<double>(rng.below(5)) - 2.0;
    input.dragX = static_cast<double>(rng.below(40)) - 20.0;
    input.pointer = at;
    input.togglePaths = rng.below(5) == 0;
    input.toggleOccupied = rng.below(5) == 0;
    input.resetView = rng.below(9) == 0;
    ASSERT_TRUE(rig.frame(input).commands.empty()) << step;
  }
}

}  // namespace
