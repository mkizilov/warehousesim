#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/cell_type.hpp"
#include "warehousesim/world/layouts.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::TaskId;
using warehousesim::robots::RobotState;
using warehousesim::simulation::PlanningStats;
using warehousesim::simulation::Simulation;
using warehousesim::simulation::SimulationConfig;
using warehousesim::tasks::TaskPriority;
using warehousesim::tasks::TaskStatus;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;
using warehousesim_test::Errand;
using warehousesim_test::hall;
using warehousesim_test::ideal;
using warehousesim_test::Job;
using warehousesim_test::layout;
using warehousesim_test::openFloor;
using warehousesim_test::P;
using warehousesim_test::R;
using warehousesim_test::runToCompletion;
using warehousesim_test::Scenario;
using warehousesim_test::soloTime;
using warehousesim_test::TrafficAudit;
using warehousesim_test::vehicles;

std::uint64_t latest(const std::vector<std::uint64_t>& done) {
  return *std::max_element(done.begin(), done.end());
}

bool allDone(const std::vector<std::uint64_t>& done) {
  return std::all_of(done.begin(), done.end(), [](std::uint64_t t) { return t > 0; });
}

SimulationConfig tableOnly(SimulationConfig config) {
  config.cooperative = false;
  return config;
}

// ---- narrow corridors --------------------------------------------------------------------------

//   Both start inside the corridor of hall(), facing each other, and meet at the niche above (7,
//   1).
std::vector<Job> meetingInTheCorridor() {
  return {{P(4, 1), P(13, 1), P(13, 2)}, {P(10, 1), P(1, 1), P(1, 2)}};
}

TEST(CooperativePlanningTest, TwoRobotsFacingEachOtherInACorridorPassAtTheNicheBySearchingForIt) {
  Scenario scenario(hall(), meetingInTheCorridor(), ideal());
  const auto done = runToCompletion(scenario, 400);
  ASSERT_TRUE(allDone(done));
  const PlanningStats& stats = scenario.sim.planningStats();
  EXPECT_GE(stats.dodges,
            1U);  // the one with the lower priority was in the way and stepped out of it
  EXPECT_GE(stats.reroutes, 1U);
  EXPECT_EQ(stats.deadlocks, 0U);
  EXPECT_EQ(stats.abandoned, 0U);
}

TEST(CooperativePlanningTest,
     ThePassingTakesLessTimeThanLettingTheRobotsMeetAndSortItOutAfterwards) {
  Scenario planned(hall(), meetingInTheCorridor(), ideal());
  const auto withSearch = runToCompletion(planned, 400);
  Scenario plain(hall(), meetingInTheCorridor(), tableOnly(ideal()));
  const auto without = runToCompletion(plain, 400);
  ASSERT_TRUE(allDone(withSearch));
  ASSERT_TRUE(allDone(without));
  EXPECT_LT(latest(withSearch), latest(without));
}

TEST(CooperativePlanningTest, TheRobotWithPriorityIsNeverDelayedByTheOneThatDodges) {
  const std::vector<Job> jobs = meetingInTheCorridor();
  const std::uint64_t alone = soloTime(hall(), jobs, 0, ideal());
  Scenario scenario(hall(), jobs, ideal());
  const auto done = runToCompletion(scenario, 400);
  EXPECT_EQ(done[0], alone);
  EXPECT_GT(done[1], 0U);
}

TEST(CooperativePlanningTest, WithTheIdsTheOtherWayRoundTheOtherRobotDodges) {
  const std::vector<Job> jobs = {meetingInTheCorridor()[1], meetingInTheCorridor()[0]};
  const std::uint64_t alone = soloTime(hall(), jobs, 0, ideal());
  Scenario scenario(hall(), jobs, ideal());
  const auto done = runToCompletion(scenario, 400);
  EXPECT_EQ(done[0], alone);
  EXPECT_GT(done[1], 0U);
  EXPECT_GE(scenario.sim.planningStats().dodges, 1U);
}

TEST(CooperativePlanningTest, VehiclesThatNeedRoomToStopPassAtTheNicheToo) {
  Scenario scenario(hall(), meetingInTheCorridor(), vehicles());
  const auto done = runToCompletion(scenario, 600);
  EXPECT_TRUE(allDone(done));
}

// A corridor with a niche every few cells, and four robots each way. Whoever can, steps into a
// niche to let the others by; the point is that all of them get through. Every robot has a goal
// cell of its own at the far end, so that nobody ends up parked on a cell somebody else still needs
// (see RobotsQueuingAroundAnIdleRobotOnTheirSharedGoalStaySafe for what happens when they do).
TEST(CooperativePlanningTest, EightRobotsSharingALongCorridorWithSomeNichesAllGetThrough) {
  const Warehouse w = layout({"....#.#.#.#....", "..............."});
  const std::vector<Point> starts = {P(0, 1),  P(1, 1),  P(0, 0),  P(2, 0),
                                     P(14, 1), P(13, 1), P(14, 0), P(12, 0)};
  const std::vector<Point> rightRoomGoals = {P(14, 0), P(14, 1), P(13, 0), P(12, 0)};
  const std::vector<Point> leftRoomGoals = {P(0, 0), P(1, 0), P(2, 0), P(3, 0)};
  std::vector<Errand> errands;
  for (std::size_t i = 0; i < 4; ++i) {
    errands.push_back({i, P(12 + static_cast<int>(i % 2), 1), rightRoomGoals[i]});
  }
  for (std::size_t i = 4; i < 8; ++i) {
    errands.push_back({i, P(static_cast<int>(i % 2), 1), leftRoomGoals[i - 4]});
  }
  Scenario scenario(w, starts, errands, ideal());
  const auto done = runToCompletion(scenario, 800);
  EXPECT_TRUE(allDone(done));
}

// Known limit: nothing sends an idle robot away. One that has delivered and is boxed in by robots
// that wait for its cell stays where it is, and the robots around it stay there too. The planner
// stays safe (runToCompletion audits every tick), and the progress monitor notices and says so.
TEST(CooperativePlanningTest, RobotsQueuingAroundAnIdleRobotOnTheirSharedGoalStaySafe) {
  const Warehouse w = layout({"....#.#.#.#....", "..............."});
  const std::vector<Point> starts = {P(0, 1),  P(1, 1),  P(0, 0),  P(2, 0),
                                     P(14, 1), P(13, 1), P(14, 0), P(12, 0)};
  std::vector<Errand> errands;
  for (std::size_t i = 0; i < 4; ++i) {
    errands.push_back({i, P(12 + static_cast<int>(i % 2), 1), P(14, 0)});
  }
  for (std::size_t i = 4; i < 8; ++i) {
    errands.push_back({i, P(static_cast<int>(i % 2), 1), P(0, 0)});
  }
  Scenario scenario(w, starts, errands, ideal());
  runToCompletion(scenario, 800);
  EXPECT_GE(scenario.sim.planningStats().prolongedBlocks, 1U);
}

// ---- intersections ------------------------------------------------------------------------------

//   Two one-cell-wide roads that cross at (4, 4).
Warehouse crossroads() {
  return layout({"####.####", "####.####", "####.####", "####.####", ".........", "####.####",
                 "####.####", "####.####", "####.####"});
}

TEST(CooperativePlanningTest, FourRobotsInTwoConvoysCrossAtAJunctionWithoutAnyoneStuckForLong) {
  // Two robots follow each other eastwards and two follow each other southwards: all four cross at
  // (4, 4), and in an order that interleaves them. Each convoy loads on its way in and delivers
  // beyond the junction, the leader deepest, so nobody has to come back out of a dead-end arm
  // (see AnIdleRobotPushedIntoADeadEndOntoAnotherRobotsGoalStaysSafe for that case).
  const std::vector<Job> jobs = {{P(1, 4), P(2, 4), P(8, 4)},
                                 {P(0, 4), P(1, 4), P(7, 4)},
                                 {P(4, 1), P(4, 2), P(4, 8)},
                                 {P(4, 0), P(4, 1), P(4, 7)}};
  for (const bool vehicle : {false, true}) {
    Scenario scenario(crossroads(), jobs, vehicle ? vehicles() : ideal());
    std::uint32_t worst = 0;
    const auto done = runToCompletion(scenario, 500, [&](const Simulation& sim) {
      for (const auto& robot : sim.snapshot().robots) {
        worst = std::max(worst, robot.stuckTicks);
      }
    });
    EXPECT_TRUE(allDone(done)) << (vehicle ? "vehicles" : "idealised");
    EXPECT_LT(worst, 20U) << (vehicle ? "vehicles" : "idealised");
    EXPECT_EQ(scenario.sim.planningStats().deadlocks, 0U);
  }
}

// Known limit, same cause: both convoys load at the end of a dead-end arm, so the robot that
// delivered nearer the junction is pushed ahead of the one behind it, into the dead end, onto that
// robot's pickup cell, and cannot get out. Safe, and noticed, but not solved.
TEST(CooperativePlanningTest, AnIdleRobotPushedIntoADeadEndOntoAnotherRobotsGoalStaysSafe) {
  const std::vector<Job> jobs = {{P(0, 4), P(8, 4), P(7, 4)},
                                 {P(1, 4), P(6, 4), P(5, 4)},
                                 {P(4, 0), P(4, 8), P(4, 7)},
                                 {P(4, 1), P(4, 6), P(4, 5)}};
  for (const bool vehicle : {false, true}) {
    Scenario scenario(crossroads(), jobs, vehicle ? vehicles() : ideal());
    runToCompletion(scenario, 500);
    EXPECT_GE(scenario.sim.planningStats().prolongedBlocks, 1U)
        << (vehicle ? "vehicles" : "idealised");
  }
}

TEST(CooperativePlanningTest, RobotsFromAllFourSidesCrossingTheMiddleOfAnOpenFloorAreAllServed) {
  // Four flows that all go through the 2x2 block in the middle of a 9x9 floor.
  const std::vector<Job> jobs = {{P(0, 4), P(8, 4), P(8, 3)},
                                 {P(8, 5), P(0, 5), P(0, 6)},
                                 {P(4, 0), P(4, 8), P(3, 8)},
                                 {P(5, 8), P(5, 0), P(6, 0)}};
  for (const bool vehicle : {false, true}) {
    Scenario scenario(openFloor(9, 9), jobs, vehicle ? vehicles() : ideal());
    const auto done = runToCompletion(scenario, 500);
    EXPECT_TRUE(allDone(done)) << (vehicle ? "vehicles" : "idealised");
    EXPECT_EQ(scenario.sim.planningStats().deadlocks, 0U);
  }
}

// ---- several destinations -----------------------------------------------------------------------

Warehouse shelfHall() {
  return layout({"..............", ".SSSS..SSSS...", ".SSSS..SSSS...", "..............",
                 ".SSSS..SSSS...", ".SSSS..SSSS...", "..............", "D....D....D..."});
}

std::vector<Errand> manyDestinations() {
  const std::vector<Point> pickups = {P(0, 2),  P(5, 1),  P(6, 4), P(11, 3), P(0, 5),  P(5, 5),
                                      P(12, 2), P(13, 5), P(5, 3), P(0, 3),  P(12, 4), P(6, 0)};
  const std::vector<Point> dropoffs = {P(0, 7), P(5, 7), P(10, 7)};
  std::vector<Errand> errands;
  for (std::size_t i = 0; i < pickups.size(); ++i) {
    errands.push_back({i % 4, pickups[i], dropoffs[(i * 2 + i / 4) % 3]});
  }
  return errands;
}

TEST(CooperativePlanningTest, FourRobotsDeliveringToThreeDifferentDropoffsAllFinishTheirErrands) {
  const std::vector<Point> starts = {P(0, 0), P(4, 0), P(8, 0), P(12, 0)};
  Scenario scenario(shelfHall(), starts, manyDestinations(), vehicles());
  const auto done = runToCompletion(scenario, 1500);
  EXPECT_TRUE(allDone(done));
  std::uint64_t deliveries = 0;
  for (const auto& robot : scenario.sim.robots()) {
    deliveries += robot.deliveries();
    EXPECT_EQ(robot.deliveries(), 3U);
  }
  EXPECT_EQ(deliveries, 12U);
}

TEST(CooperativePlanningTest, TheSameErrandsAreDoneNoSlowerThanWithTheTableAlone) {
  const std::vector<Point> starts = {P(0, 0), P(4, 0), P(8, 0), P(12, 0)};
  std::uint64_t planned = 0;
  std::uint64_t plain = 0;
  for (const bool cooperative : {true, false}) {
    SimulationConfig config = vehicles();
    config.cooperative = cooperative;
    Scenario scenario(shelfHall(), starts, manyDestinations(), config);
    const auto done = runToCompletion(scenario, 1500);
    ASSERT_TRUE(allDone(done));
    (cooperative ? planned : plain) = latest(done);
  }
  EXPECT_LE(planned, plain + plain / 20);  // within 5%, in practice faster
}

// ---- conflicting priorities ---------------------------------------------------------------------

TEST(CooperativePlanningTest,
     WhenTwoRobotsWantTheSameCellsTheLowerIdGoesFirstWhicheverWayRoundTheyAre) {
  // The same two journeys with the ids swapped: whoever has the lower id is never held up.
  const Job eastbound{P(0, 1), P(13, 0), P(14, 0)};
  const Job westbound{P(14, 2), P(1, 0), P(0, 0)};
  for (const bool eastFirst : {true, false}) {
    const std::vector<Job> jobs =
        eastFirst ? std::vector<Job>{eastbound, westbound} : std::vector<Job>{westbound, eastbound};
    const std::uint64_t alone = soloTime(hall(), jobs, 0, ideal());
    Scenario scenario(hall(), jobs, ideal());
    const auto done = runToCompletion(scenario, 400);
    EXPECT_EQ(done[0], alone) << (eastFirst ? "east first" : "west first");
    EXPECT_GT(done[1], 0U);
  }
}

TEST(CooperativePlanningTest, ARobotWithPriorityWaitsForALowerOneLoadingInItsWayAndNeverMovesIt) {
  // Robot 2 loads for a long time at (6, 1), in the corridor; robot 1 needs to get through.
  SimulationConfig config = ideal();
  config.robot.loadTicks = 15;
  const std::vector<Point> starts = {P(0, 1), P(5, 1)};
  const std::vector<Errand> errands = {{1, P(6, 1), P(7, 1)}, {0, P(13, 1), P(13, 2)}};
  Scenario scenario(hall(), starts, errands, config);
  bool movedWhileLoading = false;
  Point loadingAt = P(6, 1);
  const auto done = runToCompletion(scenario, 400, [&](const Simulation& sim) {
    const auto& loader = sim.robots()[1];
    if (loader.state() == RobotState::Loading) {
      movedWhileLoading = movedWhileLoading || loader.position() != loadingAt;
    }
  });
  EXPECT_TRUE(allDone(done));
  EXPECT_FALSE(movedWhileLoading);
  EXPECT_GE(done[1], 15U);  // robot 1 could not get by before the loading was done
}

// ---- replanning when the layout changes ---------------------------------------------------------

TEST(CooperativePlanningTest, AWallPutUpOnARobotsRouteSendsItRoundAndItStillGetsThere) {
  Simulation sim(openFloor(9, 5), {P(0, 2)}, ideal());
  const TaskId id = sim.submitTask(P(8, 2), P(8, 3));
  TrafficAudit audit(sim);
  for (int i = 0; i < 3; ++i) {
    sim.step();
    audit.check(sim);
  }
  ASSERT_TRUE(sim.placeObstacle(P(5, 2)));  // dead ahead
  EXPECT_EQ(sim.planningStats().obstacleRepairs, 1U);
  EXPECT_TRUE(std::find(sim.robots()[0].path().begin(), sim.robots()[0].path().end(), P(5, 2)) ==
              sim.robots()[0].path().end());
  for (int i = 0; i < 100 && sim.tasks().find(id)->status != TaskStatus::Completed; ++i) {
    sim.step();
    audit.check(sim);
    EXPECT_NE(sim.robots()[0].position(), P(5, 2));
  }
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

TEST(CooperativePlanningTest, AWallThatSealsTheWayMakesTheRobotGiveTheTaskBack) {
  Simulation sim(layout({"........."}), {P(0, 0)}, ideal());
  const TaskId id = sim.submitTask(P(8, 0), P(7, 0));
  sim.step();
  sim.step();
  ASSERT_EQ(sim.tasks().find(id)->status, TaskStatus::Assigned);
  ASSERT_TRUE(sim.placeObstacle(P(5, 0)));
  const auto& robot = sim.robots()[0];
  EXPECT_EQ(robot.state(), RobotState::Idle);  // it stood down at once
  EXPECT_FALSE(robot.task().has_value());
  EXPECT_NE(sim.tasks().find(id)->status, TaskStatus::Assigned);  // back in the queue
  sim.step();
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Failed);  // and rejected as impossible
  EXPECT_EQ(sim.planningStats().obstacleRepairs, 1U);
}

TEST(CooperativePlanningTest, AWallThatSealsTheWayToTheDropoffFailsTheTaskTheRobotIsCarrying) {
  SimulationConfig config = ideal();
  config.robot.loadTicks = 1;
  Simulation sim(layout({"........."}), {P(0, 0)}, config);
  const TaskId id = sim.submitTask(P(2, 0), P(8, 0));
  for (int i = 0; i < 20 && sim.robots()[0].goal() != warehousesim::robots::NavigationGoal::Dropoff;
       ++i) {
    sim.step();
  }
  ASSERT_EQ(sim.robots()[0].goal(), warehousesim::robots::NavigationGoal::Dropoff);
  ASSERT_TRUE(sim.placeObstacle(P(6, 0)));
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Failed);  // the cargo cannot be delivered
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
}

TEST(CooperativePlanningTest, TakingAWallDownOpensAShorterWayForTheNextTask) {
  // A wall with one gap at the far end: the way from (0, 0) to (0, 2) is long until the wall comes
  // down.
  Simulation sim(layout({".....", "####.", "....."}), {P(0, 0)}, ideal());
  const TaskId first = sim.submitTask(P(0, 2), P(1, 2));
  std::uint64_t slow = 0;
  for (int i = 0; i < 100 && slow == 0; ++i) {
    sim.step();
    slow = sim.tasks().find(first)->status == TaskStatus::Completed ? sim.tick() : 0;
  }
  ASSERT_GT(slow, 0U);
  ASSERT_TRUE(sim.clearObstacle(P(0, 1)));
  const TaskId second = sim.submitTask(P(0, 0), P(0, 2));
  const std::uint64_t began = sim.tick();
  std::uint64_t fast = 0;
  for (int i = 0; i < 100 && fast == 0; ++i) {
    sim.step();
    fast = sim.tasks().find(second)->status == TaskStatus::Completed ? sim.tick() - began : 0;
  }
  ASSERT_GT(fast, 0U);
  EXPECT_LT(fast, slow - 4);  // through the gap now in the wall, not round the end
}

TEST(CooperativePlanningTest, OpenTasksThatStartOrEndOnANewWallAreCancelled) {
  Simulation sim(openFloor(7, 3), {P(0, 0)}, ideal());
  const TaskId onPickup = sim.submitTask(P(3, 1), P(6, 2));
  const TaskId onDropoff = sim.submitTask(P(6, 0), P(3, 1));
  const TaskId elsewhere = sim.submitTask(P(5, 0), P(6, 2));
  ASSERT_TRUE(sim.placeObstacle(P(3, 1), CellType::Shelf));
  EXPECT_EQ(sim.tasks().find(onPickup)->status, TaskStatus::Cancelled);
  EXPECT_EQ(sim.tasks().find(onDropoff)->status, TaskStatus::Cancelled);
  EXPECT_EQ(sim.tasks().find(elsewhere)->status, TaskStatus::Pending);
}

TEST(CooperativePlanningTest, TheLayoutIsOnlyEditedWhereItIsSafeToDoSo) {
  Simulation sim(layout({"..D.", "...."}), {P(0, 0)}, ideal());
  EXPECT_FALSE(sim.placeObstacle(P(0, 0)));  // a robot stands there
  EXPECT_FALSE(sim.placeObstacle(P(2, 0)));  // a dropoff is not plain floor
  EXPECT_FALSE(sim.placeObstacle(P(9, 9)));  // off the grid
  EXPECT_TRUE(sim.placeObstacle(P(1, 1)));
  EXPECT_FALSE(sim.placeObstacle(P(1, 1)));  // already a wall
  EXPECT_FALSE(sim.clearObstacle(P(3, 0)));  // not an obstacle
  EXPECT_FALSE(sim.clearObstacle(P(-1, 0)));
  EXPECT_TRUE(sim.clearObstacle(P(1, 1)));
  EXPECT_THROW((void)sim.placeObstacle(P(3, 1), CellType::Free), std::invalid_argument);
  EXPECT_THROW((void)sim.placeObstacle(P(3, 1), CellType::Charging), std::invalid_argument);
}

TEST(CooperativePlanningTest, ARobotCannotHaveAWallPutUpOnACellItHasClaimedAhead) {
  Simulation sim(openFloor(9, 3), {P(0, 1)}, vehicles());
  (void)sim.submitTask(P(8, 1), P(8, 2));
  for (int i = 0; i < 4; ++i) {
    sim.step();
  }
  const auto holds = sim.world().holdsOf(R(1));
  ASSERT_GE(holds.size(), 2U);
  EXPECT_FALSE(sim.placeObstacle(holds.back()));
}

TEST(CooperativePlanningTest, GeneratedTasksNeverAskForACellThatHasBecomeAWall) {
  SimulationConfig config = vehicles();
  config.taskSpawnInterval = 3;
  config.seed = 5;
  Simulation sim(shelfHall(), {P(0, 0), P(4, 0)}, config);
  sim.run(50);
  for (int x = 0; x < 14; ++x) {
    (void)sim.placeObstacle(P(x, 6));  // wall off a whole row between the shelves and the dropoffs
  }
  sim.run(400);
  for (const auto& task : sim.tasks().all()) {
    if (task.createdAt > 60) {
      EXPECT_TRUE(sim.warehouse().isWalkable(task.pickup));
      EXPECT_TRUE(sim.warehouse().isWalkable(task.dropoff));
    }
  }
}

// ---- assignments changing -----------------------------------------------------------------------

TEST(CooperativePlanningTest, ATaskWithdrawnOnTheWayAndReplacedByOneElsewhereIsPickedUpFromThere) {
  Simulation sim(openFloor(12, 5), {P(0, 2), P(11, 2)}, vehicles());
  const TaskId first = sim.submitTask(P(10, 0), P(10, 4));
  TrafficAudit audit(sim);
  for (int i = 0; i < 6; ++i) {
    sim.step();
    audit.check(sim);
  }
  ASSERT_TRUE(sim.cancelTask(first));
  const TaskId second = sim.submitTask(P(2, 4), P(2, 0));
  for (int i = 0; i < 300 && sim.tasks().find(second)->status != TaskStatus::Completed; ++i) {
    sim.step();
    audit.check(sim);
  }
  EXPECT_EQ(sim.tasks().find(second)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.tasks().find(first)->status, TaskStatus::Cancelled);
}

TEST(CooperativePlanningTest, AnIdleRobotInTheWayIsMovedOffTheRouteNotAlongItIntoADeadEnd) {
  // A one-cell road with a side cell below (4, 1) and a dead end at its east end. Robot 2 is idle
  // at (4, 1); robot 1 wants to drive past it to the dead end. Pushed east, robot 2 would end up on
  // the very cell robot 1 is going to.
  const Warehouse road = layout({"#########", ".........", "####.####"});
  Simulation sim(road, {P(0, 1), P(4, 1)}, ideal());
  const TaskId id = sim.submitTask(P(8, 1), P(7, 1));
  TrafficAudit audit(sim);
  for (int i = 0; i < 80 && sim.tasks().find(id)->status != TaskStatus::Completed; ++i) {
    sim.step();
    audit.check(sim);
    if (sim.robots()[0].task().has_value()) {
      EXPECT_LE(sim.robots()[1].position().x, 4) << "tick " << sim.tick();
    }
  }
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

// ---- noticing blocking, and giving up -----------------------------------------------------------

SimulationConfig watchful() {
  SimulationConfig config = ideal();
  config.prolongedBlockTicks = 10;
  config.deadlockTicks = 5;
  config.abandonTicks = 40;
  return config;
}

TEST(CooperativePlanningTest, ARobotStuckBehindOneOutOfServiceIsCountedAndThenGivesItsTaskBack) {
  // Robot 2 is out of service in the middle of the corridor and there is no way round it.
  Simulation sim(hall(), {P(0, 1), P(6, 1)}, watchful());
  ASSERT_TRUE(sim.disableRobot(R(2)));
  const TaskId id = sim.submitTask(P(13, 1), P(13, 2));
  TrafficAudit audit(sim);
  std::uint32_t worst = 0;
  for (int i = 0; i < 70; ++i) {
    sim.step();
    audit.check(sim);
    worst = std::max(worst, sim.snapshot().robots[0].stuckTicks);
    if (i == 34) {  // stuck for 10 ticks and over, but not yet for the 40 after which it gives up
      EXPECT_EQ(sim.planningStats().prolongedBlocks, 1U);  // counted once, not at every tick
    }
  }
  EXPECT_GE(worst, 10U);
  EXPECT_GE(sim.planningStats().prolongedBlocks, 1U);
  EXPECT_GE(sim.planningStats().abandoned, 1U);
  EXPECT_NE(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.robots()[1].position(), P(6, 1));  // the robot out of service never moved
}

TEST(CooperativePlanningTest, OnceTheBlockerIsBackInServiceTheStuckRobotGetsThroughAndFinishes) {
  Simulation sim(hall(), {P(0, 1), P(6, 1)}, watchful());
  ASSERT_TRUE(sim.disableRobot(R(2)));
  const TaskId id = sim.submitTask(P(13, 1), P(13, 2));
  TrafficAudit audit(sim);
  for (int i = 0; i < 30; ++i) {
    sim.step();
    audit.check(sim);
  }
  ASSERT_NE(sim.tasks().find(id)->status, TaskStatus::Completed);
  ASSERT_TRUE(sim.restoreRobot(R(2)));
  for (int i = 0; i < 200 && sim.tasks().find(id)->status != TaskStatus::Completed; ++i) {
    sim.step();
    audit.check(sim);
  }
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

// ---- deadlocks ----------------------------------------------------------------------------------

// Four robots on a 2x2 block of an open floor, each needing the cell of the next one round the
// ring. The physical rules that normally sort this out are put out of the way (nothing waits to be
// resolved, no reservations), so that what is left is the detection and the recovery.
SimulationConfig ringConfig() {
  SimulationConfig config = ideal();
  config.reservations = false;
  config.maxBlockedTicks = 1000;
  config.deadlockTicks = 4;
  config.abandonTicks = 1000;
  return config;
}

std::vector<Errand> ringErrands() {
  // Robots at (1,1), (2,1), (2,2), (1,2): each fetches from the cell of the next.
  return {
      {0, P(2, 1), P(2, 0)}, {1, P(2, 2), P(3, 2)}, {2, P(1, 2), P(1, 3)}, {3, P(1, 1), P(0, 1)}};
}

TEST(CooperativePlanningTest, ARingOfRobotsWaitingForEachOthersCellsIsDetectedAndBrokenUp) {
  const std::vector<Point> starts = {P(1, 1), P(2, 1), P(2, 2), P(1, 2)};
  Scenario scenario(openFloor(4, 4), starts, ringErrands(), ringConfig());
  const auto done = runToCompletion(scenario, 200);
  EXPECT_TRUE(allDone(done));
  const PlanningStats& stats = scenario.sim.planningStats();
  EXPECT_GE(stats.deadlocks, 1U);  // moving one robot aside can leave two others face to face
  EXPECT_GE(stats.deadlocksRecovered, 1U);
  EXPECT_EQ(stats.deadlocksUnresolved, 0U);
}

TEST(CooperativePlanningTest, TheRobotWithTheLowestPriorityIsTheFirstOneMovedAsideToBreakARing) {
  const std::vector<Point> starts = {P(1, 1), P(2, 1), P(2, 2), P(1, 2)};
  Scenario scenario(openFloor(4, 4), starts, ringErrands(), ringConfig());
  for (int i = 0; i < 50 && scenario.sim.planningStats().deadlocksRecovered == 0; ++i) {
    scenario.sim.step();
  }
  ASSERT_EQ(scenario.sim.planningStats().deadlocksRecovered, 1U);
  EXPECT_NE(scenario.sim.robots()[3].position(), starts[3]);  // robot 4: the highest id
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(scenario.sim.robots()[i].position(), starts[i]) << "robot " << i + 1;
  }
}

TEST(CooperativePlanningTest, ARingThatStaysIsCountedOnceHoweverLongItStays) {
  const Warehouse room = layout({"####", "#..#", "#..#", "####"});
  const std::vector<Point> starts = {P(1, 1), P(2, 1), P(2, 2), P(1, 2)};
  const std::vector<Errand> errands = {
      {0, P(2, 1), P(1, 1)}, {1, P(2, 2), P(2, 1)}, {2, P(1, 2), P(2, 2)}, {3, P(1, 1), P(1, 2)}};
  Scenario scenario(room, starts, errands, ringConfig());
  (void)runToCompletion(scenario, 30);  // nobody has room to move, so it is the same ring all along
  EXPECT_EQ(scenario.sim.planningStats().deadlocks, 1U);
  EXPECT_EQ(scenario.sim.planningStats().deadlocksUnresolved, 1U);
}

TEST(CooperativePlanningTest, WithoutRecoveryTheSameRingWouldStayForEver) {
  // The same ring, but the recovery is out of reach: the detection threshold is longer than the
  // run.
  SimulationConfig config = ringConfig();
  config.deadlockTicks = 1000;
  const std::vector<Point> starts = {P(1, 1), P(2, 1), P(2, 2), P(1, 2)};
  Scenario scenario(openFloor(4, 4), starts, ringErrands(), config);
  const auto done = runToCompletion(scenario, 100);
  EXPECT_FALSE(std::any_of(done.begin(), done.end(), [](std::uint64_t t) { return t > 0; }));
  EXPECT_EQ(scenario.sim.planningStats().deadlocks, 0U);
}

TEST(CooperativePlanningTest, ARingWithNoRoomToMoveAtAllIsReportedAsUnresolvedAndNobodyIsMoved) {
  // The ring fills a walled 2x2 room, so nobody has a free neighbour: no local move can help.
  const Warehouse room = layout({"####", "#..#", "#..#", "####"});
  const std::vector<Point> starts = {P(1, 1), P(2, 1), P(2, 2), P(1, 2)};
  const std::vector<Errand> errands = {
      {0, P(2, 1), P(1, 1)}, {1, P(2, 2), P(2, 1)}, {2, P(1, 2), P(2, 2)}, {3, P(1, 1), P(1, 2)}};
  SimulationConfig config = ringConfig();
  config.abandonTicks = 25;
  Scenario scenario(room, starts, errands, config);
  const auto done = runToCompletion(scenario, 60);
  EXPECT_FALSE(std::any_of(done.begin(), done.end(), [](std::uint64_t t) { return t > 0; }));
  const PlanningStats& stats = scenario.sim.planningStats();
  EXPECT_GE(stats.deadlocks, 1U);
  EXPECT_GE(stats.deadlocksUnresolved, 1U);
  EXPECT_EQ(stats.deadlocksRecovered, 0U);
  EXPECT_GE(stats.abandoned, 4U);  // in the end each one gives its task back
  for (std::size_t i = 0; i < starts.size(); ++i) {
    EXPECT_EQ(scenario.sim.robots()[i].position(), starts[i]);
  }
}

TEST(CooperativePlanningTest, TheDefaultRulesBreakTheSameRingBeforeAnyDetectionIsNeeded) {
  SimulationConfig config = ideal();
  const std::vector<Point> starts = {P(1, 1), P(2, 1), P(2, 2), P(1, 2)};
  Scenario scenario(openFloor(4, 4), starts, ringErrands(), config);
  const auto done = runToCompletion(scenario, 200);
  EXPECT_TRUE(allDone(done));
  EXPECT_EQ(scenario.sim.planningStats().deadlocksUnresolved, 0U);
}

// ---- the limit on planning work per step --------------------------------------------------------

// The demo hall with four robots and a task every five ticks.
Simulation demoHall(SimulationConfig config, std::uint64_t seed = 1) {
  config.seed = seed;
  config.taskSpawnInterval = 5;
  config.varyTaskPriority = true;
  return Simulation(warehousesim::world::makeDemoWarehouse(), {P(0, 0), P(2, 0), P(4, 0), P(6, 0)},
                    config);
}

TEST(CooperativePlanningTest,
     NoStepSpendsMoreSearchWorkThanItsLimitAndTheRobotsPutOffStillGetServed) {
  SimulationConfig config = vehicles();
  config.planningWorkPerSearch = 120;
  config.planningWorkPerStep = 200;
  Simulation sim = demoHall(config);
  TrafficAudit audit(sim);
  std::uint64_t before = 0;
  std::uint32_t worst = 0;
  for (int tick = 0; tick < 1500; ++tick) {
    sim.step();
    audit.check(sim);
    const std::uint64_t now = sim.planningStats().work;
    ASSERT_LE(now - before, config.planningWorkPerStep) << "tick " << sim.tick();
    before = now;
    for (const auto& robot : sim.snapshot().robots) {
      worst = std::max(worst, robot.stuckTicks);
    }
  }
  const PlanningStats& stats = sim.planningStats();
  EXPECT_GT(stats.searches, 50U);
  EXPECT_GT(stats.deferred + stats.outOfBudget, 0U);  // the limit really did bite
  EXPECT_LE(stats.peakStepWork, config.planningWorkPerStep + 400);
  EXPECT_LT(worst, 40U);  // nobody was left waiting for its turn
  for (const auto& robot : sim.robots()) {
    EXPECT_GT(robot.deliveries(), 0U);
  }
}

TEST(CooperativePlanningTest, SearchesThatCannotFinishInTheirWorkLeaveTheRobotsSafeAndWorking) {
  SimulationConfig config = vehicles();
  config.planningWorkPerSearch = 8;
  config.planningWorkPerStep = 40;
  Simulation sim = demoHall(config, 2);
  TrafficAudit audit(sim);
  for (int tick = 0; tick < 800; ++tick) {
    sim.step();
    audit.check(sim);
  }
  EXPECT_GT(sim.planningStats().outOfBudget, 0U);
  EXPECT_GT(sim.snapshot().completedTasks, 20U);  // the table alone keeps them going
  EXPECT_EQ(sim.snapshot().failedTasks, 0U);
}

// Six robots cross the two-room hall through its one-cell corridor and niche (vehicles on even
// seeds), 20 seeds, with the given work limits; what the planning cost and how it ended, summed up.
struct Crossing {
  std::vector<Point> starts;
  std::vector<Errand> errands;
};

Crossing crossingOf(std::uint64_t seed) {
  const std::vector<Point> left = {P(0, 0), P(1, 0), P(2, 0), P(0, 1), P(1, 1),
                                   P(2, 1), P(0, 2), P(1, 2), P(2, 2)};
  const std::vector<Point> right = {P(12, 0), P(13, 0), P(14, 0), P(12, 1), P(13, 1),
                                    P(14, 1), P(12, 2), P(13, 2), P(14, 2)};
  Rng rng(seed);
  std::vector<Point> a = left;
  std::vector<Point> b = right;
  for (std::size_t i = a.size(); i > 1; --i) {
    std::swap(a[i - 1], a[rng.below(i)]);
    std::swap(b[i - 1], b[rng.below(i)]);
  }
  // Robots 1-3 start on the left and go right, robots 4-6 the other way; the first three of each
  // shuffled room are starts, the next three targets.
  Crossing crossing;
  for (std::size_t i = 0; i < 3; ++i) {
    crossing.starts.push_back(a[i]);
    crossing.errands.push_back({i, b[3 + i], b[6 + i]});
  }
  for (std::size_t i = 0; i < 3; ++i) {
    crossing.starts.push_back(b[i]);
    crossing.errands.push_back({3 + i, a[3 + i], a[6 + i]});
  }
  return crossing;
}

struct Tally {
  int notDone = 0;
  std::uint64_t worstStep = 0;  // the most search work spent in any one step
  std::uint64_t outOfBudget = 0;
  std::uint64_t deferred = 0;
  std::uint64_t dodges = 0;
};

Tally crossingsWithLimits(std::size_t perSearch, std::size_t perStep, std::uint32_t prolonged) {
  Tally tally;
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const Crossing crossing = crossingOf(seed);
    SimulationConfig config = seed % 2 == 0 ? vehicles() : ideal();
    config.planningWorkPerSearch = perSearch;
    config.planningWorkPerStep = perStep;
    config.prolongedBlockTicks = prolonged;
    Scenario scenario(hall(), crossing.starts, crossing.errands, config);
    std::uint64_t before = 0;
    const auto done = runToCompletion(scenario, 1500, [&](const Simulation& sim) {
      tally.worstStep = std::max(tally.worstStep, sim.planningStats().work - before);
      before = sim.planningStats().work;
    });
    tally.notDone += allDone(done) ? 0 : 1;
    const PlanningStats& stats = scenario.sim.planningStats();
    tally.outOfBudget += stats.outOfBudget;
    tally.deferred += stats.deferred;
    tally.dodges += stats.dodges;
  }
  return tally;
}

TEST(CooperativePlanningTest,
     SearchesInTheSameStepShareTheWorkOfTheStepAndStillGetEverybodyThrough) {
  // Room for one search and a half in a step: the rest are cut short or put off to the next step.
  const Tally tally = crossingsWithLimits(100, 150, 20);
  EXPECT_EQ(tally.notDone, 0);
  EXPECT_LE(tally.worstStep, 250U);  // a search can overshoot its share by a little, not by a share
  EXPECT_GT(tally.deferred, 200U);   // 269; 122 if robots put off earlier got no share of their own
  EXPECT_GT(tally.outOfBudget, 0U);
}

// A search that ran out of work is tried again with twice as much, up to eight times the first
// allowance, so that a robot that needs more than the usual is not put off for ever.
TEST(CooperativePlanningTest, ASearchThatRanOutOfWorkIsRepeatedWithMoreAndSoGetsThere) {
  const Tally tally = crossingsWithLimits(70, 4000, 100000);  // no extra allowance for stuck robots
  EXPECT_EQ(tally.notDone, 0);
  EXPECT_GT(tally.outOfBudget, 0U);
  EXPECT_GE(tally.dodges, 100U);       // 125 when a search that failed is repeated with more
  EXPECT_LT(tally.outOfBudget, 600U);  // 327; 1091 if it were repeated with the same allowance
}

TEST(CooperativePlanningTest, ARobotThatHasBeenStuckForLongMayUseTwiceTheWork) {
  const Tally patient = crossingsWithLimits(70, 4000, 100000);
  const Tally impatient = crossingsWithLimits(70, 4000, 1);  // stuck for one tick is already long
  EXPECT_LT(impatient.outOfBudget, patient.outOfBudget);
}

TEST(CooperativePlanningTest, TheWorkLimitsMustBeAtLeastOne) {
  SimulationConfig perStep = vehicles();
  perStep.planningWorkPerStep = 0;
  EXPECT_THROW(Simulation(openFloor(3, 3), {P(0, 0)}, perStep), std::invalid_argument);
  SimulationConfig perSearch = vehicles();
  perSearch.planningWorkPerSearch = 0;
  EXPECT_THROW(Simulation(openFloor(3, 3), {P(0, 0)}, perSearch), std::invalid_argument);
}

TEST(CooperativePlanningTest, TheSameSimulationRunTwiceDecidesTheSameThingsEveryTick) {
  for (const std::size_t perStep : {std::size_t{4000}, std::size_t{200}}) {
    SimulationConfig config = vehicles();
    config.planningWorkPerStep = perStep;
    Simulation a = demoHall(config, 3);
    Simulation b = demoHall(config, 3);
    for (int tick = 0; tick < 400; ++tick) {
      a.step();
      b.step();
      ASSERT_TRUE(a.snapshot() == b.snapshot()) << "work limit " << perStep << " tick " << tick;
    }
    EXPECT_EQ(warehousesim::simulation::toJson(a.snapshot()).dump(),
              warehousesim::simulation::toJson(b.snapshot()).dump());
  }
}

TEST(CooperativePlanningTest, TheSnapshotAndJsonReportThePlanningCounts) {
  Scenario scenario(hall(), meetingInTheCorridor(), ideal());
  (void)runToCompletion(scenario, 400);
  const auto snapshot = scenario.sim.snapshot();
  EXPECT_TRUE(snapshot.planning == scenario.sim.planningStats());
  EXPECT_EQ(snapshot.replans, snapshot.planning.reroutes);
  const auto json = warehousesim::simulation::toJson(snapshot);
  EXPECT_EQ(json["planning"]["reroutes"].get<std::uint64_t>(), snapshot.planning.reroutes);
  EXPECT_EQ(json["planning"]["dodges"].get<std::uint64_t>(), snapshot.planning.dodges);
  EXPECT_TRUE(json["planning"].contains("deadlocksUnresolved"));
  EXPECT_TRUE(json["robots"][0].contains("stuckTicks"));
}

// ---- solvable scenarios make progress -----------------------------------------------------------

TEST(CooperativePlanningPropertyTest,
     RandomCrossingsOfACorridorWithANicheAlwaysEndWithEveryTaskDone) {
  // Six robots in the two rooms of hall(), each sent to the opposite room: one-cell corridor, one
  // niche, so passing is possible but only just.
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const Crossing crossing = crossingOf(seed);
    Scenario scenario(hall(), crossing.starts, crossing.errands,
                      seed % 2 == 0 ? vehicles() : ideal());
    const auto done = runToCompletion(scenario, 900);
    EXPECT_TRUE(allDone(done)) << "seed " << seed;
    EXPECT_EQ(scenario.sim.planningStats().abandoned, 0U) << "seed " << seed;
  }
}

TEST(CooperativePlanningPropertyTest, NoBundledLayoutOrSeedEverCollidesWithCooperativePlanningOn) {
  for (const std::string name : {"demo", "compact", "fulfilment"}) {
    for (std::uint64_t seed = 1; seed <= 3; ++seed) {
      auto made = warehousesim::simulation::makeLayoutSimulation(name, seed);
      ASSERT_TRUE(made.has_value());
      TrafficAudit audit(*made);
      for (int tick = 0; tick < 600; ++tick) {
        made->step();
        audit.check(*made);
      }
      EXPECT_EQ(made->snapshot().failedTasks, 0U) << name << " seed " << seed;
      EXPECT_EQ(made->planningStats().deadlocksUnresolved, 0U) << name << " seed " << seed;
    }
  }
}

}  // namespace
