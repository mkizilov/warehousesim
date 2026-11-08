#include "warehousesim/traffic/cooperative_router.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/traffic/reservation_table.hpp"
#include "warehousesim/traffic/traffic_controller.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Direction;
using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::core::Tick;
using warehousesim::planning::AStarPlanner;
using warehousesim::planning::BlockedSet;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotPath;
using warehousesim::traffic::CooperativeRouter;
using warehousesim::traffic::CooperativeRouterOptions;
using warehousesim::traffic::Occupy;
using warehousesim::traffic::ReservationTable;
using warehousesim::traffic::RouteOutcome;
using warehousesim::traffic::RouteOutcomeKind;
using warehousesim::traffic::TableConstraints;
using warehousesim::traffic::TrafficController;
using warehousesim::traffic::Traverse;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

Warehouse openFloor(int width, int height) {
  return layout(std::vector<std::string>(static_cast<std::size_t>(height),
                                         std::string(static_cast<std::size_t>(width), '.')));
}

// The cells from `from` (excluded) to `to`, along x first and then along y.
RobotPath route(Point from, Point to) {
  RobotPath path;
  Point at = from;
  while (at.x != to.x) {
    at.x += at.x < to.x ? 1 : -1;
    path.push_back(at);
  }
  while (at.y != to.y) {
    at.y += at.y < to.y ? 1 : -1;
    path.push_back(at);
  }
  return path;
}

Robot robotOn(std::uint32_t id, Point start, const RobotPath& path,
              Direction heading = Direction::East) {
  Robot robot(R(id), start, {}, heading);
  if (!path.empty()) {
    robot.beginTask(TaskId{id}, path.back(), path);
  }
  return robot;
}

struct Harness {
  Warehouse floor;
  AStarPlanner planner;
  BlockedSet avoid;
  CooperativeRouter router;
  CooperativeRouterOptions options;

  explicit Harness(Warehouse w) : floor(std::move(w)) {}

  RouteOutcome reroute(const ReservationTable& table, const Robot& robot, Tick now = 0) {
    return router.reroute({floor, table, robot, now, planner, avoid, options});
  }
};

// A table in which robot 1 stands still on `cell` for the whole window.
ReservationTable tableWithParkedRobot(const Warehouse& floor, Point cell, std::size_t steps = 40) {
  ReservationTable table(floor);
  (void)table.reserve(R(1), Occupy{cell, {0, steps}});
  return table;
}

// ---- when there is nothing to do --------------------------------------------------------------

TEST(CooperativeRouterTest, ARobotThatIsNotDrivingOrHasNowhereToGoIsLeftAlone) {
  Harness h(openFloor(8, 3));
  const ReservationTable table(h.floor);
  EXPECT_EQ(h.reroute(table, robotOn(2, P(0, 0), RobotPath{})).kind, RouteOutcomeKind::Skipped);
  Robot idle(R(2), P(0, 0));
  EXPECT_EQ(h.reroute(table, idle).kind, RouteOutcomeKind::Skipped);
}

TEST(CooperativeRouterTest, AClearRouteIsKept) {
  Harness h(openFloor(8, 3));
  const ReservationTable table(h.floor);
  const RouteOutcome outcome = h.reroute(table, robotOn(2, P(0, 1), route(P(0, 1), P(6, 1))));
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::Kept);
  EXPECT_TRUE(outcome.path.empty());
  EXPECT_GT(outcome.work, 0U);
}

TEST(CooperativeRouterTest, ARouteThatOnlyHasToWaitAMomentIsKeptNotChangedForTheSakeOfIt) {
  Harness h(openFloor(8, 3));
  ReservationTable table(h.floor);
  (void)table.reserve(R(1), Occupy{P(3, 1), {2, 4}});  // crossing for two steps
  const Robot robot = robotOn(2, P(0, 1), route(P(0, 1), P(6, 1)));
  const RouteOutcome outcome = h.reroute(table, robot);
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::Kept);  // going round would take as long
}

TEST(CooperativeRouterTest, ALongerWaitIsNotWorthItWhenGoingRoundIsFaster) {
  Harness h(openFloor(8, 3));
  ReservationTable table(h.floor);
  (void)table.reserve(R(1),
                      Occupy{P(3, 1), {2, 5}});  // three steps: waiting costs 3, going round 2
  const Robot robot = robotOn(2, P(0, 1), route(P(0, 1), P(6, 1)));
  const RouteOutcome outcome = h.reroute(table, robot);
  ASSERT_EQ(outcome.kind, RouteOutcomeKind::Rerouted);
  EXPECT_EQ(outcome.waits, 0U);
  EXPECT_EQ(outcome.path.size(), 8U);
}

// ---- going round --------------------------------------------------------------------------------

TEST(CooperativeRouterTest, ARouteBlockedForGoodIsReplacedByOneAroundTheBlock) {
  Harness h(openFloor(9, 3));
  const ReservationTable table = tableWithParkedRobot(h.floor, P(3, 1));
  const Robot robot = robotOn(2, P(0, 1), route(P(0, 1), P(7, 1)));
  const RouteOutcome outcome = h.reroute(table, robot);
  ASSERT_EQ(outcome.kind, RouteOutcomeKind::Rerouted);
  EXPECT_FALSE(outcome.dodge);
  EXPECT_FALSE(outcome.partial);
  EXPECT_TRUE(std::find(outcome.path.begin(), outcome.path.end(), P(3, 1)) == outcome.path.end());
  EXPECT_EQ(outcome.path.back(), P(7, 1));
  EXPECT_EQ(outcome.path.size(), 9U);  // two longer than the 7 of the straight line
  // The robot accepts it as a route.
  Robot adopt = robot;
  EXPECT_NO_THROW(adopt.setPath(outcome.path));
}

TEST(CooperativeRouterTest, ADetourLongerThanTheAllowanceIsNotTakenAndTheRobotKeepsItsRoute) {
  Harness h(openFloor(9, 3));
  h.options.maxDetourCells = 1;
  const ReservationTable table = tableWithParkedRobot(h.floor, P(3, 1));
  const RouteOutcome outcome = h.reroute(table, robotOn(2, P(0, 1), route(P(0, 1), P(7, 1))));
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::TooLong);
  EXPECT_TRUE(outcome.path.empty());
}

TEST(CooperativeRouterTest, WithNoWayRoundTheRobotKeepsToItsRouteAndWaitsBeforeTheBlock) {
  Harness h(layout({"#########", ".........", "#########"}));
  const ReservationTable table = tableWithParkedRobot(h.floor, P(4, 1));
  const RouteOutcome outcome = h.reroute(table, robotOn(2, P(0, 1), route(P(0, 1), P(8, 1))));
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::Kept);
}

TEST(CooperativeRouterTest, ARouteBeyondTheWindowIsExtendedWithTheShortestWayOnTheEmptyMap) {
  Harness h(openFloor(30, 3));
  h.options.window = 6;
  const ReservationTable table = tableWithParkedRobot(h.floor, P(2, 1));
  const Robot robot = robotOn(2, P(0, 1), route(P(0, 1), P(25, 1)));
  const RouteOutcome outcome = h.reroute(table, robot);
  ASSERT_EQ(outcome.kind, RouteOutcomeKind::Rerouted);
  EXPECT_TRUE(outcome.partial);
  EXPECT_EQ(outcome.path.back(), P(25, 1));
  EXPECT_TRUE(std::find(outcome.path.begin(), outcome.path.end(), P(2, 1)) == outcome.path.end());
  // Every step of it is to a neighbour.
  Point at = P(0, 1);
  for (const Point cell : outcome.path) {
    EXPECT_EQ(warehousesim::core::manhattanDistance(at, cell), 1);
    at = cell;
  }
}

// ---- getting out of somebody's way --------------------------------------------------------------

//   #####.#     a one-cell corridor with a niche
//   .......
Warehouse corridorWithNiche() { return layout({"#####.#", "......."}); }

TEST(CooperativeRouterTest, ARobotInTheWayDodgesIntoTheNicheAndAllowsTheOtherToPass) {
  Harness h(corridorWithNiche());
  h.options.window = 16;
  TrafficController controller(h.floor, 16);
  const Robot east = robotOn(1, P(0, 1), route(P(0, 1), P(6, 1)));
  const Robot west = robotOn(2, P(6, 1), route(P(6, 1), P(0, 1)), Direction::West);
  controller.beginStep(0, std::vector<Robot>{east, west});
  ASSERT_FALSE(controller.plan(east).waiting());
  const auto before = controller.plan(west);
  ASSERT_TRUE(before.inTheWay);  // nowhere on its own route to wait

  const RouteOutcome outcome = h.reroute(controller.table(), west);
  ASSERT_EQ(outcome.kind, RouteOutcomeKind::Rerouted);
  EXPECT_TRUE(outcome.dodge);
  EXPECT_FALSE(outcome.partial);
  EXPECT_GE(outcome.waits, 3U);
  EXPECT_TRUE(std::find(outcome.path.begin(), outcome.path.end(), P(5, 0)) != outcome.path.end());
  EXPECT_EQ(outcome.path.back(), P(0, 1));

  Robot dodging = west;
  dodging.setPath(outcome.path);
  const auto after = controller.replan(dodging);
  EXPECT_FALSE(after.inTheWay);  // it has somewhere to wait now: in the niche
  ASSERT_TRUE(after.waiting());
  EXPECT_EQ(after.clearedCells, 2U);  // out of the corridor and into the niche
  EXPECT_EQ(controller.table().holder(P(5, 0), 8), R(2));
}

TEST(CooperativeRouterTest, WithoutANicheNothingBetterExistsAndTheRobotIsToldSo) {
  Harness h(layout({"#######", "......."}));
  h.options.window = 16;
  TrafficController controller(h.floor, 16);
  const Robot east = robotOn(1, P(0, 1), route(P(0, 1), P(6, 1)));
  const Robot west = robotOn(2, P(6, 1), route(P(6, 1), P(0, 1)), Direction::West);
  controller.beginStep(0, std::vector<Robot>{east, west});
  (void)controller.plan(east);
  (void)controller.plan(west);
  EXPECT_EQ(h.reroute(controller.table(), west).kind, RouteOutcomeKind::NoPlan);
}

TEST(CooperativeRouterTest, ACellsAlreadyGrantedAreNeverTakenBack) {
  Harness h(openFloor(9, 3));
  const ReservationTable table = tableWithParkedRobot(h.floor, P(5, 1));
  Robot robot = robotOn(2, P(0, 1), route(P(0, 1), P(8, 1)));
  robot.extendAuthority();  // (1, 1) is its to drive into
  robot.extendAuthority();  // and (2, 1)
  const RouteOutcome outcome = h.reroute(table, robot);
  ASSERT_EQ(outcome.kind, RouteOutcomeKind::Rerouted);
  ASSERT_GE(outcome.path.size(), 2U);
  EXPECT_EQ(outcome.path[0], P(1, 1));
  EXPECT_EQ(outcome.path[1], P(2, 1));
  EXPECT_TRUE(std::find(outcome.path.begin(), outcome.path.end(), P(5, 1)) == outcome.path.end());
}

// ---- budgets, determinism and the pieces --------------------------------------------------------

TEST(CooperativeRouterTest, ASearchThatRunsOutOfWorkSaysSoAndChangesNothing) {
  Harness h(openFloor(30, 6));
  h.options.maxWork = 5;
  const ReservationTable table = tableWithParkedRobot(h.floor, P(3, 2));
  const RouteOutcome outcome = h.reroute(table, robotOn(2, P(0, 2), route(P(0, 2), P(25, 2))));
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::OutOfBudget);
  EXPECT_TRUE(outcome.path.empty());
}

TEST(CooperativeRouterTest, TheSameRequestGivesTheSameOutcomeOnAReusedAndAFreshRouter) {
  Harness a(openFloor(10, 5));
  Harness b(openFloor(10, 5));
  const ReservationTable table = tableWithParkedRobot(a.floor, P(4, 2));
  const Robot robot = robotOn(2, P(0, 2), route(P(0, 2), P(9, 2)));
  const RouteOutcome first = a.reroute(table, robot);
  (void)a.reroute(table, robotOn(3, P(9, 0), route(P(9, 0), P(0, 4))));
  const RouteOutcome again = a.reroute(table, robot);
  const RouteOutcome other = b.reroute(table, robot);
  ASSERT_EQ(first.kind, RouteOutcomeKind::Rerouted);
  EXPECT_TRUE(first.path == again.path);
  EXPECT_TRUE(first.path == other.path);
  EXPECT_EQ(first.work, other.work);
}

TEST(CooperativeRouterTest, ARouteThatGetsThereAsSoonAsAnyOtherIsKeptEvenWhenItIsLonger) {
  Harness h(openFloor(9, 3));
  h.options.window = 16;
  ReservationTable table(h.floor);
  (void)table.reserve(R(1), Occupy{P(6, 1), {0, 10}});  // the goal is taken for ten steps
  // Eight cells round the top where six would do, but the robot has to wait for the goal either
  // way.
  const RobotPath longWay = {P(0, 0), P(1, 0), P(2, 0), P(3, 0),
                             P(4, 0), P(5, 0), P(6, 0), P(6, 1)};
  const RouteOutcome outcome = h.reroute(table, robotOn(2, P(0, 1), longWay));
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::Kept);
  EXPECT_TRUE(outcome.path.empty());
}

TEST(CooperativeRouterTest, TheTailBeyondTheWindowFallsBackWhenStandingRobotsBlockEveryWayRound) {
  Harness h(openFloor(30, 3));
  h.options.window = 6;
  for (int y = 0; y < 3; ++y) {
    h.avoid.insert(P(15, y));  // robots standing still right across the floor
  }
  const ReservationTable table = tableWithParkedRobot(h.floor, P(2, 1));
  const RouteOutcome outcome = h.reroute(table, robotOn(2, P(0, 1), route(P(0, 1), P(25, 1))));
  ASSERT_EQ(outcome.kind, RouteOutcomeKind::Rerouted);
  EXPECT_TRUE(outcome.partial);
  EXPECT_EQ(outcome.path.back(), P(25, 1));
}

TEST(CooperativeRouterTest, TheSearchStartsWhereTheGrantedCellsEndAndWhenTheRobotGetsThere) {
  Harness h(openFloor(9, 3));
  ReservationTable table(h.floor);
  // Robot 1 comes to stand on (3, 1) at step 2, for good. The robot below has been granted (1, 1)
  // and (2, 1) and is on the second of them at step 1 at the earliest: it can no longer get past
  // (3, 1). Planned from step 0 it would still seem to slip through in time.
  (void)table.reserve(R(1), Occupy{P(3, 1), {2, 40}});
  Robot robot = robotOn(2, P(0, 1), route(P(0, 1), P(7, 1)));
  robot.extendAuthority();
  robot.extendAuthority();
  const RouteOutcome outcome = h.reroute(table, robot);
  ASSERT_EQ(outcome.kind, RouteOutcomeKind::Rerouted);
  EXPECT_TRUE(std::find(outcome.path.begin(), outcome.path.end(), P(3, 1)) == outcome.path.end());
}

TEST(CooperativeRouterTest, WorkSpentOnTheFillAndTheFollowAlreadyUsesUpTheLimit) {
  Harness h(openFloor(5, 5));
  h.options.maxWork = 26;  // the fill of 25 cells just fits, and nothing after it does
  const ReservationTable table(h.floor);
  const RouteOutcome outcome = h.reroute(table, robotOn(2, P(0, 0), route(P(0, 0), P(4, 4))));
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::OutOfBudget);
  EXPECT_TRUE(outcome.path.empty());
}

TEST(CooperativeRouterTest, ARobotWhoseGrantedCellsEndAtItsDestinationNeedsNoSearchAtAll) {
  Harness h(openFloor(5, 3));
  const ReservationTable table(h.floor);
  Robot robot = robotOn(2, P(0, 1), route(P(0, 1), P(2, 1)));
  robot.extendAuthority();
  robot.extendAuthority();
  const RouteOutcome outcome = h.reroute(table, robot);
  EXPECT_EQ(outcome.kind, RouteOutcomeKind::Kept);
  EXPECT_EQ(outcome.work, 0U);
}

TEST(CooperativeRouterTest, OnlyTheRobotPlannedLastCanBePlannedAgain) {
  Harness h(corridorWithNiche());
  TrafficController controller(h.floor, 16);
  const Robot east = robotOn(1, P(0, 1), route(P(0, 1), P(6, 1)));
  const Robot west = robotOn(2, P(6, 1), route(P(6, 1), P(0, 1)), Direction::West);
  EXPECT_THROW((void)controller.replan(east), std::logic_error);  // before any step
  controller.beginStep(0, std::vector<Robot>{east, west});
  EXPECT_THROW((void)controller.replan(east), std::logic_error);  // nobody has planned yet
  (void)controller.plan(east);
  (void)controller.plan(west);
  EXPECT_THROW((void)controller.replan(east), std::logic_error);  // robot 2 was planned last
  EXPECT_NO_THROW((void)controller.replan(west));
}

TEST(CooperativeRouterTest, TableConstraintsLetARobotThroughItsOwnReservationsOnly) {
  const Warehouse floor = openFloor(5, 1);
  ReservationTable table(floor);
  (void)table.reserve(R(1), Occupy{P(2, 0), {3, 6}});
  const TableConstraints asOne(table, R(1));
  const TableConstraints asTwo(table, R(2));
  EXPECT_TRUE(asOne.cellFree(P(2, 0), 4));
  EXPECT_FALSE(asTwo.cellFree(P(2, 0), 4));
  EXPECT_TRUE(asTwo.cellFree(P(2, 0), 2));
  EXPECT_TRUE(asTwo.cellFree(P(2, 0), 6));
  EXPECT_TRUE(asTwo.edgeFree(P(1, 0), P(2, 0), 4));
}

TEST(FollowRouteTest, ARouteThatHasToWaitReachesItsEndLaterByTheWait) {
  const Warehouse floor = openFloor(7, 1);
  ReservationTable table(floor);
  (void)table.reserve(R(1), Occupy{P(3, 0), {2, 5}});
  const TableConstraints free(table, R(2));
  const std::vector<Point> rest = {P(1, 0), P(2, 0), P(3, 0), P(4, 0), P(5, 0)};
  const auto result =
      warehousesim::traffic::followRoute(free, P(0, 0), 0, rest, true, 12, [](Point) { return 0; });
  EXPECT_TRUE(result.feasible);
  EXPECT_TRUE(result.reaches);
  EXPECT_EQ(result.arrival, 8U);  // five moves and three steps of waiting
}

TEST(FollowRouteTest, ARouteThatEndsOnACellSomebodyNeedsLaterHasNotArrivedYet) {
  const Warehouse floor = openFloor(7, 1);
  ReservationTable table(floor);
  (void)table.reserve(R(1), Occupy{P(3, 0), {6, 10}});  // taken from step 6 to 9
  const TableConstraints free(table, R(2));
  const std::vector<Point> rest = {P(1, 0), P(2, 0), P(3, 0)};
  const auto result =
      warehousesim::traffic::followRoute(free, P(0, 0), 0, rest, true, 12, [](Point) { return 0; });
  EXPECT_TRUE(result.reaches);
  EXPECT_EQ(result.arrival,
            11U);  // not 3: it could not stay there, so it waits for the cell to clear
}

TEST(FollowRouteTest, ARouteThatWouldSwapCellsWithAnotherRobotWaitsOneStep) {
  const Warehouse floor = openFloor(4, 1);
  ReservationTable table(floor);
  (void)table.reserve(R(1), Occupy{P(2, 0), {0, 1}});
  (void)table.reserve(R(1),
                      Traverse{P(2, 0), P(1, 0), 1});  // robot 1 comes the other way in step 1
  const TableConstraints free(table, R(2));
  const auto result = warehousesim::traffic::followRoute(free, P(0, 0), 0, {P(1, 0), P(2, 0)}, true,
                                                         12, [](Point) { return 0; });
  EXPECT_TRUE(result.reaches);
  EXPECT_EQ(result.arrival, 3U);  // two moves and the step it has to wait before using the edge
}

TEST(FollowRouteTest, ARouteTheRobotCannotSurviveOnIsNotFeasible) {
  const Warehouse floor = openFloor(5, 1);
  ReservationTable table(floor);
  (void)table.reserve(R(1), Occupy{P(0, 0), {2, 100}});  // its own start is taken from step 2
  (void)table.reserve(R(1), Occupy{P(1, 0), {0, 100}});  // and the way on is blocked
  const TableConstraints free(table, R(2));
  const auto result = warehousesim::traffic::followRoute(free, P(0, 0), 0, {P(1, 0), P(2, 0)}, true,
                                                         12, [](Point) { return 2; });
  EXPECT_FALSE(result.feasible);
}

TEST(FollowRouteTest, AFarRouteReportsHowNearTheWindowLeavesTheRobot) {
  const Warehouse floor = openFloor(30, 1);
  const ReservationTable table(floor);
  const TableConstraints free(table, R(2));
  std::vector<Point> rest;
  for (int x = 1; x < 25; ++x) {
    rest.push_back(P(x, 0));
  }
  const auto result = warehousesim::traffic::followRoute(free, P(0, 0), 0, rest, false, 8,
                                                         [](Point cell) { return 24 - cell.x; });
  EXPECT_TRUE(result.feasible);
  EXPECT_FALSE(result.reaches);
  EXPECT_EQ(result.remaining, 16);
}

}  // namespace
