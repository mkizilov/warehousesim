#include "warehousesim/traffic/traffic_controller.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/robots/forecast.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/robots/robot_spec.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Direction;
using warehousesim::core::kPi;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotPath;
using warehousesim::robots::RobotSpec;
using warehousesim::traffic::ConflictKind;
using warehousesim::traffic::Occupy;
using warehousesim::traffic::Step;
using warehousesim::traffic::TrafficController;
using warehousesim::traffic::TrafficPlan;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }
TaskId T(std::uint32_t id) { return TaskId{id}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// An open floor of the given size.
Warehouse openFloor(int width, int height) {
  return layout(std::vector<std::string>(static_cast<std::size_t>(height),
                                         std::string(static_cast<std::size_t>(width), '.')));
}

RobotSpec ideal() { return RobotSpec{}; }

// The default horizon is short on purpose; tests that look at where a robot waits for a long time
// use a longer one.
constexpr std::size_t kLongHorizon = 40;

RobotSpec vehicle() {
  RobotSpec spec;
  spec.acceleration = 0.5;
  spec.deceleration = 1.0;
  spec.turnRate = kPi / 2.0;
  return spec;
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

// A robot with a journey (or, with an empty one, standing still).
Robot robotOn(std::uint32_t id, Point start, const RobotPath& path, const RobotSpec& spec = ideal(),
              Direction heading = Direction::East) {
  Robot robot(R(id), start, spec, heading);
  if (!path.empty()) {
    robot.beginTask(T(id), path.back(), path);
  }
  return robot;
}

// Plans every robot in id order, as the simulation does, and returns the plans in that order.
std::vector<TrafficPlan> planAll(TrafficController& controller, Step now,
                                 std::vector<Robot> fleet) {
  controller.beginStep(now, fleet);
  std::sort(fleet.begin(), fleet.end(),
            [](const Robot& a, const Robot& b) { return a.id() < b.id(); });
  std::vector<TrafficPlan> plans;
  for (const Robot& robot : fleet) {
    plans.push_back(controller.plan(robot));
  }
  return plans;
}

// ---- one robot, and the rules of the road -------------------------------------------------

TEST(TrafficControllerTest, ALoneRobotIsNeverHeldBackAndItsWholeJourneyIsOnTheTable) {
  const Warehouse floor = openFloor(12, 3);
  TrafficController controller(floor);
  const Robot robot = robotOn(1, P(0, 1), route(P(0, 1), P(6, 1)));
  const auto plans = planAll(controller, 10, {robot});

  ASSERT_EQ(plans.size(), 1U);
  EXPECT_EQ(plans[0].clearedCells, 6U);
  EXPECT_FALSE(plans[0].waiting());
  EXPECT_FALSE(plans[0].inTheWay);

  const auto& table = controller.table();
  EXPECT_EQ(table.holder(P(0, 1), 10), R(1));  // where it starts, now
  EXPECT_EQ(table.holder(P(1, 1), 10), R(1));  // and the cell it is entering
  EXPECT_EQ(table.holder(P(3, 1), 12), R(1));  // two steps on, three cells along
  EXPECT_FALSE(table.holder(P(3, 1), 10).has_value());
  EXPECT_FALSE(table.holder(P(0, 1), 12).has_value());  // long gone
  // It arrives and stays for the rest of the horizon.
  EXPECT_EQ(table.holder(P(6, 1), 16), R(1));
  EXPECT_EQ(table.holder(P(6, 1), 10 + controller.horizon() - 1), R(1));
  EXPECT_FALSE(table.holder(P(6, 1), 10 + controller.horizon()).has_value());
}

TEST(TrafficControllerTest, NothingCanBePlannedBeforeAStepHasBegun) {
  const Warehouse floor = openFloor(8, 3);
  TrafficController controller(floor);
  const Robot robot = robotOn(1, P(0, 0), route(P(0, 0), P(3, 0)));
  EXPECT_THROW((void)controller.plan(robot), std::logic_error);
}

TEST(TrafficControllerTest, PlanningOutOfOrderOrTwiceIsRefusedBeforeItCanGiveAnyoneAnAdvantage) {
  const Warehouse floor = openFloor(8, 3);
  TrafficController controller(floor);
  const Robot first = robotOn(1, P(0, 0), route(P(0, 0), P(3, 0)));
  const Robot second = robotOn(2, P(0, 2), route(P(0, 2), P(3, 2)));
  const std::vector<Robot> fleet = {first, second};

  controller.beginStep(0, fleet);
  (void)controller.plan(first);
  (void)controller.plan(second);
  EXPECT_THROW((void)controller.plan(first), std::logic_error);   // back to a higher priority
  EXPECT_THROW((void)controller.plan(second), std::logic_error);  // the same one again

  controller.beginStep(1, fleet);  // a new step starts the order afresh
  EXPECT_NO_THROW((void)controller.plan(first));
}

TEST(TrafficControllerTest, TwoRobotsOnTheSameCellIsAnImpossibleStateAndIsReportedAsOne) {
  const Warehouse floor = openFloor(4, 1);
  TrafficController controller(floor);
  EXPECT_THROW(
      controller.beginStep(0, std::vector<Robot>{Robot(R(1), P(1, 0)), Robot(R(2), P(1, 0))}),
      std::logic_error);
}

TEST(TrafficControllerTest, EveryStepStartsFromTheRobotsRealStateNotLastStepsReservations) {
  const Warehouse floor = openFloor(10, 1);
  TrafficController controller(floor);
  const Robot robot = robotOn(1, P(0, 0), route(P(0, 0), P(8, 0)));
  (void)planAll(controller, 0, {robot});
  const std::size_t reservedAfterPlanning = controller.table().occupancyCount();
  EXPECT_GT(reservedAfterPlanning, 4U);

  controller.beginStep(1, std::vector<Robot>{robot});  // nothing planned yet this step
  EXPECT_EQ(controller.table().occupancyCount(), 1U);  // only what the robot holds right now
  EXPECT_EQ(controller.table().holder(P(0, 0), 1), R(1));
  EXPECT_FALSE(controller.table().holder(P(5, 0), 6).has_value());
}

// ---- crossing paths -----------------------------------------------------------------------

//        . . . . N . . . .          R1 goes east along row 2, R2 south down column 4;
//        . . . . | . . . .          they would both be on (4, 2) in steps 1 and 2.
//        . . W - + - E . .
TEST(TrafficControllerTest,
     OfTwoRobotsCrossingTheOneWithTheLowerIdGoesFirstAndTheOtherWaitsBefore) {
  const Warehouse floor = openFloor(10, 8);
  TrafficController controller(floor);
  const Robot east = robotOn(1, P(2, 2), route(P(2, 2), P(8, 2)));
  const Robot south = robotOn(2, P(4, 0), route(P(4, 0), P(4, 6)), ideal(), Direction::South);
  const auto plans = planAll(controller, 0, {east, south});

  EXPECT_EQ(plans[0].clearedCells, 6U);  // nothing outranks robot 1
  EXPECT_FALSE(plans[0].waiting());

  ASSERT_TRUE(plans[1].waiting());
  EXPECT_EQ(plans[1].conflict->kind, ConflictKind::Vertex);
  EXPECT_EQ(plans[1].conflict->cell, P(4, 2));  // the intersection
  EXPECT_EQ(plans[1].conflict->step, 1U);
  EXPECT_EQ(plans[1].conflict->other, R(1));
  EXPECT_EQ(plans[1].clearedCells, 1U);  // it may enter (4, 1) and must wait there
  EXPECT_FALSE(plans[1].inTheWay);
  EXPECT_EQ(controller.table().holder(P(4, 1), 5), R(2));  // and it is waiting there
  EXPECT_FALSE(controller.table().holder(P(4, 2), 1) == R(2));
}

TEST(TrafficControllerTest, SwappingTheIdsSwapsWhoWaits) {
  const Warehouse floor = openFloor(10, 8);
  TrafficController controller(floor);
  const Robot south = robotOn(1, P(4, 0), route(P(4, 0), P(4, 6)), ideal(), Direction::South);
  const Robot east = robotOn(2, P(2, 2), route(P(2, 2), P(8, 2)));
  const auto plans = planAll(controller, 0, {south, east});
  EXPECT_FALSE(plans[0].waiting());  // now the southbound robot has the lower id
  ASSERT_TRUE(plans[1].waiting());
  EXPECT_EQ(plans[1].conflict->other, R(1));
  EXPECT_EQ(plans[1].conflict->cell, P(4, 2));
}

TEST(TrafficControllerTest, TheOrderTheFleetIsListedInDoesNotChangeWhoWins) {
  const Warehouse floor = openFloor(10, 8);
  const std::vector<Robot> fleet = {
      robotOn(3, P(0, 6), route(P(0, 6), P(9, 6))), robotOn(1, P(2, 2), route(P(2, 2), P(8, 2))),
      robotOn(2, P(4, 0), route(P(4, 0), P(4, 7)), ideal(), Direction::South)};
  std::vector<std::vector<TrafficPlan>> results;
  std::vector<std::vector<std::pair<Point, RobotId>>> tables;
  Rng rng(1);
  for (int shuffle = 0; shuffle < 8; ++shuffle) {
    std::vector<Robot> listed = fleet;
    for (std::size_t i = listed.size(); i > 1; --i) {
      std::swap(listed[i - 1], listed[rng.below(i)]);
    }
    TrafficController controller(floor);
    results.push_back(planAll(controller, 0, listed));
    tables.push_back(controller.table().occupiedAt(4));
  }
  for (std::size_t i = 1; i < results.size(); ++i) {
    ASSERT_EQ(results[i].size(), results[0].size());
    for (std::size_t r = 0; r < results[0].size(); ++r) {
      EXPECT_EQ(results[i][r].clearedCells, results[0][r].clearedCells);
      EXPECT_TRUE(results[i][r].conflict == results[0][r].conflict);
    }
    EXPECT_TRUE(tables[i] == tables[0]);
  }
}

TEST(TrafficControllerTest, ARobotThatGetsThroughFirstNeverHasToWait) {
  const Warehouse floor = openFloor(10, 8);
  TrafficController controller(floor);
  // Robot 1 reaches the crossing at (4, 2) in steps 3 and 4. Robot 2, which ranks lower, is
  // through it in steps 1 and 2: the table knows time, not just places.
  const Robot east = robotOn(1, P(0, 2), route(P(0, 2), P(8, 2)));
  const Robot south = robotOn(2, P(4, 0), route(P(4, 0), P(4, 6)), ideal(), Direction::South);
  const auto plans = planAll(controller, 0, {east, south});
  EXPECT_FALSE(plans[0].waiting());
  EXPECT_FALSE(plans[1].waiting());
  EXPECT_EQ(plans[1].clearedCells, 6U);
  EXPECT_EQ(controller.table().holder(P(4, 2), 1), R(2));
  EXPECT_EQ(controller.table().holder(P(4, 2), 3), R(1));
}

// ---- opposing traffic in a corridor ---------------------------------------------------------

TEST(TrafficControllerTest,
     InAOneWideCorridorTheRobotThatWouldMeetTheOtherHeadOnWaitsAtTheEntrance) {
  const Warehouse floor = openFloor(12, 1);
  TrafficController controller(floor, kLongHorizon);
  // Robot 1 drives east to (10, 0); robot 2 starts at (11, 0), outside, and wants to go west
  // through the same corridor. Every cell on its route is one robot 1 passes through or ends on.
  const Robot east = robotOn(1, P(0, 0), route(P(0, 0), P(10, 0)));
  const Robot west = robotOn(2, P(11, 0), route(P(11, 0), P(3, 0)), ideal(), Direction::West);
  const auto plans = planAll(controller, 0, {east, west});

  EXPECT_EQ(plans[0].clearedCells, 10U);
  ASSERT_TRUE(plans[1].waiting());
  EXPECT_EQ(plans[1].conflict->kind, ConflictKind::Swap);  // they would change places on an edge
  EXPECT_EQ(plans[1].conflict->cell, P(5, 0));
  EXPECT_EQ(plans[1].conflict->step, 5U);  // when they would meet between (5, 0) and (6, 0)
  EXPECT_EQ(plans[1].conflict->other, R(1));
  // So it waits where it is: outside, at the entrance, not in the corridor and not in the way.
  EXPECT_EQ(plans[1].clearedCells, 0U);
  EXPECT_FALSE(plans[1].inTheWay);
  EXPECT_EQ(controller.table().holder(P(11, 0), 20), R(2));
  EXPECT_EQ(controller.stats().swaps, 1U);
}

TEST(TrafficControllerTest, ARobotWaitsOnTheLastCellOfItsRouteThatIsOffTheOthersRoute) {
  const Warehouse floor = openFloor(12, 1);
  TrafficController controller(floor, kLongHorizon);
  // Robot 1 stops at (8, 0), so (9, 0) and (10, 0) on robot 2's route are free to wait on.
  const Robot east = robotOn(1, P(0, 0), route(P(0, 0), P(8, 0)));
  const Robot west = robotOn(2, P(11, 0), route(P(11, 0), P(3, 0)), ideal(), Direction::West);
  const auto plans = planAll(controller, 0, {east, west});
  ASSERT_TRUE(plans[1].waiting());
  EXPECT_EQ(plans[1].conflict->kind, ConflictKind::Swap);
  EXPECT_EQ(plans[1].clearedCells, 2U);  // down to (9, 0), next to where robot 1 ends up
  EXPECT_EQ(controller.table().holder(P(9, 0), 25), R(2));
  EXPECT_FALSE(controller.table().holder(P(8, 0), 25) == R(2));
}

TEST(TrafficControllerTest, TheWaitingRobotSetsOffAgainAsSoonAsTheOtherHasPassed) {
  const Warehouse floor = openFloor(12, 1);
  TrafficController controller(floor, 24);
  Robot east = robotOn(1, P(0, 0), route(P(0, 0), P(8, 0)));
  const Robot west = robotOn(2, P(11, 0), route(P(11, 0), P(9, 0)), ideal(), Direction::West);
  // Robot 1 is far enough along that it will have left the corridor's far end by the time robot 2
  // gets there: no clash, no waiting.
  for (int i = 0; i < 6; ++i) {
    (void)warehousesim::robots::stepUnobstructed(east);
  }
  const auto plans = planAll(controller, 6, {east, west});
  EXPECT_FALSE(plans[1].waiting());
  EXPECT_EQ(plans[1].clearedCells, 2U);
}

// ---- intersections and simultaneous arrivals ----------------------------------------------------

//           . . N . .
//           . . | . .       Three robots head for the very same cell, (4, 4), from three sides.
//           W - X - E       All three would be there in step 3.
TEST(TrafficControllerTest, RobotsArrivingAtTheSameCellTogetherAreServedInIdOrderAndTheRestWait) {
  const Warehouse floor = openFloor(9, 9);
  TrafficController controller(floor, kLongHorizon);
  const Robot fromWest = robotOn(1, P(0, 4), route(P(0, 4), P(4, 4)));
  const Robot fromNorth = robotOn(2, P(4, 0), route(P(4, 0), P(4, 4)), ideal(), Direction::South);
  const Robot fromEast = robotOn(3, P(8, 4), route(P(8, 4), P(4, 4)), ideal(), Direction::West);
  const auto plans = planAll(controller, 0, {fromNorth, fromEast, fromWest});  // listed any old way

  EXPECT_FALSE(plans[0].waiting());
  EXPECT_EQ(plans[0].clearedCells, 4U);
  for (const std::size_t waiting : {1U, 2U}) {
    ASSERT_TRUE(plans[waiting].waiting());
    EXPECT_EQ(plans[waiting].conflict->kind, ConflictKind::Vertex);
    EXPECT_EQ(plans[waiting].conflict->cell, P(4, 4));
    EXPECT_EQ(plans[waiting].conflict->other, R(1));
    EXPECT_EQ(plans[waiting].clearedCells, 3U);  // one cell short of it
  }
  // They wait on different cells, so they are not in each other's way either.
  EXPECT_EQ(controller.table().holder(P(4, 3), 30), R(2));
  EXPECT_EQ(controller.table().holder(P(5, 4), 30), R(3));
  EXPECT_EQ(controller.table().holder(P(4, 4), 30), R(1));  // robot 1 arrives and stays
  EXPECT_EQ(controller.stats().vertex, 2U);
}

TEST(TrafficControllerTest, ARobotAlreadyOnTheCellOnTheRouteOfOneWithPriorityIsInTheWay) {
  const Warehouse floor = openFloor(12, 1);
  TrafficController controller(floor);
  // Robot 1 drives east along the corridor. Robot 2 is in it and heading the other way, so there is
  // nowhere along its own route it could wait that robot 1 does not pass through.
  const Robot east = robotOn(1, P(0, 0), route(P(0, 0), P(10, 0)));
  const Robot west = robotOn(2, P(4, 0), route(P(4, 0), P(2, 0)), ideal(), Direction::West);
  const auto plans = planAll(controller, 0, {east, west});
  ASSERT_TRUE(plans[1].waiting());
  EXPECT_TRUE(plans[1].inTheWay);
  EXPECT_EQ(plans[1].clearedCells, 0U);
  EXPECT_EQ(plans[1].conflict->other, R(1));
  EXPECT_EQ(controller.stats().inTheWay, 1U);
}

// ---- where a robot waits ------------------------------------------------------------------------

TEST(TrafficControllerTest, ARobotDoesNotWaitOnACellOfAHigherPriorityRouteEvenBeyondTheHorizon) {
  const Warehouse floor = openFloor(24, 3);
  // Robot 1 drives east along row 1, a long way. Robot 2 comes down onto row 1 from above and
  // would run west along it. The two meet around x = 9, which the table (six steps) can see; but
  // every cell robot 2 could stop on in row 1 is one robot 1 will drive through later, which it
  // cannot see. Robot 2 stays where it is, off the row.
  const Robot east = robotOn(1, P(4, 1), route(P(4, 1), P(22, 1)));
  RobotPath down = {P(14, 1)};
  for (const Point cell : route(P(14, 1), P(8, 1))) {
    down.push_back(cell);
  }
  const Robot west = robotOn(2, P(14, 0), down, ideal(), Direction::South);
  TrafficController controller(floor, 6);
  const auto plans = planAll(controller, 0, {east, west});
  ASSERT_TRUE(plans[1].waiting());
  EXPECT_FALSE(plans[1].inTheWay);
  EXPECT_EQ(plans[1].clearedCells, 0U);
  EXPECT_EQ(controller.table().holder(P(14, 0), 5), R(2));
}

TEST(TrafficControllerTest, TheSameRobotDoesWaitInTheRowWhenTheOtherRouteEndsBeforeIt) {
  const Warehouse floor = openFloor(24, 3);
  // Robot 1 stops at x = 12, so the cells beyond are nobody's route and robot 2 may use them.
  const Robot east = robotOn(1, P(4, 1), route(P(4, 1), P(12, 1)));
  RobotPath down = {P(14, 1)};
  for (const Point cell : route(P(14, 1), P(8, 1))) {
    down.push_back(cell);
  }
  const Robot west = robotOn(2, P(14, 0), down, ideal(), Direction::South);
  TrafficController controller(floor, 6);
  const auto plans = planAll(controller, 0, {east, west});
  ASSERT_TRUE(plans[1].waiting());
  EXPECT_GE(plans[1].clearedCells, 1U);
}

// ---- robots moved by something else -------------------------------------------------------------

TEST(TrafficControllerTest, ARobotMovedAfterTheStepBeganIsSeenWhereItIsNowByThoseWhoPlanAfter) {
  const Warehouse floor = openFloor(10, 2);
  const Robot driver = robotOn(1, P(0, 0), route(P(0, 0), P(8, 0)));
  const Robot idle = robotOn(2, P(3, 1), RobotPath{});

  TrafficController stale(floor);
  stale.beginStep(0, std::vector<Robot>{driver, idle});
  EXPECT_FALSE(stale.plan(driver).waiting());  // nobody is on row 0

  TrafficController told(floor);
  told.beginStep(0, std::vector<Robot>{driver, idle});
  told.relocate(robotOn(2, P(4, 0), RobotPath{}));  // pushed aside, onto robot 1's road
  EXPECT_EQ(told.table().holder(P(4, 0), 0), R(2));
  EXPECT_FALSE(told.table().holder(P(3, 1), 0).has_value());  // and no longer where it was
  const TrafficPlan plan = told.plan(driver);
  ASSERT_TRUE(plan.waiting());
  EXPECT_EQ(plan.conflict->cell, P(4, 0));
  EXPECT_EQ(plan.clearedCells, 3U);

  TrafficController early(floor);
  EXPECT_THROW(early.relocate(idle), std::logic_error);
}

TEST(TrafficControllerTest, ARobotMovedOntoACellAnotherHasReservedIsLeftToTheWorldNotRefused) {
  const Warehouse floor = openFloor(10, 2);
  TrafficController controller(floor);
  const Robot driver = robotOn(1, P(0, 0), route(P(0, 0), P(8, 0)));
  const Robot idle = robotOn(2, P(3, 1), RobotPath{});
  controller.beginStep(0, std::vector<Robot>{driver, idle});
  ASSERT_FALSE(controller.plan(driver).waiting());
  // Robot 2 is moved onto (1, 0), which robot 1 has just reserved for this very step, and the table
  // is not told. Planning it must not throw: it cannot be asked to wait and reserves nothing.
  const Robot moved = robotOn(2, P(1, 0), RobotPath{});
  TrafficPlan plan;
  EXPECT_NO_THROW(plan = controller.plan(moved));
  EXPECT_TRUE(plan.inTheWay);
  EXPECT_TRUE(plan.waiting());
  EXPECT_EQ(controller.table().holder(P(1, 0), 0), R(1));
  EXPECT_EQ(controller.stats().inTheWay, 1U);
}

// ---- cells that are blocked ---------------------------------------------------------------------

TEST(TrafficControllerTest, ARouteThroughAWallIsStoppedBeforeIt) {
  const Warehouse floor = layout({"..#.."});
  TrafficController controller(floor);
  const Robot robot = robotOn(1, P(0, 0), RobotPath{P(1, 0), P(2, 0), P(3, 0)});
  const auto plans = planAll(controller, 0, {robot});
  ASSERT_TRUE(plans[0].waiting());
  EXPECT_EQ(plans[0].conflict->kind, ConflictKind::Blocked);
  EXPECT_EQ(plans[0].conflict->cell, P(2, 0));
  EXPECT_FALSE(plans[0].conflict->other.has_value());
  EXPECT_EQ(plans[0].clearedCells, 1U);
  EXPECT_EQ(controller.stats().blocked, 1U);
  EXPECT_EQ(controller.table().holder(P(1, 0), 9), R(1));
}

TEST(TrafficControllerTest, ARobotStandingStillIsAnObstacleForEveryoneWhoseRouteGoesThroughIt) {
  const Warehouse floor = openFloor(10, 1);
  TrafficController controller(floor);
  const Robot driver = robotOn(1, P(0, 0), route(P(0, 0), P(9, 0)));
  const Robot parked = robotOn(2, P(5, 0), RobotPath{});  // loading, charging or just idle
  const auto plans = planAll(controller, 0, {driver, parked});
  ASSERT_TRUE(plans[0].waiting());
  EXPECT_EQ(plans[0].conflict->kind, ConflictKind::Vertex);
  EXPECT_EQ(plans[0].conflict->cell, P(5, 0));
  EXPECT_EQ(plans[0].conflict->other, R(2));
  EXPECT_EQ(plans[0].clearedCells, 4U);  // it drives up to the cell before the parked robot
  EXPECT_FALSE(plans[0].inTheWay);
  EXPECT_FALSE(plans[1].waiting());  // the parked robot is where it is, and nobody asks it to move
}

TEST(TrafficControllerTest, ConflictsBeyondTheHorizonAreNotSeenYet) {
  const Warehouse floor = openFloor(40, 1);
  const Robot driver = robotOn(1, P(0, 0), route(P(0, 0), P(35, 0)));
  const Robot parked = robotOn(2, P(30, 0), RobotPath{});
  TrafficController shortSighted(floor, 8);
  const auto near = planAll(shortSighted, 0, {driver, parked});
  EXPECT_FALSE(near[0].waiting());  // the parked robot is 30 steps away: not in the next 8
  TrafficController farSighted(floor, 48);
  const auto far = planAll(farSighted, 0, {driver, parked});
  ASSERT_TRUE(far[0].waiting());
  EXPECT_EQ(far[0].clearedCells, 29U);
  EXPECT_THROW(TrafficController(floor, 1), std::invalid_argument);
}

// ---- vehicles ---------------------------------------------------------------------------------

TEST(TrafficControllerTest, VehiclesThatNeedRoomToBrakeReserveTheCellsAheadOfThemToo) {
  const Warehouse floor = openFloor(16, 3);
  TrafficController controller(floor);
  const Robot first = robotOn(1, P(0, 1), route(P(0, 1), P(12, 1)), vehicle());
  const auto plans = planAll(controller, 0, {first});
  ASSERT_FALSE(plans[0].waiting());
  // At cruising speed it holds its own cell and up to three ahead; the table shows that.
  std::size_t widest = 0;
  for (Step step = 0; step < 20; ++step) {
    std::size_t held = 0;
    for (const auto& [cell, owner] : controller.table().occupiedAt(step)) {
      held += owner == R(1) ? 1U : 0U;
    }
    widest = std::max(widest, held);
  }
  EXPECT_GE(widest, 3U);
  EXPECT_LE(widest, 4U);
}

TEST(TrafficControllerTest, AVehicleThatWouldBeCaughtMidRouteIsHeldBackBeforeTheClash) {
  const Warehouse floor = openFloor(12, 6);
  int clashes = 0;
  int clear = 0;
  // Sweep robot 1's start so that it reaches the crossing at (6, 3) sooner or later than robot 2.
  for (int startX = 0; startX <= 5; ++startX) {
    TrafficController controller(floor);
    const Robot east = robotOn(1, P(startX, 3), route(P(startX, 3), P(11, 3)), vehicle());
    const Robot south = robotOn(2, P(6, 0), route(P(6, 0), P(6, 5)), vehicle(), Direction::South);
    const auto plans = planAll(controller, 0, {east, south});
    EXPECT_FALSE(plans[0].waiting()) << startX;  // robot 1 outranks it and has the row to itself
    if (plans[1].waiting()) {
      ++clashes;
      EXPECT_EQ(plans[1].conflict->cell, P(6, 3)) << startX;  // they only share the crossing
      EXPECT_EQ(plans[1].conflict->other, R(1)) << startX;
      EXPECT_LE(plans[1].clearedCells, 2U) << startX;  // stopped before (6, 3), the third cell
      EXPECT_FALSE(plans[1].inTheWay) << startX;
    } else {
      ++clear;
      EXPECT_EQ(plans[1].clearedCells, 5U) << startX;
    }
  }
  EXPECT_GE(clashes, 2);  // some timings collide...
  EXPECT_GE(clear, 1);    // ...and others do not
}

// ---- the plans are consistent, whatever the traffic -------------------------------------------

TEST(TrafficControllerPropertyTest, WhateverTheTrafficNoTwoRobotsEverHaveTheSameCellInTheSameStep) {
  const Warehouse floor = layout({"..........", "..........", "....##....", "..........",
                                  "..........", "....##....", "..........", ".........."});
  Rng rng(2026);
  int delayed = 0;
  int waits = 0;
  int inTheWay = 0;
  int planned = 0;
  for (int round = 0; round < 150; ++round) {
    std::vector<Robot> fleet;
    std::vector<Point> starts;
    const std::size_t count = 2 + rng.below(4);
    while (fleet.size() < count) {
      const Point start = P(static_cast<int>(rng.below(10)), static_cast<int>(rng.below(8)));
      if (!floor.isWalkable(start) ||
          std::find(starts.begin(), starts.end(), start) != starts.end()) {
        continue;
      }
      starts.push_back(start);
      Point target = start;
      do {
        target = P(static_cast<int>(rng.below(10)), static_cast<int>(rng.below(8)));
      } while (!floor.isWalkable(target));
      // x first then y can run into the pillars; that is fine, the table calls it blocked.
      const RobotPath path = route(start, target);
      const RobotSpec spec = rng.below(2) == 0 ? ideal() : vehicle();
      fleet.push_back(robotOn(static_cast<std::uint32_t>(fleet.size() + 1), start, path, spec,
                              static_cast<Direction>(rng.below(4))));
    }

    TrafficController controller(floor);
    const std::vector<TrafficPlan> plans = planAll(controller, 5, fleet);
    const auto& table = controller.table();

    // No cell has two holders in any step.
    for (Step step = 5; step < 5 + controller.horizon(); ++step) {
      const auto held = table.occupiedAt(step);
      for (std::size_t i = 1; i < held.size(); ++i) {
        ASSERT_FALSE(held[i - 1].first == held[i].first)
            << "round " << round << " step " << step << " cell " << held[i].first.x << ","
            << held[i].first.y;
      }
    }
    // What a robot is told it may do is exactly what is on the table for it: the forecast of the
    // route cut where it was held back is reserved in full.
    for (std::size_t r = 0; r < fleet.size(); ++r) {
      if (plans[r].inTheWay) {
        continue;  // only its facts are on the table
      }
      auto holds = warehousesim::robots::forecastHolds(
          fleet[r].withRouteCutTo(plans[r].clearedCells), controller.horizon());
      if (plans[r].delaySteps > 0) {
        // It reserved its whole route, set off after waiting where it is.
        const auto whole = warehousesim::robots::forecastHolds(fleet[r], controller.horizon());
        holds.assign(plans[r].delaySteps, std::vector<Point>{fleet[r].position()});
        holds.insert(holds.end(), whole.begin(),
                     whole.begin() +
                         static_cast<std::ptrdiff_t>(controller.horizon() - plans[r].delaySteps));
        ++delayed;
      }
      for (std::size_t s = 0; s < holds.size(); ++s) {
        for (const Point cell : holds[s]) {
          ASSERT_EQ(table.holder(cell, 5 + s), fleet[r].id())
              << "round " << round << " robot " << r + 1 << " step " << s;
        }
      }
    }
    for (const TrafficPlan& plan : plans) {
      waits += plan.waiting() ? 1 : 0;
      inTheWay += plan.inTheWay ? 1 : 0;
      ++planned;
    }
  }
  EXPECT_GE(delayed, 1);  // and some robots at rest reserved the delayed route
  EXPECT_GT(waits, 100);  // plenty of the rounds really did have clashes to settle
  EXPECT_GT(planned - waits, 100);
  EXPECT_GE(inTheWay, 1);
}

}  // namespace
