#include "warehousesim/robots/forecast.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/robots/robot_spec.hpp"

namespace {

using warehousesim::core::Direction;
using warehousesim::core::kPi;
using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::forecastHolds;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotPath;
using warehousesim::robots::RobotSpec;
using warehousesim::robots::RobotState;
using warehousesim::robots::stepUnobstructed;

using Holds = std::vector<Point>;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }
TaskId T(std::uint32_t id) { return TaskId{id}; }

RobotSpec ideal() { return RobotSpec{}; }

RobotSpec vehicle() {
  RobotSpec spec;
  spec.acceleration = 0.5;
  spec.deceleration = 1.0;
  spec.turnRate = kPi / 2.0;
  return spec;
}

RobotPath eastward(Point from, int count) {
  RobotPath path;
  for (int i = 1; i <= count; ++i) {
    path.push_back(P(from.x + i, from.y));
  }
  return path;
}

// Everything observable about a robot's motion, to show that forecasting leaves it alone.
struct Motion {
  Point position;
  double speed;
  double progress;
  double heading;
  std::size_t authority;
  std::size_t pathLength;
  friend bool operator==(const Motion&, const Motion&) = default;
};
Motion motionOf(const Robot& r) {
  return {r.position(),     r.speed(),     r.segmentProgress(),
          r.headingAngle(), r.authority(), r.path().size()};
}

TEST(ForecastTest, ARobotWithNoPathHoldsItsOwnCellStepAfterStep) {
  const Robot robot(R(1), P(3, 2));
  const auto forecast = forecastHolds(robot, 5);
  ASSERT_EQ(forecast.size(), 5U);
  for (const Holds& holds : forecast) {
    EXPECT_EQ(holds, (Holds{P(3, 2)}));
  }
  EXPECT_TRUE(forecastHolds(robot, 0).empty());
}

TEST(ForecastTest, AnIdealisedRobotHoldsTheCellItLeavesAndTheOneItEntersOnEachStep) {
  Robot robot(R(1), P(0, 0), ideal(), Direction::East);
  robot.beginTask(T(1), P(4, 0), eastward(P(0, 0), 4));
  const auto forecast = forecastHolds(robot, 7);
  EXPECT_EQ(forecast[0], (Holds{P(0, 0), P(1, 0)}));
  EXPECT_EQ(forecast[1], (Holds{P(1, 0), P(2, 0)}));
  EXPECT_EQ(forecast[2], (Holds{P(2, 0), P(3, 0)}));
  EXPECT_EQ(forecast[3], (Holds{P(3, 0), P(4, 0)}));
  EXPECT_EQ(forecast[4], (Holds{P(4, 0)}));  // arrived: it stays there
  EXPECT_EQ(forecast[5], (Holds{P(4, 0)}));
  EXPECT_EQ(forecast[6], (Holds{P(4, 0)}));
}

TEST(ForecastTest, AVehicleHoldsMoreCellsAheadOfItAsItSpeedsUpBecauseItNeedsRoomToStop) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  const auto forecast = forecastHolds(robot, 16);
  EXPECT_EQ(forecast[0], (Holds{P(0, 0), P(1, 0)}));  // from rest it needs only the next cell
  EXPECT_GE(forecast[1].size(), 3U);                  // moving, it needs room to brake in
  std::size_t widest = 0;
  for (const Holds& holds : forecast) {
    widest = std::max(widest, holds.size());
    // What a robot holds is a run of consecutive cells of its route.
    for (std::size_t i = 1; i < holds.size(); ++i) {
      EXPECT_EQ(holds[i], P(holds[0].x + static_cast<int>(i), 0));
    }
  }
  EXPECT_LE(widest, 4U);  // never more than the cell it is on and three ahead
  EXPECT_EQ(forecast.back(), (Holds{P(10, 0)}));  // it is there by step 15 and stays
}

TEST(ForecastTest, ForecastingNeverTouchesTheRobotItself) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  (void)stepUnobstructed(robot);
  (void)stepUnobstructed(robot);
  const Motion before = motionOf(robot);
  (void)forecastHolds(robot, 30);
  EXPECT_TRUE(motionOf(robot) == before);
}

TEST(ForecastTest, EachStepStartsWhereTheStepBeforeLeftTheRobot) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(6, 0), eastward(P(0, 0), 6));
  const auto forecast = forecastHolds(robot, 14);
  Robot walked = robot;
  for (std::size_t s = 0; s < forecast.size(); ++s) {
    EXPECT_EQ(forecast[s].front(), walked.position()) << "step " << s;
    if (walked.hasPath()) {
      (void)stepUnobstructed(walked);
    }
  }
  EXPECT_EQ(walked.position(), P(6, 0));
}

TEST(ForecastTest, AStepReportsTheCellsPassedAndTheCellsHeld) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  const auto first = stepUnobstructed(robot);  // from rest: half a cell on, one cell held
  EXPECT_EQ(first.passed, 0U);
  EXPECT_EQ(first.heldPrefix, 1U);
  const auto second = stepUnobstructed(robot);  // reaches the first cell at full speed
  EXPECT_EQ(second.passed, 1U);
  EXPECT_GE(second.heldPrefix, 2U);  // the cell passed, and the room to stop beyond it
  EXPECT_EQ(robot.position(), P(1, 0));
}

TEST(ForecastTest, ARouteCutShortMakesTheRobotStopAtTheEndOfIt) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));

  const Robot cut = robot.withRouteCutTo(3);
  EXPECT_EQ(cut.path().size(), 3U);
  EXPECT_EQ(cut.destination(), P(3, 0));
  EXPECT_EQ(robot.path().size(), 10U);  // the original is untouched

  const auto forecast = forecastHolds(cut, 20);
  EXPECT_EQ(forecast.back(), (Holds{P(3, 0)}));  // waits where the route now ends
  for (const Holds& holds : forecast) {
    EXPECT_LE(holds.back().x, 3);  // and never reached for anything beyond it
  }

  EXPECT_EQ(robot.withRouteCutTo(50).path().size(), 10U);  // longer than the route: no change
  const Robot stay = robot.withRouteCutTo(0);              // wait right here
  EXPECT_TRUE(stay.path().empty());
  EXPECT_EQ(forecastHolds(stay, 4).back(), (Holds{P(0, 0)}));
}

TEST(ForecastTest, ARouteIsNeverCutBelowTheCellsTheRobotAlreadyHolds) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  (void)stepUnobstructed(robot);
  (void)stepUnobstructed(robot);
  (void)stepUnobstructed(robot);
  ASSERT_GE(robot.authority(), 1U);  // moving, so it holds the cell it is driving into
  ASSERT_GT(robot.speed(), 0.0);
  for (const std::size_t asked : {0U, 1U}) {
    const Robot cut = robot.withRouteCutTo(asked);
    EXPECT_EQ(cut.path().size(), std::max<std::size_t>(asked, robot.authority()));
    EXPECT_EQ(cut.authority(), robot.authority());
    EXPECT_EQ(cut.inconsistency().value_or("consistent"), "consistent");
  }
}

// ---- the same forecast in reusable storage ------------------------------------------------------

Holds holdsOf(const warehousesim::robots::ForecastBuffer& buffer, std::size_t step) {
  const auto span = buffer.holdsAt(step);
  return Holds(span.begin(), span.end());
}

TEST(ForecastBufferTest, HoldsExactlyTheForecastThatForecastHoldsGivesForEveryKindOfRobotAndRoute) {
  for (const RobotSpec& spec : {ideal(), vehicle()}) {
    for (const int cells : {1, 2, 5, 12}) {
      for (const std::size_t steps : {0U, 1U, 6U, 16U}) {
        Robot robot(R(1), P(0, 0), spec, Direction::East);
        robot.beginTask(T(1), P(cells, 0), eastward(P(0, 0), cells));
        (void)stepUnobstructed(robot);  // already under way
        const auto want = forecastHolds(robot, steps);

        Robot walker = robot;
        warehousesim::robots::ForecastBuffer buffer;
        forecastHolds(walker, steps, buffer);
        ASSERT_EQ(buffer.steps(), steps);
        for (std::size_t s = 0; s < steps; ++s) {
          ASSERT_EQ(holdsOf(buffer, s), want[s]) << cells << " cells, step " << s;
        }
        EXPECT_EQ(buffer.toVectors(), want);
      }
    }
  }
}

TEST(ForecastBufferTest, ReusingTheBufferAndTheWalkerForAnotherRobotLeavesNothingOfTheFirst) {
  Robot longRoute(R(1), P(0, 0), vehicle(), Direction::East);
  longRoute.beginTask(T(1), P(12, 0), eastward(P(0, 0), 12));
  Robot shortRoute(R(2), P(4, 3), ideal(), Direction::East);
  shortRoute.beginTask(T(2), P(6, 3), eastward(P(4, 3), 2));
  const Robot idle(R(3), P(7, 7));

  Robot walker = longRoute;
  warehousesim::robots::ForecastBuffer buffer;
  forecastHolds(walker, 14, buffer);
  EXPECT_EQ(buffer.toVectors(), forecastHolds(longRoute, 14));

  walker = shortRoute;
  forecastHolds(walker, 5, buffer);
  EXPECT_EQ(buffer.steps(), 5U);
  EXPECT_EQ(buffer.toVectors(), forecastHolds(shortRoute, 5));

  walker = idle;
  forecastHolds(walker, 3, buffer);
  EXPECT_EQ(buffer.toVectors(), forecastHolds(idle, 3));

  forecastHolds(walker, 0, buffer);
  EXPECT_EQ(buffer.steps(), 0U);
  buffer.clear();
  EXPECT_EQ(buffer.steps(), 0U);
}

TEST(ForecastBufferTest, ForecastingAWalkerNeverTouchesTheRobotItWasCopiedFrom) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  (void)stepUnobstructed(robot);
  const Motion before = motionOf(robot);
  Robot walker = robot;
  warehousesim::robots::ForecastBuffer buffer;
  forecastHolds(walker, 30, buffer);
  EXPECT_TRUE(motionOf(robot) == before);
}

TEST(ForecastBufferTest, AssignRouteCutOfMakesAReusedRobotWhatWithRouteCutToMakesOfTheOriginal) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  (void)stepUnobstructed(robot);
  (void)stepUnobstructed(robot);
  const std::size_t routeLength = robot.path().size();

  Robot reused(R(9), P(5, 5));  // not even the same robot to begin with
  for (const std::size_t cut : {0U, 1U, 3U, 7U, 50U}) {
    reused.assignRouteCutOf(robot, cut);
    const Robot want = robot.withRouteCutTo(cut);
    EXPECT_EQ(reused.id(), want.id());
    EXPECT_EQ(reused.position(), want.position());
    EXPECT_EQ(reused.destination(), want.destination());
    EXPECT_EQ(reused.authority(), want.authority());
    EXPECT_TRUE(motionOf(reused) == motionOf(want));
    EXPECT_EQ((std::vector<Point>(reused.path().begin(), reused.path().end())),
              (std::vector<Point>(want.path().begin(), want.path().end())));
    EXPECT_EQ(forecastHolds(reused, 12), forecastHolds(want, 12));
    EXPECT_EQ(reused.inconsistency().value_or("consistent"), "consistent");
  }
  EXPECT_EQ(robot.path().size(), routeLength) << "the original was never touched";
}

TEST(ForecastTest, ARobotThatWasWithdrawnForecastsItsOwnStopWithoutAskingForMore) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  (void)stepUnobstructed(robot);  // a quarter of the way into the first cell
  robot.abortTask();
  ASSERT_EQ(robot.state(), RobotState::Idle);
  ASSERT_TRUE(robot.hasPath());
  const auto forecast = forecastHolds(robot, 6);
  for (const Holds& holds : forecast) {
    EXPECT_LE(holds.back().x, 1);  // it only ever holds the cell it was already driving into
  }
  EXPECT_EQ(forecast.back(), (Holds{P(1, 0)}));
}

}  // namespace
