#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/robots/robot_spec.hpp"
#include "warehousesim/robots/robot_state.hpp"

namespace {

using warehousesim::core::angleOf;
using warehousesim::core::Direction;
using warehousesim::core::kPi;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::core::Vec2;
using warehousesim::robots::InvalidTransition;
using warehousesim::robots::MotionPhase;
using warehousesim::robots::NavigationGoal;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotPath;
using warehousesim::robots::RobotSpec;
using warehousesim::robots::RobotState;

constexpr double kTolerance = 1e-9;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }
TaskId T(std::uint32_t id) { return TaskId{id}; }

// A vehicle: 1 cell/tick at most, 0.5 up and 1 down (stops within half a cell from top speed),
// a quarter turn per tick. A small battery with round numbers.
RobotSpec vehicle() {
  RobotSpec spec;
  spec.acceleration = 0.5;
  spec.deceleration = 1.0;
  spec.turnRate = kPi / 2.0;
  spec.batteryCapacity = 1000;
  spec.moveCostPerCell = 10;
  spec.handlingDrainPerTick = 3;
  spec.loadTicks = 2;
  spec.unloadTicks = 1;
  return spec;
}

RobotSpec ideal() {
  RobotSpec spec;
  spec.batteryCapacity = 1000;
  spec.moveCostPerCell = 10;
  return spec;
}

RobotPath eastward(Point from, int count) {
  RobotPath path;
  for (int i = 1; i <= count; ++i) {
    path.push_back(P(from.x + i, from.y));
  }
  return path;
}

// x first, then y: a legal path between any two cells of an open floor.
RobotPath lPath(Point from, Point to) {
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

void expectConsistent(const Robot& robot) {
  EXPECT_EQ(robot.inconsistency().value_or("consistent"), "consistent");
}

// One step of `dt` ticks the way the simulation takes it, with nobody else around: turn to face
// the next cell, be granted what the robot wants, drive, and use any time a stop left over.
// Returns the number of cells passed.
std::uint32_t tick(Robot& robot, double dt = 1.0) {
  std::uint32_t passed = 0;
  double left = dt;
  for (int round = 0; round < 8 && left > 0.0 && robot.hasPath(); ++round) {
    if (robot.state() == RobotState::Navigating) {
      left = robot.turnTowardNext(left);
      if (left <= 0.0) {
        (void)robot.advance(0.0);
        break;
      }
      while (robot.authority() < robot.wantedAuthority(left)) {
        robot.extendAuthority();
      }
    }
    passed += robot.advance(left);
    left = robot.unusedTime();
  }
  return passed;
}

// Like tick(), but the whole path is the robot's from the start, so that only the robot's own
// limits decide what it does.
void tickWithEverythingGranted(Robot& robot, double dt) {
  while (robot.authority() < robot.path().size()) {
    robot.extendAuthority();
  }
  double left = dt;
  for (int round = 0; round < 8 && left > 0.0 && robot.hasPath(); ++round) {
    left = robot.turnTowardNext(left);
    if (left <= 0.0) {
      (void)robot.advance(0.0);
      return;
    }
    (void)robot.advance(left);
    left = robot.unusedTime();
  }
}

void runUntilStopped(Robot& robot) {
  for (int i = 0; i < 200 && robot.hasPath(); ++i) {
    (void)tick(robot);
  }
}

// ---- a straight run -----------------------------------------------------------------------

// 10 cells: 2 ticks / 1 cell speeding up, 8.5 ticks at full speed, 1 tick / 0.5 cells braking.
TEST(RobotDriveTest, AStraightRunSpeedsUpCruisesAndBrakesToStopAtTheDestination) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));

  struct Expected {
    double speed;
    int cellsPassed;  // cells entered so far
    double x;         // where the centre is, along the run
    MotionPhase phase;
  };
  const std::vector<Expected> timeline = {
      {0.5, 0, 0.25, MotionPhase::Accelerating}, {1.0, 1, 1.0, MotionPhase::Accelerating},
      {1.0, 2, 2.0, MotionPhase::Cruising},      {1.0, 3, 3.0, MotionPhase::Cruising},
      {1.0, 4, 4.0, MotionPhase::Cruising},      {1.0, 5, 5.0, MotionPhase::Cruising},
      {1.0, 6, 6.0, MotionPhase::Cruising},      {1.0, 7, 7.0, MotionPhase::Cruising},
      {1.0, 8, 8.0, MotionPhase::Cruising},      {1.0, 9, 9.0, MotionPhase::Cruising},
      {0.5, 9, 9.875, MotionPhase::Braking},     {0.0, 10, 10.0, MotionPhase::Braking},
  };
  for (std::size_t i = 0; i < timeline.size(); ++i) {
    (void)tick(robot);
    const Expected& e = timeline[i];
    EXPECT_NEAR(robot.speed(), e.speed, kTolerance) << "tick " << i + 1;
    EXPECT_EQ(robot.cellsTravelled(), static_cast<std::uint64_t>(e.cellsPassed))
        << "tick " << i + 1;
    EXPECT_NEAR(robot.pose().x, 0.5 + e.x, kTolerance) << "tick " << i + 1;
    EXPECT_EQ(robot.motion(), e.phase) << "tick " << i + 1;
    expectConsistent(robot);
  }
  EXPECT_TRUE(robot.hasArrived());
  EXPECT_EQ(robot.position(), P(10, 0));
  EXPECT_DOUBLE_EQ(robot.pose().x, 10.5);  // at rest in the middle of its cell
  EXPECT_DOUBLE_EQ(robot.pose().y, 0.5);
  EXPECT_FALSE(robot.hasPath());
  EXPECT_EQ(robot.authority(), 0U);
  EXPECT_EQ(robot.brakingViolations(), 0U);
  EXPECT_THROW((void)robot.advance(1.0), InvalidTransition);  // nothing left to drive along
}

TEST(RobotDriveTest, TheLogicalCellIsTheLastOneEnteredWhileThePoseIsBetweenCells) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(6, 0), eastward(P(0, 0), 6));
  double lastX = robot.pose().x;
  for (int i = 0; i < 12 && robot.hasPath(); ++i) {
    (void)tick(robot);
    const Vec2 pose = robot.pose();
    EXPECT_DOUBLE_EQ(pose.y, 0.5);  // straight along the row
    EXPECT_NEAR(pose.x, robot.position().x + 0.5 + robot.segmentProgress(), kTolerance);
    EXPECT_GE(robot.segmentProgress(), 0.0);
    EXPECT_LT(robot.segmentProgress(), 1.0);
    EXPECT_GE(pose.x, lastX);  // never backwards
    EXPECT_LE(pose.x - lastX, robot.maxSpeed() + kTolerance);
    lastX = pose.x;
  }
  EXPECT_EQ(robot.position(), P(6, 0));
}

TEST(RobotDriveTest, TheIdealisedRobotCrossesACellPerTickAtOnceAndNeverSlowsForCorners) {
  Robot robot(R(1), P(0, 0), ideal(), Direction::East);
  robot.beginTask(T(1), P(3, 2), RobotPath{P(1, 0), P(2, 0), P(3, 0), P(3, 1), P(3, 2)});
  std::vector<double> speeds;
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(tick(robot), 1U);
    speeds.push_back(robot.speed());
  }
  EXPECT_EQ(speeds,
            (std::vector<double>{1.0, 1.0, 1.0, 1.0, 0.0}));  // through the corner at (3, 0)
  EXPECT_EQ(robot.position(), P(3, 2));
}

// ---- authority ------------------------------------------------------------------------------

TEST(RobotAuthorityTest, ARobotBrakesToStopAtTheEdgeOfWhatItWasGranted) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  robot.extendAuthority();
  robot.extendAuthority();

  int ticks = 0;
  while (robot.isCommitted() && ticks < 20) {
    (void)robot.drive();
    ++ticks;
    EXPECT_LE(robot.position().x, 2);  // never past the second cell
    EXPECT_LE(robot.speed(), 1.0 + kTolerance);
  }
  // 2 cells from rest: 2 ticks speeding up (1 cell), half a cell at speed, 1 tick braking (0.5).
  EXPECT_EQ(ticks, 4);
  EXPECT_EQ(robot.position(), P(2, 0));
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_DOUBLE_EQ(robot.segmentProgress(), 0.0);
  EXPECT_EQ(robot.authority(), 0U);
  EXPECT_EQ(robot.path().size(), 8U);  // the rest of the journey is still ahead of it
  EXPECT_EQ(robot.state(), RobotState::Navigating);
  EXPECT_EQ(robot.brakingViolations(), 0U);

  for (int i = 0; i < 10; ++i) {  // without authority it goes nowhere
    EXPECT_EQ(robot.drive(), 0U);
  }
  EXPECT_EQ(robot.position(), P(2, 0));
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  expectConsistent(robot);
}

TEST(RobotAuthorityTest, ARobotWithNoAuthorityTurnsToFaceTheNextCellAndWaits) {
  Robot robot(R(1), P(0, 0), vehicle());  // facing north
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));
  EXPECT_EQ(robot.drive(), 0U);
  EXPECT_EQ(robot.heading(), Direction::East);  // a quarter turn per tick
  EXPECT_EQ(robot.motion(), MotionPhase::Turning);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(robot.drive(), 0U);
  }
  EXPECT_EQ(robot.position(), P(0, 0));
  EXPECT_DOUBLE_EQ(robot.segmentProgress(), 0.0);
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_EQ(robot.motion(), MotionPhase::Stopped);
  EXPECT_EQ(robot.battery().level(), 1000U);  // standing still costs nothing here
}

TEST(RobotAuthorityTest, ARobotNeverEntersACellThatWasNotGranted) {
  Rng rng(7);
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(30, 0), eastward(P(0, 0), 30));
  int grantedUpTo = 0;
  int ticks = 0;
  while (robot.hasPath() && ticks < 3000) {
    if (rng.below(3) == 0 && robot.authority() < robot.path().size()) {
      robot.extendAuthority();
      ++grantedUpTo;
    }
    (void)robot.drive();
    ++ticks;
    EXPECT_LE(robot.position().x, grantedUpTo);
    EXPECT_EQ(robot.brakingViolations(), 0U);
    if (robot.segmentProgress() > 0.0) {
      EXPECT_GE(robot.authority(), 1U);  // between two cells it holds the one it is heading for
    }
    expectConsistent(robot);
  }
  EXPECT_EQ(robot.position(), P(30, 0));
}

// ---- turning ------------------------------------------------------------------------------------

TEST(RobotTurnTest, ATurnTakesAsManyTicksAsTheTurnRateNeeds) {
  RobotSpec spec = vehicle();
  spec.turnRate = kPi / 4.0;         // a quarter turn takes two ticks
  Robot robot(R(1), P(0, 0), spec);  // facing north
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));

  (void)tick(robot);
  EXPECT_NEAR(robot.headingAngle(), -kPi / 4.0, kTolerance);  // half way round
  EXPECT_EQ(robot.motion(), MotionPhase::Turning);
  EXPECT_EQ(robot.position(), P(0, 0));
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_EQ(robot.authority(), 0U);  // still rotating: it has not asked for the cell yet

  (void)tick(robot);
  EXPECT_NEAR(robot.headingAngle(), 0.0, kTolerance);  // now facing east
  EXPECT_EQ(robot.heading(), Direction::East);
  EXPECT_EQ(robot.motion(), MotionPhase::Turning);  // the whole tick went on turning
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);

  (void)tick(robot);
  EXPECT_GT(robot.speed(), 0.0);  // and only now does it set off
  EXPECT_EQ(robot.motion(), MotionPhase::Accelerating);
}

TEST(RobotTurnTest, TimeLeftOverAfterATurnGoesIntoDriving) {
  RobotSpec spec = vehicle();
  spec.turnRate = kPi;  // a quarter turn takes half a tick
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));
  (void)tick(robot);
  EXPECT_EQ(robot.heading(), Direction::East);
  EXPECT_NEAR(robot.speed(), 0.25, kTolerance);              // half a tick of acceleration at 0.5
  EXPECT_NEAR(robot.segmentProgress(), 0.0625, kTolerance);  // a * t^2 / 2
  EXPECT_EQ(robot.motion(), MotionPhase::Accelerating);
}

TEST(RobotTurnTest, AUTurnGoesClockwiseInTwoQuarterTurns) {
  Robot robot(R(1), P(3, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(0, 0), RobotPath{P(2, 0), P(1, 0), P(0, 0)});
  (void)tick(robot);
  EXPECT_NEAR(robot.headingAngle(), angleOf(Direction::South), kTolerance);
  (void)tick(robot);
  EXPECT_NEAR(robot.headingAngle(), angleOf(Direction::West), kTolerance);
  EXPECT_EQ(robot.heading(), Direction::West);
  EXPECT_EQ(robot.position(), P(3, 0));  // still hasn't moved
  (void)tick(robot);
  EXPECT_GT(robot.speed(), 0.0);
}

TEST(RobotTurnTest, ARobotStopsAtACornerTurnsAndSetsOffAgain) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(3, 2), RobotPath{P(1, 0), P(2, 0), P(3, 0), P(3, 1), P(3, 2)});
  bool stoppedAtTheCorner = false;
  bool turnedWhileStill = false;
  for (int i = 0; i < 40 && robot.hasPath(); ++i) {
    (void)tick(robot);
    if (robot.motion() == MotionPhase::Turning) {
      EXPECT_DOUBLE_EQ(robot.speed(), 0.0);  // it never turns while moving
      turnedWhileStill = true;
    }
    if (robot.position() == P(3, 0) && robot.speed() == 0.0 && robot.segmentProgress() == 0.0) {
      stoppedAtTheCorner = true;
    }
    EXPECT_EQ(robot.brakingViolations(), 0U);
    expectConsistent(robot);
  }
  EXPECT_TRUE(stoppedAtTheCorner);
  EXPECT_TRUE(turnedWhileStill);
  EXPECT_EQ(robot.position(), P(3, 2));
  EXPECT_EQ(robot.heading(), Direction::South);
}

TEST(RobotTurnTest, ARobotThatTurnsInstantlyDoesNotStopAtCornersAndFacesItsNewDirectionAtOnce) {
  RobotSpec spec = ideal();
  spec.maxSpeed = 0.75;
  Robot robot(R(1), P(0, 0), spec, Direction::East);
  robot.beginTask(T(1), P(2, 2), RobotPath{P(1, 0), P(2, 0), P(2, 1), P(2, 2)});
  // 0.75 cells per tick: the third tick crosses the corner at (2, 0) and is already heading south.
  (void)tick(robot);
  (void)tick(robot);
  (void)tick(robot);
  EXPECT_EQ(robot.position(), P(2, 0));
  EXPECT_GT(robot.segmentProgress(), 0.0);
  EXPECT_EQ(robot.heading(), Direction::South);
  EXPECT_DOUBLE_EQ(robot.pose().x, 2.5);  // and its pose runs down the column, not across
  EXPECT_GT(robot.pose().y, 0.5);
}

// ---- cargo, state and energy ----------------------------------------------------------

TEST(RobotCargoTest, ACargoJobGoesThroughEveryStateAndEndsAsADelivery) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  EXPECT_FALSE(robot.carriesCargo());
  EXPECT_EQ(robot.deliveries(), 0U);
  robot.beginTask(T(7), P(4, 0), eastward(P(0, 0), 4));
  EXPECT_FALSE(robot.carriesCargo());  // on its way to collect it

  runUntilStopped(robot);
  ASSERT_TRUE(robot.hasArrived());
  EXPECT_EQ(robot.goal(), NavigationGoal::Pickup);
  EXPECT_EQ(robot.position(), P(4, 0));
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);

  robot.beginLoading();
  EXPECT_EQ(robot.state(), RobotState::Loading);
  EXPECT_FALSE(robot.carriesCargo());  // not until loading is done
  robot.elapseTick();
  robot.elapseTick();
  robot.finishLoading(P(4, 3), RobotPath{P(4, 1), P(4, 2), P(4, 3)});
  EXPECT_TRUE(robot.carriesCargo());
  EXPECT_EQ(robot.goal(), NavigationGoal::Dropoff);

  runUntilStopped(robot);
  ASSERT_TRUE(robot.hasArrived());
  EXPECT_TRUE(robot.carriesCargo());
  EXPECT_EQ(robot.heading(), Direction::South);
  robot.beginUnloading();
  EXPECT_EQ(robot.state(), RobotState::Unloading);
  EXPECT_TRUE(robot.carriesCargo());
  robot.elapseTick();
  robot.finishUnloading();

  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_FALSE(robot.carriesCargo());
  EXPECT_EQ(robot.deliveries(), 1U);
  EXPECT_TRUE(robot.isAvailable());
  EXPECT_EQ(robot.position(), P(4, 3));
  EXPECT_EQ(robot.battery().level(), 1000U - 7U * 10U - 3U * 3U);  // 7 cells, 3 handling ticks
  EXPECT_EQ(robot.brakingViolations(), 0U);
  expectConsistent(robot);
}

TEST(RobotCargoTest, WithdrawnCargoIsDroppedAndNotCountedAsDelivered) {
  Robot robot(R(1), P(0, 0), vehicle());
  robot.beginTask(T(1), P(1, 0), eastward(P(0, 0), 1));
  runUntilStopped(robot);
  robot.beginLoading();
  robot.elapseTick();
  robot.elapseTick();
  robot.finishLoading(P(1, 3), RobotPath{P(1, 1), P(1, 2), P(1, 3)});
  ASSERT_TRUE(robot.carriesCargo());
  robot.abortTask();
  EXPECT_FALSE(robot.carriesCargo());
  EXPECT_EQ(robot.deliveries(), 0U);
}

TEST(RobotEnergyTest, MovingCostsEnergyPerCellWhateverTheSpeedProfile) {
  RobotSpec slow = vehicle();
  slow.maxSpeed = 0.5;
  slow.acceleration = 0.125;
  slow.deceleration = 0.25;
  slow.turnRate = kPi / 4.0;
  RobotSpec instantTurn = vehicle();
  instantTurn.turnRate.reset();
  const RobotPath path = {P(1, 0), P(2, 0), P(3, 0), P(3, 1), P(3, 2), P(2, 2), P(1, 2)};
  std::vector<std::uint64_t> ticks;
  for (const RobotSpec& spec : {ideal(), vehicle(), slow, instantTurn}) {
    Robot robot(R(1), P(0, 0), spec, Direction::East);
    robot.beginTask(T(1), P(1, 2), path);
    int taken = 0;
    while (robot.hasPath() && taken < 400) {
      (void)tick(robot);
      ++taken;
    }
    EXPECT_EQ(robot.position(), P(1, 2));
    EXPECT_EQ(robot.cellsTravelled(), 7U);
    EXPECT_EQ(robot.battery().level(), 1000U - 7U * 10U);
    ticks.push_back(static_cast<std::uint64_t>(taken));
  }
  EXPECT_LT(ticks[0], ticks[1]);  // the vehicle is slower than the idealised robot...
  EXPECT_LT(ticks[1], ticks[2]);  // ...and the slow one slower still, for the same energy
}

// ---- how much authority a robot asks for ----------------------------------------------

TEST(RobotWantedAuthorityTest, AnIdealisedRobotAsksForExactlyTheCellItWillDriveInto) {
  Robot robot(R(1), P(0, 0), ideal(), Direction::East);
  robot.beginTask(T(1), P(5, 0), eastward(P(0, 0), 5));
  EXPECT_EQ(robot.wantedAuthority(1.0), 1U);
  (void)tick(robot);
  EXPECT_EQ(robot.wantedAuthority(1.0), 1U);  // one cell per tick, one cell wanted

  RobotSpec half = ideal();
  half.maxSpeed = 0.5;
  Robot slow(R(2), P(0, 0), half, Direction::East);
  slow.beginTask(T(2), P(5, 0), eastward(P(0, 0), 5));
  EXPECT_EQ(slow.wantedAuthority(1.0), 1U);
  (void)tick(slow);  // half way across the first cell
  EXPECT_EQ(slow.wantedAuthority(1.0), 1U);
  RobotSpec threeQuarters = ideal();
  threeQuarters.maxSpeed = 0.75;
  Robot quick(R(3), P(0, 0), threeQuarters, Direction::East);
  quick.beginTask(T(3), P(5, 0), eastward(P(0, 0), 5));
  (void)tick(quick);  // 0.75 of the way: the next step crosses into the second cell
  EXPECT_EQ(quick.wantedAuthority(1.0), 2U);
}

TEST(RobotWantedAuthorityTest, AVehicleAsksForRoomToStopAsWellAsRoomToDrive) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  EXPECT_EQ(robot.wantedAuthority(1.0), 1U);  // from rest it can only get as far as 0.625 cells
  (void)tick(robot);
  (void)tick(robot);  // 1 cell on, at full speed, exactly at a waypoint
  ASSERT_DOUBLE_EQ(robot.speed(), 1.0);
  // Another cell of driving plus half a cell of braking: 1.5 cells, so two.
  EXPECT_EQ(robot.wantedAuthority(1.0), 2U);
  (void)tick(robot, 0.9);
  EXPECT_NEAR(robot.segmentProgress(), 0.9, kTolerance);
  EXPECT_EQ(robot.wantedAuthority(1.0), 3U);  // 0.9 + 1 + 0.5 = 2.4 cells ahead
}

TEST(RobotWantedAuthorityTest, NeverMoreThanThePathOrPastACornerItHasToStopAt) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(2, 1), RobotPath{P(1, 0), P(2, 0), P(2, 1)});
  (void)tick(robot);
  (void)tick(robot);  // now on (1, 0) at full speed; the corner at (2, 0) is the very next cell
  ASSERT_EQ(robot.position(), P(1, 0));
  ASSERT_DOUBLE_EQ(robot.speed(), 1.0);
  // Driving on and braking would reach 1.5 cells, but the corner is a stop: nothing past it is
  // wanted yet, so the robot does not hold cells it cannot use.
  EXPECT_EQ(robot.wantedAuthority(1.0), 1U);

  // The same situation on a straight path does want the second cell.
  Robot straight(R(4), P(0, 0), vehicle(), Direction::East);
  straight.beginTask(T(4), P(3, 0), eastward(P(0, 0), 3));
  (void)tick(straight);
  (void)tick(straight);
  ASSERT_EQ(straight.position(), P(1, 0));
  EXPECT_EQ(straight.wantedAuthority(1.0), 2U);

  Robot shortRun(R(2), P(0, 0), vehicle(), Direction::East);
  shortRun.beginTask(T(2), P(1, 0), eastward(P(0, 0), 1));
  EXPECT_EQ(shortRun.wantedAuthority(1.0), 1U);  // only one cell exists

  Robot idle(R(3), P(0, 0), vehicle());
  EXPECT_EQ(idle.wantedAuthority(1.0), 0U);  // no path, nothing wanted
}

// ---- withdrawn while moving -----------------------------------------------------------

TEST(RobotCoastTest, ARobotWithdrawnBetweenCellsBrakesToTheNextWaypointInsteadOfStoppingDead) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  (void)tick(robot);  // a quarter of a cell on, at speed 0.5
  ASSERT_NEAR(robot.segmentProgress(), 0.25, kTolerance);
  const double speedAtWithdrawal = robot.speed();

  robot.abortTask();
  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_FALSE(robot.task().has_value());
  EXPECT_FALSE(robot.destination().has_value());
  EXPECT_EQ(robot.goal(), NavigationGoal::None);
  EXPECT_FALSE(robot.isAvailable());   // still rolling
  EXPECT_EQ(robot.path().size(), 1U);  // it keeps just the cell it is driving into
  EXPECT_EQ(robot.authority(), 1U);
  EXPECT_NEAR(robot.speed(), speedAtWithdrawal, kTolerance);  // not stopped dead
  expectConsistent(robot);
  EXPECT_THROW(robot.beginTask(T(2), P(0, 0), {}), InvalidTransition);  // not while it moves

  // On the way to a stop it never speeds up, not even part-way through a tick: it keeps its 0.5
  // until it must brake, 0.125 cells before the waypoint (so after a whole tick it is 0.5 further
  // on and still doing 0.5).
  (void)robot.drive();
  EXPECT_NEAR(robot.segmentProgress(), 0.75, kTolerance);
  EXPECT_NEAR(robot.speed(), 0.5, kTolerance);
  EXPECT_EQ(robot.motion(), MotionPhase::Cruising);

  double fastest = robot.speed();
  for (int i = 0; i < 10 && robot.hasPath(); ++i) {
    (void)robot.drive();
    fastest = std::max(fastest, robot.speed());
    expectConsistent(robot);
  }
  EXPECT_EQ(robot.position(), P(1, 0));  // stopped at the waypoint it had kept
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_LE(fastest, speedAtWithdrawal + kTolerance);  // it only ever slowed down
  EXPECT_EQ(robot.brakingViolations(), 0U);
  EXPECT_TRUE(robot.isAvailable());
  EXPECT_EQ(robot.battery().level(), 990U);
  EXPECT_EQ(robot.state(), RobotState::Idle);
}

TEST(RobotCoastTest, AFastRobotTooCloseToTheNextWaypointStopsAtTheOneAfter) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(10, 0), eastward(P(0, 0), 10));
  (void)tick(robot);
  (void)tick(robot);
  (void)tick(robot, 0.75);  // 0.75 into the third segment at full speed
  ASSERT_DOUBLE_EQ(robot.speed(), 1.0);
  ASSERT_NEAR(robot.segmentProgress(), 0.75, kTolerance);
  ASSERT_GE(robot.authority(), 2U);

  robot.makeUnavailable();
  // Braking from 1 takes half a cell and the next waypoint is a quarter away: it goes on to the
  // next.
  EXPECT_EQ(robot.state(), RobotState::Unavailable);
  EXPECT_EQ(robot.path().size(), 2U);
  EXPECT_EQ(robot.authority(), 2U);
  const Point firstWaypoint = robot.path().front();
  const Point secondWaypoint = robot.path().back();
  runUntilStopped(robot);
  EXPECT_EQ(robot.position(), secondWaypoint);
  EXPECT_NE(robot.position(), firstWaypoint);
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_EQ(robot.brakingViolations(), 0U);
  EXPECT_EQ(robot.state(), RobotState::Unavailable);
  robot.restore();
  EXPECT_TRUE(robot.isAvailable());
}

TEST(RobotCoastTest, ARobotThatIsStandingStillStopsImmediatelyEvenWithCellsGranted) {
  Robot robot(R(1), P(0, 0), vehicle());  // facing north, so it has to turn first
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));
  robot.extendAuthority();
  ASSERT_DOUBLE_EQ(robot.speed(), 0.0);
  robot.abortTask();
  EXPECT_EQ(robot.authority(), 0U);
  EXPECT_FALSE(robot.hasPath());
  EXPECT_TRUE(robot.isAvailable());
  expectConsistent(robot);
}

TEST(RobotCoastTest, AnIdealisedRobotBetweenCellsCoastsToTheNextWaypointToo) {
  RobotSpec spec = ideal();
  spec.maxSpeed = 0.5;
  Robot robot(R(1), P(0, 0), spec, Direction::East);
  robot.beginTask(T(1), P(5, 0), eastward(P(0, 0), 5));
  (void)tick(robot);  // half way across the first cell
  robot.abortTask();
  EXPECT_FALSE(robot.isAvailable());
  runUntilStopped(robot);
  EXPECT_EQ(robot.position(), P(1, 0));
  EXPECT_TRUE(robot.isAvailable());
}

// ---- invalid movement is refused -------------------------------------------------------------

TEST(RobotInvalidMovementTest, AuthorityCannotGoBeyondThePathAndATickIsNotNegative) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(2, 0), eastward(P(0, 0), 2));
  robot.extendAuthority();
  robot.extendAuthority();
  EXPECT_THROW(robot.extendAuthority(), InvalidTransition);
  EXPECT_EQ(robot.authority(), 2U);
  EXPECT_THROW((void)robot.advance(-0.5), std::invalid_argument);
  EXPECT_THROW((void)robot.turnTowardNext(0.0), std::invalid_argument);
  EXPECT_THROW((void)robot.turnTowardNext(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW((void)robot.drive(-1.0), std::invalid_argument);
  expectConsistent(robot);
}

TEST(RobotInvalidMovementTest, TheCellsARobotHasBeenGrantedCannotBeTakenAwayByANewRoute) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(3, 1), RobotPath{P(1, 0), P(2, 0), P(3, 0), P(3, 1)});
  robot.extendAuthority();
  robot.extendAuthority();
  // The route may change beyond the granted cells...
  EXPECT_NO_THROW(robot.setPath(RobotPath{P(1, 0), P(2, 0), P(2, 1), P(3, 1)}));
  EXPECT_EQ(robot.authority(), 2U);
  // ...but not through them.
  EXPECT_THROW(robot.setPath(RobotPath{P(1, 0), P(1, 1), P(2, 1), P(3, 1)}), std::invalid_argument);
  EXPECT_THROW(robot.setPath(RobotPath{P(0, 1), P(1, 1), P(2, 1), P(3, 1)}), std::invalid_argument);
  EXPECT_THROW(robot.setPath(RobotPath{P(1, 0)}), std::invalid_argument);  // not even the end
  EXPECT_EQ(robot.path().size(), 4U);
  EXPECT_EQ(robot.path()[1], P(2, 0));
  expectConsistent(robot);

  Robot charger(R(2), P(0, 0), vehicle(), Direction::East);
  charger.beginChargerTrip(P(2, 0), eastward(P(0, 0), 2));
  charger.extendAuthority();
  EXPECT_THROW(charger.redirectChargerTrip(P(0, 2), RobotPath{P(0, 1), P(0, 2)}),
               std::invalid_argument);
  EXPECT_NO_THROW(charger.redirectChargerTrip(P(1, 1), RobotPath{P(1, 0), P(1, 1)}));
}

TEST(RobotInvalidMovementTest, ARobotWithCellsAheadCannotBePushedAsideAndNeitherCanADrivingOne) {
  Robot robot(R(1), P(1, 1), vehicle(), Direction::East);
  robot.beginTask(T(1), P(5, 1), eastward(P(1, 1), 4));
  robot.extendAuthority();
  EXPECT_THROW(robot.sidestep(P(1, 2)), InvalidTransition);
  EXPECT_EQ(robot.position(), P(1, 1));
  EXPECT_EQ(robot.authority(), 1U);
  (void)robot.drive();  // now between cells
  ASSERT_GT(robot.segmentProgress(), 0.0);
  EXPECT_THROW(robot.sidestep(P(1, 2)), InvalidTransition);
  EXPECT_EQ(robot.position(), P(1, 1));
  expectConsistent(robot);
}

TEST(RobotInvalidMovementTest, ARobotThatHasNotStoppedCannotStartAnotherJob) {
  Robot robot(R(1), P(0, 0), vehicle(), Direction::East);
  robot.beginTask(T(1), P(6, 0), eastward(P(0, 0), 6));
  (void)tick(robot);
  robot.abortTask();
  ASSERT_TRUE(robot.hasPath());
  EXPECT_THROW(robot.beginTask(T(2), P(0, 0), {}), InvalidTransition);
  EXPECT_THROW(robot.beginChargerTrip(P(0, 0), {}), InvalidTransition);
  EXPECT_THROW(robot.beginCharging(), InvalidTransition);
  EXPECT_EQ(robot.state(), RobotState::Idle);
  expectConsistent(robot);
}

// ---- the step length does not change the motion ----------------------------------------------

struct Trace {
  Vec2 pose;
  double speed;
  double heading;
  Point position;
};

std::vector<Trace> traceWithStep(const RobotSpec& spec, double dt, int ticks) {
  Robot robot(R(1), P(0, 0), spec, Direction::East);
  robot.beginTask(T(1), P(4, 3),
                  RobotPath{P(1, 0), P(2, 0), P(3, 0), P(4, 0), P(4, 1), P(4, 2), P(4, 3)});
  std::vector<Trace> trace;
  const int stepsPerTick = static_cast<int>(std::lround(1.0 / dt));
  for (int tickNumber = 0; tickNumber < ticks; ++tickNumber) {
    for (int i = 0; i < stepsPerTick && robot.hasPath(); ++i) {
      tickWithEverythingGranted(robot, dt);
    }
    trace.push_back({robot.pose(), robot.speed(), robot.headingAngle(), robot.position()});
  }
  return trace;
}

TEST(RobotTimestepTest, TheSameJourneyInSmallerStepsPassesThroughTheSamePosesAtTheSameTimes) {
  RobotSpec turning = vehicle();
  turning.turnRate = kPi / 3.0;  // a quarter turn takes 1.5 ticks: it does not divide the step
  for (const RobotSpec& spec : {vehicle(), turning}) {
    const std::vector<Trace> reference = traceWithStep(spec, 1.0, 40);
    for (const double dt : {0.5, 0.25, 0.125, 0.2}) {
      const std::vector<Trace> split = traceWithStep(spec, dt, 40);
      ASSERT_EQ(split.size(), reference.size());
      for (std::size_t i = 0; i < reference.size(); ++i) {
        EXPECT_NEAR(split[i].pose.x, reference[i].pose.x, 1e-6) << "dt " << dt << " tick " << i + 1;
        EXPECT_NEAR(split[i].pose.y, reference[i].pose.y, 1e-6) << "dt " << dt << " tick " << i + 1;
        EXPECT_NEAR(split[i].speed, reference[i].speed, 1e-6) << "dt " << dt << " tick " << i + 1;
        EXPECT_NEAR(split[i].heading, reference[i].heading, 1e-6)
            << "dt " << dt << " tick " << i + 1;
        EXPECT_EQ(split[i].position, reference[i].position) << "dt " << dt << " tick " << i + 1;
      }
    }
  }
}

// ---- the state machine with motion, under random use -----------------------------------------

TEST(RobotDriveRandomTest, RandomJobsInterruptionsAndWithdrawalsNeverBreakAnInvariant) {
  Rng rng(2026);
  int withdrawnWhileMoving = 0;
  int deliveries = 0;
  RobotSpec spec = vehicle();
  spec.batteryCapacity = 100000;
  spec.moveCostPerCell = 1;
  spec.handlingDrainPerTick = 0;
  for (int robotNumber = 0; robotNumber < 30; ++robotNumber) {
    Robot robot(R(static_cast<std::uint32_t>(robotNumber + 1)), P(5, 5), spec);
    double fastest = 0.0;
    for (int step = 0; step < 500; ++step) {
      const std::uint64_t dice = rng.below(100);
      switch (robot.state()) {
        case RobotState::Idle:
          if (robot.isAvailable() && dice < 40) {
            const Point target =
                P(static_cast<int>(rng.below(12)), static_cast<int>(rng.below(12)));
            robot.beginTask(T(1), target, lPath(robot.position(), target));
          }
          break;
        case RobotState::Navigating:
          if (robot.hasArrived()) {
            if (robot.goal() == NavigationGoal::Pickup) {
              robot.beginLoading();
            } else {
              robot.beginUnloading();
            }
          } else if (dice < 4) {
            withdrawnWhileMoving += robot.speed() > 0.0 || robot.segmentProgress() > 0.0;
            robot.abortTask();
          } else if (dice < 6) {
            withdrawnWhileMoving += robot.speed() > 0.0 || robot.segmentProgress() > 0.0;
            robot.makeUnavailable();
          }
          break;
        case RobotState::Loading:
          if (robot.dwellRemaining() == 0) {
            const Point target =
                P(static_cast<int>(rng.below(12)), static_cast<int>(rng.below(12)));
            robot.finishLoading(target, lPath(robot.position(), target));
          }
          break;
        case RobotState::Unloading:
          if (robot.dwellRemaining() == 0) {
            robot.finishUnloading();
            ++deliveries;
          }
          break;
        case RobotState::Unavailable:
          if (dice < 30) {
            robot.restore();
          }
          break;
        case RobotState::Charging:
          break;
      }
      robot.elapseTick();
      if (robot.hasPath()) {
        (void)tick(robot);
      }
      fastest = std::max(fastest, robot.speed());
      expectConsistent(robot);
      ASSERT_EQ(robot.brakingViolations(), 0U) << "robot " << robotNumber << " step " << step;
      ASSERT_FALSE(robot.isAvailable() && robot.hasPath());
      ASSERT_LE(robot.speed(), spec.maxSpeed + kTolerance);
      ASSERT_LE(robot.authority(), robot.path().size());
    }
    EXPECT_LE(fastest, spec.maxSpeed + kTolerance);
  }
  EXPECT_GT(withdrawnWhileMoving, 20);  // plenty of withdrawals really did catch a robot in motion
  EXPECT_GT(deliveries, 100);
}

}  // namespace
