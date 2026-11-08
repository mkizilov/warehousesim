#include "warehousesim/robots/motion.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "warehousesim/core/rng.hpp"
#include "warehousesim/robots/robot_spec.hpp"

namespace {

using warehousesim::core::Rng;
using warehousesim::robots::advanceAlongRun;
using warehousesim::robots::MotionLimits;
using warehousesim::robots::MotionPhase;
using warehousesim::robots::MotionStep;
using warehousesim::robots::RobotSpec;

constexpr double kTolerance = 1e-9;

// vmax 1 cell/tick, 0.5 cells/tick^2 up, 1 down: speeding up takes 2 ticks and 1 cell, stopping
// takes 1 tick and 0.5 cells.
MotionLimits vehicle() { return {1.0, 0.5, 1.0}; }

double uniform(Rng& rng, double low, double high) {
  return low + (high - low) * static_cast<double>(rng.below(1'000'001)) / 1'000'000.0;
}

// Where a run of `distance` cells that starts at rest has got to after `until` ticks, driven in
// steps of `dt`, as (distance covered, speed, arrived).
struct Reached {
  double x = 0.0;
  double speed = 0.0;
  bool arrived = false;
};

Reached drive(const MotionLimits& limits, double distance, double dt, double until) {
  Reached state;
  const int steps = static_cast<int>(std::lround(until / dt));
  for (int i = 0; i < steps && !state.arrived; ++i) {
    const MotionStep step = advanceAlongRun(limits, state.speed, distance - state.x, dt);
    state.x += step.distance;
    state.speed = step.speed;
    state.arrived = step.arrived;
  }
  if (state.arrived) {
    state.x = distance;
    state.speed = 0.0;
  }
  return state;
}

// ---- the idealised robot ----------------------------------------------------------------

TEST(MotionTest, WithoutLimitsARobotDrivesAtMaxSpeedAndStopsOnTheSpot) {
  const MotionLimits ideal{0.5, std::nullopt, std::nullopt};
  const MotionStep first = advanceAlongRun(ideal, 0.0, 3.0, 1.0);
  EXPECT_DOUBLE_EQ(first.distance, 0.5);  // full speed from the very first instant
  EXPECT_DOUBLE_EQ(first.speed, 0.5);
  EXPECT_FALSE(first.arrived);

  const MotionStep last = advanceAlongRun(ideal, 0.5, 0.25, 1.0);
  EXPECT_TRUE(last.arrived);
  EXPECT_DOUBLE_EQ(last.distance, 0.25);
  EXPECT_DOUBLE_EQ(last.speed, 0.0);
  EXPECT_DOUBLE_EQ(last.timeUsed, 0.5);  // half the step was spent standing at the stop point
  EXPECT_DOUBLE_EQ(last.speedOnArrival, 0.5);
  EXPECT_FALSE(last.exceededDeceleration);
}

TEST(MotionTest, ArrivingExactlyAtTheEndOfTheStepIsAnArrival) {
  const MotionLimits ideal{1.0, std::nullopt, std::nullopt};
  const MotionStep step = advanceAlongRun(ideal, 0.0, 1.0, 1.0);
  EXPECT_TRUE(step.arrived);
  EXPECT_DOUBLE_EQ(step.distance, 1.0);
  EXPECT_DOUBLE_EQ(step.timeUsed, 1.0);
}

TEST(MotionTest, OnlyInstantBrakingCanArriveAtSpeed) {
  const MotionLimits instantBrakes{1.0, 0.5, std::nullopt};
  EXPECT_DOUBLE_EQ(advanceAlongRun(instantBrakes, 1.0, 0.5, 1.0).speedOnArrival, 1.0);
  const MotionStep braking = advanceAlongRun(vehicle(), 1.0, 0.5, 1.0);
  EXPECT_TRUE(braking.arrived);
  EXPECT_DOUBLE_EQ(braking.speedOnArrival, 0.0);
}

// ---- a vehicle: trapezoid and triangle profiles -------------------------------------------

// 10 cells from rest: 2 ticks / 1 cell speeding up, 8.5 ticks / 8.5 cells at full speed, then
// 1 tick / 0.5 cells braking. 11.5 ticks in all.
TEST(MotionTest, LongRunsAccelerateCruiseAndBrake) {
  struct Sample {
    double time;
    double distance;
    double speed;
  };
  const std::vector<Sample> samples = {
      {1.0, 0.25, 0.5},    // speeding up: x = a t^2 / 2
      {2.0, 1.0, 1.0},     // reaches top speed
      {6.0, 5.0, 1.0},     // cruising
      {10.5, 9.5, 1.0},    // the instant braking starts
      {11.0, 9.875, 0.5},  // halfway through braking
  };
  for (const Sample& s : samples) {
    const MotionStep step = advanceAlongRun(vehicle(), 0.0, 10.0, s.time);
    EXPECT_NEAR(step.distance, s.distance, kTolerance) << "t = " << s.time;
    EXPECT_NEAR(step.speed, s.speed, kTolerance) << "t = " << s.time;
    EXPECT_FALSE(step.arrived);
  }
  const MotionStep end = advanceAlongRun(vehicle(), 0.0, 10.0, 11.5);
  EXPECT_TRUE(end.arrived);
  EXPECT_NEAR(end.timeUsed, 11.5, kTolerance);
  EXPECT_DOUBLE_EQ(end.distance, 10.0);
  EXPECT_DOUBLE_EQ(end.speed, 0.0);
}

// 1 cell from rest never reaches top speed: it speeds up to sqrt(2/3) and brakes straight away.
TEST(MotionTest, ShortRunsPeakBelowTopSpeed) {
  const double peak = std::sqrt(2.0 / 3.0);
  const double speedUpTime = peak / 0.5;
  const MotionStep atPeak = advanceAlongRun(vehicle(), 0.0, 1.0, speedUpTime);
  EXPECT_NEAR(atPeak.speed, peak, kTolerance);
  EXPECT_NEAR(atPeak.distance, (2.0 / 3.0) / (2.0 * 0.5), kTolerance);  // v^2 / 2a
  EXPECT_LT(peak, vehicle().maxSpeed);

  const MotionStep end = advanceAlongRun(vehicle(), 0.0, 1.0, speedUpTime + peak / 1.0);
  EXPECT_TRUE(end.arrived);
  EXPECT_NEAR(end.timeUsed, speedUpTime + peak, kTolerance);
}

TEST(MotionTest, ARobotAlreadyAtSpeedOnlyNeedsToBrake) {
  // 0.5 cells is exactly the braking distance from speed 1 at deceleration 1.
  const MotionStep step = advanceAlongRun(vehicle(), 1.0, 0.5, 0.25);
  EXPECT_NEAR(step.speed, 0.75, kTolerance);
  EXPECT_NEAR(step.distance, 0.25 * 1.0 - 0.5 * 1.0 * 0.25 * 0.25, kTolerance);
  EXPECT_FALSE(step.exceededDeceleration);
}

TEST(MotionTest, InstantAccelerationStillBrakesAtTheLimit) {
  const MotionLimits limits{1.0, std::nullopt, 1.0};
  // Full speed at once, cruising for 9.5 cells, then half a cell of braking at deceleration 1.
  EXPECT_DOUBLE_EQ(advanceAlongRun(limits, 0.0, 10.0, 1.0).speed, 1.0);         // no ramp
  EXPECT_NEAR(advanceAlongRun(limits, 0.0, 10.0, 9.5).speed, 1.0, kTolerance);  // brakes from here
  EXPECT_NEAR(advanceAlongRun(limits, 0.0, 10.0, 10.0).speed, 0.5, kTolerance);
  EXPECT_NEAR(advanceAlongRun(limits, 0.0, 10.0, 10.25).speed, 0.25, kTolerance);
  EXPECT_FALSE(advanceAlongRun(limits, 0.0, 10.0, 10.4).arrived);
  EXPECT_TRUE(advanceAlongRun(limits, 0.0, 10.0, 10.5).arrived);
}

TEST(MotionTest, ASpeedAboveTheBrakingCurveIsReportedAndStillStopsExactlyAtTheLine) {
  // From speed 1 braking needs 0.5 cells at deceleration 1, but the line is only 0.25 away.
  const MotionStep step = advanceAlongRun(vehicle(), 1.0, 0.25, 1.0);
  EXPECT_TRUE(step.exceededDeceleration);
  EXPECT_TRUE(step.arrived);
  EXPECT_DOUBLE_EQ(step.distance, 0.25);  // never past the line
  EXPECT_DOUBLE_EQ(step.speed, 0.0);

  // Before it gets there the harder braking is still a straight deceleration to rest.
  const MotionStep early = advanceAlongRun(vehicle(), 1.0, 0.25, 0.25);
  EXPECT_TRUE(early.exceededDeceleration);
  EXPECT_FALSE(early.arrived);
  EXPECT_LT(early.distance, 0.25);
}

TEST(MotionTest, NothingHappensWithoutTimeAndEverythingEndsAtAZeroDistanceStop) {
  const MotionStep still = advanceAlongRun(vehicle(), 0.5, 3.0, 0.0);
  EXPECT_DOUBLE_EQ(still.distance, 0.0);
  EXPECT_DOUBLE_EQ(still.speed, 0.5);
  EXPECT_FALSE(still.arrived);

  const MotionStep there = advanceAlongRun(vehicle(), 0.0, 0.0, 1.0);
  EXPECT_TRUE(there.arrived);
  EXPECT_DOUBLE_EQ(there.distance, 0.0);
  EXPECT_FALSE(there.exceededDeceleration);
  // Moving at the stop point with limited brakes is the violation it looks like.
  EXPECT_TRUE(advanceAlongRun(vehicle(), 0.5, 0.0, 1.0).exceededDeceleration);
  EXPECT_FALSE(advanceAlongRun({1.0, std::nullopt, std::nullopt}, 0.5, 0.0, 1.0)
                   .exceededDeceleration);  // instant brakes can always stop
}

TEST(MotionTest, BadInputsAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)advanceAlongRun(vehicle(), -0.1, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun(vehicle(), 0.0, -1.0, 1.0), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun(vehicle(), 0.0, 1.0, -1.0), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun(vehicle(), nan, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun(vehicle(), 0.0, inf, 1.0), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun(vehicle(), 0.0, 1.0, nan), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun({0.0, 0.5, 1.0}, 0.0, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun({1.0, 0.0, 1.0}, 0.0, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW((void)advanceAlongRun({1.0, 0.5, -1.0}, 0.0, 1.0, 1.0), std::invalid_argument);
}

TEST(MotionTest, BrakingDistanceIsSpeedSquaredOverTwiceTheDeceleration) {
  EXPECT_DOUBLE_EQ(vehicle().brakingDistance(1.0), 0.5);
  EXPECT_DOUBLE_EQ(vehicle().brakingDistance(0.5), 0.125);
  EXPECT_DOUBLE_EQ(vehicle().brakingDistance(0.0), 0.0);
  EXPECT_DOUBLE_EQ((MotionLimits{1.0, 0.5, std::nullopt}).brakingDistance(1.0), 0.0);
}

// ---- limits are never exceeded --------------------------------------------------------------

TEST(MotionPropertyTest, RandomRunsNeverBreakTheSpeedAccelerationOrDecelerationLimits) {
  Rng rng(0xFACADE);
  int arrivals = 0;
  int cruised = 0;
  for (int run = 0; run < 400; ++run) {
    MotionLimits limits;
    limits.maxSpeed = uniform(rng, 0.05, 1.0);
    if (rng.below(4) != 0) {
      limits.acceleration = uniform(rng, 0.02, 2.0);
    }
    if (rng.below(4) != 0) {
      // Strong enough to stop within a cell, as a valid RobotSpec requires.
      limits.deceleration =
          std::max(uniform(rng, 0.02, 2.0), limits.maxSpeed * limits.maxSpeed / 2.0);
    }
    const double distance = uniform(rng, 0.2, 12.0);
    const double dt = uniform(rng, 0.05, 1.0);

    double x = 0.0;
    double speed = 0.0;
    double stepsTaken = 0;
    for (; stepsTaken < 5000; ++stepsTaken) {
      const MotionStep step = advanceAlongRun(limits, speed, distance - x, dt);
      ASSERT_FALSE(step.exceededDeceleration) << "run " << run;
      ASSERT_GE(step.distance, -kTolerance);
      ASSERT_LE(step.distance, distance - x + kTolerance);  // never past the stop point
      ASSERT_LE(step.distance, limits.maxSpeed * dt + kTolerance);
      ASSERT_LE(step.speed, limits.maxSpeed + kTolerance);
      ASSERT_GE(step.speed, 0.0);
      if (limits.acceleration) {
        ASSERT_LE(step.speed - speed, *limits.acceleration * dt + kTolerance) << "run " << run;
      }
      if (limits.deceleration) {
        ASSERT_LE(speed - step.speed, *limits.deceleration * dt + kTolerance) << "run " << run;
      }
      cruised += limits.acceleration && step.speed >= limits.maxSpeed - kTolerance;
      x += step.distance;
      speed = step.speed;
      if (step.arrived) {
        ++arrivals;
        ASSERT_NEAR(x, distance, kTolerance);
        ASSERT_DOUBLE_EQ(step.speed, 0.0);
        break;
      }
    }
    ASSERT_LT(stepsTaken, 5000) << "run " << run << " never arrived";
  }
  EXPECT_EQ(arrivals, 400);
  EXPECT_GT(cruised, 100);  // the runs really did reach top speed
}

// ---- the step length does not change the motion ------------------------------------------

TEST(MotionPropertyTest, SplittingTimeIntoSmallerStepsGivesTheSameTrajectory) {
  const std::vector<MotionLimits> models = {
      {1.0, 0.5, 1.0},                    // vehicle
      {1.0, std::nullopt, 1.0},           // instant start, limited braking
      {1.0, 0.5, std::nullopt},           // limited start, instant stop
      {1.0, std::nullopt, std::nullopt},  // idealised
      {0.5, 0.125, 0.25},                 // a slow, heavy one
  };
  for (const MotionLimits& limits : models) {
    for (const double distance : {0.5, 1.0, 2.5, 7.0, 20.0}) {
      // Reference: one step of 1.0 per tick.
      for (const double t : {1.0, 2.0, 3.0, 5.0, 8.0, 13.0, 21.0}) {
        const Reached reference = drive(limits, distance, 1.0, t);
        for (const double dt : {0.5, 0.25, 0.125, 0.2, 0.1, 0.05}) {
          const Reached split = drive(limits, distance, dt, t);
          EXPECT_NEAR(split.x, reference.x, 1e-7)
              << "distance " << distance << " t " << t << " dt " << dt;
          EXPECT_NEAR(split.speed, reference.speed, 1e-7)
              << "distance " << distance << " t " << t << " dt " << dt;
          EXPECT_EQ(split.arrived, reference.arrived)
              << "distance " << distance << " t " << t << " dt " << dt;
        }
      }
    }
  }
}

// ---- the limits in a RobotSpec -------------------------------------------------------------

TEST(RobotSpecMotionTest, ByDefaultEverythingIsInstant) {
  const RobotSpec spec;
  EXPECT_FALSE(spec.acceleration.has_value());
  EXPECT_FALSE(spec.deceleration.has_value());
  EXPECT_FALSE(spec.turnRate.has_value());
  EXPECT_NO_THROW(spec.validate());
  const MotionLimits limits = spec.motionLimits();
  EXPECT_DOUBLE_EQ(limits.maxSpeed, 1.0);
  EXPECT_FALSE(limits.acceleration.has_value());
  EXPECT_FALSE(limits.deceleration.has_value());
}

TEST(RobotSpecMotionTest, LimitsMustBeFiniteAndPositive) {
  for (const double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    RobotSpec spec;
    spec.acceleration = bad;
    EXPECT_THROW(spec.validate(), std::invalid_argument) << bad;
    spec = RobotSpec{};
    spec.deceleration = bad;
    EXPECT_THROW(spec.validate(), std::invalid_argument) << bad;
    spec = RobotSpec{};
    spec.turnRate = bad;
    EXPECT_THROW(spec.validate(), std::invalid_argument) << bad;
  }
}

TEST(RobotSpecMotionTest, ARobotMustBeAbleToStopWithinOneCell) {
  RobotSpec spec;
  spec.maxSpeed = 1.0;
  spec.deceleration = 0.5;  // 1 / (2 * 0.5) = exactly one cell: allowed
  EXPECT_NO_THROW(spec.validate());
  spec.deceleration = 0.49;  // a little more than a cell
  EXPECT_THROW(spec.validate(), std::invalid_argument);
  spec.maxSpeed = 0.5;  // half the speed needs a quarter of the distance
  EXPECT_NO_THROW(spec.validate());
  spec.deceleration = 0.125;  // 0.25 / 0.25 = exactly one cell
  EXPECT_NO_THROW(spec.validate());
  spec.deceleration = 0.1;
  EXPECT_THROW(spec.validate(), std::invalid_argument);
}

TEST(MotionPhaseTest, EveryPhaseHasAName) {
  EXPECT_STREQ(warehousesim::robots::toString(MotionPhase::Stopped), "stopped");
  EXPECT_STREQ(warehousesim::robots::toString(MotionPhase::Turning), "turning");
  EXPECT_STREQ(warehousesim::robots::toString(MotionPhase::Accelerating), "accelerating");
  EXPECT_STREQ(warehousesim::robots::toString(MotionPhase::Cruising), "cruising");
  EXPECT_STREQ(warehousesim::robots::toString(MotionPhase::Braking), "braking");
}

}  // namespace
