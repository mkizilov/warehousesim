#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/fixed_timestep.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/core/vec2.hpp"

namespace {

using warehousesim::core::angleOf;
using warehousesim::core::Direction;
using warehousesim::core::FixedTimestep;
using warehousesim::core::kPi;
using warehousesim::core::lerp;
using warehousesim::core::lerpAngle;
using warehousesim::core::manhattanDistance;
using warehousesim::core::nearestDirection;
using warehousesim::core::Point;
using warehousesim::core::PointHash;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::core::shortestRotation;
using warehousesim::core::SimClock;
using warehousesim::core::TaskId;
using warehousesim::core::Vec2;
using warehousesim::core::wrapAngle;

Point P(int x, int y) { return Point{x, y}; }

TEST(PointTest, AddsComponentwise) { EXPECT_EQ(P(1, 2) + P(3, -4), P(4, -2)); }

TEST(PointTest, OrdersLexicographically) {
  EXPECT_TRUE(P(0, 5) < P(1, 0));
  EXPECT_TRUE(P(1, 0) < P(1, 1));
  EXPECT_NE(P(1, 2), P(2, 1));
}

TEST(PointTest, ManhattanDistanceIsSymmetric) {
  EXPECT_EQ(manhattanDistance(P(0, 0), P(3, 4)), 7);
  EXPECT_EQ(manhattanDistance(P(3, 4), P(0, 0)), 7);
  EXPECT_EQ(manhattanDistance(P(-2, -2), P(-2, -2)), 0);
}

TEST(PointTest, HashDistinguishesSwappedAndNegativeCoordinates) {
  std::unordered_set<Point, PointHash> points{P(1, 2), P(2, 1), P(-1, 2), P(1, -2)};
  EXPECT_EQ(points.size(), 4U);
  EXPECT_TRUE(points.contains(P(-1, 2)));
}

TEST(DirectionTest, OffsetsMatchTheCardinalDirectionTable) {
  using warehousesim::core::Direction;
  using warehousesim::core::kAllDirections;
  using warehousesim::core::kCardinalDirections;
  using warehousesim::core::offsetOf;
  EXPECT_EQ(offsetOf(Direction::North), P(0, -1));
  EXPECT_EQ(offsetOf(Direction::East), P(1, 0));
  EXPECT_EQ(offsetOf(Direction::South), P(0, 1));
  EXPECT_EQ(offsetOf(Direction::West), P(-1, 0));
  for (std::size_t i = 0; i < kAllDirections.size(); ++i) {
    EXPECT_EQ(offsetOf(kAllDirections[i]), kCardinalDirections[i]);
  }
}

TEST(DirectionTest, DirectionBetweenFindsOnlyOrthogonalNeighbours) {
  using warehousesim::core::Direction;
  using warehousesim::core::directionBetween;
  EXPECT_EQ(directionBetween(P(2, 2), P(2, 1)), Direction::North);
  EXPECT_EQ(directionBetween(P(2, 2), P(3, 2)), Direction::East);
  EXPECT_EQ(directionBetween(P(2, 2), P(2, 3)), Direction::South);
  EXPECT_EQ(directionBetween(P(2, 2), P(1, 2)), Direction::West);
  EXPECT_FALSE(directionBetween(P(2, 2), P(2, 2)).has_value());
  EXPECT_FALSE(directionBetween(P(2, 2), P(3, 3)).has_value());
  EXPECT_FALSE(directionBetween(P(2, 2), P(4, 2)).has_value());
  EXPECT_FALSE(directionBetween(P(0, 0), P(-1, 1)).has_value());
}

TEST(DirectionSetTest, StartsEmptyAndTracksMembership) {
  using warehousesim::core::Direction;
  using warehousesim::core::DirectionSet;
  constexpr DirectionSet none;
  EXPECT_TRUE(none.empty());
  EXPECT_EQ(none.size(), 0U);
  constexpr DirectionSet all = DirectionSet::all();
  EXPECT_EQ(all.size(), 4U);
  for (const Direction direction : warehousesim::core::kAllDirections) {
    EXPECT_FALSE(none.contains(direction));
    EXPECT_TRUE(all.contains(direction));
  }
}

TEST(DirectionSetTest, WithAndWithoutReturnNewSetsAndAreIdempotent) {
  using warehousesim::core::Direction;
  using warehousesim::core::DirectionSet;
  const DirectionSet eastOnly = DirectionSet{}.with(Direction::East);
  EXPECT_TRUE(eastOnly.contains(Direction::East));
  EXPECT_EQ(eastOnly.size(), 1U);
  EXPECT_TRUE(eastOnly == eastOnly.with(Direction::East));
  const DirectionSet noEast = DirectionSet::all().without(Direction::East);
  EXPECT_FALSE(noEast.contains(Direction::East));
  EXPECT_EQ(noEast.size(), 3U);
  EXPECT_TRUE(noEast == noEast.without(Direction::East));
  EXPECT_TRUE(noEast.with(Direction::East) == DirectionSet::all());
  EXPECT_TRUE((DirectionSet{Direction::North, Direction::South}) ==
              (DirectionSet{Direction::South, Direction::North}));
}

TEST(IdsTest, DefaultIsInvalidAndDistinctTypesCompareByValue) {
  EXPECT_FALSE(RobotId{}.valid());
  EXPECT_TRUE(TaskId{3}.valid());
  EXPECT_TRUE(TaskId{3} < TaskId{4});
  EXPECT_EQ(RobotId{2}, RobotId{2});
}

TEST(RngTest, SameSeedGivesSameSequence) {
  Rng a(42);
  Rng b(42);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(a.next(), b.next());
  }
}

TEST(RngTest, DifferentSeedsDiverge) {
  Rng a(1);
  Rng b(2);
  EXPECT_NE(a.next(), b.next());
}

TEST(RngTest, MatchesKnownSplitMix64Vector) {
  // Reference values for seed 0 from the published SplitMix64 algorithm.
  Rng rng(0);
  EXPECT_EQ(rng.next(), 0xE220A8397B1DCDAFULL);
  EXPECT_EQ(rng.next(), 0x6E789E6AA1B965F4ULL);
}

TEST(RngTest, BelowStaysInRangeAndCoversAllValues) {
  Rng rng(7);
  std::set<std::uint64_t> seen;
  for (int i = 0; i < 500; ++i) {
    const std::uint64_t value = rng.below(5);
    EXPECT_LT(value, 5U);
    seen.insert(value);
  }
  EXPECT_EQ(seen.size(), 5U);
}

TEST(RngTest, BelowHandlesDegenerateBounds) {
  Rng rng(1);
  EXPECT_EQ(rng.below(0), 0U);
  EXPECT_EQ(rng.below(1), 0U);
}

TEST(SimClockTest, AdvancesInFixedSteps) {
  SimClock clock(0.5);
  EXPECT_EQ(clock.tick(), 0U);
  clock.advance();
  clock.advance();
  EXPECT_EQ(clock.tick(), 2U);
  EXPECT_DOUBLE_EQ(clock.seconds(), 1.0);
}

TEST(FixedTimestepTest, RejectsInvalidConfiguration) {
  using warehousesim::core::FixedTimestep;
  EXPECT_THROW(FixedTimestep(0.0), std::invalid_argument);
  EXPECT_THROW(FixedTimestep(-0.1), std::invalid_argument);
  EXPECT_THROW(FixedTimestep(std::nan("")), std::invalid_argument);
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)FixedTimestep(infinity), std::invalid_argument);
  EXPECT_THROW(FixedTimestep(0.1, 0), std::invalid_argument);
  EXPECT_NO_THROW(FixedTimestep(0.1, 1));
}

TEST(FixedTimestepTest, ReturnsWholeStepsAndCarriesTheRemainder) {
  warehousesim::core::FixedTimestep stepper(0.125);
  EXPECT_EQ(stepper.advance(0.25), 2U);
  EXPECT_EQ(stepper.advance(0.0625), 0U);
  EXPECT_EQ(stepper.advance(0.0625), 1U);  // 0.0625 + 0.0625 carried over
  EXPECT_EQ(stepper.advance(0.0), 0U);
}

TEST(FixedTimestepTest, ToleratesFloatingPointErrorAtStepBoundaries) {
  warehousesim::core::FixedTimestep stepper(0.1);
  // 0.3 / 0.1 is 2.9999999999999996 in binary floating point; it must still be three steps.
  EXPECT_EQ(stepper.advance(0.3), 3U);
}

// The same wall-clock span must produce the same number of steps whatever the frame pattern.
TEST(FixedTimestepTest, StepCountDoesNotDependOnFrameRate) {
  // All durations are dyadic fractions, so every sum below is exact.
  const std::vector<std::vector<double>> framePatterns = {
      {1.0 / 16},           {1.0 / 64}, {1.0 / 128}, {1.0 / 64, 1.0 / 32, 1.0 / 128, 1.0 / 256},
      {1.0 / 8, 1.0 / 512},
  };
  for (const auto& pattern : framePatterns) {
    warehousesim::core::FixedTimestep stepper(0.125);
    double elapsed = 0.0;
    std::uint64_t steps = 0;
    for (std::size_t i = 0; elapsed < 10.0; ++i) {
      const double frame = pattern[i % pattern.size()];
      elapsed += frame;
      steps += stepper.advance(frame);
    }
    EXPECT_EQ(steps, static_cast<std::uint64_t>(elapsed / 0.125));
  }
}

TEST(FixedTimestepTest, SpeedScalesElapsedTime) {
  warehousesim::core::FixedTimestep fast(0.125);
  fast.setSpeed(2.0);
  EXPECT_EQ(fast.advance(0.0625), 1U);
  EXPECT_EQ(fast.advance(0.25), 4U);

  warehousesim::core::FixedTimestep slow(0.125);
  slow.setSpeed(0.5);
  EXPECT_EQ(slow.advance(0.125), 0U);
  EXPECT_EQ(slow.advance(0.125), 1U);
}

TEST(FixedTimestepTest, SpeedIsClampedAndValidated) {
  using warehousesim::core::FixedTimestep;
  FixedTimestep stepper(0.1);
  stepper.setSpeed(1000.0);
  EXPECT_DOUBLE_EQ(stepper.speed(), FixedTimestep::kMaxSpeed);
  stepper.setSpeed(0.0001);
  EXPECT_DOUBLE_EQ(stepper.speed(), FixedTimestep::kMinSpeed);
  EXPECT_THROW(stepper.setSpeed(0.0), std::invalid_argument);
  EXPECT_THROW(stepper.setSpeed(-2.0), std::invalid_argument);
  EXPECT_THROW(stepper.setSpeed(std::nan("")), std::invalid_argument);
  EXPECT_DOUBLE_EQ(stepper.speed(), FixedTimestep::kMinSpeed);  // failed calls change nothing
  EXPECT_DOUBLE_EQ(FixedTimestep::clampSpeed(2.0), 2.0);
}

TEST(FixedTimestepTest, BacklogBeyondTheCapIsDroppedNotReplayed) {
  warehousesim::core::FixedTimestep stepper(0.125, 5);
  EXPECT_EQ(stepper.advance(100.0), 5U);
  EXPECT_EQ(stepper.advance(0.0625), 0U);  // no leftover from the dropped time
  EXPECT_EQ(stepper.advance(0.0625), 1U);
  EXPECT_EQ(stepper.advance(0.625), 5U);  // exactly at the cap is not dropped
}

TEST(FixedTimestepTest, SecondsUntilNextStepTracksTheAccumulator) {
  warehousesim::core::FixedTimestep stepper(0.125);
  EXPECT_DOUBLE_EQ(stepper.secondsUntilNextStep(), 0.125);
  EXPECT_EQ(stepper.advance(0.0625), 0U);
  EXPECT_DOUBLE_EQ(stepper.secondsUntilNextStep(), 0.0625);
  stepper.setSpeed(2.0);
  EXPECT_DOUBLE_EQ(stepper.secondsUntilNextStep(), 0.03125);  // real time at the new speed
  EXPECT_EQ(stepper.advance(0.03125), 1U);
  EXPECT_DOUBLE_EQ(stepper.secondsUntilNextStep(), 0.0625);
}

TEST(FixedTimestepTest, ResetForgetsCarriedTime) {
  warehousesim::core::FixedTimestep stepper(0.125);
  EXPECT_EQ(stepper.advance(0.0625), 0U);
  stepper.reset();
  EXPECT_EQ(stepper.advance(0.0625), 0U);
  EXPECT_DOUBLE_EQ(stepper.secondsUntilNextStep(), 0.0625);
}

TEST(FixedTimestepTest, RejectsInvalidElapsedTime) {
  warehousesim::core::FixedTimestep stepper(0.1);
  EXPECT_THROW((void)stepper.advance(-0.001), std::invalid_argument);
  EXPECT_THROW((void)stepper.advance(std::nan("")), std::invalid_argument);
  EXPECT_THROW((void)stepper.advance(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(SimClockTest, RejectsNonPositiveStep) {
  EXPECT_THROW(SimClock(0.0), std::invalid_argument);
  EXPECT_THROW(SimClock(-1.0), std::invalid_argument);
}

TEST(FixedTimestepTest, AlphaIsHowFarIntoTheNextStepTheCarriedTimeReaches) {
  FixedTimestep stepper(0.1);
  EXPECT_DOUBLE_EQ(stepper.alpha(), 0.0);
  EXPECT_EQ(stepper.advance(0.025), 0U);
  EXPECT_NEAR(stepper.alpha(), 0.25, 1e-12);
  EXPECT_EQ(stepper.advance(0.1), 1U);  // 0.125 s in all: one step, 0.025 s left over
  EXPECT_NEAR(stepper.alpha(), 0.25, 1e-12);
  stepper.setSpeed(2.0);
  EXPECT_EQ(stepper.advance(0.0125), 0U);  // 0.0125 s of real time is 0.025 s of simulated time
  EXPECT_NEAR(stepper.alpha(), 0.5, 1e-12);
}

TEST(FixedTimestepTest, AlphaAfterLooksAheadWithoutConsumingAnyTime) {
  FixedTimestep stepper(0.1);
  EXPECT_EQ(stepper.advance(0.03), 0U);
  EXPECT_NEAR(stepper.alphaAfter(0.0), 0.3, 1e-12);
  EXPECT_NEAR(stepper.alphaAfter(0.04), 0.7, 1e-12);
  EXPECT_DOUBLE_EQ(stepper.alphaAfter(10.0), 1.0);  // never past the end of the step
  EXPECT_NEAR(stepper.alpha(), 0.3, 1e-12);         // asking changed nothing
  EXPECT_EQ(stepper.advance(0.07), 1U);

  stepper.setSpeed(0.5);  // half speed: real time counts for half
  EXPECT_NEAR(stepper.alphaAfter(0.04), 0.2, 1e-12);
  EXPECT_THROW((void)stepper.alphaAfter(-0.1), std::invalid_argument);
  EXPECT_THROW((void)stepper.alphaAfter(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

TEST(FixedTimestepTest, AFrameRateOnlyChangesWhichInstantsTheDisplayIsSampledAt) {
  // However the same 1.05 s of real time is cut into frames, the simulation is owed the same
  // steps and the display ends the same fraction of the way into the next one.
  for (const double frame : {1.0 / 24.0, 1.0 / 60.0, 1.0 / 144.0, 0.3}) {
    FixedTimestep stepper(0.1);
    std::uint32_t steps = 0;
    const int frames = static_cast<int>(std::lround(1.05 / frame));
    for (int i = 0; i < frames; ++i) {
      steps += stepper.advance(frame);
    }
    const double total = frames * frame;
    EXPECT_EQ(steps, static_cast<std::uint32_t>(std::floor(total / 0.1 + 1e-9))) << frame;
    EXPECT_NEAR(stepper.alpha(), total / 0.1 - static_cast<double>(steps), 1e-6) << frame;
  }
}

TEST(FixedTimestepTest, ResetCanResumeFromAPositionInsideAStep) {
  FixedTimestep stepper(0.1);
  EXPECT_EQ(stepper.advance(0.06), 0U);
  stepper.reset(0.6);
  EXPECT_NEAR(stepper.alpha(), 0.6, 1e-12);
  EXPECT_NEAR(stepper.secondsUntilNextStep(), 0.04, 1e-12);
  EXPECT_EQ(stepper.advance(0.04), 1U);

  stepper.reset(7.0);  // clamped to the end of the step
  EXPECT_DOUBLE_EQ(stepper.alpha(), 1.0);
  stepper.reset(-3.0);
  EXPECT_DOUBLE_EQ(stepper.alpha(), 0.0);
  stepper.reset(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(stepper.alpha(), 0.0);
  stepper.reset();
  EXPECT_DOUBLE_EQ(stepper.alpha(), 0.0);
}

// ---- angles and continuous positions ---------------------------------------------------------

TEST(AngleTest, CardinalDirectionsRunClockwiseFromEastWithYPointingDown) {
  EXPECT_DOUBLE_EQ(angleOf(Direction::East), 0.0);
  EXPECT_DOUBLE_EQ(angleOf(Direction::South), kPi / 2.0);
  EXPECT_DOUBLE_EQ(angleOf(Direction::West), kPi);
  EXPECT_DOUBLE_EQ(angleOf(Direction::North), -kPi / 2.0);
  for (const Direction d : warehousesim::core::kAllDirections) {
    const Point step = warehousesim::core::offsetOf(d);
    EXPECT_NEAR(std::cos(angleOf(d)), step.x, 1e-12);
    EXPECT_NEAR(std::sin(angleOf(d)), step.y, 1e-12);
  }
}

TEST(AngleTest, WrappingKeepsEveryAngleInTheHalfOpenRangeMinusPiToPi) {
  EXPECT_DOUBLE_EQ(wrapAngle(0.0), 0.0);
  EXPECT_DOUBLE_EQ(wrapAngle(kPi), kPi);   // +pi is in the range...
  EXPECT_DOUBLE_EQ(wrapAngle(-kPi), kPi);  // ...and -pi is the same angle, so it maps to +pi
  EXPECT_NEAR(wrapAngle(3.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
  EXPECT_NEAR(wrapAngle(-3.0 * kPi / 2.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(wrapAngle(100.0 * kPi + 0.25), 0.25, 1e-9);
  Rng rng(3);
  for (int i = 0; i < 1000; ++i) {
    const double angle = (static_cast<double>(rng.below(2'000'001)) - 1'000'000.0) / 1000.0;
    const double wrapped = wrapAngle(angle);
    EXPECT_GT(wrapped, -kPi);
    EXPECT_LE(wrapped, kPi);
    EXPECT_NEAR(std::cos(wrapped), std::cos(angle), 1e-9);
    EXPECT_NEAR(std::sin(wrapped), std::sin(angle), 1e-9);
  }
}

TEST(AngleTest, TheShortestRotationTakesTheShortWayRoundAndUTurnsGoClockwise) {
  EXPECT_NEAR(shortestRotation(0.0, kPi / 2.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(shortestRotation(kPi / 2.0, 0.0), -kPi / 2.0, 1e-12);
  // North to West is a quarter turn anticlockwise, not three quarters clockwise.
  EXPECT_NEAR(shortestRotation(angleOf(Direction::North), angleOf(Direction::West)), -kPi / 2.0,
              1e-12);
  EXPECT_NEAR(shortestRotation(angleOf(Direction::West), angleOf(Direction::North)), kPi / 2.0,
              1e-12);
  EXPECT_NEAR(shortestRotation(angleOf(Direction::East), angleOf(Direction::West)), kPi, 1e-12);
  EXPECT_NEAR(shortestRotation(angleOf(Direction::West), angleOf(Direction::East)), kPi, 1e-12);
  EXPECT_NEAR(shortestRotation(angleOf(Direction::North), angleOf(Direction::South)), kPi, 1e-12);
  EXPECT_NEAR(shortestRotation(1.0, 1.0), 0.0, 1e-15);
}

TEST(AngleTest, InterpolatingAnAngleStaysOnTheShortArcAndHitsItsEndsExactly) {
  const double west = angleOf(Direction::West);
  const double north = angleOf(Direction::North);
  EXPECT_DOUBLE_EQ(lerpAngle(west, north, 0.0), west);
  EXPECT_DOUBLE_EQ(lerpAngle(west, north, 1.0), north);
  EXPECT_DOUBLE_EQ(lerpAngle(west, north, -4.0), west);  // clamped
  EXPECT_DOUBLE_EQ(lerpAngle(west, north, 9.0), north);
  // Half way from west (pi) to north (-pi/2) is north-west, 5 pi / 4 = -3 pi / 4, not east.
  EXPECT_NEAR(lerpAngle(west, north, 0.5), -3.0 * kPi / 4.0, 1e-12);
  EXPECT_NEAR(lerpAngle(0.0, kPi / 2.0, 0.25), kPi / 8.0, 1e-12);
}

TEST(AngleTest, TheNearestDirectionRoundsToTheClosestCardinal) {
  for (const Direction d : warehousesim::core::kAllDirections) {
    EXPECT_EQ(nearestDirection(angleOf(d)), d);
    EXPECT_EQ(nearestDirection(angleOf(d) + 0.3), d);
    EXPECT_EQ(nearestDirection(angleOf(d) - 0.3), d);
    EXPECT_EQ(nearestDirection(angleOf(d) + 2.0 * kPi), d);
  }
  EXPECT_EQ(nearestDirection(0.7), Direction::East);   // 40 degrees
  EXPECT_EQ(nearestDirection(0.9), Direction::South);  // 52 degrees
}

TEST(Vec2Test, ArithmeticIsComponentwiseAndLerpHitsBothEndsExactly) {
  const Vec2 a{0.1, 0.7};
  const Vec2 b{0.3, -0.2};
  EXPECT_EQ(a + b, (Vec2{0.1 + 0.3, 0.7 + -0.2}));
  EXPECT_EQ(b - a, (Vec2{0.3 - 0.1, -0.2 - 0.7}));
  EXPECT_EQ(a * 2.0, (Vec2{0.2, 1.4}));
  EXPECT_EQ(lerp(a, b, 0.0), a);  // exact, with no rounding from (b - a)
  EXPECT_EQ(lerp(a, b, 1.0), b);
  EXPECT_NEAR(lerp(a, b, 0.5).x, 0.2, 1e-15);
  EXPECT_NEAR(lerp(a, b, 0.5).y, 0.25, 1e-15);
  EXPECT_EQ(Vec2{}, (Vec2{0.0, 0.0}));
}

}  // namespace
