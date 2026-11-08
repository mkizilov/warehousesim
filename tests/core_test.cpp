#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/fixed_timestep.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"

namespace {

using warehousesim::core::manhattanDistance;
using warehousesim::core::Point;
using warehousesim::core::PointHash;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::core::SimClock;
using warehousesim::core::TaskId;

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

}  // namespace
