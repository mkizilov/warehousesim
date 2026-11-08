#include <gtest/gtest.h>

#include <set>
#include <unordered_set>
#include <vector>

#include "warehousesim/core/clock.hpp"
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

TEST(SimClockTest, RejectsNonPositiveStep) {
  EXPECT_THROW(SimClock(0.0), std::invalid_argument);
  EXPECT_THROW(SimClock(-1.0), std::invalid_argument);
}

}  // namespace
