#include "warehousesim/core/point_set.hpp"

#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "warehousesim/core/rng.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::PointSet;
using warehousesim::core::Rng;

Point P(std::int32_t x, std::int32_t y) { return Point{x, y}; }

TEST(PointSetTest, AnEmptySetHoldsNothing) {
  const PointSet set;
  EXPECT_TRUE(set.empty());
  EXPECT_EQ(set.size(), 0U);
  EXPECT_FALSE(set.contains(P(0, 0)));
  EXPECT_EQ(set.count(P(3, 4)), 0U);
  EXPECT_TRUE(set.begin() == set.end());
}

TEST(PointSetTest, InsertSaysWhetherThePointWasNew) {
  PointSet set;
  EXPECT_TRUE(set.insert(P(1, 2)));
  EXPECT_FALSE(set.insert(P(1, 2)));
  EXPECT_TRUE(set.insert(P(2, 1)));
  EXPECT_EQ(set.size(), 2U);
  EXPECT_TRUE(set.contains(P(1, 2)));
  EXPECT_TRUE(set.contains(P(2, 1)));
  EXPECT_FALSE(set.contains(P(2, 2)));
  EXPECT_EQ(set.count(P(1, 2)), 1U);
}

TEST(PointSetTest, NegativeAndHugeCoordinatesAreOrdinaryPoints) {
  PointSet set{P(-1, 0), P(0, -1), P(-1, -1), P(2147483647, -2147483647 - 1), P(0, 0)};
  EXPECT_EQ(set.size(), 5U);
  EXPECT_TRUE(set.contains(P(-1, 0)));
  EXPECT_TRUE(set.contains(P(0, -1)));
  EXPECT_TRUE(set.contains(P(2147483647, -2147483647 - 1)));
  EXPECT_FALSE(set.contains(P(1, 0)));
  EXPECT_FALSE(set.contains(P(-2147483647 - 1, 2147483647)));
}

TEST(PointSetTest, IterationIsInTheOrderOfFirstInsertion) {
  PointSet set;
  const std::vector<Point> order{P(5, 5), P(0, 0), P(9, 1), P(0, 0), P(2, 7), P(5, 5), P(1, 1)};
  for (const Point p : order) {
    (void)set.insert(p);
  }
  EXPECT_EQ((std::vector<Point>(set.begin(), set.end())),
            (std::vector<Point>{P(5, 5), P(0, 0), P(9, 1), P(2, 7), P(1, 1)}));
}

TEST(PointSetTest, BuiltFromARangeWithDuplicates) {
  const std::vector<Point> points{P(1, 1), P(2, 2), P(1, 1), P(3, 3)};
  const PointSet set(points.begin(), points.end());
  EXPECT_EQ(set.size(), 3U);
  EXPECT_TRUE(set.contains(P(3, 3)));
}

TEST(PointSetTest, ClearForgetsEverythingAndTheSetIsUsableAgain) {
  PointSet set;
  for (int i = 0; i < 100; ++i) {
    (void)set.insert(P(i, i));
  }
  set.clear();
  EXPECT_TRUE(set.empty());
  EXPECT_FALSE(set.contains(P(5, 5)));
  EXPECT_TRUE(set.insert(P(5, 5)));
  EXPECT_EQ(set.size(), 1U);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(set.contains(P(i, i)), i == 5) << "nothing of the old contents may be found";
  }
  EXPECT_TRUE(set.insert(P(7, 7)));
  EXPECT_EQ(set.size(), 2U);
}

TEST(PointSetTest, ReserveChangesNothingYouCanSee) {
  PointSet set;
  set.reserve(1000);
  EXPECT_TRUE(set.empty());
  (void)set.insert(P(1, 1));
  EXPECT_TRUE(set.contains(P(1, 1)));
  set.reserve(0);
  EXPECT_TRUE(set.contains(P(1, 1)));
}

TEST(PointSetTest, GrowingKeepsEveryPointAcrossManyRehashes) {
  PointSet set;
  std::set<Point> model;
  for (int i = 0; i < 20000; ++i) {
    const Point p = P(i % 211 - 100, i / 211 - 40);
    EXPECT_EQ(set.insert(p), model.insert(p).second);
  }
  EXPECT_EQ(set.size(), model.size());
  for (const Point p : model) {
    ASSERT_TRUE(set.contains(p)) << p.x << "," << p.y;
  }
}

TEST(PointSetTest, AgreesWithAnOrdinarySetOnRandomOperations) {
  Rng rng(77);
  for (int round = 0; round < 30; ++round) {
    PointSet set;
    std::set<Point> model;
    const std::uint64_t spread = 2 + rng.below(40);
    const std::uint64_t operations = 1 + rng.below(400);
    for (std::uint64_t i = 0; i < operations; ++i) {
      const Point p = P(static_cast<std::int32_t>(rng.below(spread)) - 3,
                        static_cast<std::int32_t>(rng.below(spread)) - 3);
      if (rng.below(4) == 0) {
        ASSERT_EQ(set.contains(p), model.contains(p));
      } else {
        ASSERT_EQ(set.insert(p), model.insert(p).second);
      }
    }
    ASSERT_EQ(set.size(), model.size());
    for (std::int32_t x = -3; x < static_cast<std::int32_t>(spread); ++x) {
      for (std::int32_t y = -3; y < static_cast<std::int32_t>(spread); ++y) {
        ASSERT_EQ(set.contains(P(x, y)), model.contains(P(x, y)));
      }
    }
  }
}

TEST(PointSetTest, ACopyIsIndependentOfTheOriginal) {
  PointSet original{P(1, 1), P(2, 2)};
  PointSet copy = original;
  (void)copy.insert(P(3, 3));
  EXPECT_EQ(original.size(), 2U);
  EXPECT_FALSE(original.contains(P(3, 3)));
  EXPECT_TRUE(copy.contains(P(1, 1)));
  EXPECT_TRUE(copy.contains(P(3, 3)));
}

}  // namespace
