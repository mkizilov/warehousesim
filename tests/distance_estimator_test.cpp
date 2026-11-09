#include "warehousesim/tasks/distance_estimator.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/testing/map_generators.hpp"
#include "warehousesim/testing/reference_search.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::DirectionSet;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::tasks::ManhattanDistance;
using warehousesim::tasks::PathDistance;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

TEST(ManhattanDistanceTest, IsStraightLineDistanceAndIgnoresWalls) {
  const Warehouse w = layout({".#.", ".#.", ".#."});
  ManhattanDistance estimator;
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(2, 2)), 4);
  EXPECT_EQ(estimator.distance(w, P(2, 2), P(0, 0)), 4);
  EXPECT_EQ(estimator.distance(w, P(1, 1), P(1, 1)), 0);
  EXPECT_TRUE(estimator.distance(w, P(0, 0), P(2, 0)).has_value());  // walled off, yet "reachable"
}

TEST(PathDistanceTest, FollowsTheRouteAroundObstacles) {
  const Warehouse w = layout({"..#..", "..#..", "....."});
  PathDistance estimator;
  EXPECT_EQ(estimator.distance(w, P(1, 0), P(3, 0)), 6);  // down, across, up
  EXPECT_EQ(estimator.distance(w, P(3, 0), P(1, 0)), 6);
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(0, 0)), 0);
}

TEST(PathDistanceTest, ReportsUnreachableAndInvalidEndpointsAsNullopt) {
  const Warehouse w = layout({".#.", ".#.", ".#."});
  PathDistance estimator;
  EXPECT_FALSE(estimator.distance(w, P(0, 0), P(2, 0)).has_value());  // wall in between
  EXPECT_FALSE(estimator.distance(w, P(0, 0), P(1, 1)).has_value());  // goal is a wall
  EXPECT_FALSE(estimator.distance(w, P(1, 1), P(0, 0)).has_value());  // start is a wall
  EXPECT_FALSE(estimator.distance(w, P(0, 0), P(9, 9)).has_value());  // outside the grid
}

TEST(PathDistanceTest, RemembersAnswersIncludingUnreachableOnes) {
  const Warehouse w = layout({".#.", "..."});
  PathDistance estimator;
  EXPECT_EQ(estimator.cacheSize(), 0U);
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(2, 0)), 4);
  EXPECT_EQ(estimator.cacheSize(), 1U);
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(2, 0)), 4);  // served from the cache
  EXPECT_EQ(estimator.cacheSize(), 1U);
  EXPECT_EQ(estimator.distance(w, P(2, 0), P(0, 0)), 4);  // a different pair
  EXPECT_EQ(estimator.cacheSize(), 2U);
  EXPECT_FALSE(estimator.distance(w, P(0, 0), P(1, 0)).has_value());
  EXPECT_EQ(estimator.cacheSize(), 3U);
  EXPECT_FALSE(estimator.distance(w, P(0, 0), P(1, 0)).has_value());
  EXPECT_EQ(estimator.cacheSize(), 3U);
  estimator.reset();
  EXPECT_EQ(estimator.cacheSize(), 0U);
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(2, 0)), 4);  // still right after a reset
}

TEST(PathDistanceTest, TheCacheIsBoundedAndStaysCorrectWhenItFillsUp) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(8, 8);
  PathDistance estimator(5);
  Rng rng(6);
  for (int i = 0; i < 300; ++i) {
    const Point from = P(static_cast<int>(rng.below(8)), static_cast<int>(rng.below(8)));
    const Point to = P(static_cast<int>(rng.below(8)), static_cast<int>(rng.below(8)));
    EXPECT_EQ(estimator.distance(w, from, to), warehousesim::core::manhattanDistance(from, to));
    EXPECT_LE(estimator.cacheSize(), 5U);
  }
}

TEST(PathDistanceTest, StaysRightWhenMoreGoalsAreAskedAboutThanItKeepsDistancesFor) {
  Rng rng(31);
  const Warehouse w = warehousesim::testing::makeRandomObstacleMap(14, 14, 200, rng);
  PathDistance estimator;
  std::vector<Point> cells;
  for (int y = 0; y < 14; ++y) {
    for (int x = 0; x < 14; ++x) {
      cells.push_back(P(x, y));
    }
  }
  // Far more than 32 goals, each asked about again and again from other starts.
  for (int round = 0; round < 3; ++round) {
    for (std::size_t g = 0; g < cells.size(); ++g) {
      const Point goal = cells[(g * 7 + static_cast<std::size_t>(round)) % cells.size()];
      const Point from = cells[rng.below(cells.size())];
      ASSERT_EQ(estimator.distance(w, from, goal),
                warehousesim::testing::shortestDistance(w, from, goal, {}, DirectionSet::all()))
          << from.x << "," << from.y << " to " << goal.x << "," << goal.y;
    }
  }
}

TEST(PathDistanceTest, AnEditedLayoutGivesNewAnswersOnceTheEstimatorIsToldToForget) {
  Warehouse w = layout({"....."});
  PathDistance estimator;
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(4, 0)), 4);
  w.set(P(2, 0), warehousesim::world::CellType::Wall);
  estimator.reset();
  EXPECT_FALSE(estimator.distance(w, P(0, 0), P(4, 0)).has_value());
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(1, 0)), 1);
  w.set(P(2, 0), warehousesim::world::CellType::Free);
  estimator.reset();
  EXPECT_EQ(estimator.distance(w, P(0, 0), P(4, 0)), 4);
}

TEST(PathDistanceTest, AnAnswerIsRememberedOnceWhateverTheDirectionOfTheQuestion) {
  const Warehouse w = layout({"...", "..."});
  PathDistance estimator;
  (void)estimator.distance(w, P(0, 0), P(2, 1));
  (void)estimator.distance(w, P(2, 1), P(0, 0));
  EXPECT_EQ(estimator.cacheSize(), 2U) << "(from, to) and (to, from) are two questions";
  (void)estimator.distance(w, P(0, 0), P(2, 1));
  EXPECT_EQ(estimator.cacheSize(), 2U);
}

TEST(PathDistanceTest, MatchesBreadthFirstSearchOnRandomMaps) {
  Rng rng(1010);
  PathDistance estimator;
  int reachable = 0;
  int unreachable = 0;
  for (int map = 0; map < 200; ++map) {
    const int width = 2 + static_cast<int>(rng.below(14));
    const int height = 2 + static_cast<int>(rng.below(14));
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        width, height, static_cast<std::uint32_t>(rng.below(450)), rng);
    estimator.reset();  // the cache is per layout
    for (int query = 0; query < 10; ++query) {
      const Point from = P(static_cast<int>(rng.below(static_cast<std::uint64_t>(width))),
                           static_cast<int>(rng.below(static_cast<std::uint64_t>(height))));
      const Point to = P(static_cast<int>(rng.below(static_cast<std::uint64_t>(width))),
                         static_cast<int>(rng.below(static_cast<std::uint64_t>(height))));
      const auto expected =
          warehousesim::testing::shortestDistance(w, from, to, {}, DirectionSet::all());
      EXPECT_EQ(estimator.distance(w, from, to), expected);
      (expected ? reachable : unreachable) += 1;
    }
  }
  EXPECT_GT(reachable, 500);
  EXPECT_GT(unreachable, 200);
}

}  // namespace
