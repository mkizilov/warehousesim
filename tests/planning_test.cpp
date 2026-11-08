#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/world/layouts.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::manhattanDistance;
using warehousesim::core::Point;
using warehousesim::planning::AStarPlanner;
using warehousesim::planning::BlockedSet;
using warehousesim::planning::Path;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

void expectValidPath(const Warehouse& w, Point start, const Path& path) {
  Point previous = start;
  for (const Point step : path) {
    EXPECT_EQ(manhattanDistance(previous, step), 1);
    EXPECT_TRUE(w.isWalkable(step));
    previous = step;
  }
}

TEST(AStarTest, StraightCorridor) {
  const Warehouse w = layout({"....."});
  const auto path = AStarPlanner{}.findPath(w, P(0, 0), P(4, 0), {});
  ASSERT_TRUE(path.has_value());
  const Path expected = {P(1, 0), P(2, 0), P(3, 0), P(4, 0)};
  EXPECT_EQ(*path, expected);
}

TEST(AStarTest, StartEqualsGoalYieldsEmptyPath) {
  const Warehouse w = layout({"..."});
  const auto path = AStarPlanner{}.findPath(w, P(1, 0), P(1, 0), {});
  ASSERT_TRUE(path.has_value());
  EXPECT_TRUE(path->empty());
}

TEST(AStarTest, RoutesAroundObstacleWithShortestLength) {
  const Warehouse w = layout({".....", "..#..", "..#..", "....."});
  const auto path = AStarPlanner{}.findPath(w, P(0, 1), P(4, 1), {});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 6U);  // Manhattan distance 4 plus a two-cell detour
  EXPECT_EQ(path->back(), P(4, 1));
  expectValidPath(w, P(0, 1), *path);
}

TEST(AStarTest, ShelvesAndWallsBlockButSpecialCellsDoNot) {
  const Warehouse w = layout({".SD.", ".#.."});
  const auto path = AStarPlanner{}.findPath(w, P(0, 0), P(3, 0), {});
  EXPECT_FALSE(path.has_value());  // shelf and wall seal the only connection
  const auto toDropoff = AStarPlanner{}.findPath(w, P(3, 1), P(2, 0), {});
  ASSERT_TRUE(toDropoff.has_value());

  const Warehouse special = layout({"PDC"});
  const auto across = AStarPlanner{}.findPath(special, P(0, 0), P(2, 0), {});
  ASSERT_TRUE(across.has_value());
  EXPECT_EQ(across->size(), 2U);  // pickup, dropoff and charging cells can all be crossed
}

TEST(AStarTest, FindsPathsOnEveryPredefinedLayout) {
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    const Warehouse w = info.make();
    const auto stations = w.cellsOfType(warehousesim::world::CellType::Dropoff);
    const auto origins = w.cellsOfType(warehousesim::world::CellType::Free);
    ASSERT_FALSE(stations.empty());
    ASSERT_FALSE(origins.empty());
    for (const Point station : stations) {
      const auto path = AStarPlanner{}.findPath(w, origins.front(), station, {});
      ASSERT_TRUE(path.has_value());
      expectValidPath(w, origins.front(), *path);
    }
  }
}

TEST(AStarTest, UnreachableOrInvalidGoalReturnsNullopt) {
  const Warehouse w = layout({".#.", ".#.", ".#."});
  EXPECT_FALSE(AStarPlanner{}.findPath(w, P(0, 0), P(2, 0), {}).has_value());
  EXPECT_FALSE(AStarPlanner{}.findPath(w, P(0, 0), P(1, 0), {}).has_value());
  EXPECT_FALSE(AStarPlanner{}.findPath(w, P(0, 0), P(9, 9), {}).has_value());
  EXPECT_FALSE(AStarPlanner{}.findPath(w, P(1, 0), P(0, 0), {}).has_value());
}

TEST(AStarTest, BlockedCellsAreAvoidedAndStartMayBeBlocked) {
  const Warehouse w = layout({"...", "..."});
  const BlockedSet blocked{P(0, 0), P(1, 0)};
  const auto path = AStarPlanner{}.findPath(w, P(0, 0), P(2, 0), blocked);
  ASSERT_TRUE(path.has_value());
  const Path expected = {P(0, 1), P(1, 1), P(2, 1), P(2, 0)};
  EXPECT_EQ(*path, expected);
}

TEST(AStarTest, BlockedGoalIsUnreachable) {
  const Warehouse w = layout({"..."});
  EXPECT_FALSE(AStarPlanner{}.findPath(w, P(0, 0), P(2, 0), BlockedSet{P(2, 0)}).has_value());
}

TEST(AStarTest, FullyBlockedCorridorIsUnreachable) {
  const Warehouse w = layout({"..."});
  EXPECT_FALSE(AStarPlanner{}.findPath(w, P(0, 0), P(2, 0), BlockedSet{P(1, 0)}).has_value());
}

TEST(AStarTest, LengthMatchesManhattanDistanceOnOpenGrid) {
  const Warehouse w(12, 9);
  for (int i = 0; i < 12; ++i) {
    const Point goal = P(11 - i, (i * 5) % 9);
    const auto path = AStarPlanner{}.findPath(w, P(i, 8 - (i % 9)), goal, {});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(static_cast<int>(path->size()), manhattanDistance(P(i, 8 - (i % 9)), goal));
  }
}

TEST(AStarTest, ResultIsDeterministicAcrossCalls) {
  const Warehouse w = warehousesim::world::makeDemoWarehouse();
  const auto first = AStarPlanner{}.findPath(w, P(0, 0), P(19, 10), {});
  const auto second = AStarPlanner{}.findPath(w, P(0, 0), P(19, 10), {});
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first, second);
}

}  // namespace
