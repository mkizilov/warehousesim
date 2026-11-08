#include "warehousesim/world/layouts.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/world/world.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::world::CellType;
using warehousesim::world::LayoutInfo;
using warehousesim::world::PlaceStatus;
using warehousesim::world::Warehouse;

// Number of walkable cells reachable from the first walkable cell by orthogonal steps.
std::size_t reachableWalkableCells(const Warehouse& w) {
  std::set<Point> visited;
  std::queue<Point> open;
  for (std::int32_t y = 0; y < w.height() && open.empty(); ++y) {
    for (std::int32_t x = 0; x < w.width() && open.empty(); ++x) {
      if (w.isWalkable(Point{x, y})) {
        open.push(Point{x, y});
        visited.insert(Point{x, y});
      }
    }
  }
  while (!open.empty()) {
    const Point current = open.front();
    open.pop();
    for (const Point next : w.walkableNeighbors(current)) {
      if (visited.insert(next).second) {
        open.push(next);
      }
    }
  }
  return visited.size();
}

std::size_t walkableCellCount(const Warehouse& w) {
  std::size_t count = 0;
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    if (warehousesim::world::isWalkable(type)) {
      count += w.cellsOfType(type).size();
    }
  }
  return count;
}

TEST(LayoutsTest, AtLeastTwoLayoutsWithUniqueNamesAndDescriptions) {
  const auto layouts = warehousesim::world::predefinedLayouts();
  EXPECT_TRUE(layouts.size() >= 2U);
  std::set<std::string> names;
  for (const LayoutInfo& layout : layouts) {
    EXPECT_FALSE(layout.name.empty());
    EXPECT_FALSE(layout.description.empty());
    EXPECT_TRUE(layout.make != nullptr);
    EXPECT_TRUE(names.insert(std::string(layout.name)).second);
  }
}

TEST(LayoutsTest, EveryLayoutIsOneConnectedWalkableRegion) {
  for (const LayoutInfo& layout : warehousesim::world::predefinedLayouts()) {
    const Warehouse w = layout.make();
    EXPECT_TRUE(walkableCellCount(w) > 0U);
    EXPECT_EQ(reachableWalkableCells(w), walkableCellCount(w));
  }
}

TEST(LayoutsTest, EveryLayoutSupportsTheWorkflowPickupToDropoff) {
  for (const LayoutInfo& layout : warehousesim::world::predefinedLayouts()) {
    const Warehouse w = layout.make();
    EXPECT_FALSE(w.cellsOfType(CellType::Dropoff).empty());
    EXPECT_FALSE(w.cellsOfType(CellType::Pickup).empty() && w.shelfAccessPoints().empty());
  }
}

TEST(LayoutsTest, EveryLayoutRoundTripsThroughItsAsciiRows) {
  for (const LayoutInfo& layout : warehousesim::world::predefinedLayouts()) {
    const Warehouse w = layout.make();
    const auto reparsed = Warehouse::fromRows(w.toRows());
    ASSERT_TRUE(reparsed.has_value());
    EXPECT_EQ(reparsed->toRows(), w.toRows());
  }
}

TEST(LayoutsTest, AWorldCanHostARobotOnEveryWalkableCellAndNoObstacleCell) {
  for (const LayoutInfo& layout : warehousesim::world::predefinedLayouts()) {
    warehousesim::world::World world(layout.make());
    std::uint32_t id = 1;
    for (std::int32_t y = 0; y < world.size().height(); ++y) {
      for (std::int32_t x = 0; x < world.size().width(); ++x) {
        const Point p{x, y};
        const PlaceStatus status = world.addRobot(RobotId{id}, p);
        if (world.isWalkable(p)) {
          EXPECT_TRUE(status == PlaceStatus::Ok);
          ++id;
        } else {
          EXPECT_TRUE(status == PlaceStatus::BlockedByObstacle);
        }
      }
    }
    EXPECT_EQ(world.occupancy().robotCount(), walkableCellCount(world.layout()));
  }
}

TEST(LayoutsTest, DemoLayoutIsTheOpenHallWithoutWallsOrPickups) {
  const Warehouse w = warehousesim::world::makeDemoWarehouse();
  EXPECT_EQ(w.width(), 20);
  EXPECT_EQ(w.height(), 12);
  EXPECT_TRUE(w.cellsOfType(CellType::Wall).empty());
  EXPECT_TRUE(w.cellsOfType(CellType::Pickup).empty());
  EXPECT_TRUE(w.cellsOfType(CellType::Charging).empty());
  EXPECT_EQ(w.cellsOfType(CellType::Dropoff).size(), 4U);
  EXPECT_EQ(w.cellsOfType(CellType::Shelf).size(), 72U);  // six shelf rows of 3 blocks x 4 cells
}

TEST(LayoutsTest, CompactLayoutHasAllSixCellTypesAndAWalledBorder) {
  const Warehouse w = warehousesim::world::makeCompactWarehouse();
  EXPECT_EQ(w.width(), 12);
  EXPECT_EQ(w.height(), 8);
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    EXPECT_FALSE(w.cellsOfType(type).empty());
  }
  for (std::int32_t x = 0; x < w.width(); ++x) {
    EXPECT_EQ(w.at(Point{x, 0}), CellType::Wall);
    EXPECT_EQ(w.at(Point{x, w.height() - 1}), CellType::Wall);
  }
  for (std::int32_t y = 0; y < w.height(); ++y) {
    EXPECT_EQ(w.at(Point{0, y}), CellType::Wall);
    EXPECT_EQ(w.at(Point{w.width() - 1, y}), CellType::Wall);
  }
}

TEST(LayoutsTest, FulfilmentLayoutHasAllSixCellTypesAndAWalledBorder) {
  const Warehouse w = warehousesim::world::makeFulfilmentWarehouse();
  EXPECT_EQ(w.width(), 28);
  EXPECT_EQ(w.height(), 16);
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    EXPECT_FALSE(w.cellsOfType(type).empty());
  }
  EXPECT_EQ(w.cellsOfType(CellType::Pickup).size(), 6U);
  EXPECT_EQ(w.cellsOfType(CellType::Charging).size(), 9U);
  EXPECT_EQ(w.cellsOfType(CellType::Dropoff).size(), 14U);
  for (std::int32_t x = 0; x < w.width(); ++x) {
    EXPECT_EQ(w.at(Point{x, 0}), CellType::Wall);
    EXPECT_EQ(w.at(Point{x, w.height() - 1}), CellType::Wall);
  }
  for (std::int32_t y = 0; y < w.height(); ++y) {
    EXPECT_EQ(w.at(Point{0, y}), CellType::Wall);
    EXPECT_EQ(w.at(Point{w.width() - 1, y}), CellType::Wall);
  }
}

TEST(LayoutsTest, LookupByNameFindsEachLayoutAndRejectsUnknownNames) {
  for (const LayoutInfo& layout : warehousesim::world::predefinedLayouts()) {
    const auto found = warehousesim::world::makePredefinedWarehouse(layout.name);
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->toRows(), layout.make().toRows());
  }
  EXPECT_FALSE(warehousesim::world::makePredefinedWarehouse("").has_value());
  EXPECT_FALSE(warehousesim::world::makePredefinedWarehouse("Demo").has_value());
  EXPECT_FALSE(warehousesim::world::makePredefinedWarehouse("nonexistent").has_value());
}

TEST(LayoutsTest, FactoriesReturnIndependentCopies) {
  Warehouse first = warehousesim::world::makeCompactWarehouse();
  first.set(Point{1, 1}, CellType::Wall);
  EXPECT_EQ(warehousesim::world::makeCompactWarehouse().at(Point{1, 1}), CellType::Charging);
}

}  // namespace
