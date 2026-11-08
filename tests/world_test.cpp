#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

TEST(WarehouseTest, NewWarehouseIsAllFloor) {
  Warehouse w(3, 2);
  EXPECT_EQ(w.width(), 3);
  EXPECT_EQ(w.height(), 2);
  EXPECT_EQ(w.at(P(2, 1)), CellType::Floor);
  EXPECT_TRUE(w.isWalkable(P(0, 0)));
}

TEST(WarehouseTest, RejectsNonPositiveDimensions) {
  EXPECT_THROW(Warehouse(0, 5), std::invalid_argument);
  EXPECT_THROW(Warehouse(5, -1), std::invalid_argument);
}

TEST(WarehouseTest, BoundsAndWalkability) {
  Warehouse w(3, 3);
  w.set(P(1, 1), CellType::Shelf);
  w.set(P(2, 2), CellType::Wall);
  w.set(P(0, 2), CellType::Station);
  EXPECT_FALSE(w.isWalkable(P(1, 1)));
  EXPECT_FALSE(w.isWalkable(P(2, 2)));
  EXPECT_TRUE(w.isWalkable(P(0, 2)));
  EXPECT_FALSE(w.inBounds(P(-1, 0)));
  EXPECT_FALSE(w.inBounds(P(3, 0)));
  EXPECT_FALSE(w.isWalkable(P(0, 3)));
}

TEST(WarehouseTest, AccessOutsideGridThrows) {
  Warehouse w(2, 2);
  EXPECT_THROW((void)w.at(P(2, 0)), std::out_of_range);
  EXPECT_THROW(w.set(P(0, -1), CellType::Wall), std::out_of_range);
}

TEST(WarehouseTest, FromRowsParsesLayoutAndRoundTrips) {
  const std::vector<std::string> rows = {"..S", "D#."};
  const auto w = Warehouse::fromRows(rows);
  ASSERT_TRUE(w.has_value());
  EXPECT_EQ(w->width(), 3);
  EXPECT_EQ(w->height(), 2);
  EXPECT_EQ(w->at(P(2, 0)), CellType::Shelf);
  EXPECT_EQ(w->at(P(0, 1)), CellType::Station);
  EXPECT_EQ(w->at(P(1, 1)), CellType::Wall);
  EXPECT_EQ(w->toRows(), rows);
}

TEST(WarehouseTest, FromRowsRejectsInvalidLayouts) {
  EXPECT_FALSE(Warehouse::fromRows({}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({""}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({"..", "."}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({"..", ".x"}).has_value());
}

TEST(WarehouseTest, CellsOfTypeAreRowMajor) {
  const auto w = Warehouse::fromRows({"S.S", ".S."});
  ASSERT_TRUE(w.has_value());
  const std::vector<Point> shelves = w->cellsOfType(CellType::Shelf);
  ASSERT_EQ(shelves.size(), 3U);
  EXPECT_EQ(shelves[0], P(0, 0));
  EXPECT_EQ(shelves[1], P(2, 0));
  EXPECT_EQ(shelves[2], P(1, 1));
}

TEST(WarehouseTest, ShelfAccessPointsAreFloorCellsNextToShelves) {
  const auto w = Warehouse::fromRows({"S..", "...", "..D"});
  ASSERT_TRUE(w.has_value());
  const std::vector<Point> expected = {P(1, 0), P(0, 1)};
  EXPECT_EQ(w->shelfAccessPoints(), expected);
}

TEST(WarehouseTest, DemoWarehouseSupportsTaskGeneration) {
  const Warehouse w = warehousesim::world::makeDemoWarehouse();
  EXPECT_EQ(w.width(), 20);
  EXPECT_EQ(w.height(), 12);
  EXPECT_FALSE(w.shelfAccessPoints().empty());
  EXPECT_FALSE(w.cellsOfType(CellType::Station).empty());
}

}  // namespace
