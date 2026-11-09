#include "warehousesim/world/warehouse.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/world/cell_type.hpp"
#include "warehousesim/world/grid_size.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::world::CellType;
using warehousesim::world::GridSize;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// ---- CellType --------------------------------------------------------------------------

TEST(CellTypeTest, OnlyWallsAndShelvesAreObstacles) {
  using warehousesim::world::isObstacle;
  using warehousesim::world::isWalkable;
  EXPECT_TRUE(isObstacle(CellType::Wall));
  EXPECT_TRUE(isObstacle(CellType::Shelf));
  EXPECT_TRUE(isWalkable(CellType::Free));
  EXPECT_TRUE(isWalkable(CellType::Pickup));
  EXPECT_TRUE(isWalkable(CellType::Dropoff));
  EXPECT_TRUE(isWalkable(CellType::Charging));
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    EXPECT_NE(isObstacle(type), isWalkable(type));
  }
}

TEST(CellTypeTest, CharactersRoundTripAndAreUnique) {
  std::set<char> seen;
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    const char c = warehousesim::world::toChar(type);
    EXPECT_TRUE(seen.insert(c).second);
    const auto parsed = warehousesim::world::cellTypeFromChar(c);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(*parsed, type);
  }
  EXPECT_EQ(seen.size(), 6U);
}

TEST(CellTypeTest, UnknownCharactersAreRejected) {
  EXPECT_FALSE(warehousesim::world::cellTypeFromChar('x').has_value());
  EXPECT_FALSE(warehousesim::world::cellTypeFromChar(' ').has_value());
  EXPECT_FALSE(warehousesim::world::cellTypeFromChar('\0').has_value());
  EXPECT_FALSE(warehousesim::world::cellTypeFromChar('s').has_value());  // case-sensitive
}

TEST(CellTypeTest, NamesAreDistinct) {
  std::set<std::string> names;
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    names.insert(warehousesim::world::toString(type));
  }
  EXPECT_EQ(names.size(), 6U);
}

// ---- GridSize --------------------------------------------------------------------------

TEST(GridSizeTest, RejectsNonPositiveDimensions) {
  EXPECT_THROW(GridSize(0, 1), std::invalid_argument);
  EXPECT_THROW(GridSize(1, 0), std::invalid_argument);
  EXPECT_THROW(GridSize(-3, 4), std::invalid_argument);
  EXPECT_THROW(GridSize(4, -3), std::invalid_argument);
  EXPECT_NO_THROW(GridSize(1, 1));
}

TEST(GridSizeTest, CellCountDoesNotOverflow32Bits) {
  const GridSize huge(std::numeric_limits<std::int32_t>::max(),
                      std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(huge.cellCount(), 2147483647ULL * 2147483647ULL);
}

TEST(GridSizeTest, ContainsIncludesEdgesAndExcludesEverythingElse) {
  const GridSize size(4, 3);
  EXPECT_TRUE(size.contains(P(0, 0)));
  EXPECT_TRUE(size.contains(P(3, 2)));
  EXPECT_FALSE(size.contains(P(4, 2)));
  EXPECT_FALSE(size.contains(P(3, 3)));
  EXPECT_FALSE(size.contains(P(-1, 0)));
  EXPECT_FALSE(size.contains(P(0, -1)));
  EXPECT_FALSE(size.contains(P(std::numeric_limits<std::int32_t>::max(), 0)));
  EXPECT_FALSE(size.contains(P(0, std::numeric_limits<std::int32_t>::min())));
}

TEST(GridSizeTest, IndexingIsRowMajorAndInvertible) {
  const GridSize size(4, 3);
  EXPECT_EQ(size.indexOf(P(0, 0)), 0U);
  EXPECT_EQ(size.indexOf(P(3, 0)), 3U);
  EXPECT_EQ(size.indexOf(P(0, 1)), 4U);
  EXPECT_EQ(size.indexOf(P(3, 2)), 11U);
  for (std::size_t i = 0; i < size.cellCount(); ++i) {
    EXPECT_EQ(size.indexOf(size.pointAt(i)), i);
  }
}

TEST(GridSizeTest, EqualityComparesBothSides) {
  EXPECT_TRUE(GridSize(3, 2) == GridSize(3, 2));
  EXPECT_FALSE(GridSize(3, 2) == GridSize(2, 3));
}

// ---- Warehouse -------------------------------------------------------------------------

TEST(WarehouseTest, NewWarehouseIsAllFree) {
  const Warehouse w(3, 2);
  EXPECT_EQ(w.width(), 3);
  EXPECT_EQ(w.height(), 2);
  EXPECT_TRUE(w.size() == GridSize(3, 2));
  for (const Point p : {P(0, 0), P(2, 0), P(0, 1), P(2, 1)}) {
    EXPECT_EQ(w.at(p), CellType::Free);
    EXPECT_TRUE(w.isWalkable(p));
    EXPECT_FALSE(w.isObstacle(p));
  }
}

TEST(WarehouseTest, RejectsNonPositiveDimensions) {
  EXPECT_THROW(Warehouse(0, 5), std::invalid_argument);
  EXPECT_THROW(Warehouse(5, -1), std::invalid_argument);
}

TEST(WarehouseTest, BoundsAreInclusiveOfEdgesAndExclusiveOfEverythingElse) {
  const Warehouse w(3, 2);
  EXPECT_TRUE(w.inBounds(P(0, 0)));
  EXPECT_TRUE(w.inBounds(P(2, 1)));
  EXPECT_FALSE(w.inBounds(P(3, 1)));
  EXPECT_FALSE(w.inBounds(P(2, 2)));
  EXPECT_FALSE(w.inBounds(P(-1, 0)));
  EXPECT_FALSE(w.inBounds(P(0, -1)));
  EXPECT_FALSE(w.inBounds(P(std::numeric_limits<std::int32_t>::min(), 0)));
  EXPECT_FALSE(w.inBounds(P(0, std::numeric_limits<std::int32_t>::max())));
}

TEST(WarehouseTest, OutOfBoundsPointsAreNeitherWalkableNorObstacles) {
  const Warehouse w(2, 2);
  EXPECT_FALSE(w.isWalkable(P(2, 0)));
  EXPECT_FALSE(w.isObstacle(P(2, 0)));
  EXPECT_FALSE(w.isWalkable(P(-1, -1)));
  EXPECT_FALSE(w.isObstacle(P(-1, -1)));
}

TEST(WarehouseTest, EveryCellTypeHasTheExpectedWalkability) {
  Warehouse w(6, 1);
  int x = 0;
  for (const CellType type : warehousesim::world::kAllCellTypes) {
    w.set(P(x, 0), type);
    const bool obstacle = type == CellType::Wall || type == CellType::Shelf;
    EXPECT_EQ(w.isObstacle(P(x, 0)), obstacle);
    EXPECT_EQ(w.isWalkable(P(x, 0)), !obstacle);
    EXPECT_EQ(w.at(P(x, 0)), type);
    ++x;
  }
}

TEST(WarehouseTest, SetModifiesOnlyTheTargetCell) {
  Warehouse w(3, 3);
  w.set(P(1, 1), CellType::Shelf);
  const std::vector<std::string> expected = {"...", ".S.", "..."};
  EXPECT_EQ(w.toRows(), expected);
  w.set(P(1, 1), CellType::Free);
  w.set(P(0, 2), CellType::Charging);
  const std::vector<std::string> expected2 = {"...", "...", "C.."};
  EXPECT_EQ(w.toRows(), expected2);
}

TEST(WarehouseTest, AccessOutsideGridThrows) {
  Warehouse w(2, 2);
  EXPECT_THROW((void)w.at(P(2, 0)), std::out_of_range);
  EXPECT_THROW((void)w.at(P(0, -1)), std::out_of_range);
  EXPECT_THROW(w.set(P(0, -1), CellType::Wall), std::out_of_range);
  EXPECT_THROW(w.set(P(2, 2), CellType::Wall), std::out_of_range);
  const std::vector<std::string> unchanged = {"..", ".."};
  EXPECT_EQ(w.toRows(), unchanged);
}

TEST(WarehouseTest, NeighborsOfInteriorCellAreInNorthEastSouthWestOrder) {
  const Warehouse w(3, 3);
  const std::vector<Point> expected = {P(1, 0), P(2, 1), P(1, 2), P(0, 1)};
  EXPECT_EQ(w.neighbors(P(1, 1)), expected);
}

TEST(WarehouseTest, NeighborsAreClippedAtCornersAndEdges) {
  const Warehouse w(3, 3);
  EXPECT_EQ(w.neighbors(P(0, 0)), (std::vector<Point>{P(1, 0), P(0, 1)}));
  EXPECT_EQ(w.neighbors(P(2, 2)), (std::vector<Point>{P(2, 1), P(1, 2)}));
  EXPECT_EQ(w.neighbors(P(2, 0)), (std::vector<Point>{P(2, 1), P(1, 0)}));
  EXPECT_EQ(w.neighbors(P(1, 0)).size(), 3U);
  EXPECT_EQ(w.neighbors(P(0, 1)).size(), 3U);
}

TEST(WarehouseTest, NeighborsOfOutOfBoundsPointAreEmpty) {
  const Warehouse w(3, 3);
  EXPECT_TRUE(w.neighbors(P(-1, 0)).empty());  // adjacent to the grid but outside it
  EXPECT_TRUE(w.neighbors(P(3, 1)).empty());
  EXPECT_TRUE(w.neighbors(P(std::numeric_limits<std::int32_t>::max(), 0)).empty());
}

TEST(WarehouseTest, SingleCellWarehouseHasNoNeighbors) {
  const Warehouse w(1, 1);
  EXPECT_TRUE(w.neighbors(P(0, 0)).empty());
  EXPECT_TRUE(w.walkableNeighbors(P(0, 0)).empty());
}

TEST(WarehouseTest, WalkableNeighborsSkipWallsAndShelvesButKeepSpecialCells) {
  const Warehouse w = layout({".#.", "S.D", "P.C"});
  // Centre (1,1): north is a wall, west a shelf, east a dropoff, south free.
  const std::vector<Point> expected = {P(2, 1), P(1, 2)};
  EXPECT_EQ(w.walkableNeighbors(P(1, 1)), expected);
}

TEST(WarehouseTest, FromRowsParsesEveryCellTypeAndRoundTrips) {
  const std::vector<std::string> rows = {".#S", "PDC"};
  const Warehouse w = layout(rows);
  EXPECT_EQ(w.width(), 3);
  EXPECT_EQ(w.height(), 2);
  EXPECT_EQ(w.at(P(0, 0)), CellType::Free);
  EXPECT_EQ(w.at(P(1, 0)), CellType::Wall);
  EXPECT_EQ(w.at(P(2, 0)), CellType::Shelf);
  EXPECT_EQ(w.at(P(0, 1)), CellType::Pickup);
  EXPECT_EQ(w.at(P(1, 1)), CellType::Dropoff);
  EXPECT_EQ(w.at(P(2, 1)), CellType::Charging);
  EXPECT_EQ(w.toRows(), rows);
}

TEST(WarehouseTest, FromRowsRejectsInvalidLayouts) {
  EXPECT_FALSE(Warehouse::fromRows({}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({""}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({"..", "."}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({".", ".."}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({"..", ".x"}).has_value());
  EXPECT_FALSE(Warehouse::fromRows({".. "}).has_value());
}

TEST(WarehouseTest, CellsOfTypeAreRowMajor) {
  const Warehouse w = layout({"S.S", ".S."});
  const std::vector<Point> shelves = w.cellsOfType(CellType::Shelf);
  ASSERT_EQ(shelves.size(), 3U);
  EXPECT_EQ(shelves[0], P(0, 0));
  EXPECT_EQ(shelves[1], P(2, 0));
  EXPECT_EQ(shelves[2], P(1, 1));
  EXPECT_TRUE(w.cellsOfType(CellType::Charging).empty());
}

TEST(WarehouseTest, ShelfAccessPointsAreFreeCellsNextToShelves) {
  const Warehouse w = layout({"S..", "...", "..D"});
  const std::vector<Point> expected = {P(1, 0), P(0, 1)};
  EXPECT_EQ(w.shelfAccessPoints(), expected);
}

TEST(WarehouseTest, AVersionNamesTheLayoutAsItIsNowAndOnlyAnEditChangesIt) {
  Warehouse a(4, 3);
  const Warehouse b(4, 3);
  EXPECT_NE(a.version(), b.version()) << "two layouts made separately are never the same layout";

  const Warehouse copy = a;
  EXPECT_EQ(copy.version(), a.version());
  Warehouse assigned(2, 2);
  assigned = a;
  EXPECT_EQ(assigned.version(), a.version());

  const std::uint64_t before = a.version();
  (void)a.at(P(1, 1));
  (void)a.neighbors(P(1, 1));
  (void)a.cellsOfType(CellType::Free);
  (void)a.toRows();
  EXPECT_EQ(a.version(), before) << "looking changes nothing";

  a.set(P(1, 1), CellType::Wall);
  EXPECT_NE(a.version(), before);
  EXPECT_EQ(copy.version(), before) << "the copy is the layout as it was";
  const std::uint64_t edited = a.version();
  a.set(P(1, 1), CellType::Free);  // back to what it looked like: still not the same version
  EXPECT_NE(a.version(), edited);
  EXPECT_NE(a.version(), before);

  EXPECT_THROW(a.set(P(9, 9), CellType::Wall), std::out_of_range);
}

TEST(WarehouseTest, ParsedLayoutsEachGetTheirOwnVersion) {
  const Warehouse first = layout({"..", ".."});
  const Warehouse second = layout({"..", ".."});
  EXPECT_NE(first.version(), second.version());
  EXPECT_EQ(first.toRows(), second.toRows());
}

TEST(WarehouseTest, VersionsAreDistinctAcrossThreadsMakingLayoutsAtOnce) {
  std::vector<std::vector<std::uint64_t>> made(4);
  std::vector<std::thread> threads;
  for (std::size_t t = 0; t < made.size(); ++t) {
    threads.emplace_back([&made, t] {
      for (int i = 0; i < 500; ++i) {
        Warehouse w(2, 2);
        w.set(P(0, 0), CellType::Wall);
        made[t].push_back(w.version());
      }
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  std::set<std::uint64_t> all;
  for (const auto& versions : made) {
    all.insert(versions.begin(), versions.end());
  }
  EXPECT_EQ(all.size(), 2000U);
}

TEST(WarehouseTest, ShelfAccessPointsExcludeSpecialCellsAndDiagonals) {
  const Warehouse w = layout({"SP.", "D..", "..."});
  // (1,0) is a Pickup and (0,1) a Dropoff, so neither counts; (1,1) is only diagonal.
  EXPECT_TRUE(w.shelfAccessPoints().empty());
}

}  // namespace
