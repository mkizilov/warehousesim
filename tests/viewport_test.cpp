#include "warehousesim/rendering/viewport.hpp"

#include <gtest/gtest.h>

#include "warehousesim/core/point.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::rendering::Rect;
using warehousesim::rendering::Viewport;

TEST(ViewportTest, FitsSquareCellsAndCentresTheGrid) {
  const Viewport viewport = Viewport::fit(100, 50, 5, 4);
  EXPECT_FLOAT_EQ(viewport.cellSize(), 12.0F);  // limited by height: floor(50 / 4)
  const Rect origin = viewport.cellRect(Point{0, 0});
  EXPECT_FLOAT_EQ(origin.x, 20.0F);  // (100 - 12 * 5) / 2
  EXPECT_FLOAT_EQ(origin.y, 1.0F);   // (50 - 12 * 4) / 2
}

TEST(ViewportTest, CellRectsTileWithoutGaps) {
  const Viewport viewport = Viewport::fit(200, 200, 4, 4);
  const Rect a = viewport.cellRect(Point{1, 2});
  const Rect b = viewport.cellRect(Point{2, 2});
  EXPECT_FLOAT_EQ(a.x + a.width, b.x);
  EXPECT_FLOAT_EQ(a.width, 50.0F);
}

TEST(ViewportTest, InsetShrinksEachSide) {
  const Viewport viewport = Viewport::fit(100, 100, 10, 10);
  const Rect inset = viewport.insetCellRect(Point{0, 0}, 0.1F);
  EXPECT_FLOAT_EQ(inset.x, 1.0F);
  EXPECT_FLOAT_EQ(inset.width, 8.0F);
}

TEST(ViewportTest, DegenerateSizesGiveZeroCellSize) {
  EXPECT_FLOAT_EQ(Viewport::fit(0, 100, 10, 10).cellSize(), 0.0F);
  EXPECT_FLOAT_EQ(Viewport::fit(100, 100, 0, 10).cellSize(), 0.0F);
}

}  // namespace
