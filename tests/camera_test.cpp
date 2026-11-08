#include "warehousesim/view/camera.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::view::Camera;
using warehousesim::view::CellRange;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenRect;
using warehousesim::view::ScreenSize;
using warehousesim::view::WorldPoint;
using warehousesim::world::GridSize;

constexpr double kEps = 1e-9;

Point P(int x, int y) { return Point{x, y}; }

// A 200x100 window over a 10x10 grid at exactly 10 pixels per cell with the grid centred, so
// the grid spans screen x 50..150 and y 0..100.
Camera tenByTen() {
  Camera camera(ScreenSize{200, 100}, GridSize(10, 10));
  camera.setZoom(10.0);
  camera.centerOn({5.0, 5.0});
  return camera;
}

// ---- fitting ----------------------------------------------------------------------------

TEST(CameraTest, StartsFittedWithTheWholeGridCentred) {
  const Camera camera(ScreenSize{200, 100}, GridSize(10, 10));
  EXPECT_TRUE(camera.fitted());
  // Limited by height: (100 - 2 * 24) / 10 = 5.2 pixels per cell.
  EXPECT_NEAR(camera.zoom(), 5.2, kEps);
  EXPECT_EQ(camera.center(), (WorldPoint{5.0, 5.0}));
  const ScreenPoint middle = camera.worldToScreen({5.0, 5.0});
  EXPECT_NEAR(middle.x, 100.0, kEps);
  EXPECT_NEAR(middle.y, 50.0, kEps);
}

TEST(CameraTest, FittedGridLiesInsideTheWindowWithAMargin) {
  const Camera camera(ScreenSize{800, 600}, GridSize(40, 10));
  const ScreenPoint topLeft = camera.worldToScreen({0.0, 0.0});
  const ScreenPoint bottomRight = camera.worldToScreen({40.0, 10.0});
  EXPECT_TRUE(topLeft.x >= Camera::kFitMarginPixels - kEps);
  EXPECT_TRUE(topLeft.y >= Camera::kFitMarginPixels - kEps);
  EXPECT_TRUE(bottomRight.x <= 800.0 - Camera::kFitMarginPixels + kEps);
  EXPECT_TRUE(bottomRight.y <= 600.0 - Camera::kFitMarginPixels + kEps);
  EXPECT_NEAR(camera.zoom(), (800.0 - 48.0) / 40.0, kEps);  // limited by width here
}

TEST(CameraTest, FitHonoursTheZoomLimits) {
  EXPECT_NEAR(Camera(ScreenSize{10, 10}, GridSize(10, 10)).zoom(), Camera::kMinZoom, kEps);
  EXPECT_NEAR(Camera(ScreenSize{800, 600}, GridSize(1, 1)).zoom(), Camera::kMaxZoom, kEps);
  const Camera tiny(ScreenSize{800, 600}, GridSize(1, 1));
  EXPECT_EQ(tiny.center(), (WorldPoint{0.5, 0.5}));
}

TEST(CameraTest, FitRestoresTheFittedViewAfterManualChanges) {
  Camera camera(ScreenSize{200, 100}, GridSize(10, 10));
  const double fittedZoom = camera.zoom();
  camera.panByPixels(30.0, 10.0);
  camera.zoomBy(3.0, ScreenPoint{10.0, 10.0});
  EXPECT_FALSE(camera.fitted());
  camera.fit();
  EXPECT_TRUE(camera.fitted());
  EXPECT_NEAR(camera.zoom(), fittedZoom, kEps);
  EXPECT_EQ(camera.center(), (WorldPoint{5.0, 5.0}));
}

// ---- coordinate transforms --------------------------------------------------------------

TEST(CameraTest, KnownWorldToScreenMapping) {
  const Camera camera = tenByTen();
  const ScreenPoint origin = camera.worldToScreen({0.0, 0.0});
  const ScreenPoint far = camera.worldToScreen({10.0, 10.0});
  const ScreenPoint middle = camera.worldToScreen({5.0, 5.0});
  EXPECT_NEAR(origin.x, 50.0, kEps);
  EXPECT_NEAR(origin.y, 0.0, kEps);
  EXPECT_NEAR(far.x, 150.0, kEps);
  EXPECT_NEAR(far.y, 100.0, kEps);
  EXPECT_NEAR(middle.x, 100.0, kEps);
  EXPECT_NEAR(middle.y, 50.0, kEps);
}

TEST(CameraTest, KnownScreenToWorldMapping) {
  const Camera camera = tenByTen();
  EXPECT_EQ(camera.screenToWorld({100.0, 50.0}), (WorldPoint{5.0, 5.0}));
  EXPECT_EQ(camera.screenToWorld({50.0, 0.0}), (WorldPoint{0.0, 0.0}));
  EXPECT_EQ(camera.screenToWorld({0.0, 100.0}), (WorldPoint{-5.0, 10.0}));  // outside the grid
}

TEST(CameraTest, YPointsDownOnScreenAndSouthInTheWorld) {
  const Camera camera = tenByTen();
  EXPECT_TRUE(camera.worldToScreen({5.0, 6.0}).y > camera.worldToScreen({5.0, 5.0}).y);
  EXPECT_TRUE(camera.worldToScreen({6.0, 5.0}).x > camera.worldToScreen({5.0, 5.0}).x);
}

TEST(CameraTest, WorldToScreenAndBackRoundTrips) {
  Camera camera(ScreenSize{813, 407}, GridSize(37, 23));
  camera.setZoom(13.7);
  camera.centerOn({11.3, 7.9});
  for (const WorldPoint p : {WorldPoint{0.0, 0.0}, WorldPoint{36.99, 22.5}, WorldPoint{-4.0, 100.0},
                             WorldPoint{11.3, 7.9}, WorldPoint{0.001, 0.002}}) {
    const WorldPoint back = camera.screenToWorld(camera.worldToScreen(p));
    EXPECT_NEAR(back.x, p.x, 1e-9);
    EXPECT_NEAR(back.y, p.y, 1e-9);
  }
  for (const ScreenPoint s :
       {ScreenPoint{0.0, 0.0}, ScreenPoint{812.5, 406.5}, ScreenPoint{-30.0, 900.0}}) {
    const ScreenPoint back = camera.worldToScreen(camera.screenToWorld(s));
    EXPECT_NEAR(back.x, s.x, 1e-9);
    EXPECT_NEAR(back.y, s.y, 1e-9);
  }
}

TEST(CameraTest, ScreenCentreAlwaysShowsTheCameraCentre) {
  Camera camera(ScreenSize{640, 480}, GridSize(20, 20));
  camera.setZoom(31.0);
  camera.centerOn({3.25, 17.5});
  const WorldPoint seen = camera.screenToWorld({320.0, 240.0});
  EXPECT_NEAR(seen.x, 3.25, kEps);
  EXPECT_NEAR(seen.y, 17.5, kEps);
}

TEST(CameraTest, CellRectsScaleWithZoomAndTileWithoutGaps) {
  const Camera camera = tenByTen();
  EXPECT_EQ(camera.cellRect(P(0, 0)), (ScreenRect{50.0, 0.0, 10.0, 10.0}));
  EXPECT_EQ(camera.cellRect(P(9, 9)), (ScreenRect{140.0, 90.0, 10.0, 10.0}));
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < 9; ++x) {
      EXPECT_EQ(camera.cellRect(P(x, y)).right(), camera.cellRect(P(x + 1, y)).x);
    }
  }
  for (int y = 0; y < 9; ++y) {
    EXPECT_EQ(camera.cellRect(P(3, y)).bottom(), camera.cellRect(P(3, y + 1)).y);
  }
}

TEST(CameraTest, CellRectsTileAtFractionalZoom) {
  Camera camera(ScreenSize{333, 217}, GridSize(25, 25));
  camera.setZoom(7.31);
  camera.centerOn({12.2, 3.7});
  for (int x = 0; x < 24; ++x) {
    EXPECT_NEAR(camera.cellRect(P(x, 4)).right(), camera.cellRect(P(x + 1, 4)).x, 1e-9);
  }
}

TEST(CameraTest, CellRectWorksOutsideTheGrid) {
  const Camera camera = tenByTen();
  EXPECT_EQ(camera.cellRect(P(-1, -1)), (ScreenRect{40.0, -10.0, 10.0, 10.0}));
}

// ---- picking ----------------------------------------------------------------------------

TEST(CameraTest, CellAtFindsTheCellUnderAScreenPosition) {
  const Camera camera = tenByTen();
  EXPECT_EQ(camera.cellAt({55.0, 5.0}), P(0, 0));
  EXPECT_EQ(camera.cellAt({50.0, 0.0}), P(0, 0));  // top-left corner belongs to the cell
  EXPECT_EQ(camera.cellAt({149.9, 99.9}), P(9, 9));
  EXPECT_EQ(camera.cellAt({100.0, 50.0}), P(5, 5));
}

TEST(CameraTest, CellAtRejectsPositionsOutsideTheGrid) {
  const Camera camera = tenByTen();
  EXPECT_FALSE(camera.cellAt({49.9, 5.0}).has_value());     // just left of the grid
  EXPECT_FALSE(camera.cellAt({55.0, -0.1}).has_value());    // just above
  EXPECT_FALSE(camera.cellAt({150.0, 50.0}).has_value());   // right edge is exclusive
  EXPECT_FALSE(camera.cellAt({100.0, 100.0}).has_value());  // bottom edge is exclusive
  EXPECT_FALSE(camera.cellAt({0.0, 50.0}).has_value());
  EXPECT_FALSE(camera.cellAt({std::nan(""), 5.0}).has_value());
}

TEST(CameraTest, CellAtAndCellRectAgree) {
  Camera camera(ScreenSize{500, 400}, GridSize(12, 9));
  camera.setZoom(23.3);
  camera.centerOn({4.4, 5.1});
  for (int y = 0; y < 9; ++y) {
    for (int x = 0; x < 12; ++x) {
      const ScreenRect rect = camera.cellRect(P(x, y));
      const ScreenPoint inside{rect.x + rect.width / 2.0, rect.y + rect.height / 2.0};
      EXPECT_EQ(camera.cellAt(inside), P(x, y));
    }
  }
}

// ---- visibility -------------------------------------------------------------------------

TEST(CameraTest, VisibleCellsCoverTheWholeGridWhenItFitsTheWindow) {
  EXPECT_EQ(tenByTen().visibleCells(), (CellRange{0, 0, 10, 10}));
}

TEST(CameraTest, VisibleCellsShrinkWhenZoomedIn) {
  Camera camera = tenByTen();
  camera.setZoom(20.0);  // 10x5 cells fit in 200x100 pixels, centred on (5, 5)
  EXPECT_EQ(camera.visibleCells(), (CellRange{0, 2, 10, 8}));
}

TEST(CameraTest, VisibleCellsFollowPanningAndClampToTheGrid) {
  Camera camera = tenByTen();
  camera.setZoom(20.0);
  camera.centerOn({2.0, 2.0});  // world x -3..7, y -0.5..4.5
  EXPECT_EQ(camera.visibleCells(), (CellRange{0, 0, 7, 5}));
  camera.centerOn({10.0, 10.0});  // x 5..15, y 7.5..12.5
  EXPECT_EQ(camera.visibleCells(), (CellRange{5, 7, 10, 10}));
}

TEST(CameraTest, VisibleCellsIncludePartiallyVisibleCells) {
  Camera camera = tenByTen();
  camera.setZoom(20.0);
  camera.centerOn({5.25, 5.0});  // x 0.25..10.25 -> columns 0..9 inclusive
  EXPECT_EQ(camera.visibleCells(), (CellRange{0, 2, 10, 8}));
  camera.setZoom(40.0);  // x 2.75..7.75 -> columns 2..7
  EXPECT_EQ(camera.visibleCells(), (CellRange{2, 3, 8, 7}));
}

TEST(CameraTest, EmptyWindowSeesNothing) {
  Camera camera(ScreenSize{0, 0}, GridSize(10, 10));
  EXPECT_TRUE(camera.visibleCells().empty());
  camera.resize(ScreenSize{-5, 100});
  EXPECT_TRUE(camera.visibleCells().empty());
  EXPECT_EQ(camera.screen(), (ScreenSize{0, 100}));
  EXPECT_TRUE(std::isfinite(camera.zoom()));
}

// ---- panning ----------------------------------------------------------------------------

TEST(CameraTest, PanningMovesTheContentWithThePointer) {
  Camera camera = tenByTen();
  const ScreenPoint before = camera.worldToScreen({4.0, 6.0});
  camera.panByPixels(20.0, -10.0);
  const ScreenPoint after = camera.worldToScreen({4.0, 6.0});
  EXPECT_NEAR(after.x - before.x, 20.0, kEps);
  EXPECT_NEAR(after.y - before.y, -10.0, kEps);
  EXPECT_FALSE(camera.fitted());
}

TEST(CameraTest, PanningIsMeasuredInPixelsRegardlessOfZoom) {
  Camera camera = tenByTen();
  camera.setZoom(40.0);
  const ScreenPoint before = camera.worldToScreen({5.0, 5.0});
  camera.panByPixels(-8.0, 0.0);
  EXPECT_NEAR(camera.worldToScreen({5.0, 5.0}).x - before.x, -8.0, kEps);
  EXPECT_NEAR(camera.center().x, 5.2, kEps);  // 8 px / 40 px-per-cell
}

TEST(CameraTest, PanningCannotLeaveTheWarehouse) {
  Camera camera = tenByTen();
  camera.panByPixels(-100000.0, 100000.0);
  EXPECT_EQ(camera.center(), (WorldPoint{10.0, 0.0}));
  camera.panByPixels(100000.0, -100000.0);
  EXPECT_EQ(camera.center(), (WorldPoint{0.0, 10.0}));
}

TEST(CameraTest, CenterOnClampsToTheGrid) {
  Camera camera = tenByTen();
  camera.centerOn({-5.0, 99.0});
  EXPECT_EQ(camera.center(), (WorldPoint{0.0, 10.0}));
  camera.centerOn({3.5, 4.5});
  EXPECT_EQ(camera.center(), (WorldPoint{3.5, 4.5}));
}

// ---- zooming ----------------------------------------------------------------------------

TEST(CameraTest, ZoomingKeepsTheWorldPointUnderTheAnchorFixed) {
  Camera camera = tenByTen();
  const ScreenPoint anchor{150.0, 25.0};
  const WorldPoint before = camera.screenToWorld(anchor);
  EXPECT_EQ(before, (WorldPoint{10.0, 2.5}));
  camera.zoomBy(2.0, anchor);
  EXPECT_NEAR(camera.zoom(), 20.0, kEps);
  const WorldPoint after = camera.screenToWorld(anchor);
  EXPECT_NEAR(after.x, before.x, kEps);
  EXPECT_NEAR(after.y, before.y, kEps);
  EXPECT_EQ(camera.center(), (WorldPoint{7.5, 3.75}));
}

TEST(CameraTest, ZoomingAboutTheWindowCentreKeepsTheCentre) {
  Camera camera = tenByTen();
  camera.centerOn({3.0, 4.0});
  camera.zoomBy(1.5, ScreenPoint{100.0, 50.0});
  EXPECT_NEAR(camera.center().x, 3.0, kEps);
  EXPECT_NEAR(camera.center().y, 4.0, kEps);
}

TEST(CameraTest, ZoomIsClampedToTheLimits) {
  Camera camera = tenByTen();
  camera.zoomBy(1e9, ScreenPoint{100.0, 50.0});
  EXPECT_DOUBLE_EQ(camera.zoom(), Camera::kMaxZoom);
  camera.zoomBy(1e-9, ScreenPoint{100.0, 50.0});
  EXPECT_DOUBLE_EQ(camera.zoom(), Camera::kMinZoom);
  camera.setZoom(1e9);
  EXPECT_DOUBLE_EQ(camera.zoom(), Camera::kMaxZoom);
  camera.setZoom(0.0001);
  EXPECT_DOUBLE_EQ(camera.zoom(), Camera::kMinZoom);
}

TEST(CameraTest, ZoomingAtALimitDoesNotDriftTheView) {
  Camera camera = tenByTen();
  camera.setZoom(Camera::kMaxZoom);
  camera.centerOn({4.0, 4.0});
  camera.zoomBy(2.0, ScreenPoint{30.0, 80.0});  // already at the maximum
  EXPECT_NEAR(camera.center().x, 4.0, kEps);
  EXPECT_NEAR(camera.center().y, 4.0, kEps);
}

TEST(CameraTest, ZoomStepsCompoundAndInvert) {
  Camera camera = tenByTen();
  const ScreenPoint anchor{120.0, 40.0};
  camera.zoomBySteps(1.0, anchor);
  EXPECT_NEAR(camera.zoom(), 10.0 * Camera::kZoomPerStep, kEps);
  camera.zoomBySteps(-1.0, anchor);
  EXPECT_NEAR(camera.zoom(), 10.0, 1e-9);
  camera.zoomBySteps(3.0, anchor);
  camera.zoomBySteps(-3.0, anchor);
  EXPECT_NEAR(camera.zoom(), 10.0, 1e-9);
  EXPECT_NEAR(camera.center().x, 5.0, 1e-9);
  EXPECT_NEAR(camera.center().y, 5.0, 1e-9);
  camera.zoomBySteps(0.0, anchor);  // a no-op
  EXPECT_NEAR(camera.zoom(), 10.0, 1e-9);
}

TEST(CameraTest, InvalidZoomRequestsAreRejectedAndChangeNothing) {
  Camera camera = tenByTen();
  const ScreenPoint anchor{100.0, 50.0};
  EXPECT_THROW(camera.zoomBy(0.0, anchor), std::invalid_argument);
  EXPECT_THROW(camera.zoomBy(-2.0, anchor), std::invalid_argument);
  EXPECT_THROW(camera.zoomBy(std::nan(""), anchor), std::invalid_argument);
  EXPECT_THROW(camera.zoomBy(std::numeric_limits<double>::infinity(), anchor),
               std::invalid_argument);
  EXPECT_THROW(camera.setZoom(0.0), std::invalid_argument);
  EXPECT_THROW(camera.setZoom(std::nan("")), std::invalid_argument);
  EXPECT_DOUBLE_EQ(camera.zoom(), 10.0);
  EXPECT_EQ(camera.center(), (WorldPoint{5.0, 5.0}));
}

// ---- resizing ---------------------------------------------------------------------------

TEST(CameraTest, ResizingAFittedCameraRefitsTheGrid) {
  Camera camera(ScreenSize{200, 100}, GridSize(10, 10));
  camera.resize(ScreenSize{1000, 700});
  EXPECT_TRUE(camera.fitted());
  EXPECT_NEAR(camera.zoom(), (700.0 - 48.0) / 10.0, kEps);
  const ScreenPoint middle = camera.worldToScreen({5.0, 5.0});
  EXPECT_NEAR(middle.x, 500.0, kEps);
  EXPECT_NEAR(middle.y, 350.0, kEps);
}

TEST(CameraTest, ResizingAManualCameraKeepsZoomAndCentre) {
  Camera camera = tenByTen();
  camera.centerOn({3.0, 4.0});
  camera.resize(ScreenSize{400, 300});
  EXPECT_FALSE(camera.fitted());
  EXPECT_DOUBLE_EQ(camera.zoom(), 10.0);
  EXPECT_EQ(camera.center(), (WorldPoint{3.0, 4.0}));
  // The camera centre now maps to the middle of the larger window.
  EXPECT_NEAR(camera.worldToScreen({3.0, 4.0}).x, 200.0, kEps);
  EXPECT_NEAR(camera.worldToScreen({3.0, 4.0}).y, 150.0, kEps);
  EXPECT_EQ(camera.screen(), (ScreenSize{400, 300}));
}

TEST(CameraTest, ResizeThenResetGivesTheSameViewAsAFreshCamera) {
  Camera camera(ScreenSize{200, 100}, GridSize(16, 9));
  camera.panByPixels(40.0, 40.0);
  camera.resize(ScreenSize{1280, 720});
  camera.fit();
  const Camera fresh(ScreenSize{1280, 720}, GridSize(16, 9));
  EXPECT_DOUBLE_EQ(camera.zoom(), fresh.zoom());
  EXPECT_EQ(camera.center(), fresh.center());
}

// ---- invariants under arbitrary use -----------------------------------------------------

TEST(CameraTest, RandomInteractionKeepsTheCameraInsideItsInvariants) {
  Rng rng(31337);
  const auto unit = [&rng] {
    return static_cast<double>(rng.below(2001)) / 1000.0 - 1.0;
  };  // [-1, 1]
  Camera camera(ScreenSize{640, 480}, GridSize(30, 20));
  for (int step = 0; step < 2000; ++step) {
    switch (rng.below(5)) {
      case 0:
        camera.panByPixels(unit() * 500.0, unit() * 500.0);
        break;
      case 1:
        camera.zoomBySteps(unit() * 6.0,
                           ScreenPoint{(unit() + 1.0) * 320.0, (unit() + 1.0) * 240.0});
        break;
      case 2:
        camera.resize(
            ScreenSize{static_cast<int>(rng.below(1500)), static_cast<int>(rng.below(1100))});
        break;
      case 3:
        camera.fit();
        break;
      default:
        camera.centerOn({unit() * 40.0, unit() * 30.0});
        break;
    }

    ASSERT_TRUE(camera.zoom() >= Camera::kMinZoom);
    ASSERT_TRUE(camera.zoom() <= Camera::kMaxZoom);
    ASSERT_TRUE(camera.center().x >= 0.0 && camera.center().x <= 30.0);
    ASSERT_TRUE(camera.center().y >= 0.0 && camera.center().y <= 20.0);

    const CellRange range = camera.visibleCells();
    ASSERT_TRUE(range.left >= 0 && range.top >= 0 && range.right <= 30 && range.bottom <= 20);

    const WorldPoint probe{unit() * 20.0 + 15.0, unit() * 10.0 + 10.0};
    const WorldPoint back = camera.screenToWorld(camera.worldToScreen(probe));
    ASSERT_NEAR(back.x, probe.x, 1e-7);
    ASSERT_NEAR(back.y, probe.y, 1e-7);

    // Every cell the camera calls visible really intersects the window.
    if (!range.empty()) {
      const ScreenRect first = camera.cellRect(P(range.left, range.top));
      const ScreenRect last = camera.cellRect(P(range.right - 1, range.bottom - 1));
      ASSERT_TRUE(first.right() > 0.0 && first.bottom() > 0.0);
      ASSERT_TRUE(last.x < camera.screen().width && last.y < camera.screen().height);
    }
  }
}

}  // namespace
