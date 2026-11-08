#include "warehousesim/view/view_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "warehousesim/core/fixed_timestep.hpp"

namespace {

using warehousesim::core::FixedTimestep;
using warehousesim::view::InputEvents;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenSize;
using warehousesim::view::ViewController;
using warehousesim::view::WorldPoint;
using warehousesim::world::GridSize;

constexpr double kEps = 1e-9;

// A large grid in an 800x600 window so panning and zooming never hit the camera's limits.
ViewController bigView(double speed = 1.0) {
  return ViewController(ScreenSize{800, 600}, GridSize(200, 200), speed);
}

// Leaves the camera at 20 px/cell centred on the middle of the 200x200 grid.
ViewController zoomedView() {
  ViewController view(ScreenSize{800, 600}, GridSize(200, 200));
  const double steps = std::log(20.0 / view.camera().zoom()) / std::log(1.15);
  InputEvents input;
  input.zoomSteps = steps;
  view.apply(input, 0.0);
  return view;
}

TEST(ViewControllerTest, StartsRunningAtTheRequestedSpeedWithAFittedCamera) {
  const ViewController view(ScreenSize{800, 600}, GridSize(20, 10), 2.0);
  EXPECT_FALSE(view.paused());
  EXPECT_DOUBLE_EQ(view.speed(), 2.0);
  EXPECT_TRUE(view.camera().fitted());
  EXPECT_FALSE(view.pointer().has_value());
}

TEST(ViewControllerTest, InitialSpeedIsClampedOrRejected) {
  EXPECT_DOUBLE_EQ(ViewController(ScreenSize{10, 10}, GridSize(2, 2), 1000.0).speed(),
                   FixedTimestep::kMaxSpeed);
  EXPECT_DOUBLE_EQ(ViewController(ScreenSize{10, 10}, GridSize(2, 2), 0.001).speed(),
                   FixedTimestep::kMinSpeed);
  EXPECT_THROW(ViewController(ScreenSize{10, 10}, GridSize(2, 2), 0.0), std::invalid_argument);
  EXPECT_THROW(ViewController(ScreenSize{10, 10}, GridSize(2, 2), std::nan("")),
               std::invalid_argument);
}

TEST(ViewControllerTest, TogglePauseFlipsTheStateEachTime) {
  ViewController view = bigView();
  InputEvents toggle;
  toggle.togglePause = true;
  view.apply(toggle, 0.016);
  EXPECT_TRUE(view.paused());
  view.apply(toggle, 0.016);
  EXPECT_FALSE(view.paused());
  view.apply(InputEvents{}, 0.016);  // no event, no change
  EXPECT_FALSE(view.paused());
}

TEST(ViewControllerTest, SpeedDoublesAndHalvesPerStep) {
  ViewController view = bigView();
  InputEvents faster;
  faster.speedSteps = 1;
  view.apply(faster, 0.016);
  EXPECT_DOUBLE_EQ(view.speed(), 2.0);
  faster.speedSteps = 2;
  view.apply(faster, 0.016);
  EXPECT_DOUBLE_EQ(view.speed(), 8.0);
  InputEvents slower;
  slower.speedSteps = -4;
  view.apply(slower, 0.016);
  EXPECT_DOUBLE_EQ(view.speed(), 0.5);
}

TEST(ViewControllerTest, SpeedStopsAtTheLimits) {
  ViewController view = bigView();
  InputEvents faster;
  faster.speedSteps = 20;
  view.apply(faster, 0.016);
  EXPECT_DOUBLE_EQ(view.speed(), FixedTimestep::kMaxSpeed);
  InputEvents slower;
  slower.speedSteps = -40;
  view.apply(slower, 0.016);
  EXPECT_DOUBLE_EQ(view.speed(), FixedTimestep::kMinSpeed);
}

TEST(ViewControllerTest, ResetViewRefitsAfterPanningAndZooming) {
  ViewController view = bigView();
  InputEvents move;
  move.dragX = 120.0;
  move.zoomSteps = 4.0;
  view.apply(move, 0.016);
  EXPECT_FALSE(view.camera().fitted());
  InputEvents reset;
  reset.resetView = true;
  view.apply(reset, 0.016);
  EXPECT_TRUE(view.camera().fitted());
}

TEST(ViewControllerTest, WheelZoomsAboutThePointer) {
  ViewController view = zoomedView();
  const ScreenPoint anchor{650.0, 120.0};
  const WorldPoint before = view.camera().screenToWorld(anchor);
  const double zoomBefore = view.camera().zoom();
  InputEvents wheel;
  wheel.zoomSteps = 2.0;
  wheel.zoomAnchor = anchor;
  view.apply(wheel, 0.016);
  EXPECT_TRUE(view.camera().zoom() > zoomBefore);
  const WorldPoint after = view.camera().screenToWorld(anchor);
  EXPECT_NEAR(after.x, before.x, kEps);
  EXPECT_NEAR(after.y, before.y, kEps);
}

TEST(ViewControllerTest, KeyboardZoomWithoutAnAnchorUsesTheWindowCentre) {
  ViewController view = zoomedView();
  const WorldPoint centre = view.camera().screenToWorld({400.0, 300.0});
  InputEvents key;
  key.zoomSteps = -3.0;
  view.apply(key, 0.016);
  const WorldPoint after = view.camera().screenToWorld({400.0, 300.0});
  EXPECT_NEAR(after.x, centre.x, kEps);
  EXPECT_NEAR(after.y, centre.y, kEps);
}

TEST(ViewControllerTest, DraggingMovesTheMapWithThePointer) {
  ViewController view = zoomedView();
  const ScreenPoint before = view.camera().worldToScreen({100.0, 100.0});
  InputEvents drag;
  drag.dragX = 37.0;
  drag.dragY = -12.0;
  view.apply(drag, 0.016);
  const ScreenPoint after = view.camera().worldToScreen({100.0, 100.0});
  EXPECT_NEAR(after.x - before.x, 37.0, kEps);
  EXPECT_NEAR(after.y - before.y, -12.0, kEps);
}

TEST(ViewControllerTest, HeldKeysPanAtAFixedPixelSpeed) {
  ViewController view = zoomedView();
  const ScreenPoint before = view.camera().worldToScreen({100.0, 100.0});
  InputEvents right;
  right.panX = 1.0;  // looking right slides the content left
  view.apply(right, 0.5);
  const ScreenPoint after = view.camera().worldToScreen({100.0, 100.0});
  EXPECT_NEAR(after.x - before.x, -ViewController::kKeyPanPixelsPerSecond * 0.5, kEps);
  EXPECT_NEAR(after.y - before.y, 0.0, kEps);

  InputEvents down;
  down.panY = 1.0;
  view.apply(down, 0.25);
  EXPECT_NEAR(view.camera().worldToScreen({100.0, 100.0}).y - after.y,
              -ViewController::kKeyPanPixelsPerSecond * 0.25, kEps);
}

TEST(ViewControllerTest, KeyPanDistanceDoesNotDependOnFrameRate) {
  ViewController slow = zoomedView();
  ViewController fast = zoomedView();
  InputEvents left;
  left.panX = -1.0;
  left.panY = 1.0;
  slow.apply(left, 0.5);
  for (int i = 0; i < 50; ++i) {
    fast.apply(left, 0.01);
  }
  EXPECT_NEAR(slow.camera().center().x, fast.camera().center().x, 1e-7);
  EXPECT_NEAR(slow.camera().center().y, fast.camera().center().y, 1e-7);
}

TEST(ViewControllerTest, NoHeldKeyOrNoElapsedTimeDoesNotPan) {
  ViewController view = zoomedView();
  const WorldPoint centre = view.camera().center();
  view.apply(InputEvents{}, 1.0);  // no key held
  InputEvents held;
  held.panX = 1.0;
  view.apply(held, 0.0);  // key held but no time passed
  EXPECT_EQ(view.camera().center(), centre);
}

TEST(ViewControllerTest, NonFiniteInputIsIgnored) {
  ViewController view = zoomedView();
  const WorldPoint centre = view.camera().center();
  const double zoom = view.camera().zoom();
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();
  InputEvents bad;
  bad.zoomSteps = nan;
  bad.dragX = inf;
  bad.dragY = nan;
  bad.panX = inf;
  bad.zoomAnchor = ScreenPoint{nan, 0.0};
  bad.pointer = ScreenPoint{inf, 0.0};
  EXPECT_NO_THROW(view.apply(bad, 0.016));
  EXPECT_EQ(view.camera().center(), centre);
  EXPECT_DOUBLE_EQ(view.camera().zoom(), zoom);
  EXPECT_FALSE(view.pointer().has_value());

  InputEvents badTime;
  badTime.panX = 1.0;
  EXPECT_NO_THROW(view.apply(badTime, nan));
  EXPECT_NO_THROW(view.apply(badTime, -1.0));
  EXPECT_EQ(view.camera().center(), centre);
}

TEST(ViewControllerTest, TracksThePointerAndForgetsItWhenItLeaves) {
  ViewController view = bigView();
  InputEvents move;
  move.pointer = ScreenPoint{12.0, 34.0};
  view.apply(move, 0.016);
  ASSERT_TRUE(view.pointer().has_value());
  EXPECT_EQ(*view.pointer(), (ScreenPoint{12.0, 34.0}));

  view.apply(InputEvents{}, 0.016);  // no new motion: the last position is kept
  EXPECT_TRUE(view.pointer().has_value());

  InputEvents leave;
  leave.pointerLeft = true;
  view.apply(leave, 0.016);
  EXPECT_FALSE(view.pointer().has_value());
}

TEST(ViewControllerTest, ResizingAFittedViewRefitsAndAManualViewKeepsItsPlace) {
  ViewController fitted(ScreenSize{200, 100}, GridSize(10, 10));
  fitted.resize(ScreenSize{1000, 800});
  EXPECT_TRUE(fitted.camera().fitted());
  EXPECT_EQ(fitted.camera().screen(), (ScreenSize{1000, 800}));

  ViewController manual = zoomedView();
  const double zoom = manual.camera().zoom();
  const WorldPoint centre = manual.camera().center();
  manual.resize(ScreenSize{1200, 900});
  EXPECT_DOUBLE_EQ(manual.camera().zoom(), zoom);
  EXPECT_EQ(manual.camera().center(), centre);
  EXPECT_EQ(manual.camera().screen(), (ScreenSize{1200, 900}));
}

}  // namespace
