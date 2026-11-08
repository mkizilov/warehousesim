#include "warehousesim/view/view_controller.hpp"

#include <cmath>

#include "warehousesim/core/fixed_timestep.hpp"

namespace warehousesim::view {

namespace {

bool isFinite(double value) { return std::isfinite(value); }

bool isFinite(const ScreenPoint& p) { return std::isfinite(p.x) && std::isfinite(p.y); }

}  // namespace

ViewController::ViewController(ScreenSize screen, world::GridSize grid, double initialSpeed)
    : camera_(screen, grid), speed_(core::FixedTimestep::clampSpeed(initialSpeed)) {}

void ViewController::apply(const InputEvents& input, double frameSeconds) {
  if (input.togglePause) {
    paused_ = !paused_;
  }
  if (input.speedSteps != 0) {
    speed_ = core::FixedTimestep::clampSpeed(speed_ * std::pow(2.0, input.speedSteps));
  }
  if (input.resetView) {
    camera_.fit();
  }

  if (input.pointerLeft) {
    pointer_.reset();
  } else if (input.pointer && isFinite(*input.pointer)) {
    pointer_ = input.pointer;
  }

  if (isFinite(input.zoomSteps) && input.zoomSteps != 0.0) {
    const ScreenSize screen = camera_.screen();
    const ScreenPoint centre{screen.width / 2.0, screen.height / 2.0};
    const ScreenPoint anchor =
        input.zoomAnchor && isFinite(*input.zoomAnchor) ? *input.zoomAnchor : centre;
    camera_.zoomBySteps(input.zoomSteps, anchor);
  }

  if (isFinite(input.dragX) && isFinite(input.dragY) &&
      (input.dragX != 0.0 || input.dragY != 0.0)) {
    camera_.panByPixels(input.dragX, input.dragY);
  }

  if (isFinite(input.panX) && isFinite(input.panY) && isFinite(frameSeconds) &&
      frameSeconds > 0.0 && (input.panX != 0.0 || input.panY != 0.0)) {
    // Holding "right" looks further right, so the content slides left.
    const double pixels = kKeyPanPixelsPerSecond * frameSeconds;
    camera_.panByPixels(-input.panX * pixels, -input.panY * pixels);
  }
}

}  // namespace warehousesim::view
