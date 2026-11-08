#pragma once

#include <optional>

#include "warehousesim/view/camera.hpp"
#include "warehousesim/world/grid_size.hpp"

namespace warehousesim::view {

// What the user did during one frame, described without any SDL types. The SDL window turns
// raw events into this; ViewController turns it into camera and playback changes. Keeping the
// two apart makes the interaction logic testable without a display.
struct InputEvents {
  bool quit = false;
  bool togglePause = false;
  bool resetView = false;                 // fit the whole warehouse
  bool togglePaths = false;               // show or hide the planned routes
  bool toggleOccupied = false;            // show or hide the cells robots hold
  int speedSteps = 0;                     // playback speed doublings (+) or halvings (-)
  double zoomSteps = 0.0;                 // wheel notches / zoom key presses; positive zooms in
  std::optional<ScreenPoint> zoomAnchor;  // where to zoom about; the window centre if unset
  double dragX = 0.0;                     // pixels the map was dragged with the mouse this frame
  double dragY = 0.0;
  double panX = 0.0;  // direction held on the keyboard, each in [-1, 1] (+x = look right)
  double panY = 0.0;
  std::optional<ScreenPoint> pointer;  // latest pointer position inside the window
  bool pointerLeft = false;            // the pointer left the window
};

// Owns the camera plus the pause, playback-speed and overlay state that the UI controls. Both
// overlays start on.
class ViewController {
 public:
  static constexpr double kKeyPanPixelsPerSecond = 600.0;

  // `initialSpeed` is clamped like FixedTimestep::clampSpeed; throws std::invalid_argument for
  // non-finite or non-positive values.
  ViewController(ScreenSize screen, world::GridSize grid, double initialSpeed = 1.0);

  void resize(ScreenSize screen) { camera_.resize(screen); }
  // Applies one frame of input. `frameSeconds` (>= 0) scales held-key panning so it moves at
  // the same speed at any frame rate; non-finite input values are ignored.
  void apply(const InputEvents& input, double frameSeconds);

  [[nodiscard]] const Camera& camera() const noexcept { return camera_; }
  [[nodiscard]] bool paused() const noexcept { return paused_; }
  [[nodiscard]] double speed() const noexcept { return speed_; }
  [[nodiscard]] bool showPaths() const noexcept { return showPaths_; }
  [[nodiscard]] bool showOccupied() const noexcept { return showOccupied_; }
  [[nodiscard]] std::optional<ScreenPoint> pointer() const noexcept { return pointer_; }

 private:
  Camera camera_;
  bool paused_ = false;
  bool showPaths_ = true;
  bool showOccupied_ = true;
  double speed_;
  std::optional<ScreenPoint> pointer_;
};

}  // namespace warehousesim::view
