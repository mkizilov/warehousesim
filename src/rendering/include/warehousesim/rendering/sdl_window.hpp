#pragma once

#include <memory>
#include <string>

#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/view/view_controller.hpp"

namespace warehousesim::rendering {

// RAII wrapper for an SDL3 window and renderer. SDL types stay out of this header, and
// everything is released in reverse order of creation (renderer, window, then SDL itself),
// including when the constructor throws part-way through. Must be used from the thread that
// created it (SDL's main-thread rule).
class SdlWindow {
 public:
  // Creates a resizable window. Throws std::runtime_error with SDL's error text on failure.
  SdlWindow(const std::string& title, int width, int height);
  ~SdlWindow();

  SdlWindow(const SdlWindow&) = delete;
  SdlWindow& operator=(const SdlWindow&) = delete;
  SdlWindow(SdlWindow&&) = delete;
  SdlWindow& operator=(SdlWindow&&) = delete;

  // Drains the SDL event queue and returns what the user did, in render-pixel coordinates.
  // Closing the window or pressing Escape sets `quit`.
  [[nodiscard]] view::InputEvents pollEvents();

  // Current drawable size in render pixels; changes when the window is resized.
  [[nodiscard]] view::ScreenSize renderSize() const;

  // Executes the commands in order and presents the frame. Throws std::runtime_error if SDL
  // reports a drawing error.
  void draw(const view::DrawList& list);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace warehousesim::rendering
