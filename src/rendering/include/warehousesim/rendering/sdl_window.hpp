#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/view/view_controller.hpp"

namespace warehousesim::rendering {

// Which program is reading the input. The fleet interface pans with a drag of the middle or right
// mouse button, reads a click of the left button as a click, and turns its keys into
// view::FleetKeys (Escape is one of them, so only closing the window quits by itself); the map
// editor paints with the left and right buttons, pans with the middle one, and turns its keys into
// view::EditorCommands.
enum class PollMode : std::uint8_t { Fleet, Editor };

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
  // Closing the window sets `quit`, and so does Escape in the map editor.
  [[nodiscard]] view::InputEvents pollEvents(PollMode mode);

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
