#pragma once

#include <cstdint>

#include "warehousesim/view/camera.hpp"

namespace warehousesim::view {

// What the user did to the map editor, described without any SDL types (see InputEvents).
//
// Mouse buttons come as a sequence of presses, moves while a button is down, and releases, in the
// order they happened, each with the pointer position at that moment: a click that is pressed and
// released within one frame is not lost, and a fast drag still has every position it passed.
enum class PointerButton : std::uint8_t { Primary, Secondary };

struct PointerAction {
  enum class Kind : std::uint8_t { Press, Move, Release };
  Kind kind = Kind::Move;
  PointerButton button = PointerButton::Primary;
  ScreenPoint at;
};

// Key commands. The window decides which keys mean what; the editor only sees the meaning.
enum class EditorCommand : std::uint8_t {
  ToolWall,
  ToolShelf,
  ToolPickup,
  ToolDropoff,
  ToolCharging,
  ToolErase,
  Undo,
  Redo,
  Save,
  Load,
  NextLayout,  // load the next predefined layout
  Clear,
  Wider,  // the map one cell wider, narrower, taller, shorter
  Narrower,
  Taller,
  Shorter
};

}  // namespace warehousesim::view
