#pragma once

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "warehousesim/view/camera.hpp"

namespace warehousesim::view {

struct Color {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  std::uint8_t a = 255;
  friend bool operator==(const Color&, const Color&) = default;
};

// Backend-neutral drawing primitives, in screen (render pixel) coordinates. The scene builder
// produces them from simulation state; SdlWindow executes them in order, later commands
// painting over earlier ones. Because they are plain data, what the scene shows can be tested
// without opening a window.
struct FillRect {
  ScreenRect rect;
  Color color;
  friend bool operator==(const FillRect&, const FillRect&) = default;
};

struct Line {
  ScreenPoint from;
  ScreenPoint to;
  Color color;
  friend bool operator==(const Line&, const Line&) = default;
};

// Monospaced debug-font text with its top-left corner at `position`. Every glyph is
// kGlyphSize pixels square.
struct Text {
  ScreenPoint position;
  std::string text;
  Color color;
  friend bool operator==(const Text&, const Text&) = default;
};

inline constexpr double kGlyphSize = 8.0;  // SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE

using DrawCommand = std::variant<FillRect, Line, Text>;

struct DrawList {
  Color background;
  std::vector<DrawCommand> commands;
};

}  // namespace warehousesim::view
