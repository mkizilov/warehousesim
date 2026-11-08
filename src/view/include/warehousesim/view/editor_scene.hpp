#pragma once

#include <array>
#include <string>

#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/view/editor_controller.hpp"

namespace warehousesim::view {

// The lines of text at the bottom of the editor window, top to bottom: file, map size, zoom, tool
// and the cell under the pointer; then the message or whether the map is ready for a simulation;
// then two lines of key help. (The tool palette is drawn between the first and the second.)
[[nodiscard]] std::array<std::string, 4> editorStatusLines(const EditorController& controller);

// Everything visible in one frame of the editor, back to front: the map with its grid, a preview of
// what the selected tool would put under the pointer, a marker on the part of the map the issues
// are about, the rulers, and the bar. Only cells inside the window are emitted.
[[nodiscard]] DrawList buildEditorScene(const EditorController& controller);

}  // namespace warehousesim::view
