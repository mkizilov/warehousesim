#include "warehousesim/view/editor_scene.hpp"

#include <iomanip>
#include <sstream>

#include "warehousesim/editor/editor_files.hpp"
#include "warehousesim/view/scene.hpp"
#include "warehousesim/view/scene_parts.hpp"

namespace warehousesim::view {

namespace {

constexpr Color kError{255, 120, 100};
constexpr Color kSelected{70, 78, 98};
constexpr Color kSelectedText{255, 255, 255};
constexpr Color kPreview{245, 245, 250};
constexpr Color kIssue{255, 70, 70};

constexpr double kLineHeight = kGlyphSize + 2.0;

// The tool's swatch colour: what the cell it makes looks like.
Color swatchOf(editor::Tool tool) { return cellColor(editor::cellTypeOf(tool)); }

std::string toolLabel(std::size_t index, editor::Tool tool) {
  return std::to_string(index + 1) + " " + editor::toString(tool);
}

void addPalette(DrawList& list, editor::Tool selected, double top) {
  double x = 4.0;
  for (std::size_t i = 0; i < editor::kAllTools.size(); ++i) {
    const editor::Tool tool = editor::kAllTools[i];
    const std::string label = toolLabel(i, tool);
    const double width = kGlyphSize + 4.0 + kGlyphSize * static_cast<double>(label.size());
    const bool isSelected = tool == selected;
    if (isSelected) {
      list.commands.emplace_back(
          FillRect{{x - 3.0, top - 1.0, width + 6.0, kGlyphSize + 4.0}, kSelected});
    }
    list.commands.emplace_back(FillRect{{x, top + 1.0, kGlyphSize, kGlyphSize}, swatchOf(tool)});
    list.commands.emplace_back(
        Text{{x + kGlyphSize + 4.0, top + 1.0}, label, isSelected ? kSelectedText : kPanelText});
    x += width + 16.0;
  }
}

}  // namespace

std::array<std::string, 4> editorStatusLines(const EditorController& controller) {
  const editor::MapEditor& editor = controller.editor();

  std::ostringstream first;
  first << controller.file().filename().string();
  if (editor.dirty()) {
    first << " *";
  }
  first << "  " << editor.layout().width() << " x " << editor.layout().height() << "  zoom "
        << std::fixed << std::setprecision(0) << controller.camera().zoom() << "  tool "
        << editor::toString(editor.tool()) << "  cell ";
  if (const auto cell = controller.hoveredCell()) {
    first << '(' << cell->x << ',' << cell->y << ") " << world::toString(editor.layout().at(*cell));
  } else {
    first << '-';
  }

  const std::string second =
      controller.message().empty() ? editor::readiness(controller.issues()) : controller.message();

  return {first.str(), second,
          "Left paint  Right erase  Middle drag/WASD pan  Wheel zoom  1-6 tool  0 fit view  "
          "Shift+arrows resize",
          "Ctrl+Z undo  Ctrl+Y redo  Ctrl+S save  Ctrl+O load  L predefined layouts  "
          "Del clear  Esc quit"};
}

DrawList buildEditorScene(const EditorController& controller) {
  DrawList list{kBackground, {}};
  const Camera& camera = controller.camera();
  const ScreenSize canvas = camera.screen();
  const ScreenSize window = controller.window();
  const editor::MapEditor& editor = controller.editor();

  if (canvas.width > 0 && canvas.height > 0) {
    const CellRange range = camera.visibleCells();
    if (!range.empty()) {
      addCells(list, editor.layout(), camera, range);
      addGridLines(list, camera, range);
      for (const world::LayoutIssue& issue : controller.issues()) {
        if (issue.where && contains(range, *issue.where)) {
          addOutline(list, camera.cellRect(*issue.where), kIssue);
        }
      }
      if (const auto cell = controller.hoveredCell()) {
        addOutline(list, camera.cellRect(*cell), kPreview);
        const ScreenRect swatch = inset(camera.cellRect(*cell), 0.3);
        list.commands.emplace_back(FillRect{swatch, swatchOf(editor.tool())});
      }
    }
    addRulers(list, camera, range, 0.0);
  }

  const double top = static_cast<double>(window.height) - kEditorBarHeight;
  list.commands.emplace_back(
      FillRect{{0.0, top, static_cast<double>(window.width), kEditorBarHeight}, kPanel});
  const std::array<std::string, 4> lines = editorStatusLines(controller);
  double y = top + 3.0;
  list.commands.emplace_back(Text{{4.0, y}, lines[0], kPanelText});
  y += kLineHeight;
  addPalette(list, editor.tool(), y);
  y += kLineHeight;
  list.commands.emplace_back(
      Text{{4.0, y}, lines[1], controller.messageIsError() ? kError : kPanelText});
  y += kLineHeight;
  list.commands.emplace_back(Text{{4.0, y}, lines[2], kPanelText});
  y += kLineHeight;
  list.commands.emplace_back(Text{{4.0, y}, lines[3], kPanelText});
  return list;
}

}  // namespace warehousesim::view
