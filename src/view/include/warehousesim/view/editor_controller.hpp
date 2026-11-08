#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/editor/map_editor.hpp"
#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/draw_list.hpp"
#include "warehousesim/view/view_controller.hpp"
#include "warehousesim/world/layout_check.hpp"

namespace warehousesim::view {

// The strip along the bottom of the editor window with the file, the tools, messages and key help.
inline constexpr int kEditorBarLines = 5;
inline constexpr double kEditorBarHeight = kEditorBarLines * (kGlyphSize + 2.0) + 6.0;

// Turns what the user did (InputEvents) into edits of a MapEditor and into camera movement. It has
// no SDL types and no drawing: buildEditorScene draws it, and the tests drive it with plain
// InputEvents.
//
//   left button    paint with the selected tool (a drag is one undo step)
//   right button   erase
//   middle button  drag the map          wheel or +/-  zoom          WASD or arrows  pan
//   1-6 tool: wall, shelf, pickup, dropoff, charging, erase          0  fit the whole map
//   Shift+arrows   one cell wider, narrower, taller, shorter         Delete  clear the map
//   Ctrl+Z / Ctrl+Y undo / redo     Ctrl+S save     Ctrl+O load      L  next predefined layout
//   Esc            quit
//
// Loading something over unsaved changes, and quitting with them, needs the same command twice in a
// row: the first time only says so.
class EditorController {
 public:
  // `window` is the whole window; the controller keeps the bottom kEditorBarHeight pixels for the
  // bar. `file` is where Save writes and Load reads.
  EditorController(ScreenSize window, editor::MapEditor editor, std::filesystem::path file);

  void resize(ScreenSize window);
  // Applies one frame of input. Commands run first, then the pointer actions in the order they
  // happened, then camera movement. Non-finite input values are ignored.
  void apply(const InputEvents& input, double frameSeconds);

  [[nodiscard]] const editor::MapEditor& editor() const noexcept { return editor_; }
  [[nodiscard]] const std::filesystem::path& file() const noexcept { return file_; }
  // The camera looks at the canvas: the window without the bar.
  [[nodiscard]] const Camera& camera() const noexcept { return view_.camera(); }
  [[nodiscard]] ScreenSize window() const noexcept { return window_; }
  [[nodiscard]] std::optional<ScreenPoint> pointer() const noexcept { return view_.pointer(); }
  // The cell under the pointer, if it is over the editable part of the canvas.
  [[nodiscard]] std::optional<core::Point> hoveredCell() const;
  // Where cells can be edited: the canvas without the rulers.
  [[nodiscard]] ScreenRect canvas() const;

  // The last thing that happened (or went wrong), for the status bar. Empty if nothing did.
  [[nodiscard]] const std::string& message() const noexcept { return message_; }
  [[nodiscard]] bool messageIsError() const noexcept { return messageIsError_; }
  // What is wrong with the map as a place for a simulation; recomputed when the map changes.
  [[nodiscard]] const std::vector<world::LayoutIssue>& issues() const noexcept { return issues_; }
  // True once the user has asked to quit (and confirmed, if there were unsaved changes).
  [[nodiscard]] bool wantsQuit() const noexcept { return quit_; }

 private:
  enum class Pending : std::uint8_t { None, Load, NextLayout, Quit };
  struct Drag {
    PointerButton button;
    editor::Tool tool;
    std::optional<core::Point> last;  // the last cell painted, to join up a fast drag
  };

  void run(EditorCommand command);
  void pointerAction(const PointerAction& action);
  void resizeBy(std::int32_t dx, std::int32_t dy);
  void fitView();
  [[nodiscard]] bool mayDiscard(Pending what, const char* question);
  [[nodiscard]] std::optional<core::Point> cellAtCanvas(ScreenPoint at) const;
  void setMessage(std::string text, bool isError);

  editor::MapEditor editor_;
  std::filesystem::path file_;
  ScreenSize window_;
  ViewController view_;
  std::optional<Drag> drag_;
  Pending pending_ = Pending::None;
  std::size_t nextLayout_ = 0;
  std::string message_;
  bool messageIsError_ = false;
  bool quit_ = false;
  std::vector<world::LayoutIssue> issues_;
  std::uint64_t issuesRevision_ = 0;
};

}  // namespace warehousesim::view
