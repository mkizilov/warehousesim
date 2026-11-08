#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/world/cell_type.hpp"
#include "warehousesim/world/layout_check.hpp"
#include "warehousesim/world/warehouse.hpp"

// The editing model of the warehouse map editor: what is being edited, the tools, strokes, undo
// and redo. It edits a world::Warehouse through that class's public interface only, so the world
// model does not know editors exist, and it has no idea of windows, mice or files.
namespace warehousesim::editor {

// What a click puts on a cell. Erase puts open floor back, which is how walls, shelves and the
// rest are removed.
enum class Tool : std::uint8_t { Wall, Shelf, Pickup, Dropoff, Charging, Erase };

inline constexpr std::array<Tool, 6> kAllTools{Tool::Wall,    Tool::Shelf,    Tool::Pickup,
                                               Tool::Dropoff, Tool::Charging, Tool::Erase};

[[nodiscard]] world::CellType cellTypeOf(Tool tool) noexcept;
[[nodiscard]] const char* toString(Tool tool) noexcept;

enum class EditStatus : std::uint8_t {
  Applied,     // at least one cell changed
  Unchanged,   // every cell already was what the tool makes it
  OutOfBounds  // a cell asked for is outside the map: nothing was changed
};

struct EditResult {
  EditStatus status = EditStatus::Unchanged;
  std::size_t changed = 0;  // cells that changed
};

enum class ResizeStatus : std::uint8_t {
  Applied,
  Unchanged,   // the map already has that size
  InvalidSize  // not from 1 to world::kMaxMapSide on each side: nothing was changed
};

struct ResizeResult {
  ResizeStatus status = ResizeStatus::Unchanged;
  std::size_t cellsLost = 0;  // cells that were not open floor and have been cropped off
};

class MapEditor {
 public:
  // How many undo steps are kept; the oldest go first. A step is a stroke, a resize or a clear.
  static constexpr std::size_t kHistoryLimit = 100;

  // Edits `layout`, which starts out clean. Throws std::invalid_argument if a side is longer than
  // world::kMaxMapSide.
  explicit MapEditor(world::Warehouse layout, std::string name = {});
  // An empty map: open floor everywhere. Throws std::invalid_argument for sides outside
  // 1..world::kMaxMapSide.
  [[nodiscard]] static MapEditor blank(std::int32_t width, std::int32_t height,
                                       std::string name = {});

  [[nodiscard]] const world::Warehouse& layout() const noexcept { return layout_; }
  [[nodiscard]] const std::string& name() const noexcept { return name_; }
  // A number that goes up every time the layout changes, by painting, resizing, undoing, redoing
  // or replacing it: whoever caches something computed from the layout can tell it is out of date.
  [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }

  [[nodiscard]] Tool tool() const noexcept { return tool_; }
  void setTool(Tool tool) noexcept { tool_ = tool; }

  // ---- painting. Nothing outside the map is ever touched: a cell outside it is refused and the
  // call changes nothing. Outside a stroke each call is its own undo step.
  EditResult paint(core::Point cell);
  EditResult paint(core::Point cell, Tool tool);
  // Every cell on the straight line from `from` to `to`, both ends included (a drag).
  EditResult paintLine(core::Point from, core::Point to, Tool tool);
  // Open floor everywhere, in one step.
  EditResult clear();

  // Everything painted between beginStroke() and endStroke() is one undo step (a mouse drag).
  // Starting a stroke while one is open continues it; anything that is not painting (resize,
  // undo, redo, replace) ends it first.
  void beginStroke() noexcept { stroke_ = true; }
  void endStroke();
  [[nodiscard]] bool strokeOpen() const noexcept { return stroke_; }

  // Changes the size of the map, keeping the cells that are inside both the old and the new map
  // (the top left corner stays where it is). New cells are open floor. One undo step.
  ResizeResult resize(std::int32_t width, std::int32_t height);

  // Starts over with another document: history is gone and the editor is clean.
  void replace(world::Warehouse layout, std::string name);

  // ---- history
  [[nodiscard]] bool canUndo() const noexcept { return !undo_.empty(); }
  [[nodiscard]] bool canRedo() const noexcept { return !redo_.empty(); }
  // Return false when there is nothing to undo or redo.
  bool undo();
  bool redo();

  // Whether the map differs from how it was when it was last marked saved (or created). Undoing
  // back to the saved state makes it clean again.
  [[nodiscard]] bool dirty() const noexcept { return state_ != saved_; }
  void markSaved() noexcept { saved_ = state_; }

  // What is wrong with the map as a place for a simulation (see world::checkLayout).
  [[nodiscard]] std::vector<world::LayoutIssue> issues() const {
    return world::checkLayout(layout_);
  }

 private:
  struct CellChange {
    core::Point cell;
    world::CellType before;
    world::CellType after;
  };
  // One undo step: either cell changes, or the layouts before and after a resize.
  struct Step {
    std::vector<CellChange> changes;
    std::optional<std::pair<world::Warehouse, world::Warehouse>> resized;
    std::uint64_t stateBefore = 0;
    std::uint64_t stateAfter = 0;
  };

  EditResult setCell(core::Point cell, world::CellType type);
  void commit();
  void record(Step step);

  world::Warehouse layout_;
  std::string name_;
  Tool tool_ = Tool::Wall;
  bool stroke_ = false;
  std::vector<CellChange> pending_;  // the open stroke, or the call being made
  std::deque<Step> undo_;
  std::vector<Step> redo_;
  // Every distinct state of the document has a number; saved_ is the one on disk.
  std::uint64_t state_ = 0;
  std::uint64_t saved_ = 0;
  std::uint64_t lastState_ = 0;
  std::uint64_t revision_ = 0;
};

}  // namespace warehousesim::editor
