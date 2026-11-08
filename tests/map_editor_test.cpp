#include "warehousesim/editor/map_editor.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/grid_size.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::editor::cellTypeOf;
using warehousesim::editor::EditResult;
using warehousesim::editor::EditStatus;
using warehousesim::editor::kAllTools;
using warehousesim::editor::MapEditor;
using warehousesim::editor::ResizeResult;
using warehousesim::editor::ResizeStatus;
using warehousesim::editor::Tool;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

using Rows = std::vector<std::string>;

Warehouse layout(const Rows& rows) {
  auto parsed = Warehouse::fromRows(rows);
  if (!parsed) {
    throw std::logic_error("bad test layout");
  }
  return *parsed;
}

// ---- tools and painting -------------------------------------------------------------------------

TEST(MapEditorToolTest, EveryToolMakesItsOwnKindOfCell) {
  EXPECT_EQ(cellTypeOf(Tool::Wall), CellType::Wall);
  EXPECT_EQ(cellTypeOf(Tool::Shelf), CellType::Shelf);
  EXPECT_EQ(cellTypeOf(Tool::Pickup), CellType::Pickup);
  EXPECT_EQ(cellTypeOf(Tool::Dropoff), CellType::Dropoff);
  EXPECT_EQ(cellTypeOf(Tool::Charging), CellType::Charging);
  EXPECT_EQ(cellTypeOf(Tool::Erase), CellType::Free);
}

TEST(MapEditorToolTest, EveryToolHasADistinctName) {
  std::set<std::string> names;
  for (const Tool tool : kAllTools) {
    EXPECT_TRUE(names.insert(warehousesim::editor::toString(tool)).second);
  }
  EXPECT_EQ(names.size(), 6U);
}

TEST(MapEditorToolTest, TheEditorStartsWithTheWallToolAndTheSelectedToolIsUsedByPaint) {
  MapEditor editor = MapEditor::blank(4, 4);
  EXPECT_EQ(editor.tool(), Tool::Wall);
  EXPECT_EQ(editor.paint({1, 1}).status, EditStatus::Applied);
  EXPECT_EQ(editor.layout().at({1, 1}), CellType::Wall);
  editor.setTool(Tool::Charging);
  EXPECT_EQ(editor.paint({2, 2}).status, EditStatus::Applied);
  EXPECT_EQ(editor.layout().at({2, 2}), CellType::Charging);
}

TEST(MapEditorPaintTest, EachKindOfObjectCanBePlacedAndRemoved) {
  MapEditor editor = MapEditor::blank(3, 1);
  for (const Tool tool : {Tool::Wall, Tool::Shelf, Tool::Pickup, Tool::Dropoff, Tool::Charging}) {
    ASSERT_EQ(editor.paint({1, 0}, tool).status, EditStatus::Applied) << toString(tool);
    EXPECT_EQ(editor.layout().at({1, 0}), cellTypeOf(tool));
    ASSERT_EQ(editor.paint({1, 0}, Tool::Erase).status, EditStatus::Applied);
    EXPECT_EQ(editor.layout().at({1, 0}), CellType::Free);
  }
}

TEST(MapEditorPaintTest, PaintingOverAnotherKindReplacesIt) {
  MapEditor editor(layout({"S"}));
  const EditResult result = editor.paint({0, 0}, Tool::Wall);
  EXPECT_EQ(result.status, EditStatus::Applied);
  EXPECT_EQ(result.changed, 1U);
  EXPECT_EQ(editor.layout().at({0, 0}), CellType::Wall);
}

TEST(MapEditorPaintTest, PaintingWhatIsAlreadyThereChangesNothingAndMakesNoUndoStep) {
  MapEditor editor(layout({"#."}));
  const EditResult same = editor.paint({0, 0}, Tool::Wall);
  EXPECT_EQ(same.status, EditStatus::Unchanged);
  EXPECT_EQ(same.changed, 0U);
  EXPECT_EQ(editor.paint({1, 0}, Tool::Erase).status, EditStatus::Unchanged);
  EXPECT_FALSE(editor.canUndo());
  EXPECT_FALSE(editor.dirty());
}

TEST(MapEditorPaintTest, CellsOutsideTheMapAreRefusedAndNothingChanges) {
  MapEditor editor = MapEditor::blank(5, 3);
  const Rows before = editor.layout().toRows();
  for (const Point cell : {Point{-1, 0}, Point{0, -1}, Point{5, 0}, Point{0, 3}, Point{5, 3},
                           Point{-100, -100}, Point{2147483647, 0}, Point{0, -2147483647 - 1}}) {
    const EditResult result = editor.paint(cell, Tool::Wall);
    EXPECT_EQ(result.status, EditStatus::OutOfBounds) << cell.x << "," << cell.y;
    EXPECT_EQ(result.changed, 0U);
  }
  EXPECT_EQ(editor.layout().toRows(), before);
  EXPECT_FALSE(editor.canUndo());
  EXPECT_FALSE(editor.dirty());
}

TEST(MapEditorPaintTest, TheFourCornersAreInsideTheMap) {
  MapEditor editor = MapEditor::blank(7, 4);
  for (const Point corner : {Point{0, 0}, Point{6, 0}, Point{0, 3}, Point{6, 3}}) {
    EXPECT_EQ(editor.paint(corner, Tool::Shelf).status, EditStatus::Applied);
  }
  EXPECT_EQ(editor.layout().cellsOfType(CellType::Shelf).size(), 4U);
}

// ---- lines --------------------------------------------------------------------------------------

TEST(MapEditorLineTest, HorizontalVerticalAndDiagonalLinesIncludeBothEnds) {
  MapEditor editor = MapEditor::blank(6, 6);
  const EditResult h = editor.paintLine({0, 0}, {5, 0}, Tool::Wall);
  EXPECT_EQ(h.changed, 6U);
  const EditResult v = editor.paintLine({0, 1}, {0, 5}, Tool::Shelf);
  EXPECT_EQ(v.changed, 5U);
  const EditResult d = editor.paintLine({1, 1}, {4, 4}, Tool::Pickup);
  EXPECT_EQ(d.changed, 4U);
  EXPECT_EQ(editor.layout().toRows(),
            (Rows{"######", "SP....", "S.P...", "S..P..", "S...P.", "S....."}));
}

TEST(MapEditorLineTest, ALineCoversTheSameCellsInEitherDirection) {
  for (const auto& [a, b] : std::vector<std::pair<Point, Point>>{{{0, 0}, {7, 3}},
                                                                 {{7, 3}, {0, 0}},
                                                                 {{2, 6}, {5, 0}},
                                                                 {{5, 0}, {2, 6}},
                                                                 {{0, 4}, {7, 4}}}) {
    MapEditor forward = MapEditor::blank(8, 8);
    MapEditor backward = MapEditor::blank(8, 8);
    forward.paintLine(a, b, Tool::Wall);
    backward.paintLine(b, a, Tool::Wall);
    EXPECT_EQ(forward.layout().cellsOfType(CellType::Wall),
              backward.layout().cellsOfType(CellType::Wall));
  }
}

TEST(MapEditorLineTest, ALineHasNoGapsSoEveryCellTouchesTheNext) {
  MapEditor editor = MapEditor::blank(30, 30);
  editor.paintLine({1, 2}, {27, 19}, Tool::Wall);
  const auto cells = editor.layout().cellsOfType(CellType::Wall);
  EXPECT_EQ(cells.size(), 27U);  // one cell per column of the longer side
  std::set<Point> remaining(cells.begin(), cells.end());
  std::vector<Point> open{{1, 2}};
  remaining.erase({1, 2});
  while (!open.empty()) {
    const Point at = open.back();
    open.pop_back();
    for (std::int32_t dx = -1; dx <= 1; ++dx) {
      for (std::int32_t dy = -1; dy <= 1; ++dy) {
        if (remaining.erase({at.x + dx, at.y + dy}) > 0) {
          open.push_back({at.x + dx, at.y + dy});
        }
      }
    }
  }
  EXPECT_TRUE(remaining.empty());
}

TEST(MapEditorLineTest, AOnePointLineIsOneCellAndCountsOnlyWhatChanged) {
  MapEditor editor(layout({".#."}));
  EXPECT_EQ(editor.paintLine({1, 0}, {1, 0}, Tool::Wall).status, EditStatus::Unchanged);
  EXPECT_EQ(editor.paintLine({2, 0}, {2, 0}, Tool::Wall).changed, 1U);
  EXPECT_EQ(editor.paintLine({0, 0}, {2, 0}, Tool::Wall).changed, 1U);  // only (0, 0) was new
}

TEST(MapEditorLineTest, ALineWithAnEndOutsideTheMapChangesNothing) {
  MapEditor editor = MapEditor::blank(5, 5);
  EXPECT_EQ(editor.paintLine({0, 0}, {5, 5}, Tool::Wall).status, EditStatus::OutOfBounds);
  EXPECT_EQ(editor.paintLine({-1, 2}, {3, 2}, Tool::Wall).status, EditStatus::OutOfBounds);
  EXPECT_TRUE(editor.layout().cellsOfType(CellType::Wall).empty());
  EXPECT_FALSE(editor.canUndo());
}

// ---- strokes, undo and redo ---------------------------------------------------------------------

TEST(MapEditorHistoryTest, EachPaintOutsideAStrokeIsItsOwnUndoStep) {
  MapEditor editor = MapEditor::blank(4, 1);
  editor.paint({0, 0});
  editor.paint({1, 0});
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#..."}));
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"...."}));
  EXPECT_FALSE(editor.undo());
}

TEST(MapEditorHistoryTest, EverythingInAStrokeIsOneUndoStep) {
  MapEditor editor = MapEditor::blank(6, 1);
  editor.beginStroke();
  EXPECT_TRUE(editor.strokeOpen());
  editor.paint({0, 0});
  editor.paint({2, 0});
  editor.paintLine({3, 0}, {5, 0}, Tool::Wall);
  EXPECT_FALSE(editor.canUndo());  // still open
  editor.endStroke();
  EXPECT_FALSE(editor.strokeOpen());
  EXPECT_TRUE(editor.canUndo());
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"......"}));
  EXPECT_FALSE(editor.canUndo());
  EXPECT_TRUE(editor.redo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#.####"}));
}

TEST(MapEditorHistoryTest, AStrokeThatChangedNothingMakesNoStep) {
  MapEditor editor(layout({"#."}));
  editor.beginStroke();
  editor.paint({0, 0}, Tool::Wall);
  editor.paint({5, 5}, Tool::Wall);  // refused
  editor.endStroke();
  EXPECT_FALSE(editor.canUndo());
  EXPECT_FALSE(editor.dirty());
}

TEST(MapEditorHistoryTest, ACellPaintedTwiceInOneStrokeIsRestoredToWhatItWasBeforeTheStroke) {
  MapEditor editor(layout({"S"}));
  editor.beginStroke();
  editor.paint({0, 0}, Tool::Wall);
  editor.paint({0, 0}, Tool::Pickup);
  editor.paint({0, 0}, Tool::Charging);
  editor.endStroke();
  EXPECT_EQ(editor.layout().at({0, 0}), CellType::Charging);
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().at({0, 0}), CellType::Shelf);
  EXPECT_TRUE(editor.redo());
  EXPECT_EQ(editor.layout().at({0, 0}), CellType::Charging);
}

TEST(MapEditorHistoryTest, UndoAndRedoEndAnOpenStrokeFirst) {
  MapEditor editor = MapEditor::blank(3, 1);
  editor.paint({0, 0});
  editor.beginStroke();
  editor.paint({1, 0});
  EXPECT_TRUE(editor.undo());  // the open stroke becomes a step and is undone at once
  EXPECT_FALSE(editor.strokeOpen());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#.."}));
  EXPECT_TRUE(editor.redo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"##."}));
}

TEST(MapEditorHistoryTest, ABeginStrokeWhileOneIsOpenContinuesIt) {
  MapEditor editor = MapEditor::blank(3, 1);
  editor.beginStroke();
  editor.paint({0, 0});
  editor.beginStroke();
  editor.paint({1, 0});
  editor.endStroke();
  EXPECT_TRUE(editor.undo());
  EXPECT_FALSE(editor.canUndo());
}

TEST(MapEditorHistoryTest, RedoIsLostWhenSomethingNewIsDone) {
  MapEditor editor = MapEditor::blank(3, 1);
  editor.paint({0, 0});
  editor.paint({1, 0});
  EXPECT_TRUE(editor.undo());
  EXPECT_TRUE(editor.canRedo());
  editor.paint({2, 0});
  EXPECT_FALSE(editor.canRedo());
  EXPECT_FALSE(editor.redo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#.#"}));
}

TEST(MapEditorHistoryTest, UndoAndRedoWithNothingToDoReturnFalseAndChangeNothing) {
  MapEditor editor(layout({"#."}));
  EXPECT_FALSE(editor.undo());
  EXPECT_FALSE(editor.redo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#."}));
}

TEST(MapEditorHistoryTest, OnlyTheLastHundredStepsAreKept) {
  MapEditor editor = MapEditor::blank(150, 1);
  for (std::int32_t x = 0; x < 150; ++x) {
    editor.paint({x, 0});
  }
  std::size_t undone = 0;
  while (editor.undo()) {
    ++undone;
  }
  EXPECT_EQ(undone, MapEditor::kHistoryLimit);
  EXPECT_EQ(editor.layout().cellsOfType(CellType::Wall).size(), 50U);  // the first 50 stay
  for (std::size_t i = 0; i < MapEditor::kHistoryLimit; ++i) {
    EXPECT_TRUE(editor.redo());
  }
  EXPECT_EQ(editor.layout().cellsOfType(CellType::Wall).size(), 150U);
}

TEST(MapEditorHistoryTest, ClearMakesEveryCellOpenFloorInOneStep) {
  MapEditor editor(layout({"#S", "PD"}));
  const EditResult result = editor.clear();
  EXPECT_EQ(result.status, EditStatus::Applied);
  EXPECT_EQ(result.changed, 4U);
  EXPECT_EQ(editor.layout().toRows(), (Rows{"..", ".."}));
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#S", "PD"}));
  EXPECT_FALSE(editor.canUndo());
}

TEST(MapEditorHistoryTest, ClearingAnEmptyMapChangesNothing) {
  MapEditor editor = MapEditor::blank(3, 3);
  EXPECT_EQ(editor.clear().status, EditStatus::Unchanged);
  EXPECT_FALSE(editor.canUndo());
}

// ---- dirty --------------------------------------------------------------------------------------

TEST(MapEditorDirtyTest, AMapIsCleanUntilItChangesAndCleanAgainWhenSaved) {
  MapEditor editor = MapEditor::blank(3, 1);
  EXPECT_FALSE(editor.dirty());
  editor.paint({0, 0});
  EXPECT_TRUE(editor.dirty());
  editor.markSaved();
  EXPECT_FALSE(editor.dirty());
  editor.paint({1, 0});
  EXPECT_TRUE(editor.dirty());
}

TEST(MapEditorDirtyTest, UndoingBackToTheSavedStateMakesTheMapCleanAndGoingFurtherDirty) {
  MapEditor editor = MapEditor::blank(3, 1);
  editor.paint({0, 0});
  editor.markSaved();
  editor.paint({1, 0});
  EXPECT_TRUE(editor.dirty());
  EXPECT_TRUE(editor.undo());
  EXPECT_FALSE(editor.dirty());
  EXPECT_TRUE(editor.undo());
  EXPECT_TRUE(editor.dirty());  // before the saved state
  EXPECT_TRUE(editor.redo());
  EXPECT_FALSE(editor.dirty());
}

TEST(MapEditorDirtyTest, ADifferentChangeAfterUndoIsNotTheSavedStateEvenIfItLooksTheSame) {
  MapEditor editor = MapEditor::blank(3, 1);
  editor.paint({0, 0});
  editor.markSaved();
  EXPECT_TRUE(editor.undo());
  EXPECT_TRUE(editor.dirty());
  editor.paint({0, 0});         // the same painting again, as a new step
  EXPECT_TRUE(editor.dirty());  // not the very state that was saved, so it counts as unsaved
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#.."}));
}

// ---- resizing -----------------------------------------------------------------------------------

TEST(MapEditorResizeTest, GrowingKeepsTheTopLeftAndAddsOpenFloor) {
  MapEditor editor(layout({"#S", "PD"}));
  const ResizeResult result = editor.resize(3, 3);
  EXPECT_EQ(result.status, ResizeStatus::Applied);
  EXPECT_EQ(result.cellsLost, 0U);
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#S.", "PD.", "..."}));
  EXPECT_EQ(editor.layout().width(), 3);
  EXPECT_EQ(editor.layout().height(), 3);
}

TEST(MapEditorResizeTest, ShrinkingCropsAndCountsTheObjectsLost) {
  MapEditor editor(layout({"#S.C", "PD..", "..#."}));
  const ResizeResult result = editor.resize(2, 2);
  EXPECT_EQ(result.status, ResizeStatus::Applied);
  EXPECT_EQ(result.cellsLost, 2U);  // C at (3, 0) and the wall at (2, 2); open floor is not counted
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#S", "PD"}));
}

TEST(MapEditorResizeTest, EachSideChangesOnItsOwn) {
  MapEditor editor(layout({"#S.", "PD."}));
  EXPECT_EQ(editor.resize(5, 2).status, ResizeStatus::Applied);
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#S...", "PD..."}));
  EXPECT_EQ(editor.resize(5, 1).status, ResizeStatus::Applied);
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#S..."}));
}

TEST(MapEditorResizeTest, TheSameSizeIsUnchangedAndMakesNoStep) {
  MapEditor editor = MapEditor::blank(4, 3);
  EXPECT_EQ(editor.resize(4, 3).status, ResizeStatus::Unchanged);
  EXPECT_FALSE(editor.canUndo());
  EXPECT_FALSE(editor.dirty());
}

TEST(MapEditorResizeTest, InvalidSizesAreRefusedAndNothingChanges) {
  MapEditor editor(layout({"#S", "PD"}));
  const Rows before = editor.layout().toRows();
  const std::int32_t limit = warehousesim::world::kMaxMapSide;
  for (const auto& [w, h] :
       std::vector<std::pair<std::int32_t, std::int32_t>>{{0, 2},
                                                          {2, 0},
                                                          {0, 0},
                                                          {-1, 2},
                                                          {2, -7},
                                                          {limit + 1, 2},
                                                          {2, limit + 1},
                                                          {2147483647, 2147483647},
                                                          {-2147483647 - 1, 1}}) {
    const ResizeResult result = editor.resize(w, h);
    EXPECT_EQ(result.status, ResizeStatus::InvalidSize) << w << " x " << h;
    EXPECT_EQ(result.cellsLost, 0U);
  }
  EXPECT_EQ(editor.layout().toRows(), before);
  EXPECT_FALSE(editor.canUndo());
  EXPECT_FALSE(editor.dirty());
}

TEST(MapEditorResizeTest, TheLimitsThemselvesAreAllowed) {
  MapEditor editor = MapEditor::blank(2, 2);
  EXPECT_EQ(
      editor.resize(warehousesim::world::kMaxMapSide, warehousesim::world::kMaxMapSide).status,
      ResizeStatus::Applied);
  EXPECT_EQ(editor.resize(1, 1).status, ResizeStatus::Applied);
  EXPECT_EQ(editor.layout().size().cellCount(), 1U);
}

TEST(MapEditorResizeTest, UndoBringsBackTheCroppedCellsExactly) {
  MapEditor editor(layout({"#S.C", "PD..", "..#."}));
  const Rows before = editor.layout().toRows();
  ASSERT_EQ(editor.resize(2, 1).status, ResizeStatus::Applied);
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#S"}));
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), before);
  EXPECT_TRUE(editor.redo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#S"}));
}

TEST(MapEditorResizeTest, CellsThatWereOutsideCanBePaintedAfterGrowing) {
  MapEditor editor = MapEditor::blank(2, 2);
  EXPECT_EQ(editor.paint({3, 3}).status, EditStatus::OutOfBounds);
  ASSERT_EQ(editor.resize(4, 4).status, ResizeStatus::Applied);
  EXPECT_EQ(editor.paint({3, 3}).status, EditStatus::Applied);
  EXPECT_EQ(editor.layout().at({3, 3}), CellType::Wall);
}

TEST(MapEditorResizeTest, PaintsAndResizesUndoInTheOrderTheyWereDone) {
  MapEditor editor = MapEditor::blank(3, 3);
  editor.paint({2, 2});
  ASSERT_EQ(editor.resize(2, 2).status, ResizeStatus::Applied);
  editor.paint({1, 1});
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"..", ".."}));
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"...", "...", "..#"}));
  EXPECT_TRUE(editor.undo());
  EXPECT_EQ(editor.layout().toRows(), (Rows{"...", "...", "..."}));
}

TEST(MapEditorResizeTest, ResizingEndsAnOpenStroke) {
  MapEditor editor = MapEditor::blank(3, 1);
  editor.beginStroke();
  editor.paint({0, 0});
  ASSERT_EQ(editor.resize(4, 1).status, ResizeStatus::Applied);
  EXPECT_FALSE(editor.strokeOpen());
  EXPECT_TRUE(editor.undo());  // the resize
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#.."}));
  EXPECT_TRUE(editor.undo());  // the stroke
  EXPECT_EQ(editor.layout().toRows(), (Rows{"..."}));
}

// ---- creating and replacing ---------------------------------------------------------------------

TEST(MapEditorCreateTest, ABlankMapIsOpenFloorAndAskedSizesAreChecked) {
  const MapEditor editor = MapEditor::blank(4, 2, "new");
  EXPECT_EQ(editor.layout().toRows(), (Rows{"....", "...."}));
  EXPECT_EQ(editor.name(), "new");
  EXPECT_FALSE(editor.dirty());
  const std::int32_t limit = warehousesim::world::kMaxMapSide;
  EXPECT_THROW((void)MapEditor::blank(0, 3), std::invalid_argument);
  EXPECT_THROW((void)MapEditor::blank(3, 0), std::invalid_argument);
  EXPECT_THROW((void)MapEditor::blank(-4, 3), std::invalid_argument);
  EXPECT_THROW((void)MapEditor::blank(limit + 1, 3), std::invalid_argument);
  EXPECT_THROW((void)MapEditor::blank(3, limit + 1), std::invalid_argument);
  EXPECT_NO_THROW((void)MapEditor::blank(limit, limit));
}

TEST(MapEditorCreateTest, ALayoutLargerThanTheFormatAllowsCannotBeEdited) {
  EXPECT_THROW(MapEditor(Warehouse(warehousesim::world::kMaxMapSide + 1, 2)),
               std::invalid_argument);
}

TEST(MapEditorCreateTest, ReplaceStartsOverCleanWithNoHistoryAndAnEndedStroke) {
  MapEditor editor = MapEditor::blank(3, 1, "old");
  editor.paint({0, 0});
  editor.beginStroke();
  editor.paint({1, 0});
  editor.replace(layout({"P.D"}), "other");
  EXPECT_EQ(editor.layout().toRows(), (Rows{"P.D"}));
  EXPECT_EQ(editor.name(), "other");
  EXPECT_FALSE(editor.dirty());
  EXPECT_FALSE(editor.canUndo());
  EXPECT_FALSE(editor.canRedo());
  EXPECT_FALSE(editor.strokeOpen());
  editor.endStroke();
  EXPECT_FALSE(editor.canUndo());  // nothing from the old stroke is left to commit
}

TEST(MapEditorCreateTest, ReplacingWithAnInvalidLayoutThrowsAndLeavesTheEditorAlone) {
  MapEditor editor = MapEditor::blank(3, 1, "keep");
  editor.paint({0, 0});
  EXPECT_THROW(editor.replace(Warehouse(600, 1), "big"), std::invalid_argument);
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#.."}));
  EXPECT_EQ(editor.name(), "keep");
  EXPECT_TRUE(editor.dirty());
  EXPECT_TRUE(editor.canUndo());
}

TEST(MapEditorCreateTest, TheEditorSaysWhetherTheMapIsReadyForASimulation) {
  MapEditor editor = MapEditor::blank(4, 1);
  EXPECT_FALSE(editor.issues().empty());
  editor.paint({0, 0}, Tool::Pickup);
  editor.paint({3, 0}, Tool::Dropoff);
  EXPECT_TRUE(editor.issues().empty());
  editor.paint({1, 0}, Tool::Wall);
  ASSERT_EQ(editor.issues().size(), 1U);  // the pickup is cut off from the dropoff
  EXPECT_FALSE(editor.issues()[0].blocksSimulation);
}

// ---- the revision -------------------------------------------------------------------------------

TEST(MapEditorRevisionTest, TheRevisionGoesUpWithEveryChangeOfTheLayoutAndOnlyThen) {
  MapEditor editor = MapEditor::blank(3, 3);
  std::uint64_t seen = editor.revision();
  const auto changed = [&] {
    const bool up = editor.revision() > seen;
    seen = editor.revision();
    return up;
  };
  const auto unchanged = [&] {
    const bool same = editor.revision() == seen;
    seen = editor.revision();
    return same;
  };

  editor.paint({0, 0}, Tool::Erase);  // already open floor
  editor.paint({7, 7}, Tool::Wall);   // outside the map
  editor.setTool(Tool::Shelf);
  editor.beginStroke();
  editor.endStroke();
  editor.markSaved();
  EXPECT_TRUE(unchanged());

  editor.paint({0, 0});
  EXPECT_TRUE(changed());
  editor.beginStroke();
  editor.paint({1, 0});  // the layout changes while the stroke is still open
  EXPECT_TRUE(changed());
  editor.endStroke();
  EXPECT_TRUE(unchanged());
  editor.paintLine({0, 2}, {2, 2}, Tool::Wall);
  EXPECT_TRUE(changed());
  EXPECT_TRUE(editor.undo());
  EXPECT_TRUE(changed());
  EXPECT_TRUE(editor.redo());
  EXPECT_TRUE(changed());
  EXPECT_EQ(editor.resize(5, 5).status, ResizeStatus::Applied);
  EXPECT_TRUE(changed());
  EXPECT_EQ(editor.resize(5, 5).status, ResizeStatus::Unchanged);
  EXPECT_EQ(editor.resize(0, 5).status, ResizeStatus::InvalidSize);
  EXPECT_TRUE(unchanged());
  EXPECT_EQ(editor.clear().status, EditStatus::Applied);
  EXPECT_TRUE(changed());
  EXPECT_EQ(editor.clear().status, EditStatus::Unchanged);
  EXPECT_TRUE(unchanged());
  EXPECT_FALSE(editor.redo());
  EXPECT_TRUE(unchanged());
  editor.replace(layout({"P.D"}), "x");
  EXPECT_TRUE(changed());
  EXPECT_FALSE(editor.undo());
  EXPECT_TRUE(unchanged());
}

// ---- a model of the whole history ---------------------------------------------------------------

// Whatever sequence of edits, resizes, undos and redos is made, the editor shows what a plain list
// of snapshots says it should, and says dirty exactly when the snapshot is not the saved one.
TEST(MapEditorPropertyTest, RandomEditsAndUndosMatchAListOfSnapshots) {
  for (std::uint64_t seed = 1; seed <= 25; ++seed) {
    warehousesim::core::Rng rng(seed);
    MapEditor editor = MapEditor::blank(6, 5);
    std::vector<Rows> states{editor.layout().toRows()};
    std::vector<int> ids{0};
    int nextId = 1;
    std::size_t at = 0;
    int savedId = 0;

    const auto pushIfChanged = [&] {
      const Rows now = editor.layout().toRows();
      if (now != states[at]) {
        states.resize(at + 1);
        ids.resize(at + 1);
        states.push_back(now);
        ids.push_back(nextId++);
        ++at;
      }
    };

    for (int step = 0; step < 70; ++step) {
      const std::uint64_t pick = rng.below(10);
      const auto x = static_cast<std::int32_t>(rng.below(10)) - 1;  // sometimes outside the map
      const auto y = static_cast<std::int32_t>(rng.below(8)) - 1;
      const Tool tool = kAllTools[rng.below(6)];
      if (pick < 4) {
        editor.paint({x, y}, tool);
        pushIfChanged();
      } else if (pick < 6) {
        editor.paintLine(
            {x, y},
            {static_cast<std::int32_t>(rng.below(6)), static_cast<std::int32_t>(rng.below(5))},
            tool);
        pushIfChanged();
      } else if (pick == 6) {
        editor.resize(static_cast<std::int32_t>(rng.below(9)),
                      static_cast<std::int32_t>(rng.below(8)));
        pushIfChanged();
      } else if (pick == 7) {
        if (editor.undo()) {
          ASSERT_GT(at, 0U);
          --at;
        }
      } else if (pick == 8) {
        if (editor.redo()) {
          ASSERT_LT(at + 1, states.size());
          ++at;
        }
      } else if (rng.below(3) == 0) {
        editor.markSaved();
        savedId = ids[at];
      } else {
        editor.clear();
        pushIfChanged();
      }
      ASSERT_EQ(editor.layout().toRows(), states[at]) << "seed " << seed << " step " << step;
      ASSERT_EQ(editor.dirty(), ids[at] != savedId) << "seed " << seed << " step " << step;
      ASSERT_EQ(editor.canUndo(), at > 0) << "seed " << seed << " step " << step;
      ASSERT_EQ(editor.canRedo(), at + 1 < states.size()) << "seed " << seed << " step " << step;
    }
  }
}

}  // namespace
