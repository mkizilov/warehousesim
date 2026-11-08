#include "warehousesim/view/editor_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "scratch_dir.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::editor::MapEditor;
using warehousesim::editor::Tool;
using warehousesim::view::EditorCommand;
using warehousesim::view::EditorController;
using warehousesim::view::InputEvents;
using warehousesim::view::PointerAction;
using warehousesim::view::PointerButton;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenSize;
using warehousesim::world::CellType;
using warehousesim_test::ScratchDir;

using Kind = PointerAction::Kind;
using Rows = std::vector<std::string>;

constexpr ScreenSize kWindow{800, 600};

EditorController make(MapEditor editor, const std::string& file = "map.json") {
  return EditorController(kWindow, std::move(editor), file);
}

EditorController blank(int width = 10, int height = 6) {
  return make(MapEditor::blank(width, height, "test"));
}

// The middle of a cell on screen.
ScreenPoint centre(const EditorController& c, Point cell) {
  const auto rect = c.camera().cellRect(cell);
  return {rect.x + rect.width / 2.0, rect.y + rect.height / 2.0};
}

InputEvents frame(std::vector<PointerAction> actions = {},
                  std::vector<EditorCommand> commands = {}) {
  InputEvents input;
  input.pointerActions = std::move(actions);
  input.commands = std::move(commands);
  return input;
}

PointerAction press(ScreenPoint at, PointerButton button = PointerButton::Primary) {
  return {Kind::Press, button, at};
}
PointerAction move(ScreenPoint at) { return {Kind::Move, PointerButton::Primary, at}; }
PointerAction release(ScreenPoint at, PointerButton button = PointerButton::Primary) {
  return {Kind::Release, button, at};
}

void send(EditorController& c, EditorCommand command) { c.apply(frame({}, {command}), 0.016); }

void click(EditorController& c, Point cell, PointerButton button = PointerButton::Primary) {
  c.apply(frame({press(centre(c, cell), button)}), 0.016);
  c.apply(frame({release(centre(c, cell), button)}), 0.016);
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// ---- painting -----------------------------------------------------------------------------------

TEST(EditorControllerTest, ALeftClickPaintsTheSelectedToolAndARightClickErases) {
  EditorController c = blank();
  click(c, {2, 1});
  EXPECT_EQ(c.editor().layout().at({2, 1}), CellType::Wall);
  send(c, EditorCommand::ToolShelf);
  click(c, {3, 1});
  EXPECT_EQ(c.editor().layout().at({3, 1}), CellType::Shelf);
  click(c, {2, 1}, PointerButton::Secondary);  // right button erases, whatever the tool
  EXPECT_EQ(c.editor().layout().at({2, 1}), CellType::Free);
  EXPECT_EQ(c.editor().tool(), Tool::Shelf);
}

TEST(EditorControllerTest, EachToolCommandSelectsItsTool) {
  EditorController c = blank();
  const std::vector<std::pair<EditorCommand, Tool>> tools{
      {EditorCommand::ToolWall, Tool::Wall},         {EditorCommand::ToolShelf, Tool::Shelf},
      {EditorCommand::ToolPickup, Tool::Pickup},     {EditorCommand::ToolDropoff, Tool::Dropoff},
      {EditorCommand::ToolCharging, Tool::Charging}, {EditorCommand::ToolErase, Tool::Erase}};
  int column = 0;
  for (const auto& [command, tool] : tools) {
    send(c, command);
    EXPECT_EQ(c.editor().tool(), tool);
    click(c, {column++, 0});
  }
  EXPECT_EQ(c.editor().layout().toRows()[0].substr(0, 6), "#SPDC.");
}

TEST(EditorControllerTest, APressAndReleaseInTheSameFramePaints) {
  EditorController c = blank();
  const ScreenPoint at = centre(c, {4, 2});
  c.apply(frame({press(at), release(at)}), 0.016);
  EXPECT_EQ(c.editor().layout().at({4, 2}), CellType::Wall);
  EXPECT_TRUE(c.editor().canUndo());
  EXPECT_FALSE(c.editor().strokeOpen());
}

TEST(EditorControllerTest, ADragIsOneUndoStepAndFastMovesAreJoinedWithoutGaps) {
  EditorController c = blank();
  c.apply(frame({press(centre(c, {0, 0})), move(centre(c, {5, 3})), move(centre(c, {9, 3}))}),
          0.016);
  EXPECT_TRUE(c.editor().strokeOpen());
  c.apply(frame({release(centre(c, {9, 3}))}), 0.016);
  EXPECT_FALSE(c.editor().strokeOpen());
  // (0,0)-(5,3) is 6 cells and (5,3)-(9,3) is 5, and they share (5,3).
  EXPECT_EQ(c.editor().layout().cellsOfType(CellType::Wall).size(), 10U);
  EXPECT_EQ(c.editor().layout().at({9, 3}), CellType::Wall);
  EXPECT_EQ(c.editor().layout().at({5, 3}), CellType::Wall);
  send(c, EditorCommand::Undo);
  EXPECT_TRUE(c.editor().layout().cellsOfType(CellType::Wall).empty());
  EXPECT_FALSE(c.editor().canUndo());
}

TEST(EditorControllerTest, ADragWithTheRightButtonErasesAlongTheWay) {
  EditorController c = make(MapEditor(*warehousesim::world::Warehouse::fromRows(
      {"##########", "##########", "##########", "##########", "##########", "##########"})));
  c.apply(frame({press(centre(c, {1, 2}), PointerButton::Secondary), move(centre(c, {8, 2})),
                 release(centre(c, {8, 2}), PointerButton::Secondary)}),
          0.016);
  EXPECT_EQ(c.editor().layout().toRows()[2], "#........#");
  EXPECT_EQ(c.editor().layout().toRows()[1], "##########");
}

TEST(EditorControllerTest, MovingWithoutAButtonDownPaintsNothing) {
  EditorController c = blank();
  c.apply(frame({move(centre(c, {1, 1})), move(centre(c, {2, 2}))}), 0.016);
  EXPECT_FALSE(c.editor().canUndo());
  EXPECT_TRUE(c.editor().layout().cellsOfType(CellType::Wall).empty());
}

TEST(EditorControllerTest, ClicksOutsideTheGridOnARulerOrOnTheBarDoNothing) {
  EditorController c = blank();
  const std::vector<ScreenPoint> nowhere{{5.0, 5.0},             // the ruler corner
                                         {400.0, 5.0},           // the column ruler
                                         {5.0, 300.0},           // the row ruler
                                         {400.0, 590.0},         // the bar
                                         {2.0, 590.0},           // the bar's corner
                                         {790.0, 300.0},         // beside the grid
                                         {400.0, 560.0},         // just under the canvas
                                         {-20.0, -20.0},         // outside the window
                                         {std::nan(""), 100.0},  // garbage
                                         {1e300, 1e300}};
  for (const ScreenPoint at : nowhere) {
    c.apply(frame({press(at), move(at), release(at)}), 0.016);
    EXPECT_FALSE(c.editor().canUndo()) << at.x << "," << at.y;
  }
  EXPECT_TRUE(c.editor().layout().cellsOfType(CellType::Wall).empty());
}

TEST(EditorControllerTest, AClickOnARulerDoesNotPaintTheCellHiddenBehindIt) {
  // Zoomed far in and moved, the map fills the canvas, so cells lie under the rulers too.
  EditorController c = blank();
  InputEvents zoom;
  zoom.zoomSteps = 30.0;
  zoom.zoomAnchor = ScreenPoint{0.0, 0.0};
  c.apply(zoom, 0.016);
  InputEvents shift;
  shift.dragX = -200.0;
  shift.dragY = -200.0;
  c.apply(shift, 0.016);
  for (const ScreenPoint at : {ScreenPoint{3.0, 100.0}, ScreenPoint{100.0, 3.0}}) {
    c.apply(frame({press(at), release(at)}), 0.016);
  }
  EXPECT_FALSE(c.editor().canUndo());
  // The same place just inside the rulers is a cell.
  c.apply(frame({press(ScreenPoint{100.0, 100.0}), release(ScreenPoint{100.0, 100.0})}), 0.016);
  EXPECT_TRUE(c.editor().canUndo());
}

TEST(EditorControllerTest, ADragThatLeavesTheCanvasDoesNotJoinUpWhereItComesBack) {
  EditorController c = blank();
  c.apply(frame({press(centre(c, {1, 1})), move(ScreenPoint{400.0, 590.0}),  // over the bar
                 move(centre(c, {8, 4})), release(centre(c, {8, 4}))}),
          0.016);
  EXPECT_EQ(c.editor().layout().cellsOfType(CellType::Wall).size(), 2U);  // no line between them
  EXPECT_EQ(c.editor().layout().at({1, 1}), CellType::Wall);
  EXPECT_EQ(c.editor().layout().at({8, 4}), CellType::Wall);
}

TEST(EditorControllerTest, ASecondButtonWhileOneIsDownIsIgnoredAndTheFirstReleaseEndsTheStroke) {
  EditorController c = blank();
  c.apply(frame({press(centre(c, {1, 1})),
                 press(centre(c, {2, 2}), PointerButton::Secondary),      // ignored
                 release(centre(c, {2, 2}), PointerButton::Secondary)}),  // not the drag's button
          0.016);
  EXPECT_TRUE(c.editor().strokeOpen());
  c.apply(frame({release(centre(c, {1, 1}))}), 0.016);
  EXPECT_FALSE(c.editor().strokeOpen());
  EXPECT_EQ(c.editor().layout().cellsOfType(CellType::Wall).size(), 1U);
}

TEST(EditorControllerTest, UndoDuringADragEndsTheDragSoLaterMovesPaintNothing) {
  EditorController c = blank();
  c.apply(frame({press(centre(c, {1, 1})), move(centre(c, {3, 1}))}), 0.016);
  send(c, EditorCommand::Undo);
  EXPECT_TRUE(c.editor().layout().cellsOfType(CellType::Wall).empty());
  c.apply(frame({move(centre(c, {5, 1})), release(centre(c, {5, 1}))}), 0.016);
  EXPECT_TRUE(c.editor().layout().cellsOfType(CellType::Wall).empty());
}

TEST(EditorControllerTest,
     ACommandInTheMiddleOfADragEndsTheStrokeSoItsReleaseHasNothingLeftToClose) {
  EditorController c = blank();
  c.apply(frame({press(centre(c, {1, 1}))}), 0.016);
  EXPECT_TRUE(c.editor().strokeOpen());
  send(c, EditorCommand::ToolShelf);
  EXPECT_FALSE(c.editor().strokeOpen());
  c.apply(frame({release(centre(c, {1, 1}))}), 0.016);
  EXPECT_FALSE(c.editor().strokeOpen());
  send(c, EditorCommand::Undo);
  EXPECT_TRUE(c.editor().layout().cellsOfType(CellType::Wall).empty());  // the stroke was one step
}

TEST(EditorControllerTest, TheEditableAreaStartsExactlyBelowTheRulersAndEndsAtTheBar) {
  // Zoomed far in and moved, so that cells lie under the rulers and under the bar's edge.
  EditorController c = blank();
  InputEvents zoom;
  zoom.zoomSteps = 30.0;
  zoom.zoomAnchor = ScreenPoint{0.0, 0.0};
  c.apply(zoom, 0.016);
  InputEvents shift;
  shift.dragX = -200.0;
  shift.dragY = -200.0;
  c.apply(shift, 0.016);
  const auto area = c.canvas();
  const auto paintedAt = [&c](ScreenPoint at) {
    const auto revision = c.editor().revision();
    c.apply(frame({press(at), release(at)}), 0.016);
    return c.editor().revision() != revision;
  };
  // Every probe that lands on a cell puts something there that was not there, so the map changes.
  send(c, EditorCommand::ToolShelf);
  EXPECT_FALSE(paintedAt({area.x - 0.01, area.y + 50.0}));
  EXPECT_TRUE(paintedAt({area.x, area.y + 50.0}));
  EXPECT_FALSE(paintedAt({area.x + 50.0, area.y - 0.01}));
  send(c, EditorCommand::ToolWall);
  EXPECT_TRUE(paintedAt({area.x + 50.0, area.y}));
  send(c, EditorCommand::ToolPickup);
  EXPECT_TRUE(paintedAt({area.x + 150.0, area.bottom() - 0.01}));
  // A probe just outside would land on the same cell, so use another tool: if it did land, it would
  // change the cell and the map.
  send(c, EditorCommand::ToolCharging);
  EXPECT_FALSE(paintedAt({area.x + 150.0, area.bottom()}));
  send(c, EditorCommand::ToolDropoff);
  EXPECT_TRUE(paintedAt({area.right() - 0.01, area.y + 120.0}));
  send(c, EditorCommand::ToolCharging);
  EXPECT_FALSE(paintedAt({area.right(), area.y + 120.0}));
}

TEST(EditorControllerTest, ThePointerOverTheMapNamesTheCellOnlyOverTheEditablePart) {
  EditorController c = blank();
  InputEvents hover;
  hover.pointer = centre(c, {3, 2});
  c.apply(hover, 0.016);
  EXPECT_EQ(c.hoveredCell(), std::optional<Point>(Point{3, 2}));
  hover.pointer = ScreenPoint{400.0, 590.0};
  c.apply(hover, 0.016);
  EXPECT_FALSE(c.hoveredCell().has_value());
  hover.pointer = ScreenPoint{3.0, 3.0};
  c.apply(hover, 0.016);
  EXPECT_FALSE(c.hoveredCell().has_value());
  hover.pointer.reset();
  hover.pointerLeft = true;
  c.apply(hover, 0.016);
  EXPECT_FALSE(c.pointer().has_value());
}

// ---- commands -----------------------------------------------------------------------------------

TEST(EditorControllerTest, UndoAndRedoWorkAndSayWhenThereIsNothingToDo) {
  EditorController c = blank();
  send(c, EditorCommand::Undo);
  EXPECT_EQ(c.message(), "nothing to undo");
  click(c, {1, 1});
  send(c, EditorCommand::Undo);
  EXPECT_EQ(c.editor().layout().at({1, 1}), CellType::Free);
  send(c, EditorCommand::Redo);
  EXPECT_EQ(c.editor().layout().at({1, 1}), CellType::Wall);
  send(c, EditorCommand::Redo);
  EXPECT_EQ(c.message(), "nothing to redo");
}

TEST(EditorControllerTest, TheMapCanBeMadeWiderNarrowerTallerAndShorterAndTheViewFollows) {
  EditorController c = blank(10, 6);
  send(c, EditorCommand::Wider);
  EXPECT_EQ(c.editor().layout().width(), 11);
  EXPECT_TRUE(contains(c.message(), "the map is now 11 x 6")) << c.message();
  send(c, EditorCommand::Taller);
  send(c, EditorCommand::Taller);
  EXPECT_EQ(c.editor().layout().height(), 8);
  send(c, EditorCommand::Narrower);
  send(c, EditorCommand::Shorter);
  EXPECT_EQ(c.editor().layout().width(), 10);
  EXPECT_EQ(c.editor().layout().height(), 7);
  EXPECT_EQ(c.camera().grid(), c.editor().layout().size());
  EXPECT_FALSE(c.messageIsError());
}

TEST(EditorControllerTest, ANewlyAddedRowCanBePaintedAtOnce) {
  EditorController c = blank(5, 3);
  send(c, EditorCommand::Taller);
  click(c, {2, 3});
  EXPECT_EQ(c.editor().layout().at({2, 3}), CellType::Wall);
}

TEST(EditorControllerTest, TheMapCannotBecomeSmallerThanOneCellOrLargerThanTheLimit) {
  EditorController tiny = make(MapEditor::blank(1, 1));
  send(tiny, EditorCommand::Narrower);
  send(tiny, EditorCommand::Shorter);
  EXPECT_EQ(tiny.editor().layout().size().cellCount(), 1U);
  EXPECT_TRUE(tiny.messageIsError());
  EXPECT_TRUE(contains(tiny.message(), "from 1 to 512")) << tiny.message();

  const int limit = warehousesim::world::kMaxMapSide;
  EditorController huge = make(MapEditor::blank(limit, limit));
  send(huge, EditorCommand::Wider);
  send(huge, EditorCommand::Taller);
  EXPECT_EQ(huge.editor().layout().width(), limit);
  EXPECT_EQ(huge.editor().layout().height(), limit);
  EXPECT_TRUE(huge.messageIsError());
}

TEST(EditorControllerTest, ShrinkingTheMapSaysWhatWasCroppedAndUndoBringsItBack) {
  EditorController c = make(MapEditor(*warehousesim::world::Warehouse::fromRows({"..#", "..S"})));
  send(c, EditorCommand::Narrower);
  EXPECT_TRUE(contains(c.message(), "2 objects were cropped")) << c.message();
  EXPECT_EQ(c.editor().layout().toRows(), (Rows{"..", ".."}));
  send(c, EditorCommand::Undo);
  EXPECT_EQ(c.editor().layout().toRows(), (Rows{"..#", "..S"}));
  EXPECT_EQ(c.camera().grid(), c.editor().layout().size());
}

TEST(EditorControllerTest, ClearEmptiesTheMapOnceAndThenSaysItIsEmpty) {
  EditorController c = make(MapEditor(*warehousesim::world::Warehouse::fromRows({"#S", "PD"})));
  send(c, EditorCommand::Clear);
  EXPECT_EQ(c.editor().layout().toRows(), (Rows{"..", ".."}));
  EXPECT_TRUE(contains(c.message(), "Ctrl+Z"));
  send(c, EditorCommand::Clear);
  EXPECT_EQ(c.message(), "the map is already empty");
}

// ---- saving, loading and unsaved changes --------------------------------------------------------

TEST(EditorControllerTest, SaveWritesTheFileAndLoadReadsItBack) {
  const ScratchDir dir("editor_controller_save");
  const std::string file = dir.file("mine.json").string();
  EditorController c = make(MapEditor::blank(6, 3, "mine"), file);
  click(c, {1, 1});
  EXPECT_TRUE(c.editor().dirty());
  send(c, EditorCommand::Save);
  EXPECT_FALSE(c.editor().dirty());
  EXPECT_TRUE(contains(c.message(), "saved ")) << c.message();
  EXPECT_FALSE(c.messageIsError());

  EditorController other = make(MapEditor::blank(2, 2), file);
  send(other, EditorCommand::Load);  // a clean editor loads without asking
  EXPECT_EQ(other.editor().layout().toRows(), c.editor().layout().toRows());
  EXPECT_EQ(other.editor().name(), "mine");
  EXPECT_EQ(other.camera().grid(), other.editor().layout().size());
  EXPECT_TRUE(other.camera().fitted());
}

TEST(EditorControllerTest, ASaveThatFailsSaysSoAndTheMapStaysUnsaved) {
  const ScratchDir dir("editor_controller_save_fails");
  EditorController c = make(MapEditor::blank(3, 3), dir.file("no/such/dir/a.json").string());
  click(c, {0, 0});
  send(c, EditorCommand::Save);
  EXPECT_TRUE(c.messageIsError());
  EXPECT_TRUE(contains(c.message(), "not saved")) << c.message();
  EXPECT_TRUE(c.editor().dirty());
}

TEST(EditorControllerTest, LoadingOverUnsavedChangesNeedsTheCommandTwiceInARow) {
  const ScratchDir dir("editor_controller_load_twice");
  const std::string file = dir.file("a.json").string();
  EditorController c = make(MapEditor::blank(4, 2, "a"), file);
  send(c, EditorCommand::Save);
  click(c, {0, 0});  // unsaved change
  send(c, EditorCommand::Load);
  EXPECT_TRUE(c.messageIsError());
  EXPECT_TRUE(contains(c.message(), "unsaved changes")) << c.message();
  EXPECT_EQ(c.editor().layout().at({0, 0}), CellType::Wall);  // still there
  send(c, EditorCommand::Load);
  EXPECT_EQ(c.editor().layout().at({0, 0}), CellType::Free);  // now discarded
  EXPECT_FALSE(c.editor().dirty());
}

TEST(EditorControllerTest, AnythingElseInBetweenTakesBackTheQuestion) {
  const ScratchDir dir("editor_controller_cancel");
  const std::string file = dir.file("a.json").string();
  EditorController c = make(MapEditor::blank(4, 2, "a"), file);
  send(c, EditorCommand::Save);
  click(c, {0, 0});
  send(c, EditorCommand::Load);
  send(c, EditorCommand::ToolShelf);  // another command
  send(c, EditorCommand::Load);       // asks again instead of discarding
  EXPECT_EQ(c.editor().layout().at({0, 0}), CellType::Wall);
  click(c, {1, 1});  // a click is something else, too
  send(c, EditorCommand::Load);
  EXPECT_EQ(c.editor().layout().at({0, 0}), CellType::Wall);
  send(c, EditorCommand::Load);
  EXPECT_EQ(c.editor().layout().at({0, 0}), CellType::Free);
}

TEST(EditorControllerTest, ALoadThatFailsLeavesTheEditorAsItWasAndReportsTheProblem) {
  const ScratchDir dir("editor_controller_load_fails");
  warehousesim_test::writeFile(dir.file("bad.json"), "{ not json");
  EditorController c = make(MapEditor::blank(3, 3, "keep"), dir.file("bad.json").string());
  send(c, EditorCommand::Load);
  EXPECT_TRUE(c.messageIsError());
  EXPECT_TRUE(contains(c.message(), "not loaded")) << c.message();
  EXPECT_EQ(c.editor().name(), "keep");
  EXPECT_EQ(c.editor().layout().width(), 3);
}

TEST(EditorControllerTest, TheLayoutKeyStepsThroughEveryPredefinedLayoutAndAround) {
  EditorController c = blank();
  const auto layouts = warehousesim::world::predefinedLayouts();
  for (int round = 0; round < 2; ++round) {
    for (const auto& layout : layouts) {
      send(c, EditorCommand::NextLayout);
      EXPECT_EQ(c.editor().name(), layout.name);
      EXPECT_EQ(c.editor().layout().toRows(), layout.make().toRows());
      EXPECT_EQ(c.camera().grid(), c.editor().layout().size());
      EXPECT_TRUE(c.camera().fitted());
      EXPECT_FALSE(c.editor().dirty());
    }
  }
}

TEST(EditorControllerTest, LoadingAMapOrALayoutShowsTheWholeMapEvenIfTheViewHadBeenZoomedAndMoved) {
  const ScratchDir dir("editor_controller_refit");
  EditorController c = make(MapEditor::blank(10, 6, "a"), dir.file("a.json").string());
  send(c, EditorCommand::Save);
  const auto zoomedAndMoved = [&c] {
    InputEvents input;
    input.zoomSteps = 6.0;
    input.dragX = 80.0;
    c.apply(input, 0.016);
    EXPECT_FALSE(c.camera().fitted());
  };
  zoomedAndMoved();
  send(c, EditorCommand::Load);
  EXPECT_TRUE(c.camera().fitted());
  zoomedAndMoved();
  send(c, EditorCommand::NextLayout);
  EXPECT_TRUE(c.camera().fitted());
}

TEST(EditorControllerTest, ChoosingAToolClearsTheLastMessage) {
  EditorController c = blank();
  send(c, EditorCommand::Undo);
  ASSERT_EQ(c.message(), "nothing to undo");
  send(c, EditorCommand::ToolShelf);
  EXPECT_TRUE(c.message().empty());
  EXPECT_FALSE(c.messageIsError());
}

TEST(EditorControllerTest, ReplacingTheMapWithALayoutAsksFirstWhenThereAreUnsavedChanges) {
  EditorController c = blank();
  click(c, {0, 0});
  send(c, EditorCommand::NextLayout);
  EXPECT_TRUE(c.messageIsError());
  EXPECT_EQ(c.editor().name(), "test");
  send(c, EditorCommand::NextLayout);
  EXPECT_EQ(c.editor().name(), warehousesim::world::predefinedLayouts()[0].name);
}

TEST(EditorControllerTest, ACustomisedLayoutIsSavedAndComesBackFromItsFile) {
  const ScratchDir dir("editor_controller_customise");
  const std::string file = dir.file("custom.json").string();
  EditorController c = make(MapEditor::blank(2, 2), file);
  send(c, EditorCommand::NextLayout);
  send(c, EditorCommand::ToolShelf);
  click(c, {5, 5});
  send(c, EditorCommand::Wider);
  send(c, EditorCommand::Save);
  const auto saved = c.editor().layout().toRows();
  EditorController again = make(MapEditor::blank(2, 2), file);
  send(again, EditorCommand::Load);
  EXPECT_EQ(again.editor().layout().toRows(), saved);
}

TEST(EditorControllerTest, QuittingWithNothingUnsavedIsImmediate) {
  EditorController c = blank();
  EXPECT_FALSE(c.wantsQuit());
  InputEvents quit;
  quit.quit = true;
  c.apply(quit, 0.016);
  EXPECT_TRUE(c.wantsQuit());
}

TEST(EditorControllerTest, QuittingWithUnsavedChangesNeedsAnotherRequest) {
  EditorController c = blank();
  click(c, {0, 0});
  InputEvents quit;
  quit.quit = true;
  c.apply(quit, 0.016);
  EXPECT_FALSE(c.wantsQuit());
  EXPECT_TRUE(c.messageIsError());
  EXPECT_TRUE(contains(c.message(), "quit without saving")) << c.message();
  c.apply(InputEvents{}, 0.016);  // time passing does not confirm it
  EXPECT_FALSE(c.wantsQuit());
  c.apply(quit, 0.016);
  EXPECT_TRUE(c.wantsQuit());
}

TEST(EditorControllerTest, AnotherActionTakesBackTheQuestionToQuit) {
  EditorController c = blank();
  click(c, {0, 0});
  InputEvents quit;
  quit.quit = true;
  c.apply(quit, 0.016);
  send(c, EditorCommand::ToolShelf);
  c.apply(quit, 0.016);
  EXPECT_FALSE(c.wantsQuit());
  c.apply(quit, 0.016);
  EXPECT_TRUE(c.wantsQuit());
}

TEST(EditorControllerTest, SavingFirstMakesQuittingImmediate) {
  const ScratchDir dir("editor_controller_quit_saved");
  EditorController c = make(MapEditor::blank(3, 3), dir.file("a.json").string());
  click(c, {0, 0});
  send(c, EditorCommand::Save);
  InputEvents quit;
  quit.quit = true;
  c.apply(quit, 0.016);
  EXPECT_TRUE(c.wantsQuit());
}

// ---- the view -----------------------------------------------------------------------------------

TEST(EditorControllerTest, TheCameraLooksAtTheWindowWithoutTheBar) {
  const EditorController c = blank();
  EXPECT_EQ(c.window(), kWindow);
  EXPECT_EQ(c.camera().screen().width, 800);
  EXPECT_EQ(c.camera().screen().height,
            600 - static_cast<int>(warehousesim::view::kEditorBarHeight));
  EXPECT_TRUE(c.camera().fitted());
}

TEST(EditorControllerTest, ResizingTheWindowRefitsAFittedView) {
  EditorController c = blank();
  c.resize(ScreenSize{1200, 900});
  EXPECT_EQ(c.window(), (ScreenSize{1200, 900}));
  EXPECT_EQ(c.camera().screen().width, 1200);
  EXPECT_TRUE(c.camera().fitted());
  c.resize(ScreenSize{10, 10});  // smaller than the bar
  EXPECT_EQ(c.camera().screen().height, 0);
  click(c, {0, 0});  // and nothing breaks
  EXPECT_FALSE(c.editor().canUndo());
}

TEST(EditorControllerTest, ZoomPanAndFitStillWorkAndPlaybackKeysDoNothing) {
  EditorController c = blank();
  const double fitted = c.camera().zoom();
  InputEvents zoom;
  zoom.zoomSteps = 3.0;
  c.apply(zoom, 0.016);
  EXPECT_GT(c.camera().zoom(), fitted);
  EXPECT_FALSE(c.camera().fitted());
  InputEvents drag;
  drag.dragX = 30.0;
  const auto before = c.camera().center();
  c.apply(drag, 0.016);
  EXPECT_FALSE(c.camera().center() == before);
  InputEvents other;  // none of this means anything to an editor
  other.togglePause = true;
  other.togglePaths = true;
  other.speedSteps = 3;
  other.resetView = true;
  c.apply(other, 0.016);
  EXPECT_TRUE(c.camera().fitted());
  EXPECT_EQ(c.editor().layout().cellsOfType(CellType::Wall).size(), 0U);
}

TEST(EditorControllerTest, ThePaintingPointIsWhereTheCameraWasWhenTheClickHappened) {
  EditorController c = blank();
  const ScreenPoint at = centre(c, {4, 2});
  InputEvents both = frame({press(at), release(at)});
  both.zoomSteps =
      6.0;  // zoom in the same frame; the click still lands on the cell it was aimed at
  both.zoomAnchor = ScreenPoint{700.0, 500.0};
  c.apply(both, 0.016);
  EXPECT_EQ(c.editor().layout().at({4, 2}), CellType::Wall);
}

TEST(EditorControllerTest, TheIssuesFollowTheMapAsItChanges) {
  EditorController c = blank(5, 1);
  EXPECT_FALSE(c.issues().empty());
  send(c, EditorCommand::ToolPickup);
  click(c, {0, 0});
  send(c, EditorCommand::ToolDropoff);
  click(c, {4, 0});
  EXPECT_TRUE(c.issues().empty());
  send(c, EditorCommand::ToolWall);
  click(c, {2, 0});
  ASSERT_EQ(c.issues().size(), 1U);  // the two ends are cut off from each other
  send(c, EditorCommand::Undo);
  EXPECT_TRUE(c.issues().empty());
}

// ---- whatever the input -------------------------------------------------------------------------

TEST(EditorControllerPropertyTest, RandomInputNeverBreaksTheEditorOrTheView) {
  const ScratchDir dir("editor_controller_fuzz");
  for (std::uint64_t seed = 1; seed <= 8; ++seed) {
    warehousesim::core::Rng rng(seed);
    EditorController c = make(MapEditor::blank(8, 5, "fuzz"), dir.file("fuzz.json").string());
    const auto randomPoint = [&rng] {
      return ScreenPoint{static_cast<double>(rng.below(900)) - 50.0,
                         static_cast<double>(rng.below(700)) - 50.0};
    };
    for (int step = 0; step < 400; ++step) {
      InputEvents input;
      for (std::uint64_t i = rng.below(4); i > 0; --i) {
        input.commands.push_back(static_cast<EditorCommand>(rng.below(16)));
      }
      for (std::uint64_t i = rng.below(5); i > 0; --i) {
        input.pointerActions.push_back({static_cast<Kind>(rng.below(3)),
                                        static_cast<PointerButton>(rng.below(2)), randomPoint()});
      }
      if (rng.below(10) == 0) {
        input.zoomSteps = static_cast<double>(rng.below(7)) - 3.0;
      }
      if (rng.below(10) == 0) {
        input.dragX = static_cast<double>(rng.below(200)) - 100.0;
      }
      if (rng.below(50) == 0) {
        input.pointerActions.push_back(
            {Kind::Press, PointerButton::Primary,
             ScreenPoint{std::numeric_limits<double>::quiet_NaN(), 1.0}});
      }
      if (rng.below(20) == 0) {
        c.resize(ScreenSize{static_cast<int>(rng.below(1200)), static_cast<int>(rng.below(900))});
      }
      c.apply(input, 0.016);
      const auto size = c.editor().layout().size();
      ASSERT_EQ(c.camera().grid(), size) << "seed " << seed << " step " << step;
      ASSERT_TRUE(warehousesim::world::isValidMapSide(size.width()));
      ASSERT_TRUE(warehousesim::world::isValidMapSide(size.height()));
      ASSERT_EQ(c.issues().empty(), c.editor().issues().empty());
    }
  }
}

}  // namespace
