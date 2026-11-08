#include "warehousesim/view/editor_scene.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "warehousesim/view/scene.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::editor::MapEditor;
using warehousesim::editor::Tool;
using warehousesim::view::buildEditorScene;
using warehousesim::view::Color;
using warehousesim::view::DrawList;
using warehousesim::view::EditorCommand;
using warehousesim::view::EditorController;
using warehousesim::view::editorStatusLines;
using warehousesim::view::FillRect;
using warehousesim::view::InputEvents;
using warehousesim::view::kEditorBarHeight;
using warehousesim::view::kGlyphSize;
using warehousesim::view::Line;
using warehousesim::view::PointerAction;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenRect;
using warehousesim::view::ScreenSize;
using warehousesim::view::Text;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

constexpr ScreenSize kWindow{800, 600};
constexpr double kEps = 1e-6;

EditorController make(MapEditor editor, const std::string& file = "maps/hall.json") {
  return EditorController(kWindow, std::move(editor), file);
}

EditorController blank() { return make(MapEditor::blank(10, 6, "test")); }

void send(EditorController& c, EditorCommand command) {
  InputEvents input;
  input.commands.push_back(command);
  c.apply(input, 0.016);
}

void click(EditorController& c, Point cell) {
  const auto rect = c.camera().cellRect(cell);
  const ScreenPoint at{rect.x + rect.width / 2.0, rect.y + rect.height / 2.0};
  InputEvents input;
  input.pointerActions.push_back(
      {PointerAction::Kind::Press, warehousesim::view::PointerButton::Primary, at});
  input.pointerActions.push_back(
      {PointerAction::Kind::Release, warehousesim::view::PointerButton::Primary, at});
  c.apply(input, 0.016);
}

void hover(EditorController& c, Point cell) {
  const auto rect = c.camera().cellRect(cell);
  InputEvents input;
  input.pointer = ScreenPoint{rect.x + rect.width / 2.0, rect.y + rect.height / 2.0};
  c.apply(input, 0.016);
}

std::vector<const Text*> texts(const DrawList& list) {
  std::vector<const Text*> found;
  for (const auto& command : list.commands) {
    if (const auto* text = std::get_if<Text>(&command)) {
      found.push_back(text);
    }
  }
  return found;
}

const Text* findText(const DrawList& list, const std::string& part) {
  for (const Text* text : texts(list)) {
    if (text->text.find(part) != std::string::npos) {
      return text;
    }
  }
  return nullptr;
}

bool near(double a, double b) { return std::fabs(a - b) < kEps; }

// The four lines of an outline just inside the cell (see view::addOutline).
bool hasOutline(const DrawList& list, const ScreenRect& cell) {
  const double margin = cell.width * 0.06;
  for (const auto& command : list.commands) {
    if (const auto* line = std::get_if<Line>(&command)) {
      if (near(line->from.x, cell.x + margin) && near(line->from.y, cell.y + margin) &&
          near(line->to.x, cell.right() - margin) && near(line->to.y, cell.y + margin)) {
        return true;
      }
    }
  }
  return false;
}

std::size_t indexOfBar(const DrawList& list) {
  const double top = 600.0 - kEditorBarHeight;
  for (std::size_t i = 0; i < list.commands.size(); ++i) {
    if (const auto* fill = std::get_if<FillRect>(&list.commands[i])) {
      if (near(fill->rect.y, top) && near(fill->rect.width, 800.0) &&
          near(fill->rect.height, kEditorBarHeight)) {
        return i;
      }
    }
  }
  return list.commands.size();
}

// ---- the picture --------------------------------------------------------------------------------

TEST(EditorSceneTest, EachObjectOnTheMapIsDrawnAtItsCellInTheColourOfItsKind) {
  EditorController c = make(MapEditor(*Warehouse::fromRows({"#SPDC", ".....", "....."})));
  const DrawList list = buildEditorScene(c);
  const std::vector<CellType> kinds{CellType::Wall, CellType::Shelf, CellType::Pickup,
                                    CellType::Dropoff, CellType::Charging};
  for (int x = 0; x < 5; ++x) {
    const ScreenRect rect = c.camera().cellRect({x, 0});
    const FillRect expected{rect,
                            warehousesim::view::cellColor(kinds[static_cast<std::size_t>(x)])};
    const bool drawn =
        std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
          const auto* fill = std::get_if<FillRect>(&command);
          return fill != nullptr && *fill == expected;
        });
    EXPECT_TRUE(drawn) << "cell " << x;
  }
}

TEST(EditorSceneTest, TheLettersOfPickupDropoffAndChargingCellsAreShownWhenZoomedIn) {
  EditorController c = make(MapEditor(*Warehouse::fromRows({"PDC"})));
  const DrawList list = buildEditorScene(c);
  for (const char* letter : {"P", "D", "C"}) {
    bool found = false;
    for (const Text* text : texts(list)) {
      found = found || text->text == letter;
    }
    EXPECT_TRUE(found) << letter;
  }
}

TEST(EditorSceneTest, TheBarIsDrawnLastOverEverythingAndAtTheBottomOfTheWindow) {
  const EditorController c = blank();
  const DrawList list = buildEditorScene(c);
  const std::size_t bar = indexOfBar(list);
  ASSERT_LT(bar, list.commands.size());
  // Everything after the panel is inside the bar; nothing of the map or the rulers comes later.
  const double top = 600.0 - kEditorBarHeight;
  for (std::size_t i = bar + 1; i < list.commands.size(); ++i) {
    if (const auto* fill = std::get_if<FillRect>(&list.commands[i])) {
      EXPECT_GE(fill->rect.y, top - kEps);
      EXPECT_LE(fill->rect.bottom(), 600.0 + kEps);
    } else if (const auto* text = std::get_if<Text>(&list.commands[i])) {
      EXPECT_GE(text->position.y, top - kEps);
      EXPECT_LE(text->position.y + kGlyphSize, 600.0 + kEps);
    } else {
      ADD_FAILURE() << "a line after the bar";
    }
  }
}

TEST(EditorSceneTest, TheRulersNumberTheColumnsAndRows) {
  const EditorController c = blank();  // 10 columns, 6 rows
  const DrawList list = buildEditorScene(c);
  const std::vector<const Text*> all = texts(list);
  const auto count = [&all](const std::string& label) {
    return std::count_if(all.begin(), all.end(),
                         [&label](const Text* text) { return text->text == label; });
  };
  EXPECT_EQ(count("0"), 2);   // column 0 and row 0
  EXPECT_EQ(count("5"), 2);   // column 5 and row 5
  EXPECT_EQ(count("9"), 1);   // column 9 only
  EXPECT_EQ(count("10"), 0);  // there is no column 10
}

TEST(EditorSceneTest, ASceneIsAPureFunctionOfTheState) {
  EditorController c = blank();
  click(c, {2, 2});
  hover(c, {3, 3});
  EXPECT_TRUE(buildEditorScene(c).commands == buildEditorScene(c).commands);
}

// ---- the tool palette ---------------------------------------------------------------------------

TEST(EditorSceneTest, ThePaletteListsEveryToolWithItsNumberAndHighlightsTheSelectedOne) {
  EditorController c = blank();
  const std::vector<std::string> labels{"1 wall",    "2 shelf",    "3 pickup",
                                        "4 dropoff", "5 charging", "6 erase"};
  const std::vector<EditorCommand> commands{EditorCommand::ToolWall,     EditorCommand::ToolShelf,
                                            EditorCommand::ToolPickup,   EditorCommand::ToolDropoff,
                                            EditorCommand::ToolCharging, EditorCommand::ToolErase};
  for (std::size_t selected = 0; selected < labels.size(); ++selected) {
    send(c, commands[selected]);
    const DrawList list = buildEditorScene(c);
    for (const std::string& label : labels) {
      EXPECT_TRUE(findText(list, label) != nullptr) << label;
    }
    // One highlight, and it lies behind the label of the selected tool and no other.
    std::vector<ScreenRect> highlights;
    for (const auto& command : list.commands) {
      if (const auto* fill = std::get_if<FillRect>(&command)) {
        if (near(fill->rect.height, kGlyphSize + 4.0)) {
          highlights.push_back(fill->rect);
        }
      }
    }
    ASSERT_EQ(highlights.size(), 1U) << labels[selected];
    for (std::size_t i = 0; i < labels.size(); ++i) {
      const Text* label = findText(list, labels[i]);
      ASSERT_TRUE(label != nullptr);
      const bool behind =
          label->position.x >= highlights[0].x && label->position.x < highlights[0].right();
      EXPECT_EQ(behind, i == selected) << labels[i] << " with " << labels[selected] << " selected";
    }
  }
}

TEST(EditorSceneTest, EachToolShowsTheColourOfWhatItMakes) {
  EditorController c = blank();
  const DrawList list = buildEditorScene(c);
  for (const Tool tool : warehousesim::editor::kAllTools) {
    const Color colour = warehousesim::view::cellColor(warehousesim::editor::cellTypeOf(tool));
    const bool swatch =
        std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
          const auto* fill = std::get_if<FillRect>(&command);
          return fill != nullptr && fill->color == colour && near(fill->rect.width, kGlyphSize) &&
                 near(fill->rect.height, kGlyphSize);
        });
    EXPECT_TRUE(swatch) << warehousesim::editor::toString(tool);
  }
}

// ---- hovering and issues ------------------------------------------------------------------------

TEST(EditorSceneTest, TheCellUnderThePointerGetsAnOutlineAndAPreviewOfTheTool) {
  EditorController c = blank();
  const DrawList without = buildEditorScene(c);
  EXPECT_FALSE(hasOutline(without, c.camera().cellRect({3, 2})));

  send(c, EditorCommand::ToolShelf);
  hover(c, {3, 2});
  const DrawList list = buildEditorScene(c);
  const ScreenRect cell = c.camera().cellRect({3, 2});
  EXPECT_TRUE(hasOutline(list, cell));
  EXPECT_FALSE(hasOutline(list, c.camera().cellRect({4, 2})));
  const double margin = cell.width * 0.3;
  const FillRect preview{
      {cell.x + margin, cell.y + margin, cell.width - 2.0 * margin, cell.height - 2.0 * margin},
      warehousesim::view::cellColor(CellType::Shelf)};
  EXPECT_TRUE(std::any_of(list.commands.begin(), list.commands.end(), [&](const auto& command) {
    const auto* fill = std::get_if<FillRect>(&command);
    return fill != nullptr && near(fill->rect.x, preview.rect.x) &&
           near(fill->rect.width, preview.rect.width) && fill->color == preview.color;
  }));
}

TEST(EditorSceneTest, ThePointerOverTheBarShowsNoPreview) {
  EditorController c = blank();
  InputEvents input;
  input.pointer = ScreenPoint{400.0, 590.0};
  c.apply(input, 0.016);
  const DrawList list = buildEditorScene(c);
  for (int y = 0; y < 6; ++y) {
    for (int x = 0; x < 10; ++x) {
      EXPECT_FALSE(hasOutline(list, c.camera().cellRect({x, y})));
    }
  }
}

TEST(EditorSceneTest, AnIssueWithAPlaceMarksThatCellAndAReadyMapHasNoMarker) {
  EditorController c = make(MapEditor(*Warehouse::fromRows({"P.#.D"})));
  ASSERT_EQ(c.issues().size(), 1U);
  ASSERT_TRUE(c.issues()[0].where.has_value());
  const Point where = *c.issues()[0].where;
  EXPECT_EQ(where, (Point{3, 0}));
  const DrawList list = buildEditorScene(c);
  EXPECT_TRUE(hasOutline(list, c.camera().cellRect(where)));
  EXPECT_FALSE(hasOutline(list, c.camera().cellRect({0, 0})));

  EditorController ready = make(MapEditor(*Warehouse::fromRows({"P..D"})));
  const DrawList clean = buildEditorScene(ready);
  for (int x = 0; x < 4; ++x) {
    EXPECT_FALSE(hasOutline(clean, ready.camera().cellRect({x, 0})));
  }
}

// ---- the text -----------------------------------------------------------------------------------

TEST(EditorSceneTest, TheFirstLineNamesTheFileTheSizeTheToolAndTheCell) {
  EditorController c = blank();
  hover(c, {3, 2});
  const auto lines = editorStatusLines(c);
  EXPECT_TRUE(lines[0].find("hall.json") == 0) << lines[0];  // the file name, not the folder
  EXPECT_TRUE(lines[0].find("maps/") == std::string::npos);
  EXPECT_TRUE(lines[0].find("10 x 6") != std::string::npos);
  EXPECT_TRUE(lines[0].find("tool wall") != std::string::npos);
  EXPECT_TRUE(lines[0].find("cell (3,2) free") != std::string::npos) << lines[0];
  EXPECT_TRUE(lines[0].find('*') == std::string::npos);  // nothing unsaved
}

TEST(EditorSceneTest, UnsavedChangesAreMarkedAndTheCellFollowsThePointer) {
  EditorController c = blank();
  click(c, {3, 2});
  hover(c, {3, 2});
  const auto lines = editorStatusLines(c);
  EXPECT_TRUE(lines[0].find("hall.json *") == 0) << lines[0];
  EXPECT_TRUE(lines[0].find("cell (3,2) wall") != std::string::npos) << lines[0];
  InputEvents away;
  away.pointerLeft = true;
  c.apply(away, 0.016);
  EXPECT_TRUE(editorStatusLines(c)[0].find("cell -") != std::string::npos);
}

TEST(EditorSceneTest, WithNoMessageTheSecondLineSaysWhetherTheMapIsReady) {
  EditorController c = blank();
  EXPECT_TRUE(editorStatusLines(c)[1].find("not ready: there is no dropoff") == 0)
      << editorStatusLines(c)[1];
  EditorController ready = make(MapEditor(*Warehouse::fromRows({"P..D"})));
  EXPECT_EQ(editorStatusLines(ready)[1], "ready for a simulation");
}

TEST(EditorSceneTest, AMessageTakesTheSecondLineAndAnErrorIsDrawnInAnotherColour) {
  EditorController c = make(MapEditor::blank(1, 1));
  send(c, EditorCommand::Narrower);  // refused
  ASSERT_TRUE(c.messageIsError());
  EXPECT_EQ(editorStatusLines(c)[1], c.message());
  const DrawList error = buildEditorScene(c);
  const Text* shown = findText(error, "from 1 to 512");
  ASSERT_TRUE(shown != nullptr);

  send(c, EditorCommand::Wider);  // fine
  ASSERT_FALSE(c.messageIsError());
  const DrawList fine = buildEditorScene(c);
  const Text* ok = findText(fine, "the map is now 2 x 1");
  ASSERT_TRUE(ok != nullptr);
  EXPECT_FALSE(shown->color == ok->color);
}

TEST(EditorSceneTest, TheHelpLinesMentionEveryWayOfEditing) {
  const EditorController c = blank();
  const auto lines = editorStatusLines(c);
  const std::string help = lines[2] + " " + lines[3];
  for (const char* part : {"paint", "erase", "pan", "zoom", "1-6 tool", "Shift+arrows resize",
                           "undo", "redo", "save", "load", "predefined layouts", "clear", "quit"}) {
    EXPECT_TRUE(help.find(part) != std::string::npos) << part;
  }
}

// ---- odd windows --------------------------------------------------------------------------------

TEST(EditorSceneTest, AWindowSmallerThanTheBarDrawsOnlyTheBarAndNothingBreaks) {
  EditorController c = blank();
  click(c, {0, 0});  // a wall, so that drawing the map would show
  for (const ScreenSize size : {ScreenSize{10, 10}, ScreenSize{0, 0}, ScreenSize{800, 40}}) {
    c.resize(size);
    const DrawList list = buildEditorScene(c);
    EXPECT_FALSE(list.commands.empty());
    // Not the little swatch of the wall tool in the palette.
    const bool anyCell =
        std::any_of(list.commands.begin(), list.commands.end(), [](const auto& command) {
          const auto* fill = std::get_if<FillRect>(&command);
          return fill != nullptr && fill->color == warehousesim::view::cellColor(CellType::Wall) &&
                 fill->rect.width > kGlyphSize + 1.0;
        });
    EXPECT_FALSE(anyCell);
  }
}

TEST(EditorSceneTest, ABigMapZoomedInEmitsOnlyTheCellsInView) {
  EditorController c =
      make(MapEditor::blank(warehousesim::world::kMaxMapSide, warehousesim::world::kMaxMapSide));
  InputEvents zoom;
  zoom.zoomSteps = 40.0;
  c.apply(zoom, 0.016);
  const DrawList list = buildEditorScene(c);
  EXPECT_LT(list.commands.size(), 400U);
}

}  // namespace
