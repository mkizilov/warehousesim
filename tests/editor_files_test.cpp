#include "warehousesim/editor/editor_files.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include "scratch_dir.hpp"
#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using warehousesim::editor::loadMap;
using warehousesim::editor::loadPredefined;
using warehousesim::editor::MapEditor;
using warehousesim::editor::readiness;
using warehousesim::editor::saveMap;
using warehousesim::editor::Tool;
using warehousesim::world::CellType;
using warehousesim_test::readFile;
using warehousesim_test::ScratchDir;
using warehousesim_test::writeFile;

using Rows = std::vector<std::string>;

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// ---- saving -------------------------------------------------------------------------------------

TEST(EditorFilesTest, SavingWritesTheMapMarksItSavedAndSaysWhetherItIsReady) {
  const ScratchDir dir("editor_save");
  MapEditor editor = MapEditor::blank(4, 1, "mine");
  editor.paint({0, 0}, Tool::Pickup);
  editor.paint({3, 0}, Tool::Dropoff);
  ASSERT_TRUE(editor.dirty());
  const auto outcome = saveMap(editor, dir.file("mine.json"));
  EXPECT_TRUE(outcome.ok) << outcome.message;
  EXPECT_FALSE(editor.dirty());
  EXPECT_TRUE(contains(outcome.message, "saved")) << outcome.message;
  EXPECT_TRUE(contains(outcome.message, "(4 x 1)"));
  EXPECT_TRUE(contains(outcome.message, "ready for a simulation"));
  EXPECT_EQ(readFile(dir.file("mine.json")),
            warehousesim::mapio::serializeMap({"mine", editor.layout()}));
}

TEST(EditorFilesTest, AMapThatIsNotReadyForASimulationCanStillBeSavedAsWorkInProgress) {
  const ScratchDir dir("editor_save_unfinished");
  MapEditor editor = MapEditor::blank(4, 4);
  editor.paint({1, 1}, Tool::Wall);
  const auto outcome = saveMap(editor, dir.file("wip.json"));
  EXPECT_TRUE(outcome.ok);
  EXPECT_TRUE(contains(outcome.message, "not ready: there is no dropoff")) << outcome.message;
  EXPECT_FALSE(editor.dirty());
}

TEST(EditorFilesTest, AFailedSaveSaysSoAndLeavesTheEditorDirty) {
  const ScratchDir dir("editor_save_fails");
  MapEditor editor = MapEditor::blank(3, 1, "x");
  editor.paint({0, 0});
  const auto outcome = saveMap(editor, dir.file("missing/dir/a.json"));
  EXPECT_FALSE(outcome.ok);
  EXPECT_TRUE(contains(outcome.message, "not saved: ")) << outcome.message;
  EXPECT_TRUE(editor.dirty());
  EXPECT_FALSE(std::filesystem::exists(dir.file("missing")));
}

TEST(EditorFilesTest, AMapWhoseNameCannotBeWrittenIsNotSaved) {
  const ScratchDir dir("editor_save_bad_name");
  MapEditor editor = MapEditor::blank(3, 1, std::string(200, 'n'));
  editor.paint({0, 0});
  const auto outcome = saveMap(editor, dir.file("a.json"));
  EXPECT_FALSE(outcome.ok);
  EXPECT_TRUE(contains(outcome.message, "/name")) << outcome.message;
  EXPECT_FALSE(std::filesystem::exists(dir.file("a.json")));
  EXPECT_TRUE(editor.dirty());
}

// ---- loading a file -----------------------------------------------------------------------------

TEST(EditorFilesTest, LoadingReplacesTheMapCleanAndWithoutHistory) {
  const ScratchDir dir("editor_load");
  MapEditor saved = MapEditor::blank(5, 2, "stored");
  saved.paint({4, 1}, Tool::Charging);
  ASSERT_TRUE(saveMap(saved, dir.file("stored.json")).ok);

  MapEditor editor = MapEditor::blank(2, 2, "other");
  editor.paint({0, 0});
  const auto outcome = loadMap(editor, dir.file("stored.json"));
  EXPECT_TRUE(outcome.ok) << outcome.message;
  EXPECT_TRUE(contains(outcome.message, "(5 x 2)"));
  EXPECT_EQ(editor.layout().toRows(), saved.layout().toRows());
  EXPECT_EQ(editor.name(), "stored");
  EXPECT_FALSE(editor.dirty());
  EXPECT_FALSE(editor.canUndo());
}

TEST(EditorFilesTest, AMapWithNoNameInTheFileIsNamedAfterTheFile) {
  const ScratchDir dir("editor_load_unnamed");
  writeFile(dir.file("hall-3.json"),
            R"({"format": "warehousesim-map", "version": 1, "width": 2, "height": 1})");
  MapEditor editor = MapEditor::blank(1, 1);
  ASSERT_TRUE(loadMap(editor, dir.file("hall-3.json")).ok);
  EXPECT_EQ(editor.name(), "hall-3");
}

TEST(EditorFilesTest, ALongFileNameIsShortenedToWhatTheFormatAllows) {
  const ScratchDir dir("editor_load_long_name");
  const std::string stem(120, 'f');
  writeFile(dir.file(stem + ".json"),
            R"({"format": "warehousesim-map", "version": 1, "width": 2, "height": 1})");
  MapEditor editor = MapEditor::blank(1, 1);
  ASSERT_TRUE(loadMap(editor, dir.file(stem + ".json")).ok);
  EXPECT_EQ(editor.name().size(), warehousesim::mapio::kMaxNameBytes);
  EXPECT_TRUE(saveMap(editor, dir.file("again.json")).ok);  // and it can be saved again
}

TEST(EditorFilesTest, AFileThatCannotBeLoadedLeavesTheEditorExactlyAsItWas) {
  const ScratchDir dir("editor_load_fails");
  writeFile(dir.file("bad.json"),
            R"({"format": "warehousesim-map", "version": 1, "width": 3, "height": 3,
                "walls": [[0, 0], [9, 9]], "shelves": [[0, 0]]})");
  writeFile(dir.file("junk.json"), "}{");
  writeFile(dir.file("future.json"), R"({"format": "warehousesim-map", "version": 9})");

  MapEditor editor = MapEditor::blank(3, 1, "mine");
  editor.paint({0, 0});
  editor.paint({1, 0});
  ASSERT_TRUE(editor.undo());
  for (const char* name : {"bad.json", "junk.json", "future.json", "missing.json"}) {
    const auto outcome = loadMap(editor, dir.file(name));
    EXPECT_FALSE(outcome.ok) << name;
    EXPECT_TRUE(contains(outcome.message, "not loaded: ")) << outcome.message;
    EXPECT_EQ(editor.layout().toRows(), (Rows{"#.."})) << name;
    EXPECT_EQ(editor.name(), "mine");
    EXPECT_TRUE(editor.dirty());
    EXPECT_TRUE(editor.canUndo());  // history is intact
    EXPECT_TRUE(editor.canRedo());
  }
}

TEST(EditorFilesTest, AFailedLoadNamesTheFirstProblemAndHowManyMoreThereAre) {
  const ScratchDir dir("editor_load_problems");
  writeFile(dir.file("bad.json"),
            R"({"format": "warehousesim-map", "version": 1, "width": 3, "height": 3,
                "walls": [[0, 0], [9, 9]], "shelves": [[0, 0]]})");
  MapEditor editor = MapEditor::blank(3, 1);
  const auto outcome = loadMap(editor, dir.file("bad.json"));
  EXPECT_TRUE(contains(outcome.message, "/walls/1: cell (9, 9) is outside the 3 x 3 grid"))
      << outcome.message;
  EXPECT_TRUE(contains(outcome.message, "(and 1 more)")) << outcome.message;
}

// ---- predefined scenarios -----------------------------------------------------------------------

TEST(EditorFilesTest, EveryPredefinedLayoutCanBeLoadedByName) {
  MapEditor editor = MapEditor::blank(2, 2);
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    const auto outcome = loadPredefined(editor, info.name);
    EXPECT_TRUE(outcome.ok) << outcome.message;
    EXPECT_EQ(editor.layout().toRows(), info.make().toRows()) << info.name;
    EXPECT_EQ(editor.name(), info.name);
    EXPECT_FALSE(editor.dirty());
    EXPECT_FALSE(editor.canUndo());
    EXPECT_EQ(readiness(editor), "ready for a simulation") << info.name;
  }
}

TEST(EditorFilesTest, AnUnknownPredefinedNameListsTheKnownOnesAndChangesNothing) {
  MapEditor editor = MapEditor::blank(3, 1, "mine");
  editor.paint({0, 0});
  for (const char* name : {"", "Demo", "nowhere", "demo "}) {
    const auto outcome = loadPredefined(editor, name);
    EXPECT_FALSE(outcome.ok) << name;
    EXPECT_TRUE(contains(outcome.message, "there are: demo, compact, fulfilment"))
        << outcome.message;
  }
  EXPECT_EQ(editor.layout().toRows(), (Rows{"#.."}));
  EXPECT_EQ(editor.name(), "mine");
  EXPECT_TRUE(editor.dirty());
}

TEST(EditorFilesTest, ACustomisedPredefinedLayoutCanBeSavedAndLoadedBack) {
  const ScratchDir dir("editor_customise");
  MapEditor editor = MapEditor::blank(1, 1);
  ASSERT_TRUE(loadPredefined(editor, "compact").ok);
  const auto original = editor.layout().toRows();

  editor.paint({3, 3}, Tool::Wall);
  editor.paint({4, 3}, Tool::Erase);
  ASSERT_EQ(editor.resize(14, 9).status, warehousesim::editor::ResizeStatus::Applied);
  editor.paint({13, 8}, Tool::Charging);
  const auto customised = editor.layout().toRows();
  ASSERT_NE(customised, original);
  ASSERT_TRUE(saveMap(editor, dir.file("custom.json")).ok);

  MapEditor reopened = MapEditor::blank(1, 1);
  ASSERT_TRUE(loadMap(reopened, dir.file("custom.json")).ok);
  EXPECT_EQ(reopened.layout().toRows(), customised);
  EXPECT_EQ(reopened.name(), "compact");
}

// ---- readiness text -----------------------------------------------------------------------------

TEST(EditorFilesTest, ReadinessSaysReadyNotReadyOrReadyBut) {
  MapEditor editor = MapEditor::blank(4, 1);
  EXPECT_TRUE(contains(readiness(editor), "not ready: there is no dropoff"));
  editor.paint({0, 0}, Tool::Pickup);
  editor.paint({3, 0}, Tool::Dropoff);
  EXPECT_EQ(readiness(editor), "ready for a simulation");
  editor.paint({1, 0}, Tool::Wall);
  EXPECT_TRUE(contains(readiness(editor), "ready, but the walkable cells form 2 separate regions"))
      << readiness(editor);
}

}  // namespace
