#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "scratch_dir.hpp"
#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

namespace fs = std::filesystem;
using warehousesim::mapio::loadMapFile;
using warehousesim::mapio::MapDocument;
using warehousesim::mapio::saveMapFile;
using warehousesim::mapio::serializeMap;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;
using warehousesim_test::readFile;
using warehousesim_test::ScratchDir;
using warehousesim_test::writeFile;

MapDocument sample() {
  Warehouse w(6, 3);
  w.set({0, 0}, CellType::Wall);
  w.set({1, 1}, CellType::Shelf);
  w.set({2, 2}, CellType::Pickup);
  w.set({3, 2}, CellType::Dropoff);
  w.set({4, 2}, CellType::Charging);
  return {"sample", w};
}

std::size_t entriesIn(const fs::path& directory) {
  std::size_t count = 0;
  for (const auto& entry : fs::directory_iterator(directory)) {
    (void)entry;
    ++count;
  }
  return count;
}

TEST(MapFileIoTest, ASavedMapIsLoadedBackUnchanged) {
  const ScratchDir dir("map_io_round_trip");
  const MapDocument original = sample();
  ASSERT_FALSE(saveMapFile(dir.file("a.json"), original).has_value());
  const auto load = loadMapFile(dir.file("a.json"));
  ASSERT_TRUE(load.ok()) << load.describe();
  EXPECT_EQ(load.document->name, original.name);
  EXPECT_EQ(load.document->layout.toRows(), original.layout.toRows());
}

TEST(MapFileIoTest, EveryPredefinedLayoutSurvivesAFileRoundTrip) {
  const ScratchDir dir("map_io_predefined");
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    const fs::path path = dir.file(std::string(info.name) + ".json");
    ASSERT_FALSE(saveMapFile(path, {std::string(info.name), info.make()}).has_value());
    const auto load = loadMapFile(path);
    ASSERT_TRUE(load.ok()) << load.describe();
    EXPECT_EQ(load.document->layout.toRows(), info.make().toRows()) << info.name;
  }
}

TEST(MapFileIoTest, TheFileHoldsExactlyTheSerializedText) {
  const ScratchDir dir("map_io_text");
  ASSERT_FALSE(saveMapFile(dir.file("a.json"), sample()).has_value());
  EXPECT_EQ(readFile(dir.file("a.json")), serializeMap(sample()));
}

TEST(MapFileIoTest, SavingReplacesAnExistingMapAndLeavesNoTemporaryFileBehind) {
  const ScratchDir dir("map_io_replace");
  writeFile(dir.file("a.json"), "old contents");
  ASSERT_FALSE(saveMapFile(dir.file("a.json"), sample()).has_value());
  EXPECT_EQ(readFile(dir.file("a.json")), serializeMap(sample()));
  EXPECT_EQ(entriesIn(dir.path()), 1U);
}

TEST(MapFileIoTest, AMapThatCannotBeWrittenLeavesTheExistingFileUntouched) {
  const ScratchDir dir("map_io_invalid_save");
  ASSERT_FALSE(saveMapFile(dir.file("a.json"), sample()).has_value());
  const std::string before = readFile(dir.file("a.json"));

  const auto tooLong = saveMapFile(dir.file("a.json"), {std::string(200, 'n'), sample().layout});
  ASSERT_TRUE(tooLong.has_value());
  EXPECT_TRUE(tooLong->message.find("cannot be written") != std::string::npos) << tooLong->text();
  const auto tooWide = saveMapFile(dir.file("a.json"), {"", Warehouse(600, 3)});
  ASSERT_TRUE(tooWide.has_value());
  EXPECT_TRUE(tooWide->message.find("/width") != std::string::npos) << tooWide->text();

  EXPECT_EQ(readFile(dir.file("a.json")), before);
  EXPECT_EQ(entriesIn(dir.path()), 1U);
}

TEST(MapFileIoTest, SavingIntoAMissingDirectoryFailsAndCreatesNothing) {
  const ScratchDir dir("map_io_missing_directory");
  const auto problem = saveMapFile(dir.file("no/such/dir/a.json"), sample());
  ASSERT_TRUE(problem.has_value());
  EXPECT_TRUE(problem->message.find("cannot create") != std::string::npos) << problem->text();
  EXPECT_EQ(entriesIn(dir.path()), 0U);
}

TEST(MapFileIoTest, ARenameThatFailsRemovesTheTemporaryFileAndKeepsWhatWasThere) {
  const ScratchDir dir("map_io_rename_fails");
  fs::create_directory(dir.file("a.json"));  // a directory is in the way of the map
  writeFile(dir.file("a.json") / "keep.txt", "keep");
  const auto problem = saveMapFile(dir.file("a.json"), sample());
  ASSERT_TRUE(problem.has_value());
  EXPECT_TRUE(problem->message.find("cannot replace") != std::string::npos) << problem->text();
  EXPECT_EQ(entriesIn(dir.path()), 1U);  // no a.json.tmp left over
  EXPECT_EQ(readFile(dir.file("a.json") / "keep.txt"), "keep");
}

TEST(MapFileIoTest, AMissingFileIsAProblemNotAnException) {
  const ScratchDir dir("map_io_missing");
  const auto load = loadMapFile(dir.file("nothing.json"));
  EXPECT_FALSE(load.ok());
  ASSERT_EQ(load.problems.size(), 1U);
  EXPECT_TRUE(load.problems[0].message.find("there is no such file") != std::string::npos)
      << load.describe();
}

TEST(MapFileIoTest, ADirectoryIsNotAMap) {
  const ScratchDir dir("map_io_directory");
  const auto load = loadMapFile(dir.path());
  EXPECT_FALSE(load.ok());
  EXPECT_TRUE(load.describe().find("is not a regular file") != std::string::npos)
      << load.describe();
}

TEST(MapFileIoTest, AnEmptyFileAndAGarbageFileAreMalformed) {
  const ScratchDir dir("map_io_garbage");
  writeFile(dir.file("empty.json"), "");
  writeFile(dir.file("garbage.json"), "this is not a map");
  writeFile(dir.file("binary.json"), std::string("\x00\x01\x02\xFF\xFE", 5));
  for (const char* name : {"empty.json", "garbage.json", "binary.json"}) {
    const auto load = loadMapFile(dir.file(name));
    EXPECT_FALSE(load.ok()) << name;
    EXPECT_TRUE(load.describe().find("not valid JSON") != std::string::npos) << name;
  }
}

TEST(MapFileIoTest, AFileOverTheSizeLimitIsRefusedBeforeItIsRead) {
  const ScratchDir dir("map_io_too_big");
  writeFile(dir.file("big.json"), std::string(warehousesim::mapio::kMaxFileBytes + 1, ' '));
  const auto load = loadMapFile(dir.file("big.json"));
  EXPECT_FALSE(load.ok());
  EXPECT_TRUE(load.describe().find("the limit is") != std::string::npos) << load.describe();
}

TEST(MapFileIoTest, AFileOfExactlyTheSizeLimitIsStillRead) {
  const ScratchDir dir("map_io_exact_limit");
  std::string text = serializeMap(sample());
  text += std::string(warehousesim::mapio::kMaxFileBytes - text.size(), ' ');
  writeFile(dir.file("limit.json"), text);
  EXPECT_TRUE(loadMapFile(dir.file("limit.json")).ok());
}

TEST(MapFileIoTest, AFileWithAnInvalidMapReportsItsProblemsLikeParsedText) {
  const ScratchDir dir("map_io_invalid_file");
  writeFile(dir.file("bad.json"),
            R"({"format": "warehousesim-map", "version": 1, "width": 3, "height": 3,
                "walls": [[7, 7]]})");
  const auto load = loadMapFile(dir.file("bad.json"));
  EXPECT_FALSE(load.ok());
  EXPECT_EQ(load.describe(), "/walls/0: cell (7, 7) is outside the 3 x 3 grid");
}

TEST(MapFileIoTest, AFileEditedByHandAndSavedWithWindowsLineEndingsIsStillAMap) {
  const ScratchDir dir("map_io_crlf");
  std::string text = serializeMap(sample());
  std::string crlf;
  for (const char c : text) {
    crlf += c == '\n' ? std::string("\r\n") : std::string(1, c);
  }
  writeFile(dir.file("crlf.json"), crlf);
  const auto load = loadMapFile(dir.file("crlf.json"));
  ASSERT_TRUE(load.ok()) << load.describe();
  EXPECT_EQ(load.document->layout.toRows(), sample().layout.toRows());
}

}  // namespace
