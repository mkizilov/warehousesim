#include "warehousesim/mapio/map_file.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::mapio::MapDocument;
using warehousesim::mapio::MapLoad;
using warehousesim::mapio::MapProblem;
using warehousesim::mapio::parseMap;
using warehousesim::mapio::serializeMap;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

Warehouse layout(const std::vector<std::string>& rows) {
  auto parsed = Warehouse::fromRows(rows);
  if (!parsed) {
    throw std::logic_error("bad test layout");
  }
  return *parsed;
}

// A valid document with the given body after the header (the body starts with a comma or is empty).
std::string doc(const std::string& body) {
  return R"({"format": "warehousesim-map", "version": 1)" + body + "}";
}

std::string sized(int width, int height, const std::string& more = "") {
  return doc(", \"width\": " + std::to_string(width) + ", \"height\": " + std::to_string(height) +
             more);
}

MapLoad parseOk(const std::string& text) {
  MapLoad load = parseMap(text);
  EXPECT_TRUE(load.ok()) << load.describe();
  return load;
}

// True if some problem is at `location` and its message contains `fragment`.
bool hasProblem(const MapLoad& load, const std::string& location, const std::string& fragment) {
  for (const MapProblem& problem : load.problems) {
    if (problem.location == location && problem.message.find(fragment) != std::string::npos) {
      return true;
    }
  }
  return false;
}

// Rejected, and with this one problem at this place.
void expectRejected(const std::string& text, const std::string& location,
                    const std::string& fragment) {
  const MapLoad load = parseMap(text);
  EXPECT_FALSE(load.ok()) << text;
  EXPECT_FALSE(load.problems.empty());
  EXPECT_TRUE(hasProblem(load, location, fragment))
      << "expected \"" << fragment << "\" at \"" << location << "\" in:\n"
      << load.describe();
}

void expectSameDocument(const MapDocument& a, const MapDocument& b) {
  EXPECT_EQ(a.name, b.name);
  EXPECT_EQ(a.layout.toRows(), b.layout.toRows());
}

// ---- round trips --------------------------------------------------------------------------------

TEST(MapFileRoundTripTest, EveryPredefinedLayoutComesBackUnchanged) {
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    const MapDocument original{std::string(info.name), info.make()};
    const MapLoad load = parseOk(serializeMap(original));
    ASSERT_TRUE(load.ok());
    expectSameDocument(*load.document, original);
  }
}

TEST(MapFileRoundTripTest, TheTextIsCanonicalSoSavingALoadedMapChangesNothing) {
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    const std::string first = serializeMap({std::string(info.name), info.make()});
    const MapLoad load = parseOk(first);
    ASSERT_TRUE(load.ok());
    EXPECT_EQ(serializeMap(*load.document), first) << info.name;
  }
}

TEST(MapFileRoundTripTest, RandomMapsOfRandomSizesComeBackUnchanged) {
  warehousesim::core::Rng rng(7);
  for (int round = 0; round < 60; ++round) {
    const auto width = static_cast<std::int32_t>(1 + rng.below(40));
    const auto height = static_cast<std::int32_t>(1 + rng.below(40));
    Warehouse w(width, height);
    for (std::int32_t y = 0; y < height; ++y) {
      for (std::int32_t x = 0; x < width; ++x) {
        w.set({x, y}, warehousesim::world::kAllCellTypes[rng.below(6)]);
      }
    }
    const MapDocument original{"random " + std::to_string(round), w};
    const MapLoad load = parseOk(serializeMap(original));
    ASSERT_TRUE(load.ok());
    expectSameDocument(*load.document, original);
  }
}

TEST(MapFileRoundTripTest, TheSmallestAndTheLargestMapsRoundTrip) {
  for (const std::int32_t side : {1, warehousesim::world::kMaxMapSide}) {
    Warehouse w(side, side);
    w.set({0, 0}, CellType::Pickup);
    w.set({side - 1, side - 1}, CellType::Dropoff);
    const MapLoad load = parseOk(serializeMap({"", w}));
    ASSERT_TRUE(load.ok());
    expectSameDocument(*load.document, MapDocument{"", w});
  }
}

TEST(MapFileRoundTripTest, AMapWithEveryCellFilledStaysWithinTheSizeLimit) {
  Warehouse w(warehousesim::world::kMaxMapSide, warehousesim::world::kMaxMapSide);
  for (std::int32_t y = 0; y < w.height(); ++y) {
    for (std::int32_t x = 0; x < w.width(); ++x) {
      w.set({x, y}, CellType::Wall);
    }
  }
  const std::string text = serializeMap({"full", w});
  EXPECT_LT(text.size(), warehousesim::mapio::kMaxFileBytes);
  const MapLoad load = parseOk(text);
  ASSERT_TRUE(load.ok());
  EXPECT_EQ(load.document->layout.cellsOfType(CellType::Wall).size(), w.size().cellCount());
}

TEST(MapFileRoundTripTest, NamesWithQuotesBackslashesAndNonAsciiTextSurvive) {
  for (const std::string& name :
       {std::string("quote \" and backslash \\"), std::string("Lagerhalle \xC3\xA4\xE2\x82\xAC"),
        std::string("slash / and ~ tilde"), std::string(warehousesim::mapio::kMaxNameBytes, 'n'),
        std::string()}) {
    const MapLoad load = parseOk(serializeMap({name, layout({"P.D"})}));
    ASSERT_TRUE(load.ok());
    EXPECT_EQ(load.document->name, name);
  }
}

TEST(MapFileFormatTest, TheTextIsPinnedExactlyForASmallMap) {
  const std::string expected =
      "{\n"
      "  \"format\": \"warehousesim-map\",\n"
      "  \"version\": 1,\n"
      "  \"name\": \"tiny\",\n"
      "  \"width\": 3,\n"
      "  \"height\": 2,\n"
      "  \"walls\": [\n"
      "    [0, 0]\n"
      "  ],\n"
      "  \"shelves\": [\n"
      "    [0, 1]\n"
      "  ],\n"
      "  \"pickups\": [\n"
      "    [2, 0]\n"
      "  ],\n"
      "  \"dropoffs\": [\n"
      "    [1, 1]\n"
      "  ],\n"
      "  \"chargers\": [\n"
      "    [2, 1]\n"
      "  ]\n"
      "}\n";
  EXPECT_EQ(serializeMap({"tiny", layout({"#.P", "SDC"})}), expected);
}

TEST(MapFileFormatTest, EmptyListsAreWrittenAsEmptyAndCellsAreListedRowByRow) {
  const std::string text = serializeMap({"", layout({"..", "#.", ".#"})});
  EXPECT_TRUE(text.find("\"walls\": [\n    [0, 1], [1, 2]\n  ],") != std::string::npos) << text;
  EXPECT_TRUE(text.find("\"shelves\": [],") != std::string::npos);
  EXPECT_TRUE(text.find("\"chargers\": []\n}") != std::string::npos);
}

TEST(MapFileFormatTest, LongListsAreWrappedAtOneHundredColumns) {
  Warehouse w(60, 4);
  for (std::int32_t x = 0; x < 60; ++x) {
    w.set({x, 1}, CellType::Wall);
  }
  const std::string text = serializeMap({"", w});
  std::size_t lines = 0;
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t end = text.find('\n', start);
    EXPECT_LE(end - start, 100U);
    ++lines;
    start = end + 1;
  }
  EXPECT_GT(lines, 12U);  // the wall list needed more than one line
}

TEST(MapFileFormatTest, AnyOrderOfKeysAndAnyWhitespaceIsAccepted) {
  const MapLoad load = parseOk(
      "{\"chargers\":[[0,0]],\"height\":1,\"width\":3,\n\t\"dropoffs\": [[2, 0]],"
      "\"version\":1,\"name\":\"x\",\"format\":\"warehousesim-map\"}");
  ASSERT_TRUE(load.ok());
  EXPECT_EQ(load.document->layout.toRows(), (std::vector<std::string>{"C.D"}));
  EXPECT_EQ(load.document->name, "x");
}

TEST(MapFileFormatTest, TheNameAndAllTheListsAreOptional) {
  const MapLoad load = parseOk(sized(4, 2));
  ASSERT_TRUE(load.ok());
  EXPECT_EQ(load.document->name, "");
  EXPECT_EQ(load.document->layout.toRows(), (std::vector<std::string>{"....", "...."}));
}

// ---- malformed input ----------------------------------------------------------------------------

TEST(MapFileMalformedTest, TextThatIsNotJsonIsRejectedWithoutThrowing) {
  for (const std::string text : {"", "   ", "{", "}", "not json", "{\"format\": }", "[1, 2",
                                 "{\"a\": 1,}", "{'format': 'x'}", "\x01\x02\x03", "nul"}) {
    const MapLoad load = parseMap(text);
    EXPECT_FALSE(load.ok()) << "[" << text << "]";
    ASSERT_EQ(load.problems.size(), 1U);
    EXPECT_TRUE(load.problems[0].message.find("not valid JSON") != std::string::npos)
        << load.describe();
  }
}

TEST(MapFileMalformedTest, TrailingGarbageAfterTheDocumentIsRejected) {
  expectRejected(sized(2, 2) + " extra", "", "not valid JSON");
  expectRejected(sized(2, 2) + sized(2, 2), "", "not valid JSON");
}

TEST(MapFileMalformedTest, AMapMustBeAnObject) {
  expectRejected("[]", "", "must be a JSON object, not an array");
  expectRejected("42", "", "must be a JSON object, not a number");
  expectRejected("\"map\"", "", "must be a JSON object, not a string");
  expectRejected("null", "", "must be a JSON object, not null");
  expectRejected("true", "", "must be a JSON object, not a boolean");
}

TEST(MapFileMalformedTest, InvalidUtf8InTheTextIsRejected) {
  expectRejected(doc(", \"name\": \"bad \xFF byte\", \"width\": 1, \"height\": 1"), "",
                 "not valid JSON");
}

TEST(MapFileMalformedTest, TheFormatMarkerIsRequiredAndExact) {
  expectRejected(R"({"version": 1, "width": 1, "height": 1})", "/format", "required");
  expectRejected(R"({"format": "other", "version": 1, "width": 1, "height": 1})", "/format",
                 "must be \"warehousesim-map\", not \"other\"");
  expectRejected(R"({"format": 7, "version": 1, "width": 1, "height": 1})", "/format",
                 "not a number");
  expectRejected(R"({"format": "Warehousesim-Map", "version": 1, "width": 1, "height": 1})",
                 "/format", "must be");
}

TEST(MapFileMalformedTest, AWrongFormatMarkerReportsNothingElseAboutTheDocument) {
  const MapLoad load = parseMap(R"({"format": "other", "version": 1, "width": 0})");
  ASSERT_EQ(load.problems.size(), 1U);
  EXPECT_EQ(load.problems[0].location, "/format");
}

TEST(MapFileMalformedTest, AMissingFormatMarkerReportsNothingElseAboutTheDocumentEither) {
  const MapLoad load = parseMap(R"({"version": 1, "width": 0, "walls": 5})");
  EXPECT_FALSE(load.ok());
  ASSERT_EQ(load.problems.size(), 1U) << load.describe();
  EXPECT_EQ(load.problems[0].location, "/format");
}

TEST(MapFileMalformedTest, ADuplicateKeyIsReportedTogetherWithTheOtherProblemsOfTheDocument) {
  const MapLoad load = parseMap(
      R"({"format": "warehousesim-map", "version": 1, "width": 0, "width": 0, "height": 3})");
  EXPECT_FALSE(load.ok());
  EXPECT_TRUE(hasProblem(load, "/width", "duplicate key")) << load.describe();
  EXPECT_TRUE(hasProblem(load, "/width", "must be from 1 to 512")) << load.describe();
}

TEST(MapFileMalformedTest, TheVersionIsRequiredAnIntegerAndOneOfThoseKnown) {
  expectRejected(R"({"format": "warehousesim-map", "width": 1, "height": 1})", "/version",
                 "required");
  expectRejected(R"({"format": "warehousesim-map", "version": "1", "width": 1, "height": 1})",
                 "/version", "must be an integer, not a string");
  expectRejected(R"({"format": "warehousesim-map", "version": 1.0, "width": 1, "height": 1})",
                 "/version", "must be an integer");
  expectRejected(R"({"format": "warehousesim-map", "version": null, "width": 1, "height": 1})",
                 "/version", "not null");
  expectRejected(R"({"format": "warehousesim-map", "version": 0, "width": 1, "height": 1})",
                 "/version", "unsupported version 0");
  expectRejected(R"({"format": "warehousesim-map", "version": -1, "width": 1, "height": 1})",
                 "/version", "unsupported version -1");
}

TEST(MapFileMalformedTest, ANewerVersionIsNamedAsSuchAndItsContentIsNotJudged) {
  const MapLoad load = parseMap(
      R"({"format": "warehousesim-map", "version": 2, "width": "wide", "layers": [1, 2]})");
  EXPECT_FALSE(load.ok());
  ASSERT_EQ(load.problems.size(), 1U);
  EXPECT_EQ(load.problems[0].location, "/version");
  EXPECT_TRUE(load.problems[0].message.find(
                  "unsupported version 2; this program reads version 1") != std::string::npos)
      << load.describe();
}

TEST(MapFileMalformedTest, AVersionBeyondTheIntegerRangeIsUnsupportedNotAnOverflow) {
  expectRejected(R"({"format": "warehousesim-map", "version": 99999999999999999999999})",
                 "/version", "must be an integer");
  expectRejected(
      R"({"format": "warehousesim-map", "version": 18446744073709551615, "width": 1, "height": 1})",
      "/version", "unsupported version");
}

TEST(MapFileMalformedTest, DimensionsAreRequired) {
  expectRejected(doc(", \"height\": 3"), "/width", "required");
  expectRejected(doc(", \"width\": 3"), "/height", "required");
  expectRejected(doc(""), "/width", "required");
}

TEST(MapFileMalformedTest, DimensionsOutsideOneToTheLimitAreRejected) {
  for (const int bad : {0, -1, -512, 513, 1000, 2147483647}) {
    expectRejected(sized(bad, 4), "/width", "must be from 1 to 512, not " + std::to_string(bad));
    expectRejected(sized(4, bad), "/height", "must be from 1 to 512, not " + std::to_string(bad));
  }
  EXPECT_TRUE(parseMap(sized(512, 512)).ok());
  EXPECT_TRUE(parseMap(sized(1, 1)).ok());
}

TEST(MapFileMalformedTest, BothBadDimensionsAreReportedTogether) {
  const MapLoad load = parseMap(sized(0, 600));
  EXPECT_TRUE(hasProblem(load, "/width", "not 0"));
  EXPECT_TRUE(hasProblem(load, "/height", "not 600"));
}

TEST(MapFileMalformedTest, DimensionsOfTheWrongTypeAreRejected) {
  expectRejected(doc(", \"width\": \"20\", \"height\": 4"), "/width",
                 "must be an integer, not a string");
  expectRejected(doc(", \"width\": 2.5, \"height\": 4"), "/width", "not the number 2.5");
  expectRejected(doc(", \"width\": 3.0, \"height\": 4"), "/width", "must be an integer");
  expectRejected(doc(", \"width\": null, \"height\": 4"), "/width", "not null");
  expectRejected(doc(", \"width\": [3], \"height\": 4"), "/width", "not an array");
  expectRejected(doc(", \"width\": 3, \"height\": {\"n\": 4}"), "/height", "not an object");
  expectRejected(doc(", \"width\": 3, \"height\": true"), "/height", "not a boolean");
  expectRejected(doc(", \"width\": 99999999999999999999999, \"height\": 4"), "/width", "integer");
}

TEST(MapFileMalformedTest, ObjectsOutsideTheGridAreRejectedForEveryKindOfObject) {
  for (const std::string key : {"walls", "shelves", "pickups", "dropoffs", "chargers"}) {
    for (const std::string cell : {"[20, 0]", "[0, 12]", "[-1, 0]", "[0, -1]", "[20, 12]",
                                   "[2147483648, 0]", "[0, 9223372036854775807]"}) {
      const MapLoad load = parseMap(sized(20, 12, ", \"" + key + "\": [[0, 0], " + cell + "]"));
      EXPECT_FALSE(load.ok()) << key << " " << cell;
      EXPECT_TRUE(hasProblem(load, "/" + key + "/1", "is outside the 20 x 12 grid"))
          << key << " " << cell << "\n"
          << load.describe();
    }
  }
}

TEST(MapFileMalformedTest, TheLastCellInsideTheGridIsAccepted) {
  const MapLoad load = parseOk(sized(20, 12, ", \"walls\": [[19, 11], [0, 0]]"));
  ASSERT_TRUE(load.ok());
  EXPECT_EQ(load.document->layout.at({19, 11}), CellType::Wall);
}

TEST(MapFileMalformedTest, AnOutOfBoundsMessageSaysWhichCell) {
  expectRejected(sized(5, 5, ", \"shelves\": [[1, 1], [7, -3]]"), "/shelves/1",
                 "cell (7, -3) is outside the 5 x 5 grid");
}

TEST(MapFileMalformedTest, ACellMustBeAPairOfIntegers) {
  expectRejected(sized(9, 9, ", \"walls\": [[1]]"), "/walls/0",
                 "must be a pair [x, y], not a list of 1 values");
  expectRejected(sized(9, 9, ", \"walls\": [[1, 2, 3]]"), "/walls/0", "a list of 3 values");
  expectRejected(sized(9, 9, ", \"walls\": [[]]"), "/walls/0", "a list of 0 values");
  expectRejected(sized(9, 9, ", \"walls\": [\"1,2\"]"), "/walls/0", "not a string");
  expectRejected(sized(9, 9, ", \"walls\": [{\"x\": 1, \"y\": 2}]"), "/walls/0", "not an object");
  expectRejected(sized(9, 9, ", \"walls\": [7]"), "/walls/0", "not a number");
  expectRejected(sized(9, 9, ", \"walls\": [null]"), "/walls/0", "not null");
  expectRejected(sized(9, 9, ", \"walls\": [[1.5, 2]]"), "/walls/0/0",
                 "must be an integer, not the number 1.5");
  expectRejected(sized(9, 9, ", \"walls\": [[1, 2.0]]"), "/walls/0/1", "must be an integer");
  expectRejected(sized(9, 9, ", \"walls\": [[\"a\", 2]]"), "/walls/0/0", "not a string");
  expectRejected(sized(9, 9, ", \"walls\": [[1, null]]"), "/walls/0/1", "not null");
  expectRejected(sized(9, 9, ", \"walls\": [[true, false]]"), "/walls/0/0", "not a boolean");
}

TEST(MapFileMalformedTest, AListMustBeAList) {
  for (const std::string key : {"walls", "shelves", "pickups", "dropoffs", "chargers"}) {
    expectRejected(sized(9, 9, ", \"" + key + "\": \"none\""), "/" + key,
                   "must be a list of [x, y] pairs, not a string");
    expectRejected(sized(9, 9, ", \"" + key + "\": {\"0\": [1, 1]}"), "/" + key, "not an object");
    expectRejected(sized(9, 9, ", \"" + key + "\": null"), "/" + key, "not null");
    expectRejected(sized(9, 9, ", \"" + key + "\": 3"), "/" + key, "not a number");
  }
}

TEST(MapFileMalformedTest, ACellListedTwiceInOneListIsRejected) {
  expectRejected(sized(9, 9, ", \"walls\": [[3, 3], [4, 4], [3, 3]]"), "/walls/2",
                 "cell (3, 3) is already listed as a wall at /walls/0");
}

TEST(MapFileMalformedTest, ACellInTwoListsIsRejectedWhateverTheKinds) {
  const std::vector<std::string> keys{"walls", "shelves", "pickups", "dropoffs", "chargers"};
  for (std::size_t a = 0; a < keys.size(); ++a) {
    for (std::size_t b = a + 1; b < keys.size(); ++b) {
      const MapLoad load =
          parseMap(sized(5, 5, ", \"" + keys[a] + "\": [[2, 2]], \"" + keys[b] + "\": [[2, 2]]"));
      EXPECT_FALSE(load.ok()) << keys[a] << " " << keys[b];
      EXPECT_TRUE(hasProblem(load, "/" + keys[b] + "/0", "is already listed as a"))
          << keys[a] << " " << keys[b] << "\n"
          << load.describe();
      EXPECT_TRUE(hasProblem(load, "/" + keys[b] + "/0", "at /" + keys[a] + "/0"));
    }
  }
}

TEST(MapFileMalformedTest, UnknownKeysAreRejectedAndTheKnownOnesAreListed) {
  const MapLoad load = parseMap(sized(3, 3, ", \"robots\": [[0, 0]]"));
  EXPECT_FALSE(load.ok());
  EXPECT_TRUE(hasProblem(load, "/robots", "unknown key"));
  EXPECT_TRUE(hasProblem(load, "/robots", "format, version, name, width, height, walls"));
  expectRejected(sized(3, 3, ", \"Walls\": []"), "/Walls", "unknown key");
  expectRejected(sized(3, 3, ", \"a/b~c\": 1"), "/a~1b~0c", "unknown key");
}

TEST(MapFileMalformedTest, DuplicateKeysAreRejectedNotSilentlyResolved) {
  expectRejected(R"({"format": "warehousesim-map", "version": 1, "width": 3, "width": 4,
                     "height": 3})",
                 "/width", "duplicate key");
  expectRejected(sized(3, 3, ", \"walls\": [[0, 0]], \"walls\": []"), "/walls", "duplicate key");
  expectRejected(R"({"format": "warehousesim-map", "format": "warehousesim-map", "version": 1,
                     "width": 3, "height": 3})",
                 "/format", "duplicate key");
}

TEST(MapFileMalformedTest, TheNameMustBeShortCleanText) {
  expectRejected(sized(3, 3, ", \"name\": 5"), "/name", "must be text, not a number");
  expectRejected(sized(3, 3, ", \"name\": null"), "/name", "not null");
  expectRejected(sized(3, 3, ", \"name\": [\"a\"]"), "/name", "not an array");
  expectRejected(sized(3, 3, ", \"name\": \"" + std::string(81, 'x') + "\""), "/name",
                 "at most 80 bytes, not 81");
  expectRejected(sized(3, 3, ", \"name\": \"bell\\u0007\""), "/name", "control characters");
  expectRejected(sized(3, 3, ", \"name\": \"line\\nbreak\""), "/name", "control characters");
  expectRejected(sized(3, 3, ", \"name\": \"del\\u007f\""), "/name", "control characters");
  EXPECT_TRUE(parseMap(sized(3, 3, ", \"name\": \"" + std::string(80, 'x') + "\"")).ok());
}

TEST(MapFileMalformedTest, EveryProblemIsReportedNotJustTheFirst) {
  const MapLoad load = parseMap(R"({
    "format": "warehousesim-map", "version": 1, "name": 3, "width": 4, "height": 4,
    "walls": [[9, 9], "x", [1, 1], [1, 1]],
    "shelves": 5,
    "extra": 1
  })");
  EXPECT_FALSE(load.ok());
  EXPECT_TRUE(hasProblem(load, "/name", "must be text"));
  EXPECT_TRUE(hasProblem(load, "/walls/0", "outside the 4 x 4 grid"));
  EXPECT_TRUE(hasProblem(load, "/walls/1", "must be a pair"));
  EXPECT_TRUE(hasProblem(load, "/walls/3", "already listed"));
  EXPECT_TRUE(hasProblem(load, "/shelves", "must be a list"));
  EXPECT_TRUE(hasProblem(load, "/extra", "unknown key"));
  EXPECT_EQ(load.problems.size(), 6U);
}

TEST(MapFileMalformedTest, WithoutAUsableSizeCellsAreStillCheckedForTheirShape) {
  const MapLoad load = parseMap(sized(0, 4, ", \"walls\": [[1], [2, 3]]"));
  EXPECT_TRUE(hasProblem(load, "/width", "must be from 1 to 512"));
  EXPECT_TRUE(hasProblem(load, "/walls/0", "must be a pair"));
  EXPECT_EQ(load.problems.size(), 2U);  // [2, 3] cannot be compared with a grid that is not there
}

TEST(MapFileMalformedTest, ThereIsALimitToHowManyProblemsAreListed) {
  std::string walls = "[";
  for (int i = 0; i < 100; ++i) {
    walls += std::string(i == 0 ? "" : ",") + "[1000, 1000]";
  }
  walls += "]";
  const MapLoad load = parseMap(sized(5, 5, ", \"walls\": " + walls));
  EXPECT_FALSE(load.ok());
  ASSERT_EQ(load.problems.size(), warehousesim::mapio::kMaxReportedProblems + 1);
  EXPECT_EQ(load.problems.back().message, "... and 80 more");
}

TEST(MapFileMalformedTest, DeeplyNestedDocumentsAreRefusedBeforeTheyAreBuilt) {
  const std::string deep(100000, '[');
  const MapLoad load = parseMap(deep);
  EXPECT_FALSE(load.ok());
  ASSERT_EQ(load.problems.size(), 1U);
  EXPECT_TRUE(load.problems[0].message.find("nested more than 4 levels") != std::string::npos);
  expectRejected(sized(3, 3, ", \"walls\": [[[[1]]]]"), "", "nested more than 4 levels");
  expectRejected(R"({"a": {"b": {"c": {"d": {"e": 1}}}}})", "", "nested more than");
}

TEST(MapFileMalformedTest,
     ADocumentNestedExactlyAsDeepAsTheLimitIsJudgedBySchemaNotRefusedForItsDepth) {
  const MapLoad load = parseMap(sized(3, 3, ", \"walls\": [[[1]]]"));  // four levels
  EXPECT_FALSE(load.ok());
  EXPECT_TRUE(hasProblem(load, "/walls/0", "must be a pair [x, y], not a list of 1 values"))
      << load.describe();
  EXPECT_FALSE(hasProblem(load, "", "nested more than"));
}

TEST(MapFileMalformedTest, BracketsInsideStringsDoNotCountAsNesting) {
  const MapLoad load = parseOk(sized(3, 3, ", \"name\": \"[[[[[[[[[[ {{{{{ \\\" ]]]]\""));
  ASSERT_TRUE(load.ok());
  EXPECT_EQ(load.document->name, "[[[[[[[[[[ {{{{{ \" ]]]]");
}

TEST(MapFileMalformedTest, ADocumentOverTheSizeLimitIsRefusedWithoutBeingParsed) {
  const std::string huge(warehousesim::mapio::kMaxFileBytes + 1, ' ');
  const MapLoad load = parseMap(huge);
  EXPECT_FALSE(load.ok());
  ASSERT_EQ(load.problems.size(), 1U);
  EXPECT_TRUE(load.problems[0].message.find("the limit is 4194304") != std::string::npos)
      << load.describe();
}

TEST(MapFileMalformedTest, ADocumentOfExactlyTheSizeLimitIsStillRead) {
  std::string text = sized(2, 2);
  text += std::string(warehousesim::mapio::kMaxFileBytes - text.size(), ' ');
  ASSERT_EQ(text.size(), warehousesim::mapio::kMaxFileBytes);
  EXPECT_TRUE(parseMap(text).ok());
}

TEST(MapFileMalformedTest, NumbersAtTheEdgesOfTheIntegerRangesAreHandled) {
  expectRejected(sized(3, 3, ", \"walls\": [[-9223372036854775808, 0]]"), "/walls/0", "outside");
  expectRejected(sized(3, 3, ", \"walls\": [[18446744073709551615, 0]]"), "/walls/0", "outside");
  expectRejected(sized(3, 3, ", \"walls\": [[1e400, 0]]"), "",
                 "not valid JSON");  // overflows a double
  expectRejected(sized(3, 3, ", \"walls\": [[1e30, 0]]"), "/walls/0/0", "integer");
}

TEST(MapFileMalformedTest, ProblemsPrintWithTheirLocation) {
  const MapLoad load = parseMap(sized(3, 3, ", \"walls\": [[5, 5]]"));
  EXPECT_EQ(load.describe(), "/walls/0: cell (5, 5) is outside the 3 x 3 grid");
  EXPECT_EQ((MapProblem{"", "plain"}).text(), "plain");
  EXPECT_EQ((MapProblem{"/a", "b"}).text(), "/a: b");
  EXPECT_TRUE(parseOk(sized(3, 3)).describe().empty());
}

// ---- writing ------------------------------------------------------------------------------------

TEST(MapFileWriteTest, ADocumentThatCannotBeWrittenIsRefusedWithAReason) {
  const Warehouse ok = layout({"P.D"});
  EXPECT_THROW((void)serializeMap({std::string(81, 'n'), ok}), std::invalid_argument);
  EXPECT_THROW((void)serializeMap({"tab\there", ok}), std::invalid_argument);
  EXPECT_THROW((void)serializeMap({"bad \xFF utf8", ok}), std::invalid_argument);
  EXPECT_THROW((void)serializeMap({"", Warehouse(513, 1)}), std::invalid_argument);
  EXPECT_THROW((void)serializeMap({"", Warehouse(1, 1000)}), std::invalid_argument);
}

TEST(MapFileWriteTest, ValidateNamesEveryProblemWithAnObjectLocation) {
  const auto problems = warehousesim::mapio::validate({std::string(90, 'n'), Warehouse(600, 700)});
  ASSERT_EQ(problems.size(), 3U);
  EXPECT_EQ(problems[0].location, "/name");
  EXPECT_EQ(problems[1].location, "/width");
  EXPECT_EQ(problems[2].location, "/height");
  EXPECT_TRUE(warehousesim::mapio::validate({"fine", layout({"P.D"})}).empty());
}

TEST(MapFileWriteTest, WhatIsWrittenIsAlwaysAcceptedAgain) {
  warehousesim::core::Rng rng(11);
  for (int round = 0; round < 20; ++round) {
    Warehouse w(static_cast<std::int32_t>(1 + rng.below(30)),
                static_cast<std::int32_t>(1 + rng.below(30)));
    for (std::size_t i = 0; i < w.size().cellCount(); ++i) {
      w.set(w.size().pointAt(i), warehousesim::world::kAllCellTypes[rng.below(6)]);
    }
    EXPECT_TRUE(parseMap(serializeMap({"n", w})).ok());
  }
}

}  // namespace
