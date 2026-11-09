#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "warehousesim/world/warehouse.hpp"

// The warehouse map file: a versioned JSON document with an explicit schema, and the reading,
// validating and writing of it. The world model knows nothing about files; this module turns a
// world::Warehouse into text and back, and refuses anything that is not a valid map.
//
// Format, version 1 (a JSON object; unknown keys, duplicate keys and wrong types are errors):
//
//   {
//     "format":   "warehousesim-map",            required, exactly this text
//     "version":  1,                             required, the integer 1
//     "name":     "my warehouse",                optional, at most 80 bytes, no control characters
//     "width":    20,                            required, integer, 1 to 512
//     "height":   12,                            required, integer, 1 to 512
//     "walls":    [[x, y], ...],                 each optional (default none): cells of that kind
//     "shelves":  [[x, y], ...],                 x is the column and y the row, both counted from 0
//     "pickups":  [[x, y], ...],                 at the top left; every cell must be inside the
//     grid "dropoffs": [[x, y], ...],                 and may be listed once, in one list only
//     "chargers": [[x, y], ...]                  cells in no list are open floor
//   }
//
// A change that old readers cannot understand gets a new version number; a reader that sees a newer
// version than it knows says so instead of guessing.
namespace warehousesim::mapio {

inline constexpr std::string_view kFormatName = "warehousesim-map";
inline constexpr int kFormatVersion = 1;
inline constexpr std::size_t kMaxNameBytes = 80;
// A 512 x 512 map with every cell listed takes under 3 MB; anything bigger is not a map.
inline constexpr std::size_t kMaxFileBytes = std::size_t{4} * 1024 * 1024;
// Document, list, pair: the schema never goes deeper than 3; a little more is allowed so that the
// error says what is wrong instead of the nesting being the error.
inline constexpr int kMaxNesting = 4;
// Reading stops collecting after this many problems and says how many more there were.
inline constexpr std::size_t kMaxReportedProblems = 20;

struct MapDocument {
  std::string name;  // free text for people; empty is fine
  world::Warehouse layout;
};

// One thing wrong with a map, and where: `location` is a JSON Pointer into the document
// ("/walls/3/0"), or empty for the document as a whole.
struct MapProblem {
  std::string location;
  std::string message;

  // "/walls/3: cell (25, 2) is outside the 20 x 12 grid", or just the message without a location.
  [[nodiscard]] std::string text() const;
};

// The outcome of reading a map: the document if there is one, otherwise what is wrong with it.
// Exactly one of the two is set.
struct MapLoad {
  std::optional<MapDocument> document;
  std::vector<MapProblem> problems;

  [[nodiscard]] bool ok() const noexcept { return document.has_value(); }
  // Every problem on a line of its own; empty when ok().
  [[nodiscard]] std::string describe() const;
};

// Reads a map from text. Never throws for bad input: a document that is too big, not JSON, nested
// too deeply, or does not fit the schema comes back as problems. Every problem the schema check
// can find is reported (up to kMaxReportedProblems), not just the first.
[[nodiscard]] MapLoad parseMap(std::string_view text);

// What stops `document` from being written as a valid map: a name that is too long, holds control
// characters or is not UTF-8, or a layout wider or taller than the format allows. Empty when it is
// fine.
[[nodiscard]] std::vector<MapProblem> validate(const MapDocument& document);

// The document as map text. The text is canonical: cells are listed row by row, so equal
// documents give identical text and parseMap(serializeMap(d)) gives back d. Throws
// std::invalid_argument (with the first problem as its message) if validate() finds any.
[[nodiscard]] std::string serializeMap(const MapDocument& document);

// Reads a map file. Problems with the file itself (missing, not a regular file, too big,
// unreadable) are reported like problems with the content.
[[nodiscard]] MapLoad loadMapFile(const std::filesystem::path& path);

// Validates and writes a map file. The text goes to a temporary file next to `path` first and is
// moved into place only when it has been written completely, so a failure never leaves a
// half-written map under the real name. Returns nullopt on success.
[[nodiscard]] std::optional<MapProblem> saveMapFile(const std::filesystem::path& path,
                                                    const MapDocument& document);

}  // namespace warehousesim::mapio
