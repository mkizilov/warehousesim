#include "warehousesim/mapio/map_file.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace warehousesim::mapio {

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;
using world::CellType;

// The lists of cells in the file, in the order they are read and written, with the cell type each
// one stands for. Cells in none of them are open floor.
struct ListSpec {
  std::string_view key;
  CellType type;
  std::string_view noun;
};
constexpr std::array<ListSpec, 5> kLists{{{"walls", CellType::Wall, "wall"},
                                          {"shelves", CellType::Shelf, "shelf"},
                                          {"pickups", CellType::Pickup, "pickup"},
                                          {"dropoffs", CellType::Dropoff, "dropoff"},
                                          {"chargers", CellType::Charging, "charging station"}}};

constexpr std::string_view kKeyList =
    "format, version, name, width, height, walls, shelves, pickups, dropoffs, chargers";

// Collects problems, keeping the first kMaxReportedProblems and counting the rest.
class Problems {
 public:
  void add(std::string location, std::string message) {
    ++total_;
    if (items_.size() < kMaxReportedProblems) {
      items_.push_back({std::move(location), std::move(message)});
    }
  }
  [[nodiscard]] bool any() const noexcept { return total_ > 0; }
  [[nodiscard]] std::vector<MapProblem> take() {
    if (total_ > items_.size()) {
      items_.push_back({"", "... and " + std::to_string(total_ - items_.size()) + " more"});
    }
    return std::move(items_);
  }

 private:
  std::vector<MapProblem> items_;
  std::size_t total_ = 0;
};

MapLoad failed(std::vector<MapProblem> problems) {
  return MapLoad{std::nullopt, std::move(problems)};
}
MapLoad failed(std::string location, std::string message) {
  std::vector<MapProblem> problems;
  problems.push_back({std::move(location), std::move(message)});
  return failed(std::move(problems));
}

// RFC 6901: '~' and '/' inside a key are escaped.
std::string pointerEscape(std::string_view key) {
  std::string out;
  for (const char c : key) {
    if (c == '~') {
      out += "~0";
    } else if (c == '/') {
      out += "~1";
    } else {
      out += c;
    }
  }
  return out;
}

std::string shortQuote(std::string_view text) {
  constexpr std::size_t kLongest = 40;
  std::string out = "\"";
  out.append(text.substr(0, kLongest));
  out += text.size() > kLongest ? "...\"" : "\"";
  return out;
}

// "a string", "a number", ... for messages about the wrong kind of value.
std::string describeValue(const Json& value) {
  if (value.is_number_float()) {
    return "the number " + value.dump();
  }
  if (value.is_null()) {
    return "null";
  }
  const std::string type = value.type_name();
  return std::string(type == "array" || type == "object" ? "an " : "a ") + type;
}

// The deepest nesting of [] and {} in the text, ignoring string contents. Checked before parsing so
// that a document of a million open brackets is refused without building anything.
int nestingDepth(std::string_view text) {
  int depth = 0;
  int deepest = 0;
  bool inString = false;
  bool escaped = false;
  for (const char c : text) {
    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
    } else if (c == '"') {
      inString = true;
    } else if (c == '[' || c == '{') {
      deepest = std::max(deepest, ++depth);
      if (deepest > kMaxNesting) {
        return deepest;
      }
    } else if ((c == ']' || c == '}') && depth > 0) {
      --depth;
    }
  }
  return deepest;
}

struct ParsedJson {
  std::optional<Json> value;
  std::vector<std::string> duplicateKeys;
  std::string error;
};

// Parses the text and notes keys that occur twice in one object (the parser itself keeps the last
// silently, which would hide a typo).
ParsedJson parseJson(std::string_view text) {
  ParsedJson parsed;
  std::vector<std::set<std::string>> keysOfOpenObjects;
  const auto watch = [&](int /*depth*/, Json::parse_event_t event, Json& value) {
    switch (event) {
      case Json::parse_event_t::object_start:
        keysOfOpenObjects.emplace_back();
        break;
      case Json::parse_event_t::key:
        if (!keysOfOpenObjects.back().insert(value.get<std::string>()).second) {
          parsed.duplicateKeys.push_back(value.get<std::string>());
        }
        break;
      case Json::parse_event_t::object_end:
        keysOfOpenObjects.pop_back();
        break;
      default:
        break;
    }
    return true;
  };
  try {
    parsed.value = Json::parse(text.begin(), text.end(), watch, true, false);
  } catch (const Json::exception& error) {
    parsed.error = error.what();
  }
  return parsed;
}

// A JSON integer as a 64-bit one; values beyond the range saturate (they are out of every
// range the format has anyway). Floats such as 3.0 are not integers here.
std::optional<std::int64_t> integerOf(const Json& value) {
  if (value.is_number_unsigned()) {
    const auto unsignedValue = value.get<std::uint64_t>();
    constexpr auto kLargest = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
    return unsignedValue > kLargest ? std::numeric_limits<std::int64_t>::max()
                                    : static_cast<std::int64_t>(unsignedValue);
  }
  if (value.is_number_integer()) {
    return value.get<std::int64_t>();
  }
  return std::nullopt;
}

bool hasControlCharacter(std::string_view text) {
  return std::any_of(text.begin(), text.end(), [](char c) {
    const auto byte = static_cast<unsigned char>(c);
    return byte < 0x20 || byte == 0x7F;
  });
}

// Whether the text can go into a JSON string as it is (the writer refuses invalid UTF-8).
bool isUtf8(const std::string& text) {
  try {
    (void)Json(text).dump();
  } catch (const Json::type_error&) {
    return false;
  }
  return true;
}

std::string cellText(std::int64_t x, std::int64_t y) {
  return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

// ---- reading the schema -------------------------------------------------------------------------

// The header: "format" and "version". False if the reader should not go on to the rest.
bool readHeader(const Json& root, Problems& problems) {
  bool usable = true;
  const auto format = root.find("format");
  if (format == root.end()) {
    problems.add("/format", "required; it must be \"" + std::string(kFormatName) + "\"");
    usable = false;
  } else if (!format->is_string() || format->get<std::string>() != kFormatName) {
    problems.add("/format", "must be \"" + std::string(kFormatName) + "\", not " +
                                (format->is_string() ? shortQuote(format->get<std::string>())
                                                     : describeValue(*format)));
    usable = false;
  }

  const auto version = root.find("version");
  if (version == root.end()) {
    problems.add("/version",
                 "required; this program reads version " + std::to_string(kFormatVersion));
    return false;
  }
  const auto number = integerOf(*version);
  if (!number) {
    problems.add("/version", "must be an integer, not " + describeValue(*version));
    return false;
  }
  if (*number != kFormatVersion) {
    problems.add("/version", "unsupported version " + std::to_string(*number) +
                                 "; this program reads version " + std::to_string(kFormatVersion));
    return false;
  }
  return usable;
}

std::optional<std::int32_t> readSide(const Json& root, std::string_view key, Problems& problems) {
  const std::string location = "/" + std::string(key);
  const auto found = root.find(std::string(key));
  if (found == root.end()) {
    problems.add(location, "required");
    return std::nullopt;
  }
  const auto number = integerOf(*found);
  if (!number) {
    problems.add(location, "must be an integer, not " + describeValue(*found));
    return std::nullopt;
  }
  if (!world::isValidMapSide(*number)) {
    problems.add(location, "must be from 1 to " + std::to_string(world::kMaxMapSide) + ", not " +
                               std::to_string(*number));
    return std::nullopt;
  }
  return static_cast<std::int32_t>(*number);
}

void readName(const Json& root, std::string& name, Problems& problems) {
  const auto found = root.find("name");
  if (found == root.end()) {
    return;
  }
  if (!found->is_string()) {
    problems.add("/name", "must be text, not " + describeValue(*found));
    return;
  }
  name = found->get<std::string>();
  if (name.size() > kMaxNameBytes) {
    problems.add("/name", "must be at most " + std::to_string(kMaxNameBytes) + " bytes, not " +
                              std::to_string(name.size()));
  } else if (hasControlCharacter(name)) {
    problems.add("/name", "must not contain control characters");
  }
}

struct Listed {
  CellType type;
  core::Point cell;
};

// Where a cell was first listed, to say so when it is listed again.
struct FirstSeen {
  bool taken = false;
  std::uint8_t list = 0;
  std::size_t index = 0;
};

void readLists(const Json& root, const std::optional<world::GridSize>& grid, Problems& problems,
               std::vector<Listed>& cells) {
  std::vector<FirstSeen> seen(grid ? grid->cellCount() : 0);
  for (std::size_t listIndex = 0; listIndex < kLists.size(); ++listIndex) {
    const ListSpec& spec = kLists[listIndex];
    const auto found = root.find(std::string(spec.key));
    if (found == root.end()) {
      continue;
    }
    const std::string listLocation = "/" + std::string(spec.key);
    if (!found->is_array()) {
      problems.add(listLocation, "must be a list of [x, y] pairs, not " + describeValue(*found));
      continue;
    }
    for (std::size_t i = 0; i < found->size(); ++i) {
      const Json& element = (*found)[i];
      const std::string location = listLocation + "/" + std::to_string(i);
      if (!element.is_array() || element.size() != 2) {
        problems.add(location, "must be a pair [x, y], not " +
                                   (element.is_array()
                                        ? "a list of " + std::to_string(element.size()) + " values"
                                        : describeValue(element)));
        continue;
      }
      const auto x = integerOf(element[0]);
      const auto y = integerOf(element[1]);
      if (!x) {
        problems.add(location + "/0", "must be an integer, not " + describeValue(element[0]));
      }
      if (!y) {
        problems.add(location + "/1", "must be an integer, not " + describeValue(element[1]));
      }
      if (!x || !y || !grid) {
        continue;  // without a valid size there is nothing to compare the cell with
      }
      const core::Point cell{static_cast<std::int32_t>(std::clamp<std::int64_t>(
                                 *x, std::numeric_limits<std::int32_t>::min(),
                                 std::numeric_limits<std::int32_t>::max())),
                             static_cast<std::int32_t>(std::clamp<std::int64_t>(
                                 *y, std::numeric_limits<std::int32_t>::min(),
                                 std::numeric_limits<std::int32_t>::max()))};
      if (!grid->contains(cell)) {
        problems.add(location, "cell " + cellText(*x, *y) + " is outside the " +
                                   std::to_string(grid->width()) + " x " +
                                   std::to_string(grid->height()) + " grid");
        continue;
      }
      FirstSeen& first = seen[grid->indexOf(cell)];
      if (first.taken) {
        const ListSpec& before = kLists[first.list];
        problems.add(location, "cell " + cellText(*x, *y) + " is already listed as a " +
                                   std::string(before.noun) + " at /" + std::string(before.key) +
                                   "/" + std::to_string(first.index));
        continue;
      }
      first = {true, static_cast<std::uint8_t>(listIndex), i};
      cells.push_back({spec.type, cell});
    }
  }
}

MapLoad readVersion1(const Json& root) {
  Problems problems;
  for (const auto& item : root.items()) {
    const std::string& key = item.key();
    const bool known = key == "format" || key == "version" || key == "name" || key == "width" ||
                       key == "height" ||
                       std::any_of(kLists.begin(), kLists.end(),
                                   [&key](const ListSpec& l) { return l.key == key; });
    if (!known) {
      problems.add("/" + pointerEscape(key),
                   "unknown key; the keys of a map are: " + std::string(kKeyList));
    }
  }

  const auto width = readSide(root, "width", problems);
  const auto height = readSide(root, "height", problems);
  std::string name;
  readName(root, name, problems);

  std::optional<world::GridSize> grid;
  if (width && height) {
    grid.emplace(*width, *height);
  }
  std::vector<Listed> cells;
  readLists(root, grid, problems, cells);
  if (problems.any()) {
    return failed(problems.take());
  }

  world::Warehouse layout(*grid);
  for (const Listed& listed : cells) {
    layout.set(listed.cell, listed.type);
  }
  return MapLoad{MapDocument{std::move(name), std::move(layout)}, {}};
}

// ---- writing ------------------------------------------------------------------------------------

// One list of cells, wrapped to 100 columns so that a big map stays a readable file.
void appendList(std::string& out, std::string_view key, const std::vector<core::Point>& cells,
                bool last) {
  constexpr std::size_t kIndent = 4;
  constexpr std::size_t kWidth = 100;
  out += "  \"" + std::string(key) + "\": [";
  if (cells.empty()) {
    out += ']';
  } else {
    out += "\n" + std::string(kIndent, ' ');
    std::size_t column = kIndent;
    for (std::size_t i = 0; i < cells.size(); ++i) {
      const std::string item =
          "[" + std::to_string(cells[i].x) + ", " + std::to_string(cells[i].y) + "]";
      if (i > 0) {
        out += ',';
        ++column;
        // The item may need a comma of its own after it if another item follows.
        const std::size_t trailingComma = i + 1 < cells.size() ? 1 : 0;
        if (column + 1 + item.size() + trailingComma > kWidth) {
          out += "\n" + std::string(kIndent, ' ');
          column = kIndent;
        } else {
          out += ' ';
          ++column;
        }
      }
      out += item;
      column += item.size();
    }
    out += "\n  ]";
  }
  out += last ? "\n" : ",\n";
}

}  // namespace

std::string MapProblem::text() const {
  return location.empty() ? message : location + ": " + message;
}

std::string MapLoad::describe() const {
  std::string out;
  for (const MapProblem& problem : problems) {
    out += out.empty() ? "" : "\n";
    out += problem.text();
  }
  return out;
}

MapLoad parseMap(std::string_view text) {
  if (text.size() > kMaxFileBytes) {
    return failed("", "the map is " + std::to_string(text.size()) + " bytes; the limit is " +
                          std::to_string(kMaxFileBytes));
  }
  if (nestingDepth(text) > kMaxNesting) {
    return failed("", "the document is nested more than " + std::to_string(kMaxNesting) +
                          " levels deep; a map never is");
  }
  ParsedJson parsed = parseJson(text);
  if (!parsed.value) {
    return failed("", "not valid JSON: " + parsed.error);
  }
  const Json& root = *parsed.value;
  if (!root.is_object()) {
    return failed("", "a map must be a JSON object, not " + describeValue(root));
  }

  Problems header;
  for (const std::string& key : parsed.duplicateKeys) {
    header.add("/" + pointerEscape(key), "duplicate key");
  }
  if (!readHeader(root, header)) {
    return failed(header.take());  // another version may have a different shape: say no more
  }
  MapLoad body = readVersion1(root);
  if (!header.any()) {
    return body;
  }
  std::vector<MapProblem> problems = header.take();
  if (!body.ok()) {
    problems.insert(problems.end(), body.problems.begin(), body.problems.end());
  }
  return failed(std::move(problems));
}

std::vector<MapProblem> validate(const MapDocument& document) {
  std::vector<MapProblem> problems;
  if (document.name.size() > kMaxNameBytes) {
    problems.push_back({"/name", "must be at most " + std::to_string(kMaxNameBytes) +
                                     " bytes, not " + std::to_string(document.name.size())});
  } else if (hasControlCharacter(document.name)) {
    problems.push_back({"/name", "must not contain control characters"});
  } else if (!isUtf8(document.name)) {
    problems.push_back({"/name", "must be valid UTF-8"});
  }
  if (!world::isValidMapSide(document.layout.width())) {
    problems.push_back({"/width", "must be from 1 to " + std::to_string(world::kMaxMapSide) +
                                      ", not " + std::to_string(document.layout.width())});
  }
  if (!world::isValidMapSide(document.layout.height())) {
    problems.push_back({"/height", "must be from 1 to " + std::to_string(world::kMaxMapSide) +
                                       ", not " + std::to_string(document.layout.height())});
  }
  return problems;
}

std::string serializeMap(const MapDocument& document) {
  const std::vector<MapProblem> problems = validate(document);
  if (!problems.empty()) {
    throw std::invalid_argument("the map cannot be written: " + problems.front().text());
  }
  std::string out = "{\n";
  out += "  \"format\": \"" + std::string(kFormatName) + "\",\n";
  out += "  \"version\": " + std::to_string(kFormatVersion) + ",\n";
  out += "  \"name\": " + Json(document.name).dump() + ",\n";
  out += "  \"width\": " + std::to_string(document.layout.width()) + ",\n";
  out += "  \"height\": " + std::to_string(document.layout.height()) + ",\n";
  for (std::size_t i = 0; i < kLists.size(); ++i) {
    appendList(out, kLists[i].key, document.layout.cellsOfType(kLists[i].type),
               i + 1 == kLists.size());
  }
  out += "}\n";
  return out;
}

MapLoad loadMapFile(const fs::path& path) {
  const std::string shown = path.string();
  std::error_code error;
  const fs::file_status status = fs::status(path, error);
  if (error || !fs::exists(status)) {
    return failed("", "cannot open " + shown + ": there is no such file");
  }
  if (!fs::is_regular_file(status)) {
    return failed("", shown + " is not a regular file");
  }
  const std::uintmax_t size = fs::file_size(path, error);
  if (error) {
    return failed("", "cannot read " + shown + ": " + error.message());
  }
  if (size > kMaxFileBytes) {
    return failed("", shown + " is " + std::to_string(size) + " bytes; the limit is " +
                          std::to_string(kMaxFileBytes));
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return failed("", "cannot open " + shown);
  }
  std::string text(static_cast<std::size_t>(size), '\0');
  in.read(text.data(), static_cast<std::streamsize>(text.size()));
  if (in.gcount() != static_cast<std::streamsize>(text.size())) {
    return failed("", "could not read all of " + shown);
  }
  return parseMap(text);
}

std::optional<MapProblem> saveMapFile(const fs::path& path, const MapDocument& document) {
  const std::vector<MapProblem> problems = validate(document);
  if (!problems.empty()) {
    return MapProblem{"", "the map cannot be written: " + problems.front().text()};
  }
  const std::string text = serializeMap(document);
  fs::path temporary = path;
  temporary += ".tmp";

  std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
  if (!out) {
    return MapProblem{"", "cannot create " + temporary.string()};
  }
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
  out.close();
  std::error_code error;
  if (out.fail()) {
    fs::remove(temporary, error);
    return MapProblem{"", "could not write " + temporary.string()};
  }
  fs::rename(temporary, path, error);
  if (error) {
    const std::string reason = error.message();
    fs::remove(temporary, error);
    return MapProblem{"", "cannot replace " + path.string() + ": " + reason};
  }
  return std::nullopt;
}

}  // namespace warehousesim::mapio
