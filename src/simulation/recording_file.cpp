#include "warehousesim/simulation/recording_file.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>

namespace warehousesim::simulation {

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;
using core::Point;

// Reading stops at the first thing that is wrong. This carries it out of the helpers below.
struct Reject {
  std::string location;
  std::string message;
};

[[noreturn]] void reject(std::string location, std::string message) {
  throw Reject{std::move(location), std::move(message)};
}

// ---- reading a JSON document strictly (the same rules as the map reader in mapio) ---------------

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

std::string join(const std::string& path, std::string_view key) {
  return path + "/" + pointerEscape(key);
}

std::string join(const std::string& path, std::size_t index) {
  return path + "/" + std::to_string(index);
}

// "a string", "the number 1.5", ... for messages about the wrong kind of value.
std::string describe(const Json& value) {
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
      if (deepest > kMaxRecordingNesting) {
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

// ---- typed access with the place in the document in every message -------------------------------

std::string commaList(std::initializer_list<std::string_view> names) {
  std::string out;
  for (const std::string_view name : names) {
    out += out.empty() ? "" : ", ";
    out += name;
  }
  return out;
}

// A JSON object whose keys are all among `keys`.
class Object {
 public:
  Object(const Json& value, std::string path, std::initializer_list<std::string_view> keys)
      : value_(value), path_(std::move(path)) {
    if (!value.is_object()) {
      reject(path_, "must be an object, not " + describe(value));
    }
    for (const auto& item : value.items()) {
      if (std::find(keys.begin(), keys.end(), item.key()) == keys.end()) {
        reject(join(path_, item.key()), "unknown key; the keys here are: " + commaList(keys));
      }
    }
  }

  [[nodiscard]] const Json* find(std::string_view key) const {
    const auto found = value_.find(std::string(key));
    return found == value_.end() ? nullptr : &*found;
  }
  [[nodiscard]] const Json& need(std::string_view key) const {
    const Json* found = find(key);
    if (found == nullptr) {
      reject(join(path_, key), "required");
    }
    return *found;
  }
  [[nodiscard]] std::string at(std::string_view key) const { return join(path_, key); }
  [[nodiscard]] const std::string& path() const noexcept { return path_; }

 private:
  const Json& value_;
  std::string path_;
};

void requireArray(const Json& value, const std::string& path) {
  if (!value.is_array()) {
    reject(path, "must be a list, not " + describe(value));
  }
}

bool asBool(const Json& value, const std::string& path) {
  if (!value.is_boolean()) {
    reject(path, "must be true or false, not " + describe(value));
  }
  return value.get<bool>();
}

// The result refers into `value`; `path` only words the message. Copy the result when `path` is
// built in the call: the compiler cannot tell it does not refer into that temporary.
const std::string& asText(const Json& value, const std::string& path) {
  if (!value.is_string()) {
    reject(path, "must be text, not " + describe(value));
  }
  return value.get_ref<const std::string&>();
}

std::uint64_t asUnsigned(const Json& value, const std::string& path, std::uint64_t min,
                         std::uint64_t max) {
  const auto fail = [&] {
    reject(path, "must be a whole number from " + std::to_string(min) + " to " +
                     std::to_string(max) + ", not " +
                     (value.is_number() ? value.dump() : describe(value)));
  };
  if (!value.is_number_integer()) {
    fail();
  }
  if (!value.is_number_unsigned()) {
    fail();  // an integer that is not unsigned is negative
  }
  const auto number = value.get<std::uint64_t>();
  if (number < min || number > max) {
    fail();
  }
  return number;
}

double asNumber(const Json& value, const std::string& path) {
  if (!value.is_number()) {
    reject(path, "must be a number, not " + describe(value));
  }
  const double number = value.get<double>();
  if (!std::isfinite(number)) {
    reject(path, "must be a finite number");
  }
  return number;
}

Point asPoint(const Json& value, const std::string& path) {
  if (!value.is_array() || value.size() != 2) {
    reject(path, "must be a pair [x, y], not " +
                     (value.is_array() ? "a list of " + std::to_string(value.size()) + " values"
                                       : describe(value)));
  }
  constexpr auto kLeast = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min());
  constexpr auto kMost = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max());
  std::int32_t coordinate[2] = {0, 0};
  for (std::size_t i = 0; i < 2; ++i) {
    const Json& number = value[i];
    std::optional<std::int64_t> whole;
    if (number.is_number_unsigned()) {
      const auto unsignedValue = number.get<std::uint64_t>();
      if (unsignedValue <= static_cast<std::uint64_t>(kMost)) {
        whole = static_cast<std::int64_t>(unsignedValue);
      }
    } else if (number.is_number_integer()) {
      whole = number.get<std::int64_t>();
    }
    if (!whole || *whole < kLeast || *whole > kMost) {
      reject(join(path, i), "must be a whole number from " + std::to_string(kLeast) + " to " +
                                std::to_string(kMost) + ", not " +
                                (number.is_number() ? number.dump() : describe(number)));
    }
    coordinate[i] = static_cast<std::int32_t>(*whole);
  }
  return {coordinate[0], coordinate[1]};
}

template <typename T>
void readUnsigned(const Object& object, std::string_view key, T& out, std::uint64_t min = 0,
                  std::uint64_t max = std::numeric_limits<T>::max()) {
  out = static_cast<T>(asUnsigned(object.need(key), object.at(key), min, max));
}

void readDouble(const Object& object, std::string_view key, double& out) {
  out = asNumber(object.need(key), object.at(key));
}

void readOptionalDouble(const Object& object, std::string_view key, std::optional<double>& out) {
  const Json& value = object.need(key);
  out = value.is_null() ? std::nullopt : std::optional<double>(asNumber(value, object.at(key)));
}

void readOptionalUnits(const Object& object, std::string_view key,
                       std::optional<robots::Battery::Units>& out) {
  const Json& value = object.need(key);
  if (value.is_null()) {
    out = std::nullopt;
    return;
  }
  out = static_cast<robots::Battery::Units>(
      asUnsigned(value, object.at(key), 0, std::numeric_limits<robots::Battery::Units>::max()));
}

// ---- names in the file --------------------------------------------------------------------------

struct PriorityName {
  tasks::TaskPriority priority;
  std::string_view name;
};
constexpr std::array<PriorityName, 4> kPriorities{{{tasks::TaskPriority::Low, "low"},
                                                   {tasks::TaskPriority::Normal, "normal"},
                                                   {tasks::TaskPriority::High, "high"},
                                                   {tasks::TaskPriority::Urgent, "urgent"}}};

struct ObstacleName {
  world::CellType type;
  std::string_view name;
};
constexpr std::array<ObstacleName, 2> kObstacles{
    {{world::CellType::Wall, "wall"}, {world::CellType::Shelf, "shelf"}}};

std::string_view nameOf(tasks::TaskPriority priority) {
  for (const PriorityName& entry : kPriorities) {
    if (entry.priority == priority) {
      return entry.name;
    }
  }
  return "normal";
}

std::string_view nameOf(world::CellType type) {
  for (const ObstacleName& entry : kObstacles) {
    if (entry.type == type) {
      return entry.name;
    }
  }
  return "wall";
}

tasks::TaskPriority readPriority(const Json& value, const std::string& path) {
  const std::string& text = asText(value, path);
  for (const PriorityName& entry : kPriorities) {
    if (entry.name == text) {
      return entry.priority;
    }
  }
  reject(path, "must be one of low, normal, high, urgent, not \"" + text.substr(0, 40) + "\"");
}

world::CellType readObstacle(const Json& value, const std::string& path) {
  const std::string& text = asText(value, path);
  for (const ObstacleName& entry : kObstacles) {
    if (entry.name == text) {
      return entry.type;
    }
  }
  reject(path, "must be wall or shelf, not \"" + text.substr(0, 40) + "\"");
}

constexpr std::string_view kHexDigits = "0123456789abcdef";

std::uint64_t readChecksum(const Json& value, const std::string& path) {
  const std::string& text = asText(value, path);
  if (text.size() != 16 || text.find_first_not_of(kHexDigits) != std::string::npos) {
    reject(path, "must be 16 lowercase hexadecimal digits, not \"" + text.substr(0, 40) + "\"");
  }
  std::uint64_t number = 0;
  for (const char digit : text) {
    number = (number << 4) | static_cast<std::uint64_t>(kHexDigits.find(digit));
  }
  return number;
}

// ---- the scenario -------------------------------------------------------------------------------

Json pointJson(Point p) { return Json::array({p.x, p.y}); }

Json optionalJson(const std::optional<double>& value) {
  return value ? Json(*value) : Json(nullptr);
}

Json optionalJson(const std::optional<robots::Battery::Units>& value) {
  return value ? Json(*value) : Json(nullptr);
}

Json robotSpecJson(const robots::RobotSpec& spec) {
  return Json{{"maxSpeed", spec.maxSpeed},
              {"acceleration", optionalJson(spec.acceleration)},
              {"deceleration", optionalJson(spec.deceleration)},
              {"turnRate", optionalJson(spec.turnRate)},
              {"batteryCapacity", spec.batteryCapacity},
              {"initialCharge", optionalJson(spec.initialCharge)},
              {"idleDrainPerTick", spec.idleDrainPerTick},
              {"moveCostPerCell", spec.moveCostPerCell},
              {"handlingDrainPerTick", spec.handlingDrainPerTick},
              {"chargePerTick", spec.chargePerTick},
              {"lowBatteryLevel", spec.lowBatteryLevel},
              {"loadTicks", spec.loadTicks},
              {"unloadTicks", spec.unloadTicks}};
}

Json configJson(const SimulationConfig& config) {
  return Json{{"taskSpawnInterval", config.taskSpawnInterval},
              {"varyTaskPriority", config.varyTaskPriority},
              {"maxBlockedTicks", config.maxBlockedTicks},
              {"robot", robotSpecJson(config.robot)},
              {"reservations", config.reservations},
              {"reservationHorizon", config.reservationHorizon},
              {"maxDetourCells", config.maxDetourCells},
              {"cooperative", config.cooperative},
              {"maxDodgeCells", config.maxDodgeCells},
              {"planningWorkPerStep", config.planningWorkPerStep},
              {"planningWorkPerSearch", config.planningWorkPerSearch},
              {"replanInterval", config.replanInterval},
              {"prolongedBlockTicks", config.prolongedBlockTicks},
              {"deadlockTicks", config.deadlockTicks},
              {"priorityBoostTicks", config.priorityBoostTicks},
              {"abandonTicks", config.abandonTicks}};
}

constexpr std::uint64_t kBig = 1'000'000'000;  // an upper bound for counts and tick intervals

robots::RobotSpec readRobotSpec(const Json& value, const std::string& path) {
  const Object object(
      value, path,
      {"maxSpeed", "acceleration", "deceleration", "turnRate", "batteryCapacity", "initialCharge",
       "idleDrainPerTick", "moveCostPerCell", "handlingDrainPerTick", "chargePerTick",
       "lowBatteryLevel", "loadTicks", "unloadTicks"});
  robots::RobotSpec spec;
  readDouble(object, "maxSpeed", spec.maxSpeed);
  readOptionalDouble(object, "acceleration", spec.acceleration);
  readOptionalDouble(object, "deceleration", spec.deceleration);
  readOptionalDouble(object, "turnRate", spec.turnRate);
  readUnsigned(object, "batteryCapacity", spec.batteryCapacity);
  readOptionalUnits(object, "initialCharge", spec.initialCharge);
  readUnsigned(object, "idleDrainPerTick", spec.idleDrainPerTick);
  readUnsigned(object, "moveCostPerCell", spec.moveCostPerCell);
  readUnsigned(object, "handlingDrainPerTick", spec.handlingDrainPerTick);
  readUnsigned(object, "chargePerTick", spec.chargePerTick);
  readUnsigned(object, "lowBatteryLevel", spec.lowBatteryLevel);
  readUnsigned(object, "loadTicks", spec.loadTicks);
  readUnsigned(object, "unloadTicks", spec.unloadTicks);
  return spec;
}

SimulationConfig readConfig(const Json& value, const std::string& path, std::uint64_t seed) {
  const Object object(
      value, path,
      {"taskSpawnInterval", "varyTaskPriority", "maxBlockedTicks", "robot", "reservations",
       "reservationHorizon", "maxDetourCells", "cooperative", "maxDodgeCells",
       "planningWorkPerStep", "planningWorkPerSearch", "replanInterval", "prolongedBlockTicks",
       "deadlockTicks", "priorityBoostTicks", "abandonTicks"});
  SimulationConfig config;
  config.seed = seed;
  readUnsigned(object, "taskSpawnInterval", config.taskSpawnInterval, 0, kBig);
  config.varyTaskPriority = asBool(object.need("varyTaskPriority"), object.at("varyTaskPriority"));
  readUnsigned(object, "maxBlockedTicks", config.maxBlockedTicks, 0, kBig);
  config.robot = readRobotSpec(object.need("robot"), object.at("robot"));
  config.reservations = asBool(object.need("reservations"), object.at("reservations"));
  readUnsigned(object, "reservationHorizon", config.reservationHorizon, 0, 1000);
  readUnsigned(object, "maxDetourCells", config.maxDetourCells, 0, kBig);
  config.cooperative = asBool(object.need("cooperative"), object.at("cooperative"));
  readUnsigned(object, "maxDodgeCells", config.maxDodgeCells, 0, kBig);
  readUnsigned(object, "planningWorkPerStep", config.planningWorkPerStep, 0, kBig);
  readUnsigned(object, "planningWorkPerSearch", config.planningWorkPerSearch, 0, kBig);
  readUnsigned(object, "replanInterval", config.replanInterval, 0, kBig);
  readUnsigned(object, "prolongedBlockTicks", config.prolongedBlockTicks, 0, kBig);
  readUnsigned(object, "deadlockTicks", config.deadlockTicks, 0, kBig);
  readUnsigned(object, "priorityBoostTicks", config.priorityBoostTicks, 0, kBig);
  readUnsigned(object, "abandonTicks", config.abandonTicks, 0, kBig);
  return config;
}

world::Warehouse readLayout(const Json& value, const std::string& path) {
  requireArray(value, path);
  const Json& rows = value;
  if (rows.empty() || rows.size() > static_cast<std::size_t>(world::kMaxMapSide)) {
    reject(path, "must have 1 to " + std::to_string(world::kMaxMapSide) + " rows, not " +
                     std::to_string(rows.size()));
  }
  std::vector<std::string> texts;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    texts.push_back(asText(rows[i], join(path, i)));
    if (texts.back().size() != texts.front().size()) {
      reject(join(path, i), "must be as long as the first row (" +
                                std::to_string(texts.front().size()) + " cells), not " +
                                std::to_string(texts.back().size()));
    }
  }
  if (texts.front().empty() ||
      texts.front().size() > static_cast<std::size_t>(world::kMaxMapSide)) {
    reject(path, "rows must be 1 to " + std::to_string(world::kMaxMapSide) + " cells long");
  }
  std::optional<world::Warehouse> layout = world::Warehouse::fromRows(texts);
  if (!layout) {
    reject(path, "a row has a character that is not a cell: use . # S P D C");
  }
  return *std::move(layout);
}

ScenarioSpec readScenario(const Json& value, const std::string& path) {
  const Object object(value, path, {"seed", "layout", "robots", "config"});
  ScenarioSpec scenario{world::Warehouse(1, 1), {}, {}};
  const std::uint64_t seed = asUnsigned(object.need("seed"), object.at("seed"), 0,
                                        std::numeric_limits<std::uint64_t>::max());
  scenario.warehouse = readLayout(object.need("layout"), object.at("layout"));
  const Json& robots = object.need("robots");
  requireArray(robots, object.at("robots"));
  for (std::size_t i = 0; i < robots.size(); ++i) {
    scenario.robotStarts.push_back(asPoint(robots[i], join(object.at("robots"), i)));
  }
  scenario.config = readConfig(object.need("config"), object.at("config"), seed);
  return scenario;
}

// ---- commands, events, checkpoints --------------------------------------------------------------

RecordedCommand readCommand(const Json& value, const std::string& path) {
  if (!value.is_object()) {
    reject(path, "must be an object, not " + describe(value));
  }
  const auto kind = value.find("command");
  if (kind == value.end()) {
    reject(join(path, "command"), "required");
  }
  const std::string name = asText(*kind, join(path, "command"));

  RecordedCommand entry;
  const auto header = [&entry](const Object& object) {
    readUnsigned(object, "tick", entry.tick);
    entry.accepted = asBool(object.need("accepted"), object.at("accepted"));
  };
  if (name == "submitTask") {
    const Object object(value, path,
                        {"tick", "command", "accepted", "pickup", "dropoff", "priority"});
    header(object);
    entry.command = SubmitTask{asPoint(object.need("pickup"), object.at("pickup")),
                               asPoint(object.need("dropoff"), object.at("dropoff")),
                               readPriority(object.need("priority"), object.at("priority"))};
  } else if (name == "setTaskPriority") {
    const Object object(value, path, {"tick", "command", "accepted", "task", "priority"});
    header(object);
    std::uint32_t task = 0;
    readUnsigned(object, "task", task);
    entry.command = SetTaskPriority{core::TaskId{task},
                                    readPriority(object.need("priority"), object.at("priority"))};
  } else if (name == "cancelTask") {
    const Object object(value, path, {"tick", "command", "accepted", "task"});
    header(object);
    std::uint32_t task = 0;
    readUnsigned(object, "task", task);
    entry.command = CancelTask{core::TaskId{task}};
  } else if (name == "setRobotService") {
    const Object object(value, path, {"tick", "command", "accepted", "robot", "inService"});
    header(object);
    std::uint32_t robot = 0;
    readUnsigned(object, "robot", robot);
    entry.command = SetRobotService{core::RobotId{robot},
                                    asBool(object.need("inService"), object.at("inService"))};
  } else if (name == "placeObstacle") {
    const Object object(value, path, {"tick", "command", "accepted", "cell", "type"});
    header(object);
    entry.command = PlaceObstacle{asPoint(object.need("cell"), object.at("cell")),
                                  readObstacle(object.need("type"), object.at("type"))};
  } else if (name == "clearObstacle") {
    const Object object(value, path, {"tick", "command", "accepted", "cell"});
    header(object);
    entry.command = ClearObstacle{asPoint(object.need("cell"), object.at("cell"))};
  } else {
    reject(join(path, "command"),
           "unknown command \"" + name.substr(0, 40) +
               "\"; the commands are: submitTask, setTaskPriority, cancelTask, setRobotService, "
               "placeObstacle, clearObstacle");
  }
  return entry;
}

TaskEvent readTaskEvent(const Json& value, const std::string& path) {
  if (!value.is_object()) {
    reject(path, "must be an object, not " + describe(value));
  }
  const auto kind = value.find("event");
  if (kind == value.end()) {
    reject(join(path, "event"), "required");
  }
  const std::string name = asText(*kind, join(path, "event"));
  if (name != "created" && name != "cancelled") {
    reject(join(path, "event"), "must be created or cancelled, not \"" + name.substr(0, 40) + "\"");
  }
  const bool created = name == "created";
  TaskEvent event;
  event.kind = created ? TaskEventKind::Created : TaskEventKind::Cancelled;
  const Object object =
      created ? Object(value, path,
                       {"tick", "event", "task", "source", "pickup", "dropoff", "priority"})
              : Object(value, path, {"tick", "event", "task", "source"});
  readUnsigned(object, "tick", event.tick);
  std::uint32_t task = 0;
  readUnsigned(object, "task", task);
  event.task = core::TaskId{task};
  const std::string source = asText(object.need("source"), object.at("source"));
  if (source != "command" && source != "engine") {
    reject(object.at("source"), "must be command or engine, not \"" + source.substr(0, 40) + "\"");
  }
  event.source = source == "command" ? EventSource::FromCommand : EventSource::Engine;
  if (created) {
    event.pickup = asPoint(object.need("pickup"), object.at("pickup"));
    event.dropoff = asPoint(object.need("dropoff"), object.at("dropoff"));
    event.priority = readPriority(object.need("priority"), object.at("priority"));
  }
  return event;
}

Checkpoint readCheckpoint(const Json& value, const std::string& path) {
  const Object object(value, path, {"tick", "checksum"});
  Checkpoint checkpoint;
  readUnsigned(object, "tick", checkpoint.tick);
  checkpoint.checksum = readChecksum(object.need("checksum"), object.at("checksum"));
  return checkpoint;
}

// The version, as an error to give up on or nothing. Looked at before anything else in the
// document, since another version may have another shape.
void checkHeader(const Json& root) {
  const auto format = root.find("format");
  if (format == root.end()) {
    reject("/format", "required; it must be \"" + std::string(kRecordingFormatName) + "\"");
  }
  if (!format->is_string() || format->get<std::string>() != kRecordingFormatName) {
    reject("/format",
           "must be \"" + std::string(kRecordingFormatName) + "\", not " +
               (format->is_string() ? "\"" + format->get<std::string>().substr(0, 40) + "\""
                                    : describe(*format)));
  }
  const auto version = root.find("version");
  if (version == root.end()) {
    reject("/version",
           "required; this program reads version " + std::to_string(kRecordingFormatVersion));
  }
  if (!version->is_number_integer()) {
    reject("/version", "must be an integer, not " + describe(*version));
  }
  if (version->get<std::int64_t>() != kRecordingFormatVersion) {
    reject("/version", "unsupported version " + version->dump() + "; this program reads version " +
                           std::to_string(kRecordingFormatVersion));
  }
}

Recording readRecording(const Json& root) {
  checkHeader(root);
  const Object object(root, "",
                      {"format", "version", "engine", "scenario", "ticks", "checkpointInterval",
                       "commands", "taskEvents", "checkpoints", "end"});
  Recording recording;
  readUnsigned(object, "engine", recording.engineRevision, 0, 1'000'000);
  recording.scenario = readScenario(object.need("scenario"), object.at("scenario"));
  readUnsigned(object, "ticks", recording.ticks, 0, kBig);
  if (object.find("checkpointInterval") != nullptr) {
    readUnsigned(object, "checkpointInterval", recording.checkpointInterval, 0, kBig);
  }
  const auto each = [&object](std::string_view key, auto&& read) {
    if (const Json* list = object.find(key)) {
      const Json& items = *list;
      requireArray(items, object.at(key));
      for (std::size_t i = 0; i < items.size(); ++i) {
        read(items[i], join(object.at(key), i));
      }
    }
  };
  each("commands", [&](const Json& v, const std::string& p) {
    recording.commands.push_back(readCommand(v, p));
  });
  recording.taskEventsRecorded = object.find("taskEvents") != nullptr;
  each("taskEvents", [&](const Json& v, const std::string& p) {
    recording.taskEvents.push_back(readTaskEvent(v, p));
  });
  each("checkpoints", [&](const Json& v, const std::string& p) {
    recording.checkpoints.push_back(readCheckpoint(v, p));
  });
  if (const Json* end = object.find("end")) {
    recording.end = readCheckpoint(*end, object.at("end"));
  }
  return recording;
}

RecordingLoad failed(std::string location, std::string message) {
  return RecordingLoad{std::nullopt, RecordingProblem{std::move(location), std::move(message)}};
}

// ---- writing ------------------------------------------------------------------------------------

Json commandJson(const RecordedCommand& entry) {
  Json json = std::visit(
      [](const auto& command) -> Json {
        using T = std::decay_t<decltype(command)>;
        if constexpr (std::is_same_v<T, SubmitTask>) {
          return Json{{"command", "submitTask"},
                      {"pickup", pointJson(command.pickup)},
                      {"dropoff", pointJson(command.dropoff)},
                      {"priority", std::string(nameOf(command.priority))}};
        } else if constexpr (std::is_same_v<T, SetTaskPriority>) {
          return Json{{"command", "setTaskPriority"},
                      {"task", command.task.value},
                      {"priority", std::string(nameOf(command.priority))}};
        } else if constexpr (std::is_same_v<T, CancelTask>) {
          return Json{{"command", "cancelTask"}, {"task", command.task.value}};
        } else if constexpr (std::is_same_v<T, SetRobotService>) {
          return Json{{"command", "setRobotService"},
                      {"robot", command.robot.value},
                      {"inService", command.inService}};
        } else if constexpr (std::is_same_v<T, PlaceObstacle>) {
          return Json{{"command", "placeObstacle"},
                      {"cell", pointJson(command.cell)},
                      {"type", std::string(nameOf(command.type))}};
        } else if constexpr (std::is_same_v<T, ClearObstacle>) {
          return Json{{"command", "clearObstacle"}, {"cell", pointJson(command.cell)}};
        } else {
          static_assert(std::is_same_v<T, ResetScenario>);
          throw std::invalid_argument("a reset is not part of a recording");
        }
      },
      entry.command);
  json["tick"] = entry.tick;
  json["accepted"] = entry.accepted;
  return json;
}

Json taskEventJson(const TaskEvent& event) {
  Json json{{"tick", event.tick},
            {"event", toString(event.kind)},
            {"task", event.task.value},
            {"source", toString(event.source)}};
  if (event.kind == TaskEventKind::Created) {
    json["pickup"] = pointJson(event.pickup);
    json["dropoff"] = pointJson(event.dropoff);
    json["priority"] = std::string(nameOf(event.priority));
  }
  return json;
}

Json checkpointJson(const Checkpoint& checkpoint) {
  return Json{{"tick", checkpoint.tick}, {"checksum", checksumText(checkpoint.checksum)}};
}

// A list with an entry to a line, so that a long recording stays a readable file.
void appendList(std::string& out, std::string_view key, const std::vector<Json>& items, bool last) {
  out += "  \"" + std::string(key) + "\": [";
  if (items.empty()) {
    out += ']';
  } else {
    out += '\n';
    for (std::size_t i = 0; i < items.size(); ++i) {
      out += "    " + items[i].dump() + (i + 1 < items.size() ? ",\n" : "\n");
    }
    out += "  ]";
  }
  out += last ? "\n" : ",\n";
}

}  // namespace

RecordingLoad parseRecording(std::string_view text) {
  if (text.size() > kMaxRecordingFileBytes) {
    return failed("", "the recording is " + std::to_string(text.size()) + " bytes; the limit is " +
                          std::to_string(kMaxRecordingFileBytes));
  }
  if (nestingDepth(text) > kMaxRecordingNesting) {
    return failed("", "the document is nested more than " + std::to_string(kMaxRecordingNesting) +
                          " levels deep; a recording never is");
  }
  ParsedJson parsed = parseJson(text);
  if (!parsed.value) {
    return failed("", "not valid JSON: " + parsed.error);
  }
  const Json& root = *parsed.value;
  if (!root.is_object()) {
    return failed("", "a recording must be a JSON object, not " + describe(root));
  }
  try {
    if (!parsed.duplicateKeys.empty()) {
      reject("", "the key \"" + parsed.duplicateKeys.front().substr(0, 40) +
                     "\" occurs twice in one object");
    }
    Recording recording = readRecording(root);
    if (auto found = validate(recording)) {
      return RecordingLoad{std::nullopt, std::move(found)};
    }
    // Everything it says is allowed; whether it can be built is the simulation's say.
    try {
      Simulation built(recording.scenario);
    } catch (const std::exception& error) {
      reject("/scenario", std::string("cannot be built: ") + error.what());
    }
    return RecordingLoad{std::move(recording), std::nullopt};
  } catch (Reject& problem) {
    return failed(std::move(problem.location), std::move(problem.message));
  }
}

std::string serializeRecording(const Recording& recording) {
  if (const auto found = validate(recording)) {
    throw std::invalid_argument("the recording cannot be written: " + found->text());
  }
  std::string out = "{\n";
  out += "  \"format\": \"" + std::string(kRecordingFormatName) + "\",\n";
  out += "  \"version\": " + std::to_string(kRecordingFormatVersion) + ",\n";
  out += "  \"engine\": " + std::to_string(recording.engineRevision) + ",\n";

  const ScenarioSpec& scenario = recording.scenario;
  out += "  \"scenario\": {\n";
  out += "    \"seed\": " + std::to_string(scenario.config.seed) + ",\n";
  out += "    \"layout\": [\n";
  const std::vector<std::string> rows = scenario.warehouse.toRows();
  for (std::size_t i = 0; i < rows.size(); ++i) {
    out += "      " + Json(rows[i]).dump() + (i + 1 < rows.size() ? ",\n" : "\n");
  }
  out += "    ],\n";
  Json starts = Json::array();
  for (const Point start : scenario.robotStarts) {
    starts.push_back(pointJson(start));
  }
  out += "    \"robots\": " + starts.dump() + ",\n";
  out += "    \"config\": " + configJson(scenario.config).dump() + "\n";
  out += "  },\n";

  out += "  \"ticks\": " + std::to_string(recording.ticks) + ",\n";
  out += "  \"checkpointInterval\": " + std::to_string(recording.checkpointInterval) + ",\n";
  std::vector<Json> items;
  items.reserve(recording.commands.size());
  for (const RecordedCommand& entry : recording.commands) {
    items.push_back(commandJson(entry));
  }
  appendList(out, "commands", items, false);
  items.clear();
  if (recording.taskEventsRecorded) {
    for (const TaskEvent& event : recording.taskEvents) {
      items.push_back(taskEventJson(event));
    }
    appendList(out, "taskEvents", items, false);
    items.clear();
  }
  for (const Checkpoint& checkpoint : recording.checkpoints) {
    items.push_back(checkpointJson(checkpoint));
  }
  appendList(out, "checkpoints", items, !recording.end);
  if (recording.end) {
    out += "  \"end\": " + checkpointJson(*recording.end).dump() + "\n";
  }
  out += "}\n";
  return out;
}

RecordingLoad loadRecordingFile(const fs::path& path) {
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
  if (size > kMaxRecordingFileBytes) {
    return failed("", shown + " is " + std::to_string(size) + " bytes; the limit is " +
                          std::to_string(kMaxRecordingFileBytes));
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
  return parseRecording(text);
}

std::optional<RecordingProblem> saveRecordingFile(const fs::path& path,
                                                  const Recording& recording) {
  if (const auto found = validate(recording)) {
    return RecordingProblem{"", "the recording cannot be written: " + found->text()};
  }
  const std::string text = serializeRecording(recording);
  fs::path temporary = path;
  temporary += ".tmp";

  std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
  if (!out) {
    return RecordingProblem{"", "cannot create " + temporary.string()};
  }
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
  out.close();
  std::error_code error;
  if (out.fail()) {
    fs::remove(temporary, error);
    return RecordingProblem{"", "could not write " + temporary.string()};
  }
  fs::rename(temporary, path, error);
  if (error) {
    const std::string reason = error.message();
    fs::remove(temporary, error);
    return RecordingProblem{"", "cannot replace " + path.string() + ": " + reason};
  }
  return std::nullopt;
}

}  // namespace warehousesim::simulation
