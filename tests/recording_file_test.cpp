#include "warehousesim/simulation/recording_file.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "recording_support.hpp"
#include "scratch_dir.hpp"
#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/rng.hpp"

namespace {

using namespace warehousesim_test;
using Json = nlohmann::json;
using warehousesim::simulation::Checkpoint;
using warehousesim::simulation::EventSource;
using warehousesim::simulation::kMaxRecordingFileBytes;
using warehousesim::simulation::loadRecordingFile;
using warehousesim::simulation::parseRecording;
using warehousesim::simulation::RecordedCommand;
using warehousesim::simulation::RecordingLoad;
using warehousesim::simulation::saveRecordingFile;
using warehousesim::simulation::ScenarioSpec;
using warehousesim::simulation::serializeRecording;
using warehousesim::simulation::TaskEvent;
using warehousesim::simulation::TaskEventKind;
using warehousesim::simulation::validate;

// ---- a small recording written out by hand ------------------------------------------------------

// Every kind of command, a refused one, tasks made and cancelled, checkpoints and an end.
Recording small() {
  Recording recording;
  recording.scenario = ScenarioSpec{*warehousesim::world::Warehouse::fromRows({"P.D", "C.."}),
                                    {P(0, 1), P(2, 1)},
                                    SimulationConfig{}};
  recording.scenario.config.seed = 42;
  recording.ticks = 20;
  recording.checkpointInterval = 10;
  recording.commands = {
      {2, SubmitTask{P(0, 0), P(2, 0), TaskPriority::High}, true},
      {3, SetTaskPriority{TaskId{1}, TaskPriority::Urgent}, true},
      {5, CancelTask{TaskId{1}}, true},
      {6, SetRobotService{RobotId{2}, false}, true},
      {7, PlaceObstacle{P(1, 1), CellType::Wall}, true},
      {8, ClearObstacle{P(1, 1)}, true},
      {9, CancelTask{TaskId{9}}, false},
  };
  TaskEvent created;
  created.tick = 2;
  created.kind = TaskEventKind::Created;
  created.task = TaskId{1};
  created.source = EventSource::Command;
  created.pickup = P(0, 0);
  created.dropoff = P(2, 0);
  created.priority = TaskPriority::High;
  TaskEvent cancelled;
  cancelled.tick = 5;
  cancelled.kind = TaskEventKind::Cancelled;
  cancelled.task = TaskId{1};
  cancelled.source = EventSource::Command;
  recording.taskEvents = {created, cancelled};
  recording.checkpoints = {{0, 0x0000000000000001ULL}, {10, 0x00000000000000ffULL}};
  recording.end = Checkpoint{20, 0xabcdef0123456789ULL};
  return recording;
}

const char* const kSmallText = R"({
  "format": "warehousesim-recording",
  "version": 1,
  "engine": 2,
  "scenario": {
    "seed": 42,
    "layout": [
      "P.D",
      "C.."
    ],
    "robots": [[0,1],[2,1]],
    "config": {"abandonTicks":150,"cooperative":true,"deadlockTicks":6,"maxBlockedTicks":3,"maxDetourCells":4,"maxDodgeCells":12,"planningWorkPerSearch":1500,"planningWorkPerStep":4000,"priorityBoostTicks":40,"prolongedBlockTicks":20,"replanInterval":4,"reservationHorizon":12,"reservations":true,"robot":{"acceleration":null,"batteryCapacity":10000,"chargePerTick":500,"deceleration":null,"handlingDrainPerTick":0,"idleDrainPerTick":0,"initialCharge":null,"loadTicks":0,"lowBatteryLevel":0,"maxSpeed":1.0,"moveCostPerCell":0,"turnRate":null,"unloadTicks":0},"taskSpawnInterval":0,"varyTaskPriority":false}
  },
  "ticks": 20,
  "checkpointInterval": 10,
  "commands": [
    {"accepted":true,"command":"submitTask","dropoff":[2,0],"pickup":[0,0],"priority":"high","tick":2},
    {"accepted":true,"command":"setTaskPriority","priority":"urgent","task":1,"tick":3},
    {"accepted":true,"command":"cancelTask","task":1,"tick":5},
    {"accepted":true,"command":"setRobotService","inService":false,"robot":2,"tick":6},
    {"accepted":true,"cell":[1,1],"command":"placeObstacle","tick":7,"type":"wall"},
    {"accepted":true,"cell":[1,1],"command":"clearObstacle","tick":8},
    {"accepted":false,"command":"cancelTask","task":9,"tick":9}
  ],
  "taskEvents": [
    {"dropoff":[2,0],"event":"created","pickup":[0,0],"priority":"high","source":"command","task":1,"tick":2},
    {"event":"cancelled","source":"command","task":1,"tick":5}
  ],
  "checkpoints": [
    {"checksum":"0000000000000001","tick":0},
    {"checksum":"00000000000000ff","tick":10}
  ],
  "end": {"checksum":"abcdef0123456789","tick":20}
}
)";

// Field by field, so that a field added to a struct and forgotten by the format is found here.
void expectSameConfig(const SimulationConfig& a, const SimulationConfig& b) {
  EXPECT_EQ(a.seed, b.seed);
  EXPECT_EQ(a.taskSpawnInterval, b.taskSpawnInterval);
  EXPECT_EQ(a.varyTaskPriority, b.varyTaskPriority);
  EXPECT_EQ(a.maxBlockedTicks, b.maxBlockedTicks);
  EXPECT_EQ(a.reservations, b.reservations);
  EXPECT_EQ(a.reservationHorizon, b.reservationHorizon);
  EXPECT_EQ(a.maxDetourCells, b.maxDetourCells);
  EXPECT_EQ(a.cooperative, b.cooperative);
  EXPECT_EQ(a.maxDodgeCells, b.maxDodgeCells);
  EXPECT_EQ(a.planningWorkPerStep, b.planningWorkPerStep);
  EXPECT_EQ(a.planningWorkPerSearch, b.planningWorkPerSearch);
  EXPECT_EQ(a.replanInterval, b.replanInterval);
  EXPECT_EQ(a.prolongedBlockTicks, b.prolongedBlockTicks);
  EXPECT_EQ(a.deadlockTicks, b.deadlockTicks);
  EXPECT_EQ(a.priorityBoostTicks, b.priorityBoostTicks);
  EXPECT_EQ(a.abandonTicks, b.abandonTicks);
  EXPECT_EQ(a.robot.maxSpeed, b.robot.maxSpeed);
  EXPECT_EQ(a.robot.acceleration, b.robot.acceleration);
  EXPECT_EQ(a.robot.deceleration, b.robot.deceleration);
  EXPECT_EQ(a.robot.turnRate, b.robot.turnRate);
  EXPECT_EQ(a.robot.batteryCapacity, b.robot.batteryCapacity);
  EXPECT_EQ(a.robot.initialCharge, b.robot.initialCharge);
  EXPECT_EQ(a.robot.idleDrainPerTick, b.robot.idleDrainPerTick);
  EXPECT_EQ(a.robot.moveCostPerCell, b.robot.moveCostPerCell);
  EXPECT_EQ(a.robot.handlingDrainPerTick, b.robot.handlingDrainPerTick);
  EXPECT_EQ(a.robot.chargePerTick, b.robot.chargePerTick);
  EXPECT_EQ(a.robot.lowBatteryLevel, b.robot.lowBatteryLevel);
  EXPECT_EQ(a.robot.loadTicks, b.robot.loadTicks);
  EXPECT_EQ(a.robot.unloadTicks, b.robot.unloadTicks);
}

void expectSameRecording(const Recording& a, const Recording& b) {
  EXPECT_EQ(a.engineRevision, b.engineRevision);
  EXPECT_EQ(a.scenario.warehouse.toRows(), b.scenario.warehouse.toRows());
  EXPECT_EQ(a.scenario.robotStarts, b.scenario.robotStarts);
  expectSameConfig(a.scenario.config, b.scenario.config);
  EXPECT_EQ(a.ticks, b.ticks);
  EXPECT_EQ(a.checkpointInterval, b.checkpointInterval);
  EXPECT_TRUE(a.commands == b.commands);
  EXPECT_EQ(a.taskEventsRecorded, b.taskEventsRecorded);
  EXPECT_TRUE(a.taskEvents == b.taskEvents);
  EXPECT_TRUE(a.checkpoints == b.checkpoints);
  EXPECT_TRUE(a.end == b.end);
}

// ---- the text of the format is a contract -------------------------------------------------------

TEST(RecordingFormatTest, TheTextIsPinnedExactlyForASmallRecording) {
  EXPECT_EQ(serializeRecording(small()), kSmallText);
}

TEST(RecordingFormatTest, TheNameOfEachCommandInTheFileIsTheNameTheEngineGivesIt) {
  const Recording recording = small();
  const Json written = Json::parse(serializeRecording(recording));
  ASSERT_EQ(written["commands"].size(), recording.commands.size());
  std::set<std::size_t> kinds;
  for (std::size_t i = 0; i < recording.commands.size(); ++i) {
    EXPECT_EQ(written["commands"][i]["command"].get<std::string>(),
              warehousesim::simulation::commandName(recording.commands[i].command))
        << i;
    kinds.insert(recording.commands[i].command.index());
  }
  EXPECT_EQ(kinds.size(), std::variant_size_v<Command> - 1);  // every kind but the reset
}

TEST(RecordingFormatTest, ThePinnedTextReadsBackAsWhatWasWritten) {
  const RecordingLoad load = parseRecording(kSmallText);
  ASSERT_TRUE(load.ok()) << load.problem->text();
  expectSameRecording(*load.recording, small());
}

TEST(RecordingFormatTest, ReadingThenWritingChangesNothing) {
  const RecordingLoad load = parseRecording(kSmallText);
  ASSERT_TRUE(load.ok());
  EXPECT_EQ(serializeRecording(*load.recording), kSmallText);
}

TEST(RecordingFormatTest, EveryFieldOfTheConfigurationSurvivesTheRoundTrip) {
  SimulationConfig config;
  config.seed = 0xFFFFFFFFFFFFFFF0ULL;  // beyond what a double holds
  config.taskSpawnInterval = 7;
  config.varyTaskPriority = true;
  config.maxBlockedTicks = 9;
  config.reservations = false;
  config.reservationHorizon = 33;
  config.maxDetourCells = 5;
  config.cooperative = false;
  config.maxDodgeCells = 13;
  config.planningWorkPerStep = 4001;
  config.planningWorkPerSearch = 1501;
  config.replanInterval = 5;
  config.prolongedBlockTicks = 21;
  config.deadlockTicks = 7;
  config.priorityBoostTicks = 41;
  config.abandonTicks = 151;
  config.robot.maxSpeed = 0.75;
  config.robot.acceleration = 0.5;
  config.robot.deceleration = 1.25;
  config.robot.turnRate = warehousesim::core::kPi / 2.0 + 0.1;  // not a short decimal
  config.robot.batteryCapacity = 20001;
  config.robot.initialCharge = 15000;
  config.robot.idleDrainPerTick = 2;
  config.robot.moveCostPerCell = 16;
  config.robot.handlingDrainPerTick = 3;
  config.robot.chargePerTick = 251;
  config.robot.lowBatteryLevel = 3001;
  config.robot.loadTicks = 3;
  config.robot.unloadTicks = 4;
  Recording recording = small();
  recording.scenario.config = config;
  const RecordingLoad load = parseRecording(serializeRecording(recording));
  ASSERT_TRUE(load.ok()) << load.problem->text();
  expectSameConfig(load.recording->scenario.config, config);
  EXPECT_EQ(load.recording->scenario.config.robot.turnRate, config.robot.turnRate);  // bit for bit
}

TEST(RecordingFormatTest, TheFormatCoversEveryFieldOfTheConfigurationAndTheRobotModel) {
  // Structured bindings need exactly as many names as there are fields: adding a field to either
  // struct stops this compiling until it is added here, and so until it is added to the format.
  const SimulationConfig config{};
  [[maybe_unused]] const auto& [seed, spawn, vary, blocked, robot, reservations, horizon, detour,
                                cooperative, dodge, workStep, workSearch, replan, prolonged,
                                deadlock, boost, abandon] = config;
  const warehousesim::robots::RobotSpec spec{};
  [[maybe_unused]] const auto& [maxSpeed, acceleration, deceleration, turnRate, capacity, charge,
                                idle, move, handling, chargeRate, low, load, unload] = spec;
  const Json written = Json::parse(serializeRecording(small()));
  EXPECT_EQ(written["scenario"]["config"].size(), 16U);  // all but the seed, which stands apart
  EXPECT_EQ(written["scenario"]["config"]["robot"].size(), 13U);
}

TEST(RecordingFormatTest, EqualRecordingsGiveEqualTextAndEveryRunOfABundledLayoutRoundTrips) {
  for (const std::string& name : bundledLayouts()) {
    const Recording recording = busyRecording(name, 3, 120);
    const std::string text = serializeRecording(recording);
    EXPECT_EQ(text, serializeRecording(busyRecording(name, 3, 120))) << name;
    const RecordingLoad load = parseRecording(text);
    ASSERT_TRUE(load.ok()) << name << ": " << load.problem->text();
    expectSameRecording(*load.recording, recording);
    EXPECT_EQ(serializeRecording(*load.recording), text) << name;
  }
}

TEST(RecordingFormatTest, TheFileHasNoClockNoPathAndNothingOfTheMachine) {
  const std::string text = serializeRecording(busyRecording("compact", 3, 100));
  for (const char* forbidden : {"/Users", "/home", "C:\\", "time", "date", "host", "user", "pid"}) {
    EXPECT_EQ(text.find(forbidden), std::string::npos) << forbidden;
  }
}

TEST(RecordingFormatTest, ASeedOfAnySizeKeepsEveryBit) {
  for (const std::uint64_t seed :
       {std::uint64_t{0}, std::uint64_t{1}, std::uint64_t{1} << 53, (std::uint64_t{1} << 53) + 1,
        std::numeric_limits<std::uint64_t>::max()}) {
    Recording recording = small();
    recording.scenario.config.seed = seed;
    const RecordingLoad load = parseRecording(serializeRecording(recording));
    ASSERT_TRUE(load.ok()) << seed;
    EXPECT_EQ(load.recording->scenario.config.seed, seed);
  }
}

TEST(RecordingFormatTest, ARecordingWithNothingToCheckIsAScenarioScript) {
  const std::string text = R"({
    "format": "warehousesim-recording", "version": 1, "engine": 1,
    "scenario": {"seed": 3, "layout": ["P.D"], "robots": [[1, 0]],
      "config": )" + Json::parse(kSmallText)["scenario"]["config"].dump() +
                           R"(},
    "ticks": 30,
    "commands": [{"tick": 2, "command": "submitTask", "accepted": true,
                  "pickup": [0, 0], "dropoff": [2, 0], "priority": "normal"}]
  })";
  const RecordingLoad load = parseRecording(text);
  ASSERT_TRUE(load.ok()) << load.problem->text();
  EXPECT_TRUE(load.recording->checkpoints.empty());
  EXPECT_FALSE(load.recording->taskEventsRecorded);
  EXPECT_TRUE(load.recording->taskEvents.empty());
  EXPECT_FALSE(load.recording->end.has_value());
  EXPECT_EQ(load.recording->checkpointInterval, 0U);
  EXPECT_EQ(load.recording->commands.size(), 1U);
  // Written back it stays a script: the tasks are still not claimed to be anything.
  EXPECT_EQ(serializeRecording(*load.recording).find("taskEvents"), std::string::npos);
}

TEST(RecordingFormatTest, NoTasksListedIsNotTheSameAsTheListLeftOut) {
  Recording kept = small();
  kept.taskEvents.clear();
  kept.commands.clear();  // nothing made, nothing cancelled
  ASSERT_TRUE(kept.taskEventsRecorded);
  const std::string keptText = serializeRecording(kept);
  EXPECT_NE(keptText.find("\"taskEvents\": []"), std::string::npos);
  EXPECT_TRUE(parseRecording(keptText).recording->taskEventsRecorded);

  Recording left = kept;
  left.taskEventsRecorded = false;
  const std::string leftText = serializeRecording(left);
  EXPECT_EQ(leftText.find("taskEvents"), std::string::npos);
  EXPECT_FALSE(parseRecording(leftText).recording->taskEventsRecorded);
  EXPECT_EQ(serializeRecording(*parseRecording(leftText).recording), leftText);

  Recording contradiction = small();
  contradiction.taskEventsRecorded = false;  // but it lists events
  const auto problem = validate(contradiction);
  ASSERT_TRUE(problem.has_value());
  EXPECT_EQ(problem->location, "/taskEvents");
}

TEST(RecordingFormatTest, ARecordingThatCannotBeWrittenIsRefusedWithTheReason) {
  Recording late = small();
  late.commands[0].tick = 99;
  try {
    (void)serializeRecording(late);
    ADD_FAILURE() << "should have thrown";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("/commands/0/tick"), std::string::npos)
        << error.what();
  }
}

// ---- reading is strict --------------------------------------------------------------------------

struct Case {
  std::string name;
  std::function<void(Json&)> change;
  std::string location;
  std::string fragment;
};

Json smallJson() { return Json::parse(kSmallText); }

void expectRefused(const Case& c) {
  Json document = smallJson();
  c.change(document);
  const RecordingLoad load = parseRecording(document.dump());
  ASSERT_FALSE(load.ok()) << c.name << " was accepted";
  ASSERT_TRUE(load.problem.has_value()) << c.name;
  EXPECT_EQ(load.problem->location, c.location) << c.name << ": " << load.problem->text();
  EXPECT_NE(load.problem->message.find(c.fragment), std::string::npos)
      << c.name << ": " << load.problem->text() << " (wanted: " << c.fragment << ")";
}

#define SET(path, value) [](Json& d) { d[Json::json_pointer(path)] = value; }
#define ERASE(parent, key) [](Json& d) { d[Json::json_pointer(parent)].erase(key); }

TEST(RecordingFormatTest, TheHeaderIsCheckedBeforeAnythingElse) {
  expectRefused({"another format", SET("/format", "warehousesim-map"), "/format",
                 "must be \"warehousesim-recording\", not \"warehousesim-map\""});
  expectRefused({"a format that is not text", SET("/format", 7), "/format", "not a number"});
  expectRefused({"no format", ERASE("", "format"), "/format", "required"});
  expectRefused({"no version", ERASE("", "version"), "/version", "required"});
  expectRefused({"a newer version", SET("/version", 2), "/version",
                 "unsupported version 2; this program reads version 1"});
  expectRefused({"version 0", SET("/version", 0), "/version", "unsupported version 0"});
  expectRefused({"a version that is text", SET("/version", "1"), "/version", "must be an integer"});
  // A newer version may have other keys: it is refused for its version, not for what it has.
  expectRefused({"a newer version with other keys",
                 [](Json& d) {
                   d["version"] = 2;
                   d["tracks"] = 1;
                 },
                 "/version", "unsupported version 2"});
}

TEST(RecordingFormatTest, UnknownAndMissingKeysAreNamed) {
  expectRefused(
      {"unknown key", SET("/colour", 1), "/colour", "unknown key; the keys here are: format"});
  expectRefused({"unknown key with a slash", SET("/a~1b", 1), "/a~1b", "unknown key"});
  expectRefused({"no engine", ERASE("", "engine"), "/engine", "required"});
  expectRefused({"a negative engine", SET("/engine", -1), "/engine",
                 "must be a whole number from 0 to 1000000, not -1"});
  expectRefused({"no scenario", ERASE("", "scenario"), "/scenario", "required"});
  expectRefused({"no ticks", ERASE("", "ticks"), "/ticks", "required"});
  expectRefused({"too many ticks", SET("/ticks", 2000000000), "/ticks",
                 "must be a whole number from 0 to 1000000000, not 2000000000"});
  expectRefused({"negative ticks", SET("/ticks", -1), "/ticks", "not -1"});
  expectRefused({"fractional ticks", SET("/ticks", 2.5), "/ticks", "not 2.5"});
  expectRefused({"ticks as text", SET("/ticks", "20"), "/ticks", "not a string"});
  expectRefused({"an interval as text", SET("/checkpointInterval", "x"), "/checkpointInterval",
                 "not a string"});
  expectRefused({"the scenario is not an object", SET("/scenario", 3), "/scenario",
                 "must be an object, not a number"});
  expectRefused(
      {"unknown scenario key", SET("/scenario/mood", 1), "/scenario/mood", "unknown key"});
  expectRefused({"no seed", ERASE("/scenario", "seed"), "/scenario/seed", "required"});
}

TEST(RecordingFormatTest, TheSeedMustBeAWholeNumberThatFitsSixtyFourBits) {
  expectRefused({"a negative seed", SET("/scenario/seed", -1), "/scenario/seed",
                 "from 0 to 18446744073709551615, not -1"});
  expectRefused({"a fractional seed", SET("/scenario/seed", 1.5), "/scenario/seed", "not 1.5"});
  expectRefused({"a seed as text", SET("/scenario/seed", "7"), "/scenario/seed", "not a string"});
  expectRefused(
      {"a seed that is null", SET("/scenario/seed", nullptr), "/scenario/seed", "not null"});
}

TEST(RecordingFormatTest, TheLayoutIsRowsOfCellsOfOneLengthWithinTheLimits) {
  expectRefused({"no rows", SET("/scenario/layout", Json::array()), "/scenario/layout",
                 "must have 1 to 512 rows, not 0"});
  expectRefused({"layout as text", SET("/scenario/layout", "P.D"), "/scenario/layout",
                 "must be a list, not a string"});
  expectRefused({"a row that is not text", SET("/scenario/layout/1", 5), "/scenario/layout/1",
                 "must be text, not a number"});
  expectRefused({"a short row", SET("/scenario/layout/1", "C."), "/scenario/layout/1",
                 "must be as long as the first row (3 cells), not 2"});
  expectRefused({"an unknown cell", SET("/scenario/layout/0", "PXD"), "/scenario/layout",
                 "not a cell: use . # S P D C"});
  expectRefused({"empty rows", [](Json& d) { d["scenario"]["layout"] = Json::array({"", ""}); },
                 "/scenario/layout", "rows must be 1 to 512 cells long"});
  expectRefused({"too many rows",
                 [](Json& d) { d["scenario"]["layout"] = Json(513, std::string("...")); },
                 "/scenario/layout", "must have 1 to 512 rows, not 513"});
  expectRefused({"rows too long",
                 [](Json& d) { d["scenario"]["layout"] = Json::array({std::string(513, '.')}); },
                 "/scenario/layout", "rows must be 1 to 512 cells long"});
}

TEST(RecordingFormatTest, RobotStartsArePairsOfWholeNumbersThatMakeASimulation) {
  expectRefused(
      {"robots as text", SET("/scenario/robots", "x"), "/scenario/robots", "must be a list"});
  expectRefused({"a start that is one number", SET("/scenario/robots/0", Json::array({1})),
                 "/scenario/robots/0", "must be a pair [x, y], not a list of 1 values"});
  expectRefused({"a start that is not a list", SET("/scenario/robots/0", 4), "/scenario/robots/0",
                 "must be a pair [x, y], not a number"});
  expectRefused({"a fractional coordinate", SET("/scenario/robots/0", Json::array({1.5, 2})),
                 "/scenario/robots/0/0", "not 1.5"});
  expectRefused({"a coordinate beyond 32 bits",
                 SET("/scenario/robots/1", Json::array({1, 99999999999})), "/scenario/robots/1/1",
                 "from -2147483648 to 2147483647, not 99999999999"});
  expectRefused({"a coordinate that is huge and unsigned",
                 SET("/scenario/robots/1", Json::array({std::uint64_t{1} << 63, 0})),
                 "/scenario/robots/1/0", "from -2147483648 to 2147483647"});
  expectRefused({"a coordinate far below 32 bits",
                 SET("/scenario/robots/1", Json::array({1, -99999999999LL})),
                 "/scenario/robots/1/1", "from -2147483648 to 2147483647, not -99999999999"});
  // The limits themselves are numbers; it is the map that has no such cell.
  expectRefused({"the lowest coordinate",
                 SET("/scenario/robots/1", Json::array({-2147483648LL, 0})), "/scenario",
                 "cannot be built: robot start cell is not walkable"});
  expectRefused({"one below it", SET("/scenario/robots/1", Json::array({-2147483649LL, 0})),
                 "/scenario/robots/1/0", "not -2147483649"});
  expectRefused({"the highest coordinate", SET("/scenario/robots/1", Json::array({0, 2147483647})),
                 "/scenario", "cannot be built: robot start cell is not walkable"});
  expectRefused({"one above it", SET("/scenario/robots/1", Json::array({0, 2147483648LL})),
                 "/scenario/robots/1/1", "not 2147483648"});
  expectRefused({"a start off the map", SET("/scenario/robots/0", Json::array({9, 9})), "/scenario",
                 "cannot be built: robot start cell is not walkable"});
  expectRefused({"two robots in one cell", SET("/scenario/robots/1", Json::array({0, 1})),
                 "/scenario", "cannot be built: two robots share a start cell"});
  expectRefused({"a robot on a wall",
                 [](Json& d) {
                   d["scenario"]["layout"][0] = "P#D";
                   d["scenario"]["robots"][0] = Json::array({1, 0});
                 },
                 "/scenario", "cannot be built: robot start cell is not walkable"});
}

TEST(RecordingFormatTest, TheConfigurationNeedsEveryKeyOfTheRightKindAndOneThatCanBeBuilt) {
  expectRefused({"no config", ERASE("/scenario", "config"), "/scenario/config", "required"});
  expectRefused({"a config key missing", ERASE("/scenario/config", "cooperative"),
                 "/scenario/config/cooperative", "required"});
  expectRefused({"a robot key missing", ERASE("/scenario/config/robot", "maxSpeed"),
                 "/scenario/config/robot/maxSpeed", "required"});
  expectRefused({"an optional key missing", ERASE("/scenario/config/robot", "turnRate"),
                 "/scenario/config/robot/turnRate", "required"});
  expectRefused({"an unknown config key", SET("/scenario/config/bogus", 1),
                 "/scenario/config/bogus", "unknown key"});
  expectRefused({"an unknown robot key", SET("/scenario/config/robot/bogus", 1),
                 "/scenario/config/robot/bogus", "unknown key"});
  expectRefused({"a flag as a number", SET("/scenario/config/reservations", 1),
                 "/scenario/config/reservations", "must be true or false, not a number"});
  expectRefused({"a negative count", SET("/scenario/config/abandonTicks", -5),
                 "/scenario/config/abandonTicks", "from 0 to 1000000000, not -5"});
  expectRefused({"a horizon beyond the limit", SET("/scenario/config/reservationHorizon", 1001),
                 "/scenario/config/reservationHorizon", "from 0 to 1000, not 1001"});
  expectRefused({"a horizon too short to use", SET("/scenario/config/reservationHorizon", 1),
                 "/scenario",
                 "cannot be built: the reservation horizon must be at least two steps"});
  expectRefused({"no room to plan", SET("/scenario/config/planningWorkPerStep", 0), "/scenario",
                 "cannot be built: the planning work limits must be positive"});
  expectRefused({"no deadlock time", SET("/scenario/config/deadlockTicks", 0), "/scenario",
                 "cannot be built: deadlockTicks must be positive"});
  expectRefused({"a speed that is text", SET("/scenario/config/robot/maxSpeed", "fast"),
                 "/scenario/config/robot/maxSpeed", "must be a number, not a string"});
  expectRefused({"a speed of no robot", SET("/scenario/config/robot/maxSpeed", 2.0), "/scenario",
                 "cannot be built:"});
  expectRefused({"an acceleration that is text", SET("/scenario/config/robot/acceleration", "x"),
                 "/scenario/config/robot/acceleration", "must be a number"});
  expectRefused({"a battery below zero", SET("/scenario/config/robot/batteryCapacity", -1),
                 "/scenario/config/robot/batteryCapacity", "from 0 to 4294967295, not -1"});
  expectRefused({"a charge too big", SET("/scenario/config/robot/initialCharge", 4294967296ULL),
                 "/scenario/config/robot/initialCharge", "from 0 to 4294967295"});
}

TEST(RecordingFormatTest, EachCommandNeedsExactlyItsOwnKeys) {
  expectRefused({"a command that is not an object", SET("/commands/0", 5), "/commands/0",
                 "must be an object, not a number"});
  expectRefused({"a command without a name", ERASE("/commands/0", "command"), "/commands/0/command",
                 "required"});
  expectRefused({"a name that is not text", SET("/commands/0/command", 3), "/commands/0/command",
                 "must be text"});
  expectRefused({"a command nobody knows", SET("/commands/0/command", "dance"),
                 "/commands/0/command", "unknown command \"dance\"; the commands are: submitTask"});
  expectRefused({"a reset is not a command here", SET("/commands/0/command", "resetScenario"),
                 "/commands/0/command", "unknown command \"resetScenario\""});
  expectRefused(
      {"an extra key", SET("/commands/0/colour", 1), "/commands/0/colour", "unknown key"});
  expectRefused({"a key of another command", SET("/commands/2/cell", Json::array({0, 0})),
                 "/commands/2/cell", "unknown key"});
  expectRefused({"no tick", ERASE("/commands/0", "tick"), "/commands/0/tick", "required"});
  expectRefused(
      {"no accepted", ERASE("/commands/0", "accepted"), "/commands/0/accepted", "required"});
  expectRefused({"accepted as text", SET("/commands/0/accepted", "yes"), "/commands/0/accepted",
                 "must be true or false"});
  expectRefused({"no pickup", ERASE("/commands/0", "pickup"), "/commands/0/pickup", "required"});
  expectRefused({"a priority nobody knows", SET("/commands/0/priority", "super"),
                 "/commands/0/priority",
                 "must be one of low, normal, high, urgent, not \"super\""});
  expectRefused({"a priority in capitals", SET("/commands/0/priority", "High"),
                 "/commands/0/priority", "not \"High\""});
  expectRefused({"a task number below zero", SET("/commands/2/task", -3), "/commands/2/task",
                 "from 0 to 4294967295, not -3"});
  expectRefused({"a robot as text", SET("/commands/3/robot", "two"), "/commands/3/robot",
                 "must be a whole number"});
  expectRefused({"a service as a number", SET("/commands/3/inService", 0), "/commands/3/inService",
                 "must be true or false"});
  expectRefused({"an obstacle that is a pickup", SET("/commands/4/type", "pickup"),
                 "/commands/4/type", "must be wall or shelf, not \"pickup\""});
  expectRefused({"a cell that is one number", SET("/commands/5/cell", Json::array({1})),
                 "/commands/5/cell", "must be a pair"});
}

TEST(RecordingFormatTest, EventsAndCheckpointsAreCheckedLikeCommands) {
  expectRefused({"an event that is neither", SET("/taskEvents/0/event", "born"),
                 "/taskEvents/0/event", "must be created or cancelled, not \"born\""});
  expectRefused({"an event without a name", ERASE("/taskEvents/0", "event"), "/taskEvents/0/event",
                 "required"});
  expectRefused({"a source nobody knows", SET("/taskEvents/0/source", "me"), "/taskEvents/0/source",
                 "must be command or engine, not \"me\""});
  expectRefused({"a created event without a pickup", ERASE("/taskEvents/0", "pickup"),
                 "/taskEvents/0/pickup", "required"});
  expectRefused({"a cancelled event with a pickup",
                 SET("/taskEvents/1/pickup", Json::array({0, 0})), "/taskEvents/1/pickup",
                 "unknown key"});
  expectRefused({"a checksum that is too short", SET("/checkpoints/0/checksum", "00000000000001"),
                 "/checkpoints/0/checksum", "must be 16 lowercase hexadecimal digits"});
  expectRefused({"a checksum in capitals", SET("/checkpoints/0/checksum", "ABCDEF0123456789"),
                 "/checkpoints/0/checksum", "must be 16 lowercase hexadecimal digits"});
  expectRefused({"a checksum that is a number", SET("/checkpoints/0/checksum", 1),
                 "/checkpoints/0/checksum", "must be text"});
  expectRefused({"a checkpoint without a tick", ERASE("/checkpoints/0", "tick"),
                 "/checkpoints/0/tick", "required"});
  expectRefused(
      {"the end without a checksum", ERASE("/end", "checksum"), "/end/checksum", "required"});
}

TEST(RecordingFormatTest, WhatDoesNotAddUpIsRefusedWithWhereItIsWrong) {
  expectRefused({"a command after the end", SET("/commands/6/tick", 21), "/commands/6/tick",
                 "after the end of the run (tick 20)"});
  expectRefused({"commands out of order", SET("/commands/1/tick", 1), "/commands/1/tick",
                 "before the command before it"});
  expectRefused({"an event after the end", SET("/taskEvents/1/tick", 21), "/taskEvents/1/tick",
                 "after the end of the run"});
  expectRefused({"tasks made out of numbering", SET("/taskEvents/0/task", 2), "/taskEvents/0/task",
                 "tasks are numbered 1, 2, 3"});
  expectRefused({"a task cancelled before it was made", SET("/taskEvents/1/task", 3),
                 "/taskEvents/1/task", "has not been made"});
  expectRefused({"checkpoints out of order", SET("/checkpoints/1/tick", 0), "/checkpoints/1/tick",
                 "increasing tick"});
  expectRefused({"a checkpoint after the end", SET("/checkpoints/1/tick", 25),
                 "/checkpoints/1/tick", "after the end of the run"});
  expectRefused({"an end that is not the end", SET("/end/tick", 19), "/end/tick",
                 "the end checkpoint is at tick 19"});
}

TEST(RecordingFormatTest, ADocumentThatIsNotARecordingIsSaidSoAtTheTop) {
  const auto problem = [](const std::string& text) {
    const RecordingLoad load = parseRecording(text);
    EXPECT_FALSE(load.ok()) << text.substr(0, 40);
    return load.problem.value_or(warehousesim::simulation::RecordingProblem{"", "(none)"});
  };
  EXPECT_NE(problem("").message.find("not valid JSON"), std::string::npos);
  EXPECT_NE(problem("{ nope").message.find("not valid JSON"), std::string::npos);
  EXPECT_EQ(problem("[]").message, "a recording must be a JSON object, not an array");
  EXPECT_EQ(problem("7").message, "a recording must be a JSON object, not a number");
  EXPECT_EQ(problem("null").message, "a recording must be a JSON object, not null");
  EXPECT_EQ(problem("{}").location, "/format");
}

TEST(RecordingFormatTest, AKeyThatOccursTwiceIsRefusedNotQuietlyResolved) {
  std::string text = kSmallText;
  const std::size_t at = text.find("\"ticks\": 20,");
  ASSERT_NE(at, std::string::npos);
  text.replace(at, 12, "\"ticks\": 20, \"ticks\": 21,");
  const RecordingLoad load = parseRecording(text);
  ASSERT_FALSE(load.ok());
  EXPECT_EQ(load.problem->message, "the key \"ticks\" occurs twice in one object");

  std::string nested = kSmallText;
  const std::size_t inner = nested.find("\"seed\": 42,");
  nested.replace(inner, 11, "\"seed\": 42, \"seed\": 43,");
  EXPECT_FALSE(parseRecording(nested).ok());
}

TEST(RecordingFormatTest, DeepNestingAndHugeDocumentsAreStoppedBeforeAnythingIsBuilt) {
  const RecordingLoad deep = parseRecording(std::string(100000, '[') + std::string(100000, ']'));
  ASSERT_FALSE(deep.ok());
  EXPECT_NE(deep.problem->message.find("nested more than 6 levels deep"), std::string::npos);

  const RecordingLoad bytes = parseRecording(std::string(kMaxRecordingFileBytes + 1, ' '));
  ASSERT_FALSE(bytes.ok());
  EXPECT_NE(bytes.problem->message.find("the limit is 67108864"), std::string::npos);

  // Brackets inside text do not count as nesting.
  Json document = smallJson();
  document["scenario"]["config"]["robot"].erase("maxSpeed");  // refused, but for this reason only
  document["notes"] = "[[[[[[[[[[[[";
  const RecordingLoad quoted = parseRecording(document.dump());
  ASSERT_FALSE(quoted.ok());
  EXPECT_EQ(quoted.problem->message.find("nested"), std::string::npos);
}

TEST(RecordingFormatTest, TheTextOfAProblemNamesWhereItIs) {
  Json document = smallJson();
  document["commands"][2]["task"] = -1;
  const RecordingLoad load = parseRecording(document.dump());
  ASSERT_FALSE(load.ok());
  EXPECT_EQ(load.problem->text(),
            "/commands/2/task: must be a whole number from 0 to 4294967295, not -1");
}

// ---- reading what is not the file's fault: damage -----------------------------------------------

TEST(RecordingFormatFuzzTest, DamagedFilesNeverThrowAndWhatIsAcceptedIsConsistent) {
  const std::string text = serializeRecording(busyRecording("compact", 2, 90));
  warehousesim::core::Rng rng(12345);
  std::size_t accepted = 0;
  for (int round = 0; round < 400; ++round) {
    std::string damaged = text;
    const int kind = static_cast<int>(rng.below(5));
    const std::size_t at = static_cast<std::size_t>(rng.below(damaged.size()));
    switch (kind) {
      case 0:  // one byte becomes another
        damaged[at] = static_cast<char>(rng.below(256));
        break;
      case 1:  // a stretch is cut out
        damaged.erase(at, static_cast<std::size_t>(rng.below(40)));
        break;
      case 2:  // the file ends early
        damaged.resize(at);
        break;
      case 3:  // a stretch is repeated
        damaged.insert(at, damaged.substr(at, static_cast<std::size_t>(rng.below(60))));
        break;
      default:  // a digit becomes another digit
        for (std::size_t i = at; i < damaged.size(); ++i) {
          if (damaged[i] >= '0' && damaged[i] <= '9') {
            damaged[i] = static_cast<char>('0' + rng.below(10));
            break;
          }
        }
        break;
    }
    RecordingLoad load;
    ASSERT_NO_THROW(load = parseRecording(damaged)) << "round " << round;
    ASSERT_NE(load.ok(), load.problem.has_value()) << "exactly one of the two must be set";
    if (load.ok()) {
      ++accepted;
      EXPECT_FALSE(validate(*load.recording).has_value()) << "round " << round;
      const std::string again = serializeRecording(*load.recording);
      const RecordingLoad reread = parseRecording(again);
      ASSERT_TRUE(reread.ok()) << "round " << round;
      EXPECT_EQ(serializeRecording(*reread.recording), again);
    }
  }
  EXPECT_LT(accepted, 400U) << "damage that is never noticed is no test";
  EXPECT_GT(accepted, 0U) << "damage that is always noticed leaves the accepting side untested";
}

// ---- files --------------------------------------------------------------------------------------

TEST(RecordingFileTest, ASavedRecordingLoadsBackAndLeavesNoTemporaryFile) {
  ScratchDir dir("RecordingFileTest_roundtrip");
  const Recording recording = busyRecording("fulfilment", 5, 100);
  const std::filesystem::path path = dir.file("run.json");
  EXPECT_FALSE(saveRecordingFile(path, recording).has_value());
  EXPECT_EQ(readFile(path), serializeRecording(recording));
  EXPECT_FALSE(std::filesystem::exists(dir.file("run.json.tmp")));
  const RecordingLoad load = loadRecordingFile(path);
  ASSERT_TRUE(load.ok()) << load.problem->text();
  expectSameRecording(*load.recording, recording);
}

TEST(RecordingFileTest, SavingReplacesTheOldFileWholeOrNotAtAll) {
  ScratchDir dir("RecordingFileTest_replace");
  const std::filesystem::path path = dir.file("run.json");
  writeFile(path, "old contents");
  Recording bad = small();
  bad.commands[0].tick = 99;
  const auto problem = saveRecordingFile(path, bad);
  ASSERT_TRUE(problem.has_value());
  EXPECT_NE(problem->message.find("cannot be written"), std::string::npos) << problem->text();
  EXPECT_EQ(readFile(path), "old contents");
  EXPECT_FALSE(std::filesystem::exists(dir.file("run.json.tmp")));
  EXPECT_FALSE(saveRecordingFile(path, small()).has_value());
  EXPECT_EQ(readFile(path), kSmallText);
}

TEST(RecordingFileTest, ASaveThatCannotHappenSaysWhy) {
  ScratchDir dir("RecordingFileTest_nowhere");
  const auto missingFolder = saveRecordingFile(dir.file("no-such-folder/run.json"), small());
  ASSERT_TRUE(missingFolder.has_value());
  EXPECT_NE(missingFolder->message.find("cannot create"), std::string::npos)
      << missingFolder->text();
  std::filesystem::create_directory(dir.file("folder"));
  const auto folder = saveRecordingFile(dir.file("folder"), small());
  ASSERT_TRUE(folder.has_value());
  EXPECT_TRUE(std::filesystem::is_directory(dir.file("folder")));
  EXPECT_FALSE(std::filesystem::exists(dir.file("folder.tmp")));
}

TEST(RecordingFileTest, AFileThatCannotBeReadSaysWhy) {
  ScratchDir dir("RecordingFileTest_unreadable");
  const RecordingLoad missing = loadRecordingFile(dir.file("nothing.json"));
  ASSERT_FALSE(missing.ok());
  EXPECT_NE(missing.problem->message.find("there is no such file"), std::string::npos);
  std::filesystem::create_directory(dir.file("folder"));
  const RecordingLoad folder = loadRecordingFile(dir.file("folder"));
  ASSERT_FALSE(folder.ok());
  EXPECT_NE(folder.problem->message.find("is not a regular file"), std::string::npos);
  writeFile(dir.file("bad.json"), "{ not json");
  const RecordingLoad bad = loadRecordingFile(dir.file("bad.json"));
  ASSERT_FALSE(bad.ok());
  EXPECT_NE(bad.problem->message.find("not valid JSON"), std::string::npos);
  writeFile(dir.file("empty.json"), "");
  EXPECT_FALSE(loadRecordingFile(dir.file("empty.json")).ok());
}

}  // namespace
