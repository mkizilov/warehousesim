// Whole runs, from a layout to the final numbers, audited on every tick: different layouts, fleet
// sizes and task loads, then runs that are pushed around with every kind of command and recorded
// and replayed. Nothing here knows how the engine works inside; it only asks what must be true of
// any run (see simulation/audit.hpp) and that the same run happens again.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/simulation/audit.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/recording.hpp"
#include "warehousesim/simulation/replay.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/world/layout_check.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

namespace sim = warehousesim::simulation;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::Tick;
using warehousesim::world::CellType;

// The result of a run that was audited all the way.
struct Outcome {
  sim::Snapshot snapshot;
  std::uint64_t checksum = 0;
  std::string audit;  // empty if nothing was wrong
  std::uint64_t deliveries = 0;
};

Outcome runAudited(sim::Simulation simulation, Tick ticks) {
  sim::InvariantAuditor auditor(simulation.warehouse());
  auditor.observe(simulation);
  for (Tick t = 0; t < ticks; ++t) {
    simulation.step();
    auditor.observe(simulation);
  }
  Outcome outcome;
  outcome.snapshot = simulation.snapshot();
  outcome.checksum = simulation.stateChecksum();
  outcome.audit = auditor.ok() ? "" : auditor.summary();
  for (const sim::RobotSnapshot& robot : outcome.snapshot.robots) {
    outcome.deliveries += robot.deliveries;
  }
  return outcome;
}

sim::Simulation fleetOn(const std::string& layout, std::size_t robots, Tick spawn,
                        std::uint64_t seed) {
  sim::ScenarioOptions options;
  options.robots = robots;
  options.taskSpawnInterval = spawn;
  return (*sim::layoutScenario(layout, options))(seed);
}

// ---- layouts x fleet sizes x task loads ---------------------------------------------------------

TEST(EndToEndTest, EveryLayoutFleetAndLoadRunsCleanAndGetsWorkDone) {
  constexpr Tick kTicks = 500;
  std::size_t runs = 0;
  for (const char* layout : {"demo", "compact", "fulfilment"}) {
    for (const std::size_t robots : {1U, 4U, 16U}) {
      for (const Tick spawn : {1U, 5U, 25U}) {
        for (const std::uint64_t seed : {1U, 2U}) {
          const Outcome outcome = runAudited(fleetOn(layout, robots, spawn, seed), kTicks);
          const std::string name = std::string(layout) + " x" + std::to_string(robots) +
                                   ", a task every " + std::to_string(spawn) + ", seed " +
                                   std::to_string(seed);
          EXPECT_EQ(outcome.audit, "") << name;
          EXPECT_EQ(outcome.snapshot.tick, kTicks) << name;
          EXPECT_EQ(outcome.snapshot.robots.size(), robots) << name;
          EXPECT_GT(outcome.snapshot.completedTasks, 0U) << name << " delivered nothing";
          EXPECT_EQ(outcome.deliveries, outcome.snapshot.completedTasks)
              << name << ": the robots' deliveries are the completed tasks";
          ++runs;
        }
      }
    }
  }
  EXPECT_EQ(runs, 54U);
}

TEST(EndToEndTest, WithoutAnyTaskNothingHappensAndNobodyMoves) {
  for (const char* layout : {"demo", "compact", "fulfilment"}) {
    sim::Simulation simulation = fleetOn(layout, 6, 0, 3);
    const std::vector<sim::RobotSnapshot> start = simulation.snapshot().robots;
    const Outcome outcome = runAudited(std::move(simulation), 300);
    EXPECT_EQ(outcome.audit, "") << layout;
    EXPECT_EQ(outcome.snapshot.completedTasks + outcome.snapshot.failedTasks, 0U) << layout;
    EXPECT_TRUE(outcome.snapshot.activeTasks.empty()) << layout;
    for (std::size_t i = 0; i < start.size(); ++i) {
      EXPECT_EQ(outcome.snapshot.robots[i].position, start[i].position)
          << layout << ": robot " << i + 1 << " moved with nothing to do";
    }
  }
}

TEST(EndToEndTest, AMoreGenerousLoadIsServedBetterByABiggerFleet) {
  std::vector<std::size_t> completed;
  for (const std::size_t robots : {1U, 2U, 4U, 8U}) {
    const Outcome outcome = runAudited(fleetOn("fulfilment", robots, 1, 1), 800);
    ASSERT_EQ(outcome.audit, "") << robots << " robots";
    completed.push_back(outcome.snapshot.completedTasks);
  }
  EXPECT_LT(completed[0], completed[1]);
  EXPECT_LT(completed[1], completed[2]);
  EXPECT_LT(completed[2], completed[3]);
}

// A fleet three times bigger than a light load needs serves nearly all of it. It does not serve
// all of it: robots that have finished stay where they are, and a few end up on the dropoff cells
// that others are heading for (README, limitations), so a task or two is given up after the robot
// has been stuck for too long. What must hold is that every failure is such an abandonment.
TEST(EndToEndTest, ALightLoadIsMostlyServedAndEveryFailureIsAnExplainedAbandonment) {
  const Outcome outcome = runAudited(fleetOn("fulfilment", 16, 25, 1), 1500);
  ASSERT_EQ(outcome.audit, "");
  const sim::Snapshot& end = outcome.snapshot;
  EXPECT_GE(end.completedTasks, 50U);  // a task every 25 ticks: 60 were made
  EXPECT_LE(end.activeTasks.size(), 6U);
  EXPECT_EQ(end.failedTasks, end.planning.abandoned) << "a task failed for another reason";
  EXPECT_EQ(end.completedTasks + end.failedTasks + end.cancelledTasks + end.activeTasks.size(),
            60U);
}

TEST(EndToEndTest, HundredsOfRobotsOnTheLargeHallStayCollisionFree) {
  for (const std::size_t robots : {100U, 256U}) {
    for (const Tick spawn : {1U, 5U}) {
      const Outcome outcome = runAudited(fleetOn("large", robots, spawn, 1), 300);
      const std::string name =
          "large x" + std::to_string(robots) + ", a task every " + std::to_string(spawn);
      EXPECT_EQ(outcome.audit, "") << name;
      EXPECT_EQ(outcome.snapshot.robots.size(), robots) << name;
      EXPECT_GT(outcome.snapshot.completedTasks, 0U) << name;
    }
  }
}

// Found by the audited scenario campaign (docs/benchmarks.md): a robot that has taken its own step
// in a tick and is then pushed aside by a blocked robot with a higher id has moved twice in one
// tick. The campaign showed it in 5 of 328 runs, all in crowded maps; these are three of them, each
// run to just past the tick at which the unfixed engine first did it (368, 996 and 993).
TEST(EndToEndTest, ARobotThatHasMovedIsNeverPushedAsideInTheSameTick) {
  struct Run {
    const char* map;  // an example file
    std::size_t robots;
    Tick spawn;
    std::uint64_t seed;
    Tick ticks;
  };
  const Run runs[] = {
      {"single-lane.json", 16, 5, 2, 400},
      {"charging-hub.json", 16, 25, 2, 1000},
      {"choke-point.json", 64, 25, 2, 1000},
  };
  for (const Run& run : runs) {
    sim::ScenarioOptions options;
    options.robots = run.robots;
    options.taskSpawnInterval = run.spawn;
    const std::string map = run.map;
    const auto load =
        warehousesim::mapio::loadMapFile(std::filesystem::path(WAREHOUSESIM_EXAMPLES_DIR) / map);
    ASSERT_TRUE(load.ok()) << map << ":\n" << load.describe();
    const Outcome outcome =
        runAudited(sim::mapScenario(load.document->layout, options)(run.seed), run.ticks);
    EXPECT_EQ(outcome.audit, "") << map << " x" << run.robots << ", a task every " << run.spawn;
  }
}

TEST(EndToEndTest, RunningTheSameScenarioTwiceGivesTheSameRunToTheLastBit) {
  for (const char* layout : {"compact", "fulfilment"}) {
    const Outcome a = runAudited(fleetOn(layout, 8, 2, 7), 600);
    const Outcome b = runAudited(fleetOn(layout, 8, 2, 7), 600);
    EXPECT_EQ(a.checksum, b.checksum) << layout;
    EXPECT_TRUE(a.snapshot == b.snapshot) << layout;
    const Outcome other = runAudited(fleetOn(layout, 8, 2, 8), 600);
    EXPECT_NE(a.checksum, other.checksum) << layout << ": another seed is another run";
  }
}

// ---- pushed around with commands, recorded and replayed -----------------------------------------

std::vector<Point> walkableCells(const warehousesim::world::Warehouse& warehouse) {
  std::vector<Point> cells;
  for (std::int32_t y = 0; y < warehouse.height(); ++y) {
    for (std::int32_t x = 0; x < warehouse.width(); ++x) {
      if (warehouse.isWalkable(Point{x, y})) {
        cells.push_back(Point{x, y});
      }
    }
  }
  return cells;
}

// A random command of any kind that the engine understands (some of them will be refused, which is
// part of the point).
sim::Command randomCommand(Rng& rng, const std::vector<Point>& cells, std::size_t robots,
                           std::vector<Point>& walls, std::size_t tasksMade) {
  const auto cell = [&] { return cells[rng.below(cells.size())]; };
  const auto priority = [&] {
    return static_cast<warehousesim::tasks::TaskPriority>(rng.below(4));
  };
  const auto task = [&] {
    return warehousesim::core::TaskId{static_cast<std::uint32_t>(1 + rng.below(tasksMade + 2))};
  };
  switch (rng.below(7)) {
    case 0:
    case 1:
      return sim::SubmitTask{cell(), cell(), priority()};
    case 2:
      return sim::SetTaskPriority{task(), priority()};
    case 3:
      return sim::CancelTask{task()};
    case 4:
      return sim::SetRobotService{
          warehousesim::core::RobotId{static_cast<std::uint32_t>(1 + rng.below(robots))},
          rng.below(3) != 0};
    case 5: {
      const Point at = cell();
      walls.push_back(at);
      return sim::PlaceObstacle{at, rng.below(2) == 0 ? CellType::Wall : CellType::Shelf};
    }
    default:
      if (walls.empty()) {
        return sim::SubmitTask{cell(), cell(), priority()};
      }
      return sim::ClearObstacle{walls[rng.below(walls.size())]};
  }
}

TEST(EndToEndTest, AnyMixOfCommandsLeavesARunSafeAndItsRecordingReplaysExactly) {
  std::size_t accepted = 0;
  std::size_t refused = 0;
  for (const char* layout : {"demo", "compact", "fulfilment"}) {
    for (const std::uint64_t seed : {1U, 2U, 3U}) {
      sim::Recorder recorder(*sim::makeLayoutSimulation(layout, seed), 50);
      sim::InvariantAuditor auditor(recorder.simulation().warehouse());
      const std::vector<Point> cells = walkableCells(recorder.simulation().warehouse());
      const std::size_t robots = recorder.simulation().robots().size();
      std::vector<Point> walls;
      Rng rng(seed * 1000 + 17);

      auditor.observe(recorder.simulation());
      for (Tick tick = 0; tick < 600; ++tick) {
        if (rng.below(5) == 0) {
          const sim::CommandOutcome outcome = recorder.apply(
              randomCommand(rng, cells, robots, walls, recorder.simulation().tasks().all().size()));
          (outcome.accepted ? accepted : refused) += 1;
        }
        recorder.step();
        auditor.observe(recorder.simulation());
      }
      const std::string name = std::string(layout) + ", seed " + std::to_string(seed);
      EXPECT_TRUE(auditor.ok()) << name << ": " << auditor.summary();

      const sim::Recording recording = recorder.recording();
      const sim::ReplayResult replayed = sim::replay(recording);
      ASSERT_TRUE(replayed.ok()) << name << ": " << replayed.divergence->what;
      EXPECT_EQ(replayed.checksum, recorder.simulation().stateChecksum()) << name;
      EXPECT_TRUE(replayed.endVerified) << name;

      sim::ReplayOptions threaded;
      threaded.planningThreads = 2;
      const sim::ReplayResult onThreads = sim::replay(recording, threaded);
      ASSERT_TRUE(onThreads.ok()) << name << ": " << onThreads.divergence->what;
      EXPECT_EQ(onThreads.checksum, replayed.checksum) << name << ": threads changed the replay";
    }
  }
  EXPECT_GT(accepted, 150U) << "commands of every kind were really applied";
  EXPECT_GT(refused, 20U) << "and the engine really refused some";
}

// ---- the example maps ---------------------------------------------------------------------------

TEST(EndToEndTest, EveryExampleMapIsValidAndRunsCleanOnAFleetAndALoad) {
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(WAREHOUSESIM_EXAMPLES_DIR)) {
    if (entry.path().extension() == ".json") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  ASSERT_GE(files.size(), 6U) << "the examples directory " << WAREHOUSESIM_EXAMPLES_DIR;

  for (const std::filesystem::path& file : files) {
    const warehousesim::mapio::MapLoad load = warehousesim::mapio::loadMapFile(file);
    ASSERT_TRUE(load.ok()) << file << ":\n" << load.describe();
    EXPECT_FALSE(warehousesim::world::blocksSimulation(
        warehousesim::world::checkLayout(load.document->layout)))
        << file;
    // Saved by the editor's own writer, so loading and saving changes nothing.
    std::ifstream in(file, std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_EQ(warehousesim::mapio::serializeMap(*load.document), text)
        << file << " is not in the canonical form";

    for (const std::size_t robots : {1U, 6U}) {
      sim::ScenarioOptions options;
      options.robots = robots;
      options.taskSpawnInterval = 4;
      const Outcome outcome = runAudited(sim::mapScenario(load.document->layout, options)(1), 600);
      EXPECT_EQ(outcome.audit, "") << file.filename() << " x" << robots;
      EXPECT_GT(outcome.snapshot.completedTasks, 0U) << file.filename() << " x" << robots;
    }
  }
}

}  // namespace
