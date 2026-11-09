// Randomly generated warehouses: shelf blocks, stray walls, docks and chargers in places nobody
// chose. Some are fine, some are useless (no dropoff, cut in two), and what must hold of all of
// them is checked: the map file reads back what was written, the simulator refuses exactly the
// layouts that cannot host it, shortest routes are shortest (against an independent breadth-first
// search), and a run with a random fleet and load is collision-free and replays to the same end.
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/simulation/audit.hpp"
#include "warehousesim/simulation/recording.hpp"
#include "warehousesim/simulation/replay.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/tasks/distance_estimator.hpp"
#include "warehousesim/testing/reference_search.hpp"
#include "warehousesim/world/layout_check.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

namespace sim = warehousesim::simulation;
using warehousesim::core::DirectionSet;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::Tick;
using warehousesim::world::Warehouse;

// A hall of 10..40 by 8..24 cells: sometimes walled in, a few shelf blocks, a sprinkling of walls,
// and up to three dropoffs, pickups and chargers dropped on free cells (possibly none at all).
Warehouse randomHall(Rng& rng) {
  const auto width = static_cast<std::size_t>(10 + rng.below(31));
  const auto height = static_cast<std::size_t>(8 + rng.below(17));
  std::vector<std::string> rows(height, std::string(width, '.'));
  if (rng.below(4) != 0) {
    for (std::size_t x = 0; x < width; ++x) {
      rows[0][x] = rows[height - 1][x] = '#';
    }
    for (std::size_t y = 0; y < height; ++y) {
      rows[y][0] = rows[y][width - 1] = '#';
    }
  }
  for (std::uint64_t block = rng.below(8); block > 0; --block) {
    const std::size_t w = 2 + rng.below(8);
    const std::size_t h = 1 + rng.below(2);
    const std::size_t x0 = rng.below(width - w);
    const std::size_t y0 = rng.below(height - h);
    for (std::size_t y = y0; y < y0 + h; ++y) {
      for (std::size_t x = x0; x < x0 + w; ++x) {
        rows[y][x] = 'S';
      }
    }
  }
  const std::uint64_t stray = rng.below(9);  // percent of cells that are a wall
  for (std::string& row : rows) {
    for (char& c : row) {
      if (c == '.' && rng.below(100) < stray) {
        c = '#';
      }
    }
  }
  const auto drop = [&](char kind, std::uint64_t count) {
    for (std::uint64_t i = 0; i < count; ++i) {
      for (int attempt = 0; attempt < 20; ++attempt) {
        char& c = rows[rng.below(height)][rng.below(width)];
        if (c == '.') {
          c = kind;
          break;
        }
      }
    }
  };
  drop('D', rng.below(4));
  drop('P', rng.below(4));
  drop('C', rng.below(4));
  return *Warehouse::fromRows(rows);
}

bool usable(const Warehouse& hall) {
  return !warehousesim::world::blocksSimulation(warehousesim::world::checkLayout(hall));
}

bool connected(const Warehouse& hall) {
  for (const auto& issue : warehousesim::world::checkLayout(hall)) {
    if (issue.kind == warehousesim::world::LayoutIssueKind::DisconnectedRegions) {
      return false;
    }
  }
  return true;
}

constexpr std::uint64_t kSeed = 20260511;

TEST(RandomWorldsTest, AMapFileReadsBackExactlyWhatWasWrittenForAnyHall) {
  Rng rng(kSeed);
  for (int i = 0; i < 120; ++i) {
    const Warehouse hall = randomHall(rng);
    const warehousesim::mapio::MapDocument document{"hall " + std::to_string(i), hall};
    const std::string text = warehousesim::mapio::serializeMap(document);
    const warehousesim::mapio::MapLoad load = warehousesim::mapio::parseMap(text);
    ASSERT_TRUE(load.ok()) << "hall " << i << ":\n" << load.describe();
    EXPECT_EQ(load.document->layout.toRows(), hall.toRows()) << "hall " << i;
    EXPECT_EQ(load.document->name, document.name);
    EXPECT_EQ(warehousesim::mapio::serializeMap(*load.document), text)
        << "hall " << i << ": writing is not canonical";
  }
}

TEST(RandomWorldsTest, TheSimulatorRefusesExactlyTheHallsThatCannotHostIt) {
  Rng rng(kSeed + 1);
  int accepted = 0;
  int refused = 0;
  for (int i = 0; i < 150; ++i) {
    const Warehouse hall = randomHall(rng);
    bool threw = false;
    try {
      (void)sim::mapScenario(hall);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    EXPECT_EQ(threw, !usable(hall)) << "hall " << i;
    (threw ? refused : accepted) += 1;
  }
  EXPECT_GT(accepted, 30);
  EXPECT_GT(refused, 15);
}

TEST(RandomWorldsTest, ShortestRoutesAreShortestAndLegalAgainstAnIndependentSearch) {
  Rng rng(kSeed + 2);
  warehousesim::planning::AStarPlanner planner;
  warehousesim::planning::Path path;
  int halls = 0;
  int reachablePairs = 0;
  int unreachablePairs = 0;
  for (int i = 0; i < 100; ++i) {
    const Warehouse hall = randomHall(rng);
    std::vector<Point> walkable;
    for (std::int32_t y = 0; y < hall.height(); ++y) {
      for (std::int32_t x = 0; x < hall.width(); ++x) {
        if (hall.isWalkable(Point{x, y})) {
          walkable.push_back(Point{x, y});
        }
      }
    }
    if (walkable.size() < 2) {
      continue;
    }
    ++halls;
    warehousesim::tasks::PathDistance estimator;
    for (int pair = 0; pair < 40; ++pair) {
      const Point from = walkable[rng.below(walkable.size())];
      const Point to = walkable[rng.below(walkable.size())];
      const auto expected =
          warehousesim::testing::shortestDistance(hall, from, to, {}, DirectionSet::all());
      const auto result = planner.search(hall, from, to, {}, path);
      ASSERT_EQ(result.status == warehousesim::planning::SearchStatus::Found, expected.has_value())
          << "hall " << i << " from (" << from.x << "," << from.y << ") to (" << to.x << "," << to.y
          << ")";
      EXPECT_EQ(estimator.distance(hall, from, to), expected) << "hall " << i;
      if (!expected) {
        ++unreachablePairs;
        continue;
      }
      ++reachablePairs;
      ASSERT_EQ(result.cost, *expected) << "hall " << i << ": not the shortest";
      ASSERT_EQ(path.size(), static_cast<std::size_t>(*expected));
      Point at = from;
      for (const Point step : path) {
        ASSERT_EQ(warehousesim::core::manhattanDistance(at, step), 1) << "hall " << i;
        ASSERT_TRUE(hall.isWalkable(step)) << "hall " << i << ": the route goes through a wall";
        at = step;
      }
      ASSERT_EQ(at, to);
    }
    if (connected(hall)) {
      // A hall in one piece has a route between any two of its cells.
      EXPECT_TRUE(warehousesim::testing::shortestDistance(hall, walkable.front(), walkable.back(),
                                                          {}, DirectionSet::all())
                      .has_value())
          << "hall " << i;
    }
  }
  EXPECT_GT(halls, 80);
  EXPECT_GT(reachablePairs, 1500);
  EXPECT_GT(unreachablePairs, 50) << "the cut-in-two halls really were met";
}

TEST(RandomWorldsTest, ARunOnAnyUsableHallWithAnyFleetAndLoadIsSafeAndReplaysExactly) {
  Rng rng(kSeed + 3);
  int runs = 0;
  int connectedRuns = 0;
  for (int i = 0; i < 200 && runs < 40; ++i) {
    const Warehouse hall = randomHall(rng);
    if (!usable(hall)) {
      continue;
    }
    sim::ScenarioOptions options;
    options.robots = 1 + rng.below(10);
    options.taskSpawnInterval = 1 + rng.below(10);
    sim::Simulation simulation = [&] {
      try {
        return sim::mapScenario(hall, options)(i);
      } catch (const std::invalid_argument&) {  // no room for so many: take one
        options.robots = 1;
        return sim::mapScenario(hall, options)(i);
      }
    }();
    const std::size_t robots = simulation.robots().size();

    sim::Recorder recorder(std::move(simulation), 100);
    sim::InvariantAuditor auditor(recorder.simulation().warehouse());
    auditor.observe(recorder.simulation());
    for (Tick t = 0; t < 300; ++t) {
      recorder.step();
      auditor.observe(recorder.simulation());
    }
    const std::string name = "hall " + std::to_string(i) + ", " + std::to_string(robots) +
                             " robots, a task every " + std::to_string(*options.taskSpawnInterval);
    EXPECT_TRUE(auditor.ok()) << name << ": " << auditor.summary();

    const sim::Snapshot end = recorder.simulation().snapshot();
    if (connected(hall)) {
      ++connectedRuns;
      EXPECT_GT(end.completedTasks, 0U) << name << " delivered nothing";
    }
    const sim::ReplayResult replayed = sim::replay(recorder.recording());
    ASSERT_TRUE(replayed.ok()) << name << ": " << replayed.divergence->what;
    EXPECT_EQ(replayed.checksum, recorder.simulation().stateChecksum()) << name;
    ++runs;
  }
  EXPECT_EQ(runs, 40);
  EXPECT_GT(connectedRuns, 10);
}

}  // namespace
