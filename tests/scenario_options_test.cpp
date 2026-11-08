#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using namespace warehousesim_test;
using warehousesim::simulation::kMaxScenarioRobots;
using warehousesim::simulation::layoutScenario;
using warehousesim::simulation::makeLayoutSimulation;
using warehousesim::simulation::makeMapSimulation;
using warehousesim::simulation::mapScenario;
using warehousesim::simulation::ScenarioOptions;
using warehousesim::world::CellType;

std::vector<Point> startsOf(const Simulation& sim) {
  std::vector<Point> starts;
  for (const auto& robot : sim.robots()) {
    starts.push_back(robot.position());
  }
  return starts;
}

std::vector<std::string> layouts() {
  std::vector<std::string> names;
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    names.emplace_back(info.name);
  }
  return names;
}

// ---- the options that change nothing ------------------------------------------------------------

TEST(ScenarioOptionsTest, NoOptionsIsExactlyTheScenarioAsItAlwaysWas) {
  for (const std::string& name : layouts()) {
    Simulation legacy = *makeLayoutSimulation(name, 4);
    Simulation viaOptions = makeMapSimulation(*warehousesim::world::makePredefinedWarehouse(name),
                                              4, ScenarioOptions{});
    for (int i = 0; i < 300; ++i) {
      legacy.step();
      viaOptions.step();
      ASSERT_TRUE(legacy.snapshot() == viaOptions.snapshot()) << name << " tick " << i + 1;
    }
  }
}

TEST(ScenarioOptionsTest, AskingForTheNumberOfRobotsTheLayoutAlreadyGetsChangesNothing) {
  for (const std::string& name : layouts()) {
    Simulation legacy = *makeLayoutSimulation(name, 4);
    ScenarioOptions options;
    options.robots = legacy.robots().size();
    Simulation same = (*layoutScenario(name, options))(4);
    EXPECT_EQ(startsOf(legacy), startsOf(same)) << name;
    for (int i = 0; i < 200; ++i) {
      legacy.step();
      same.step();
    }
    EXPECT_TRUE(legacy.snapshot() == same.snapshot()) << name;
  }
}

// ---- the size of the fleet ----------------------------------------------------------------------

TEST(ScenarioOptionsTest, AFleetOfAnySizeTheLayoutHasRoomForIsExactlyThatBig) {
  for (const std::string& name : layouts()) {
    for (const std::size_t count : {1U, 2U, 3U, 4U, 5U, 6U, 8U, 12U}) {
      ScenarioOptions options;
      options.robots = count;
      const Simulation sim = (*layoutScenario(name, options))(1);
      ASSERT_EQ(sim.robots().size(), count) << name;
      for (std::size_t i = 0; i < count; ++i) {
        EXPECT_EQ(sim.robots()[i].id().value, i + 1);  // ids 1..N in order
      }
    }
  }
}

TEST(ScenarioOptionsTest, StartCellsAreDistinctOpenAndNeverNextToEachOtherOnTheFloor) {
  for (const std::string& name : layouts()) {
    ScenarioOptions options;
    options.robots = 8;
    const Simulation sim = (*layoutScenario(name, options))(1);
    const std::vector<Point> starts = startsOf(sim);
    EXPECT_EQ(std::set<Point>(starts.begin(), starts.end()).size(), starts.size()) << name;
    const auto chargers = sim.warehouse().cellsOfType(CellType::Charging);
    for (const Point cell : starts) {
      EXPECT_TRUE(sim.warehouse().isWalkable(cell)) << name;
    }
    // The ones not on a charger keep a cell between them (and between them and a charger).
    for (std::size_t i = 0; i < starts.size(); ++i) {
      for (std::size_t j = i + 1; j < starts.size(); ++j) {
        const bool bothChargers =
            std::find(chargers.begin(), chargers.end(), starts[i]) != chargers.end() &&
            std::find(chargers.begin(), chargers.end(), starts[j]) != chargers.end();
        if (!bothChargers) {
          EXPECT_GE(warehousesim::core::manhattanDistance(starts[i], starts[j]), 2)
              << name << " robots " << i + 1 << " and " << j + 1;
        }
      }
    }
  }
}

TEST(ScenarioOptionsTest, ChargingStationsComeFirstAndThenOpenFloor) {
  ScenarioOptions options;
  options.robots = 4;
  const Simulation compact = (*layoutScenario("compact", options))(1);  // two chargers
  const auto chargers = compact.warehouse().cellsOfType(CellType::Charging);
  ASSERT_EQ(chargers.size(), 2U);
  EXPECT_EQ(compact.robots()[0].position(), chargers[0]);
  EXPECT_EQ(compact.robots()[1].position(), chargers[1]);
  EXPECT_EQ(compact.warehouse().at(compact.robots()[2].position()), CellType::Free);
  EXPECT_EQ(compact.warehouse().at(compact.robots()[3].position()), CellType::Free);
}

TEST(ScenarioOptionsTest, AFleetBiggerThanThereIsRoomForIsRefusedWithTheReason) {
  // Three cells in a row. Robots are put on open floor first and keep a cell between them, so the
  // middle one takes the room that P and D would otherwise need.
  const auto cramped = layout({"P.D", "###"});
  ScenarioOptions options;
  options.robots = 3;
  try {
    (void)mapScenario(cramped, options);
    FAIL() << "should have been refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("room for only 1 robots, not 3"), std::string::npos)
        << error.what();
  }
}

TEST(ScenarioOptionsTest, ZeroRobotsAndMoreThanTheMaximumAreRefused) {
  ScenarioOptions options;
  options.robots = 0;
  EXPECT_THROW((void)layoutScenario("demo", options), std::invalid_argument);
  options.robots = kMaxScenarioRobots + 1;
  EXPECT_THROW((void)layoutScenario("demo", options), std::invalid_argument);
  options.robots = 1'000'000;
  EXPECT_THROW((void)layoutScenario("fulfilment", options), std::invalid_argument);
  options.robots = kMaxScenarioRobots;
  EXPECT_NO_THROW((void)layoutScenario("fulfilment", options));
}

TEST(ScenarioOptionsTest, ALayoutThatCannotHostASimulationIsStillRefusedWhateverTheOptions) {
  ScenarioOptions options;
  options.robots = 1;
  EXPECT_THROW((void)mapScenario(layout({"P..."}), options), std::invalid_argument);  // no dropoff
  EXPECT_THROW((void)mapScenario(layout({"...D"}), options),
               std::invalid_argument);  // nothing to collect
}

TEST(ScenarioOptionsTest, AnUnknownLayoutHasNoScenarioWhateverTheOptions) {
  ScenarioOptions options;
  options.robots = 2;
  EXPECT_FALSE(layoutScenario("nowhere", options).has_value());
}

TEST(ScenarioOptionsTest, ARefusedFleetIsRefusedWhenTheFactoryIsMadeNotWhenItIsFirstUsed) {
  ScenarioOptions options;
  options.robots = 500;
  EXPECT_THROW((void)mapScenario(layout({"P.D"}), options), std::invalid_argument);
}

// ---- the rate of tasks --------------------------------------------------------------------------

TEST(ScenarioOptionsTest, TheSpawnIntervalDecidesHowManyTasksAreMade) {
  const auto tasksAfter = [](std::optional<warehousesim::core::Tick> interval) {
    ScenarioOptions options;
    options.taskSpawnInterval = interval;
    Simulation sim = (*layoutScenario("demo", options))(1);
    sim.run(200);
    return sim.tasks().all().size();
  };
  EXPECT_EQ(tasksAfter(std::nullopt), 40U);  // every 5 ticks, from tick 0
  EXPECT_EQ(tasksAfter(5), 40U);
  EXPECT_EQ(tasksAfter(20), 10U);
  EXPECT_EQ(tasksAfter(100), 2U);
  EXPECT_EQ(tasksAfter(0), 0U);  // none
}

TEST(ScenarioOptionsTest, TheFactoryMakesTheSameScenarioForTheSameSeedAndAnotherForAnother) {
  ScenarioOptions options;
  options.robots = 6;
  const auto scenario = *layoutScenario("fulfilment", options);
  Simulation a = scenario(3);
  Simulation b = scenario(3);
  Simulation c = scenario(4);
  a.run(150);
  b.run(150);
  c.run(150);
  EXPECT_TRUE(a.snapshot() == b.snapshot());
  EXPECT_FALSE(a.snapshot() == c.snapshot());
  EXPECT_EQ(a.robots().size(), 6U);
}

TEST(ScenarioOptionsTest, MoreRobotsAreNeverSlowerToStartOnTheSameTaskStream) {
  // The task stream depends on the seed alone, not on the fleet, so fleets of different sizes
  // are asked to do the same work: that is what makes a comparison of them fair.
  const auto submitted = [](std::size_t robots) {
    ScenarioOptions options;
    options.robots = robots;
    Simulation sim = (*layoutScenario("demo", options))(9);
    sim.run(100);
    std::vector<std::pair<Point, Point>> stream;
    for (const auto& task : sim.tasks().all()) {
      stream.emplace_back(task.pickup, task.dropoff);
    }
    return stream;
  };
  EXPECT_EQ(submitted(1), submitted(4));
  EXPECT_EQ(submitted(4), submitted(8));
}

}  // namespace
