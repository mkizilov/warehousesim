#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/world/layout_check.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using namespace warehousesim_test;
using warehousesim::simulation::makeDemoSimulation;
using warehousesim::simulation::makeLayoutSimulation;
using warehousesim::simulation::makeMapSimulation;

std::vector<Point> startsOf(const Simulation& sim) {
  std::vector<Point> starts;
  for (const Robot& robot : sim.robots()) {
    starts.push_back(robot.position());
  }
  return starts;
}

std::string json(const Simulation& sim) {
  return warehousesim::simulation::toJson(sim.snapshot()).dump();
}

TEST(MapSimulationTest, ThePredefinedLayoutsStartTheirRobotsWhereTheyAlwaysDid) {
  EXPECT_EQ(startsOf(makeDemoSimulation(1)),
            (std::vector<Point>{P(0, 0), P(2, 0), P(4, 0), P(6, 0)}));
  for (const char* name : {"compact", "fulfilment"}) {
    const auto sim = makeLayoutSimulation(name, 1);
    ASSERT_TRUE(sim.has_value());
    const auto chargers = warehousesim::world::makePredefinedWarehouse(name)->cellsOfType(
        warehousesim::world::CellType::Charging);
    const std::size_t robots = std::min<std::size_t>(chargers.size(), 4);
    ASSERT_GT(robots, 0U);
    EXPECT_EQ(startsOf(*sim),
              (std::vector<Point>(chargers.begin(),
                                  chargers.begin() + static_cast<std::ptrdiff_t>(robots))))
        << name;
  }
}

TEST(MapSimulationTest, TheDemoSimulationIsTheDemoWarehouseRunAsAnyOtherMap) {
  Simulation a = makeDemoSimulation(5);
  Simulation b = makeMapSimulation(warehousesim::world::makeDemoWarehouse(), 5);
  for (int i = 0; i < 300; ++i) {
    a.step();
    b.step();
  }
  EXPECT_EQ(json(a), json(b));
}

TEST(MapSimulationTest, UnknownLayoutNamesAreStillNotFound) {
  EXPECT_FALSE(makeLayoutSimulation("nowhere", 1).has_value());
  EXPECT_FALSE(makeLayoutSimulation("", 1).has_value());
}

TEST(MapSimulationTest, AMapWithoutChargersStartsRobotsOnOpenFloorSpacedApart) {
  const Simulation sim =
      makeMapSimulation(layout({"P.........D", "...........", "SSSSSSSSSSS"}), 1);
  EXPECT_EQ(startsOf(sim), (std::vector<Point>{P(1, 0), P(3, 0), P(5, 0), P(7, 0)}));
}

TEST(MapSimulationTest, AMapWithChargersStartsUpToFourRobotsOnThemAndGivesThemABattery) {
  const Warehouse w = layout({"P......D", "CCCCCC.."});
  Simulation sim = makeMapSimulation(w, 1);
  EXPECT_EQ(startsOf(sim), (std::vector<Point>{P(0, 1), P(1, 1), P(2, 1), P(3, 1)}));
  // With an energy model robots lose charge as they idle and drive.
  bool drained = false;
  for (int i = 0; i < 200; ++i) {
    sim.step();
    for (const auto& robot : sim.snapshot().robots) {
      drained = drained || robot.batteryLevel < robot.batteryCapacity;
    }
  }
  EXPECT_TRUE(drained);
}

TEST(MapSimulationTest, WithoutChargersRobotsHaveNoBatteryModelAndNeverRunDown) {
  Simulation sim = makeMapSimulation(layout({"P.........D", "..........."}), 1);
  for (int i = 0; i < 400; ++i) {
    sim.step();
  }
  for (const auto& robot : sim.snapshot().robots) {
    EXPECT_EQ(robot.batteryLevel, robot.batteryCapacity);
  }
}

TEST(MapSimulationTest, ACrampedMapGetsAsManyRobotsAsFitWithoutTouching) {
  EXPECT_EQ(startsOf(makeMapSimulation(layout({"PD"}), 1)).size(), 1U);
  EXPECT_EQ(startsOf(makeMapSimulation(layout({"P.D"}), 1)), (std::vector<Point>{P(1, 0)}));
  EXPECT_EQ(startsOf(makeMapSimulation(layout({"P.....D"}), 1)),
            (std::vector<Point>{P(1, 0), P(3, 0), P(5, 0)}));
}

TEST(MapSimulationTest, AMapWithOnlyPickupAndDropoffCellsStillStartsARobot) {
  Simulation sim = makeMapSimulation(layout({"PD"}), 1);
  ASSERT_EQ(sim.robots().size(), 1U);
  EXPECT_EQ(sim.robots()[0].position(), P(0, 0));
  for (int i = 0; i < 300; ++i) {
    sim.step();
  }
  EXPECT_GT(sim.snapshot().completedTasks, 0U);
  EXPECT_EQ(sim.snapshot().failedTasks, 0U);
}

TEST(MapSimulationTest, LayoutsThatCannotHostASimulationAreRefusedWithTheReason) {
  const auto message = [](const Warehouse& w) {
    try {
      (void)makeMapSimulation(w, 1);
    } catch (const std::invalid_argument& error) {
      return std::string(error.what());
    }
    return std::string();
  };
  EXPECT_TRUE(message(layout({"###", "#S#"})).find("no walkable cell") != std::string::npos);
  EXPECT_TRUE(message(layout({"P..."})).find("no dropoff") != std::string::npos);
  EXPECT_TRUE(message(layout({"...D"})).find("no pickup") != std::string::npos);
  EXPECT_TRUE(message(Warehouse(4, 4)).find("cannot host a simulation") != std::string::npos);
  EXPECT_TRUE(message(layout({"P.D"})).empty());
}

TEST(MapSimulationTest, ALayoutWithSeparateRegionsIsAcceptedAndRuns) {
  Simulation sim = makeMapSimulation(layout({"P.#.D", "..#..", "..#.."}), 3);
  TrafficAudit audit(sim);
  for (int i = 0; i < 300; ++i) {
    sim.step();
    audit.check(sim);
  }
  SUCCEED();
}

TEST(MapSimulationTest, AMapReadFromAFileRunsLikeTheSameLayoutBuiltInCode) {
  const Warehouse built = layout({"P.....D", ".SSSSS.", "......."});
  const auto load =
      warehousesim::mapio::parseMap(warehousesim::mapio::serializeMap({"hall", built}));
  ASSERT_TRUE(load.ok()) << load.describe();
  Simulation fromFile = makeMapSimulation(load.document->layout, 9);
  Simulation fromCode = makeMapSimulation(built, 9);
  for (int i = 0; i < 400; ++i) {
    fromFile.step();
    fromCode.step();
  }
  EXPECT_EQ(json(fromFile), json(fromCode));
  EXPECT_GT(fromFile.snapshot().completedTasks, 0U);
}

TEST(MapSimulationTest, TheSameMapAndSeedGiveTheSameRunAndAnotherSeedAnotherOne) {
  const Warehouse w = layout({"P......D", "..SSSS..", "........", "C......."});
  Simulation a = makeMapSimulation(w, 1);
  Simulation b = makeMapSimulation(w, 1);
  Simulation c = makeMapSimulation(w, 2);
  for (int i = 0; i < 300; ++i) {
    a.step();
    b.step();
    c.step();
  }
  EXPECT_EQ(json(a), json(b));
  EXPECT_NE(json(a), json(c));
}

TEST(MapSimulationTest, EveryEditedMapOfAnyShapeThatIsReadyRunsWithoutBreakingAnyRule) {
  for (const auto& rows : std::vector<std::vector<std::string>>{{"PD"},
                                                                {"P", "D"},
                                                                {"P.", ".D"},
                                                                {"D.P", "...", "C.."},
                                                                {"#P#", "#.#", "#D#"},
                                                                {"PSSSD"},
                                                                {"P.S.D", "..S..", "C...."}}) {
    auto parsed = Warehouse::fromRows(rows);
    ASSERT_TRUE(parsed.has_value());
    if (warehousesim::world::blocksSimulation(warehousesim::world::checkLayout(*parsed))) {
      EXPECT_THROW((void)makeMapSimulation(*parsed, 1), std::invalid_argument);
      continue;
    }
    Simulation sim = makeMapSimulation(*parsed, 1);
    TrafficAudit audit(sim);
    for (int i = 0; i < 200; ++i) {
      sim.step();
      audit.check(sim);
    }
  }
}

}  // namespace
