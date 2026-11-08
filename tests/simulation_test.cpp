#include "warehousesim/simulation/simulation.hpp"

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::PointHash;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::RobotStatus;
using warehousesim::simulation::makeDemoSimulation;
using warehousesim::simulation::Simulation;
using warehousesim::simulation::SimulationConfig;
using warehousesim::simulation::toJson;
using warehousesim::tasks::TaskStatus;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

TEST(SimulationTest, RejectsInvalidRobotStarts) {
  EXPECT_THROW(Simulation(layout({".S."}), {P(1, 0)}), std::invalid_argument);
  EXPECT_THROW(Simulation(layout({"..."}), {P(5, 0)}), std::invalid_argument);
  EXPECT_THROW(Simulation(layout({"..."}), {P(0, 0), P(0, 0)}), std::invalid_argument);
}

TEST(SimulationTest, RejectsTasksOnNonWalkableCells) {
  Simulation sim(layout({".S."}), {P(0, 0)});
  EXPECT_THROW((void)sim.submitTask(P(1, 0), P(2, 0)), std::invalid_argument);
  EXPECT_THROW((void)sim.submitTask(P(0, 0), P(9, 0)), std::invalid_argument);
}

TEST(SimulationTest, RobotsGetSequentialIds) {
  const Simulation sim(layout({"..."}), {P(2, 0), P(0, 0)});
  ASSERT_EQ(sim.robots().size(), 2U);
  EXPECT_EQ(sim.robots()[0].id(), RobotId{1});
  EXPECT_EQ(sim.robots()[0].position(), P(2, 0));
  EXPECT_EQ(sim.robots()[1].id(), RobotId{2});
}

TEST(SimulationTest, SingleRobotCompletesTaskInExpectedTicks) {
  Simulation sim(layout({"....."}), {P(0, 0)});
  const TaskId id = sim.submitTask(P(2, 0), P(4, 0));

  sim.run(4);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::InProgress);
  EXPECT_EQ(sim.robots()[0].position(), P(4, 0));
  EXPECT_EQ(sim.robots()[0].status(), RobotStatus::ToDropoff);

  sim.step();
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.robots()[0].status(), RobotStatus::Idle);
  EXPECT_EQ(sim.tick(), 5U);
  EXPECT_EQ(sim.robots()[0].distanceTravelled(), 4U);
}

TEST(SimulationTest, TaskWithPickupEqualToDropoffCompletes) {
  Simulation sim(layout({"..."}), {P(1, 0)});
  const TaskId id = sim.submitTask(P(1, 0), P(1, 0));
  sim.step();
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

TEST(SimulationTest, UnreachablePickupFailsTaskAndKeepsRobotIdle) {
  Simulation sim(layout({".#."}), {P(0, 0)});
  const TaskId id = sim.submitTask(P(2, 0), P(0, 0));
  sim.step();
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Failed);
  EXPECT_EQ(sim.robots()[0].status(), RobotStatus::Idle);
  EXPECT_EQ(sim.snapshot().failedTasks, 1U);
}

TEST(SimulationTest, UnreachableDropoffFailsTaskAfterPickup) {
  Simulation sim(layout({"..#."}), {P(0, 0)});
  const TaskId id = sim.submitTask(P(1, 0), P(3, 0));
  sim.run(3);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Failed);
  EXPECT_EQ(sim.robots()[0].status(), RobotStatus::Idle);
}

TEST(SimulationTest, TwoRobotsShareWorkWithoutOverlapping) {
  Simulation sim(layout({"......", "......"}), {P(0, 0), P(5, 1)});
  const TaskId left = sim.submitTask(P(1, 0), P(0, 1));
  const TaskId right = sim.submitTask(P(4, 1), P(5, 0));
  sim.step();
  EXPECT_EQ(sim.tasks().find(left)->assignee, RobotId{1});
  EXPECT_EQ(sim.tasks().find(right)->assignee, RobotId{2});
  sim.run(20);
  EXPECT_EQ(sim.snapshot().completedTasks, 2U);
}

TEST(SimulationTest, IdleRobotInTheWayStepsAside) {
  Simulation sim(layout({"......", "......"}), {P(0, 0), P(1, 0)});
  // Robot 1 is busy; robot 2 sits idle on its straight-line route.
  const TaskId id = sim.submitTask(P(4, 0), P(5, 0));
  sim.run(40);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.tasks().find(id)->assignee, RobotId{2});
}

TEST(SimulationTest, BusyRobotsNeverShareACell) {
  SimulationConfig config;
  config.seed = 5;
  config.taskSpawnInterval = 2;
  Simulation sim(layout({"........", "........", "........", "S.S.S.D."}),
                 {P(0, 0), P(1, 0), P(2, 0), P(3, 0), P(4, 0)}, config);
  for (int i = 0; i < 300; ++i) {
    sim.step();
    std::set<Point> positions;
    for (const auto& robot : sim.robots()) {
      EXPECT_TRUE(sim.warehouse().isWalkable(robot.position()));
      positions.insert(robot.position());
    }
    ASSERT_EQ(positions.size(), sim.robots().size());
  }
  EXPECT_GT(sim.snapshot().completedTasks, 0U);
}

TEST(SimulationTest, DemoScenarioCompletesTasksAndAccountsForEveryTask) {
  Simulation sim = makeDemoSimulation(1);
  sim.run(400);
  const auto snapshot = sim.snapshot();
  EXPECT_GT(snapshot.completedTasks, 10U);
  EXPECT_EQ(snapshot.completedTasks + snapshot.failedTasks + snapshot.activeTasks.size(),
            sim.tasks().all().size());
  EXPECT_EQ(snapshot.failedTasks, 0U);
}

TEST(SimulationTest, DemoScenarioNeverDeadlocks) {
  // Regression: head-on standoffs near the single-cell stations used to livelock.
  for (std::uint64_t seed = 1; seed <= 6; ++seed) {
    Simulation sim = makeDemoSimulation(seed);
    sim.run(1500);
    const std::size_t before = sim.snapshot().completedTasks;
    sim.run(500);
    EXPECT_GT(sim.snapshot().completedTasks, before + 30U);
  }
}

TEST(SimulationTest, SameSeedIsDeterministic) {
  Simulation a = makeDemoSimulation(123);
  Simulation b = makeDemoSimulation(123);
  for (int i = 0; i < 300; ++i) {
    a.step();
    b.step();
    ASSERT_TRUE(a.snapshot() == b.snapshot());
  }
  EXPECT_EQ(toJson(a.snapshot()).dump(), toJson(b.snapshot()).dump());
}

TEST(SimulationTest, DifferentSeedsProduceDifferentRuns) {
  Simulation a = makeDemoSimulation(1);
  Simulation b = makeDemoSimulation(2);
  a.run(100);
  b.run(100);
  EXPECT_FALSE(a.snapshot() == b.snapshot());
}

TEST(SimulationTest, SpawningDisabledWithoutShelvesOrStations) {
  SimulationConfig config;
  config.taskSpawnInterval = 1;
  Simulation sim(layout({"..."}), {P(0, 0)}, config);
  sim.run(10);
  EXPECT_TRUE(sim.tasks().all().empty());
}

TEST(SerializationTest, WarehouseRoundTrips) {
  const Warehouse original = warehousesim::world::makeDemoWarehouse();
  const auto json = toJson(original);
  EXPECT_EQ(json["width"].get<int>(), 20);
  EXPECT_EQ(json["height"].get<int>(), 12);
  const Warehouse parsed = warehousesim::simulation::warehouseFromJson(json);
  EXPECT_EQ(parsed.toRows(), original.toRows());
}

TEST(SerializationTest, WarehouseFromTextParsesAndRejectsBadDocuments) {
  using warehousesim::simulation::warehouseFromJson;
  const auto ok = nlohmann::json::parse(R"({"rows": ["S.", ".D"]})");
  EXPECT_EQ(warehouseFromJson(ok).width(), 2);

  EXPECT_THROW((void)warehouseFromJson(nlohmann::json::parse(R"({})")), std::invalid_argument);
  EXPECT_THROW((void)warehouseFromJson(nlohmann::json::parse(R"({"rows": "S."})")),
               std::invalid_argument);
  EXPECT_THROW((void)warehouseFromJson(nlohmann::json::parse(R"({"rows": ["..", "."]})")),
               std::invalid_argument);
  EXPECT_THROW((void)warehouseFromJson(nlohmann::json::parse(R"({"rows": ["?"]})")),
               std::invalid_argument);
  EXPECT_THROW((void)warehouseFromJson(nlohmann::json::parse(R"({"width": 9, "rows": ["."]})")),
               std::invalid_argument);
}

TEST(SerializationTest, SnapshotJsonHasStableShape) {
  Simulation sim(layout({"...."}), {P(0, 0)});
  (void)sim.submitTask(P(2, 0), P(3, 0));
  sim.step();
  const auto json = toJson(sim.snapshot());
  EXPECT_EQ(json["tick"].get<int>(), 1);
  ASSERT_EQ(json["robots"].size(), 1U);
  EXPECT_EQ(json["robots"][0]["status"].get<std::string>(), "to_pickup");
  EXPECT_EQ(json["robots"][0]["task"].get<int>(), 1);
  EXPECT_EQ(json["robots"][0]["position"]["x"].get<int>(), 1);
  ASSERT_EQ(json["activeTasks"].size(), 1U);
  EXPECT_EQ(json["activeTasks"][0]["status"].get<std::string>(), "assigned");
  EXPECT_EQ(json["completedTasks"].get<int>(), 0);
}

}  // namespace
