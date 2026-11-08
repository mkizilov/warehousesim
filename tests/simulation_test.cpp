#include "warehousesim/simulation/simulation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/world/layouts.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::PointHash;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Battery;
using warehousesim::robots::NavigationGoal;
using warehousesim::robots::RobotState;
using warehousesim::simulation::makeDemoSimulation;
using warehousesim::simulation::makeLayoutSimulation;
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
  EXPECT_THROW(Simulation(layout({".#."}), {P(1, 0)}), std::invalid_argument);
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
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Navigating);
  EXPECT_EQ(sim.robots()[0].goal(), NavigationGoal::Dropoff);

  sim.step();
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
  EXPECT_EQ(sim.tick(), 5U);
  EXPECT_EQ(sim.robots()[0].cellsTravelled(), 4U);
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
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
  EXPECT_EQ(sim.snapshot().failedTasks, 1U);
}

TEST(SimulationTest, UnreachableDropoffFailsTaskAfterPickup) {
  Simulation sim(layout({"..#."}), {P(0, 0)});
  const TaskId id = sim.submitTask(P(1, 0), P(3, 0));
  sim.run(3);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Failed);
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
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

TEST(SimulationTest, WorldOccupancyMatchesRobotPositionsEveryTick) {
  Simulation sim = makeDemoSimulation(11);
  for (int i = 0; i < 400; ++i) {
    sim.step();
    const auto& occupancy = sim.world().occupancy();
    ASSERT_EQ(occupancy.robotCount(), sim.robots().size());
    for (const auto& robot : sim.robots()) {
      ASSERT_EQ(occupancy.positionOf(robot.id()), robot.position());
      ASSERT_EQ(occupancy.occupantAt(robot.position()), robot.id());
      ASSERT_TRUE(sim.world().isWalkable(robot.position()));
    }
  }
}

TEST(SimulationTest, SpawnedTasksUseExplicitPickupCellsWhenTheLayoutHasThem) {
  SimulationConfig config;
  config.taskSpawnInterval = 1;
  Simulation sim(layout({"S.....D", "..P...."}), {P(3, 1)}, config);
  sim.run(40);
  ASSERT_FALSE(sim.tasks().all().empty());
  for (const auto& task : sim.tasks().all()) {
    EXPECT_EQ(task.pickup, P(2, 1));  // the Pickup cell, not an aisle beside the shelf
    EXPECT_EQ(task.dropoff, P(6, 0));
  }
  EXPECT_GT(sim.snapshot().completedTasks, 0U);
}

TEST(SimulationTest, PredefinedLayoutsRunWithoutFailuresOrOverlap) {
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    auto sim = makeLayoutSimulation(info.name, 3);
    ASSERT_TRUE(sim.has_value());
    for (int i = 0; i < 400; ++i) {
      sim->step();
      std::set<Point> positions;
      for (const auto& robot : sim->robots()) {
        ASSERT_TRUE(sim->world().isWalkable(robot.position()));
        positions.insert(robot.position());
      }
      ASSERT_EQ(positions.size(), sim->robots().size());
    }
    EXPECT_EQ(sim->snapshot().failedTasks, 0U);
    EXPECT_GT(sim->snapshot().completedTasks, 5U);
  }
}

TEST(SimulationTest, LayoutScenarioLookup) {
  EXPECT_FALSE(makeLayoutSimulation("nonexistent", 1).has_value());
  auto named = makeLayoutSimulation("demo", 4);
  ASSERT_TRUE(named.has_value());
  Simulation direct = makeDemoSimulation(4);
  named->run(50);
  direct.run(50);
  EXPECT_TRUE(named->snapshot() == direct.snapshot());
  auto compact = makeLayoutSimulation("compact", 1);
  ASSERT_TRUE(compact.has_value());
  EXPECT_EQ(compact->robots().size(), 2U);  // one robot per charging station, up to four
}

TEST(SimulationTest, SpawningDisabledWithoutShelvesOrStations) {
  SimulationConfig config;
  config.taskSpawnInterval = 1;
  Simulation sim(layout({"..."}), {P(0, 0)}, config);
  sim.run(10);
  EXPECT_TRUE(sim.tasks().all().empty());
}

// ---- robot state machine inside the simulation -------------------------------------------

TEST(SimulationRobotTest, ATaskTakesTheRobotThroughEveryStateInOrder) {
  SimulationConfig config;
  config.robot.loadTicks = 2;
  config.robot.unloadTicks = 1;
  Simulation sim(layout({"......"}), {P(0, 0)}, config);
  const TaskId id = sim.submitTask(P(2, 0), P(5, 0));

  using S = RobotState;
  std::vector<S> states;
  std::vector<TaskStatus> taskStates;
  for (int i = 0; i < 10; ++i) {
    sim.step();
    states.push_back(sim.robots()[0].state());
    taskStates.push_back(sim.tasks().find(id)->status);
  }
  // Two steps to the pickup, two ticks loading, three steps to the dropoff, one tick unloading.
  EXPECT_EQ(states,
            (std::vector<S>{S::Navigating, S::Navigating, S::Loading, S::Loading, S::Navigating,
                            S::Navigating, S::Navigating, S::Unloading, S::Idle, S::Idle}));
  using T = TaskStatus;
  EXPECT_EQ(taskStates, (std::vector<T>{T::Assigned, T::Assigned, T::Assigned, T::Assigned,
                                        T::InProgress, T::InProgress, T::InProgress, T::InProgress,
                                        T::Completed, T::Completed}));
  EXPECT_EQ(sim.robots()[0].position(), P(5, 0));
  EXPECT_FALSE(sim.robots()[0].task().has_value());
}

TEST(SimulationRobotTest, NavigatingRobotsReportTheirGoalAndLoadingRobotsStayPut) {
  SimulationConfig config;
  config.robot.loadTicks = 3;
  Simulation sim(layout({"......"}), {P(0, 0)}, config);
  (void)sim.submitTask(P(2, 0), P(5, 0));
  sim.step();
  EXPECT_EQ(sim.robots()[0].goal(), NavigationGoal::Pickup);
  EXPECT_EQ(sim.robots()[0].destination(), P(2, 0));
  sim.run(2);  // arrives and starts loading
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Loading);
  EXPECT_EQ(sim.robots()[0].goal(), NavigationGoal::None);
  EXPECT_EQ(sim.robots()[0].position(), P(2, 0));
  EXPECT_DOUBLE_EQ(sim.robots()[0].speed(), 0.0);
  sim.run(3);  // finishes loading and heads for the dropoff
  EXPECT_EQ(sim.robots()[0].goal(), NavigationGoal::Dropoff);
  EXPECT_EQ(sim.robots()[0].destination(), P(5, 0));
}

TEST(SimulationRobotTest, EnergyIsDrawnForMovingHandlingAndIdling) {
  SimulationConfig config;
  config.robot.loadTicks = 2;
  config.robot.unloadTicks = 1;
  config.robot.moveCostPerCell = 7;
  config.robot.handlingDrainPerTick = 2;
  config.robot.idleDrainPerTick = 1;
  Simulation sim(layout({"......"}), {P(0, 0)}, config);
  (void)sim.submitTask(P(2, 0), P(5, 0));
  sim.run(10);
  // 5 cells x 7, 3 handling ticks x 2, and the 2 ticks after the task during which it idles.
  EXPECT_EQ(sim.robots()[0].battery().level(), 10000U - (35U + 6U + 2U));
  EXPECT_EQ(sim.robots()[0].cellsTravelled(), 5U);
}

TEST(SimulationRobotTest, ASlowRobotMovesEveryOtherTickAndReportsItsSpeed) {
  SimulationConfig config;
  config.robot.maxSpeed = 0.5;
  Simulation sim(layout({"...."}), {P(0, 0)}, config);
  (void)sim.submitTask(P(2, 0), P(2, 0));

  std::vector<Point> positions;
  std::vector<double> speeds;
  for (int i = 0; i < 4; ++i) {
    sim.step();
    positions.push_back(sim.robots()[0].position());
    speeds.push_back(sim.robots()[0].speed());
  }
  EXPECT_EQ(positions, (std::vector<Point>{P(0, 0), P(1, 0), P(1, 0), P(2, 0)}));
  EXPECT_EQ(speeds, (std::vector<double>{0.5, 0.5, 0.5, 0.0}));  // stopped on arrival
  EXPECT_EQ(sim.robots()[0].heading(), warehousesim::core::Direction::East);
  sim.step();
  EXPECT_EQ(sim.tasks().find(TaskId{1})->status, TaskStatus::Completed);
}

TEST(SimulationRobotTest, SnapshotCarriesTheFullRobotPicture) {
  SimulationConfig config;
  config.robot.maxSpeed = 0.5;
  config.robot.batteryCapacity = 500;
  config.robot.moveCostPerCell = 10;
  Simulation sim(layout({"..."}), {P(0, 0)}, config);
  (void)sim.submitTask(P(2, 0), P(2, 0));
  sim.run(2);
  const auto robot = sim.snapshot().robots.at(0);
  EXPECT_EQ(robot.position, P(1, 0));
  EXPECT_EQ(robot.heading, warehousesim::core::Direction::East);
  EXPECT_EQ(robot.state, RobotState::Navigating);
  EXPECT_EQ(robot.goal, NavigationGoal::Pickup);
  EXPECT_DOUBLE_EQ(robot.speed, 0.5);
  EXPECT_EQ(robot.batteryLevel, 490U);
  EXPECT_EQ(robot.batteryCapacity, 500U);
  EXPECT_EQ(robot.task, TaskId{1});
}

TEST(SimulationRobotTest, ALowBatteryRobotGoesToChargeBeforeTakingWork) {
  SimulationConfig config;
  config.robot.moveCostPerCell = 10;
  config.robot.chargePerTick = 500;
  config.robot.lowBatteryLevel = 3000;
  config.robot.initialCharge = 2500;  // below the reserve
  Simulation sim(layout({"C...."}), {P(3, 0)}, config);
  const TaskId id = sim.submitTask(P(4, 0), P(4, 0));

  std::vector<RobotState> states;
  Battery::Units previous = 0;
  bool chargedMonotonically = true;
  for (int i = 0; i < 45; ++i) {
    sim.step();
    const auto& robot = sim.robots()[0];
    states.push_back(robot.state());
    if (robot.state() == RobotState::Charging) {
      EXPECT_EQ(robot.position(), P(0, 0));
      EXPECT_EQ(sim.tasks().find(id)->status,
                TaskStatus::Pending);  // not given to a charging robot
      chargedMonotonically = chargedMonotonically && robot.battery().level() >= previous;
    }
    previous = robot.battery().level();
  }
  EXPECT_EQ(states.front(), RobotState::Navigating);
  EXPECT_TRUE(chargedMonotonically);
  const auto firstCharging = std::ranges::find(states, RobotState::Charging);
  ASSERT_TRUE(firstCharging != states.end());
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);  // done once charged
  EXPECT_FALSE(sim.robots()[0].needsCharge());
}

TEST(SimulationRobotTest, ARobotThatStartsOnAChargerChargesWithoutMoving) {
  SimulationConfig config;
  config.robot.chargePerTick = 1000;
  config.robot.lowBatteryLevel = 5000;
  config.robot.initialCharge = 1000;
  Simulation sim(layout({"C.."}), {P(0, 0)}, config);
  sim.step();
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Charging);
  EXPECT_EQ(sim.robots()[0].position(), P(0, 0));
  EXPECT_EQ(sim.robots()[0].battery().level(), 2000U);  // 1000 + 1000 gained this tick
  sim.run(7);
  EXPECT_EQ(sim.robots()[0].battery().level(), 9000U);
  sim.run(2);  // reaches full on the first tick and stops charging on the next
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
  EXPECT_TRUE(sim.robots()[0].battery().isFull());
}

TEST(SimulationRobotTest, LowRobotsPickADifferentChargerWhenTheNearestIsTaken) {
  SimulationConfig config;
  config.robot.lowBatteryLevel = 5000;
  config.robot.initialCharge = 1000;
  Simulation sim(layout({"C.....C"}), {P(0, 0), P(2, 0)}, config);
  sim.run(8);
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Charging);
  EXPECT_EQ(sim.robots()[0].position(), P(0, 0));
  // Robot 2 is nearest to the charger at (0,0), but robot 1 is on it, so it goes to (6,0).
  EXPECT_EQ(sim.robots()[1].state(), RobotState::Charging);
  EXPECT_EQ(sim.robots()[1].position(), P(6, 0));
}

TEST(SimulationRobotTest, WithoutChargersTheLowBatteryReserveIsIgnored) {
  SimulationConfig config;
  config.robot.lowBatteryLevel = 5000;
  config.robot.initialCharge = 1000;
  Simulation sim(layout({"...."}), {P(0, 0)}, config);
  EXPECT_FALSE(sim.robots()[0].needsCharge());
  EXPECT_EQ(sim.robots()[0].spec().lowBatteryLevel, 0U);
  const TaskId id = sim.submitTask(P(1, 0), P(2, 0));
  sim.run(10);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

TEST(SimulationRobotTest, AFlatBatteryTakesTheRobotOutOfServiceAndReleasesItsTask) {
  SimulationConfig config;
  config.robot.batteryCapacity = 100;
  config.robot.moveCostPerCell = 40;
  Simulation sim(layout({"......"}), {P(0, 0)}, config);
  const TaskId id = sim.submitTask(P(5, 0), P(5, 0));
  sim.run(4);  // 100 -> 60 -> 20 -> 0 after three cells; the next turn notices
  const auto& robot = sim.robots()[0];
  EXPECT_EQ(robot.state(), RobotState::Unavailable);
  EXPECT_EQ(robot.position(), P(3, 0));
  EXPECT_FALSE(robot.task().has_value());
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Pending);  // available to someone else
  EXPECT_FALSE(sim.tasks().find(id)->assignee.has_value());
  sim.run(5);
  EXPECT_EQ(robot.state(), RobotState::Unavailable);  // and it stays that way
  EXPECT_TRUE(sim.world().isOccupied(P(3, 0)));       // still occupying its cell
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Pending);

  EXPECT_TRUE(sim.restoreRobot(RobotId{1}));  // an operator swaps the battery
  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_TRUE(robot.battery().isFull());
  sim.run(8);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

TEST(SimulationRobotTest, DisablingARobotReturnsAnUncollectedTaskAndFailsACarriedOne) {
  SimulationConfig config;
  Simulation sim(layout({"......."}), {P(0, 0), P(6, 0)}, config);
  const TaskId pickupBound = sim.submitTask(P(2, 0), P(3, 0));
  sim.step();
  ASSERT_EQ(sim.tasks().find(pickupBound)->assignee, RobotId{1});
  EXPECT_TRUE(sim.disableRobot(RobotId{1}));
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Unavailable);
  EXPECT_EQ(sim.tasks().find(pickupBound)->status, TaskStatus::Pending);
  EXPECT_FALSE(sim.disableRobot(RobotId{1}));  // already out of service
  sim.run(1);
  EXPECT_EQ(sim.tasks().find(pickupBound)->assignee, RobotId{2});  // the other robot takes over

  // A task that is already being carried cannot be handed over: it fails.
  Simulation carrying(layout({"......."}), {P(0, 0)}, config);
  const TaskId carried = carrying.submitTask(P(1, 0), P(6, 0));
  carrying.run(3);
  ASSERT_EQ(carrying.tasks().find(carried)->status, TaskStatus::InProgress);
  EXPECT_TRUE(carrying.disableRobot(RobotId{1}));
  EXPECT_EQ(carrying.tasks().find(carried)->status, TaskStatus::Failed);
  EXPECT_EQ(carrying.snapshot().failedTasks, 1U);
}

TEST(SimulationRobotTest, RestoringAnIdleRobotChangesNothingAndUnknownIdsThrow) {
  Simulation sim(layout({"..."}), {P(0, 0)});
  EXPECT_FALSE(sim.restoreRobot(RobotId{1}));
  EXPECT_THROW((void)sim.disableRobot(RobotId{9}), std::out_of_range);
  EXPECT_THROW((void)sim.restoreRobot(RobotId{}), std::out_of_range);
  EXPECT_TRUE(sim.disableRobot(RobotId{1}));
  EXPECT_TRUE(sim.restoreRobot(RobotId{1}));
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
}

TEST(SimulationRobotTest, OtherRobotsRouteAroundAnUnavailableOne) {
  Simulation sim(layout({"......", "......"}), {P(2, 0), P(0, 0)});
  ASSERT_TRUE(sim.disableRobot(RobotId{1}));
  const TaskId id = sim.submitTask(P(5, 0), P(5, 1));
  sim.run(40);
  EXPECT_EQ(sim.robots()[0].position(), P(2, 0));  // never pushed: it is out of service
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Unavailable);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

TEST(SimulationRobotTest, RobotsBusyInPlaceAreNeverPushedAside) {
  SimulationConfig config;
  config.robot.loadTicks = 6;
  Simulation sim(layout({"........"}), {P(2, 0), P(0, 0)}, config);
  (void)sim.submitTask(P(2, 0), P(7, 0));  // robot 1 loads right where it stands...
  (void)sim.submitTask(P(5, 0), P(6, 0));  // ...while robot 2's way east is blocked by it
  for (int i = 0; i < 6; ++i) {
    sim.step();
    EXPECT_EQ(sim.robots()[0].state(), RobotState::Loading);
    EXPECT_EQ(sim.robots()[0].position(), P(2, 0));
  }
  EXPECT_EQ(sim.robots()[1].position(), P(1, 0));  // stuck behind it for the whole time
  EXPECT_EQ(sim.robots()[1].state(), RobotState::Navigating);
  EXPECT_TRUE(sim.robots()[1].blockedTicks() > 0);
  sim.step();
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Navigating);  // loading is over, it moves on
}

TEST(SimulationRobotTest, AnInvalidRobotSpecIsRejectedWhenTheSimulationIsBuilt) {
  SimulationConfig config;
  config.robot.maxSpeed = 2.0;
  EXPECT_THROW(Simulation(layout({"..."}), {P(0, 0)}, config), std::invalid_argument);
  config = SimulationConfig{};
  config.robot.chargePerTick = 0;
  EXPECT_THROW(Simulation(layout({"..."}), {P(0, 0)}, config), std::invalid_argument);
}

// Cross-checks robots, tasks and the world against each other after every tick.
void expectWorldConsistent(const Simulation& sim) {
  std::set<std::uint32_t> heldTasks;
  for (const auto& robot : sim.robots()) {
    EXPECT_EQ(robot.inconsistency().value_or("consistent"), "consistent");
    EXPECT_EQ(sim.world().occupancy().positionOf(robot.id()), robot.position());
    EXPECT_TRUE(sim.world().isWalkable(robot.position()));
    EXPECT_TRUE(robot.battery().level() <= robot.battery().capacity());
    if (robot.state() == RobotState::Charging) {
      EXPECT_EQ(sim.warehouse().at(robot.position()), warehousesim::world::CellType::Charging);
    }
    if (!robot.task()) {
      continue;
    }
    EXPECT_TRUE(heldTasks.insert(robot.task()->value).second);  // no task is held twice
    const auto* task = sim.tasks().find(*robot.task());
    ASSERT_TRUE(task != nullptr);
    EXPECT_EQ(task->assignee, robot.id());
    const bool beforePickup =
        robot.state() == RobotState::Loading ||
        (robot.state() == RobotState::Navigating && robot.goal() == NavigationGoal::Pickup);
    EXPECT_EQ(task->status, beforePickup ? TaskStatus::Assigned : TaskStatus::InProgress);
  }
  for (const auto& task : sim.tasks().all()) {
    const bool open = task.status == TaskStatus::Assigned || task.status == TaskStatus::InProgress;
    EXPECT_EQ(open, heldTasks.contains(task.id.value));  // every open task has exactly one holder
  }
}

TEST(SimulationRobotTest, RobotsTasksAndTheWorldStayConsistentOnEveryLayout) {
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    for (std::uint64_t seed = 1; seed <= 3; ++seed) {
      auto sim = makeLayoutSimulation(info.name, seed);
      ASSERT_TRUE(sim.has_value());
      for (int i = 0; i < 700; ++i) {
        sim->step();
        expectWorldConsistent(*sim);
      }
    }
  }
}

TEST(SimulationRobotTest, RealisticScenariosKeepRobotsChargedWorkingAndInService) {
  for (const char* name : {"compact", "fulfilment"}) {
    for (std::uint64_t seed = 1; seed <= 2; ++seed) {
      auto sim = makeLayoutSimulation(name, seed);
      ASSERT_TRUE(sim.has_value());
      std::set<RobotState> seen;
      Battery::Units lowest = 10000;
      for (int i = 0; i < 3000; ++i) {
        sim->step();
        for (const auto& robot : sim->robots()) {
          seen.insert(robot.state());
          lowest = std::min(lowest, robot.battery().level());
        }
      }
      EXPECT_EQ(seen.count(RobotState::Unavailable), 0U);  // nobody ran flat
      EXPECT_TRUE(seen.contains(RobotState::Charging));
      EXPECT_TRUE(seen.contains(RobotState::Loading));
      EXPECT_TRUE(seen.contains(RobotState::Unloading));
      EXPECT_TRUE(seen.contains(RobotState::Navigating));
      EXPECT_GT(lowest, 0U);
      EXPECT_EQ(sim->snapshot().failedTasks, 0U);
      EXPECT_GT(sim->snapshot().completedTasks, 40U);
    }
  }
}

TEST(SimulationRobotTest, BatteryAndStateAreDeterministicForAGivenSeed) {
  auto a = makeLayoutSimulation("fulfilment", 9);
  auto b = makeLayoutSimulation("fulfilment", 9);
  auto c = makeLayoutSimulation("fulfilment", 10);
  ASSERT_TRUE(a && b && c);
  for (int i = 0; i < 1200; ++i) {
    a->step();
    b->step();
    c->step();
    ASSERT_TRUE(a->snapshot() == b->snapshot());
  }
  EXPECT_FALSE(a->snapshot() == c->snapshot());
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
  EXPECT_EQ(json["robots"][0]["state"].get<std::string>(), "navigating");
  EXPECT_EQ(json["robots"][0]["goal"].get<std::string>(), "pickup");
  EXPECT_EQ(json["robots"][0]["heading"].get<std::string>(), "east");
  EXPECT_DOUBLE_EQ(json["robots"][0]["speed"].get<double>(), 1.0);
  EXPECT_EQ(json["robots"][0]["battery"]["level"].get<int>(), 10000);
  EXPECT_EQ(json["robots"][0]["battery"]["capacity"].get<int>(), 10000);
  EXPECT_EQ(json["robots"][0]["task"].get<int>(), 1);
  EXPECT_EQ(json["robots"][0]["position"]["x"].get<int>(), 1);
  ASSERT_EQ(json["activeTasks"].size(), 1U);
  EXPECT_EQ(json["activeTasks"][0]["status"].get<std::string>(), "assigned");
  EXPECT_EQ(json["completedTasks"].get<int>(), 0);
}

}  // namespace
