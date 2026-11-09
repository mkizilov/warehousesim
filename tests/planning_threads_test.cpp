// Planning on several threads must change how long a step takes and nothing else: the state after
// any number of steps, tick by tick, is the one a single thread gives, and recordings made either
// way replay either way.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "recording_support.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/simulation/replay.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

namespace sim = warehousesim::simulation;
using sim::SimulationRunner;
using std::chrono::milliseconds;
using warehousesim::core::Point;

constexpr milliseconds kTimeout{20000};

// A fleet of `robots` on a bundled layout, planning on `workers` threads besides the stepping one.
sim::Simulation fleet(const std::string& layout, std::size_t robots, std::uint64_t spawn,
                      std::uint64_t seed, std::size_t workers, std::size_t minBatch = 1) {
  sim::ScenarioOptions options;
  options.robots = robots;
  options.taskSpawnInterval = spawn;
  sim::Simulation simulation = (*sim::layoutScenario(layout, options))(seed);
  simulation.usePlanningThreads(workers, minBatch);
  return simulation;
}

struct Case {
  const char* layout;
  std::size_t robots;
  std::uint64_t spawn;
  std::uint64_t seed;
};

// The free cell that the most routes still ahead of the robots pass through, if any can be built
// on.
std::optional<Point> busiestFreeCell(const sim::Simulation& simulation) {
  const auto size = simulation.warehouse().size();
  std::vector<std::uint32_t> use(size.cellCount(), 0);
  for (const auto& robot : simulation.robots()) {
    if (robot.state() != warehousesim::robots::RobotState::Navigating) {
      continue;
    }
    for (std::size_t i = robot.authority(); i + 1 < robot.path().size(); ++i) {
      ++use[size.indexOf(robot.path()[i])];
    }
  }
  std::optional<std::size_t> best;
  for (std::size_t i = 0; i < use.size(); ++i) {
    if (use[i] > 0 &&
        simulation.warehouse().at(size.pointAt(i)) == warehousesim::world::CellType::Free &&
        !simulation.world().isOccupied(size.pointAt(i)) && (!best || use[i] > use[*best])) {
      best = i;
    }
  }
  return best ? std::optional<Point>(size.pointAt(*best)) : std::nullopt;
}

// ---- the run is the same on every number of threads ---------------------------------------------

TEST(PlanningThreadsTest, EveryTickOfARunIsTheSameOnAnyNumberOfPlanningThreads) {
  for (const Case& c :
       {Case{"compact", 12, 2, 5}, Case{"fulfilment", 24, 1, 3}, Case{"demo", 4, 5, 1}}) {
    for (const std::size_t workers : {1U, 2U, 3U}) {
      sim::Simulation single = fleet(c.layout, c.robots, c.spawn, c.seed, 0);
      sim::Simulation threaded = fleet(c.layout, c.robots, c.spawn, c.seed, workers);
      for (int tick = 0; tick < 250; ++tick) {
        single.step();
        threaded.step();
        ASSERT_TRUE(single.snapshot() == threaded.snapshot())
            << c.layout << " x" << c.robots << ", " << workers << " threads, tick " << tick + 1;
        ASSERT_EQ(single.stateChecksum(), threaded.stateChecksum())
            << c.layout << ", " << workers << " threads, tick " << tick + 1;
      }
      ASSERT_TRUE(threaded.batchStats().has_value());
      EXPECT_GT(threaded.batchStats()->batches, 0U) << "the threaded run really asked for batches";
      EXPECT_EQ(threaded.planningThreads(), workers);
    }
  }
}

TEST(PlanningThreadsTest, AHundredRobotsOnTheLargeHallComeOutTheSameOnEveryThreadCount) {
  std::optional<std::uint64_t> checksum;
  std::optional<sim::Snapshot> reference;
  for (const std::size_t workers : {0U, 1U, 2U}) {
    for (const std::size_t minBatch : {1U, 16U}) {
      sim::Simulation simulation = fleet("large", 100, 1, 1, workers, minBatch);
      simulation.run(400);
      if (!checksum) {
        checksum = simulation.stateChecksum();
        reference = simulation.snapshot();
        EXPECT_GT(reference->completedTasks, 0U) << "the fleet got some work done";
        EXPECT_EQ(reference->robots.size(), 100U);
      }
      EXPECT_EQ(simulation.stateChecksum(), *checksum)
          << workers << " threads, batches from " << minBatch;
      EXPECT_TRUE(simulation.snapshot() == *reference)
          << workers << " threads, batches from " << minBatch;
      if (workers > 0 && minBatch == 1) {
        EXPECT_GT(simulation.batchStats()->parallelBatches, 0U);
      }
    }
  }
}

TEST(PlanningThreadsTest, ObstaclesPlacedAndClearedMidRunRepairRoutesTheSameWayOnAnyThreadCount) {
  sim::Simulation single = fleet("fulfilment", 24, 1, 3, 0);
  sim::Simulation threaded = fleet("fulfilment", 24, 1, 3, 2);
  int placed = 0;
  std::optional<Point> wall;
  for (int tick = 1; tick <= 220; ++tick) {
    if (tick % 40 == 0) {
      const std::optional<Point> cell = busiestFreeCell(single);
      ASSERT_TRUE(cell.has_value());
      ASSERT_EQ(busiestFreeCell(threaded), cell) << "the two runs agree on where the traffic is";
      const bool a = single.placeObstacle(*cell);
      const bool b = threaded.placeObstacle(*cell);
      ASSERT_EQ(a, b);
      if (a) {
        wall = cell;
        ++placed;
      }
    } else if (tick % 40 == 5 && wall) {
      ASSERT_EQ(single.clearObstacle(*wall), threaded.clearObstacle(*wall));
      wall.reset();
    }
    single.step();
    threaded.step();
    ASSERT_TRUE(single.snapshot() == threaded.snapshot()) << "tick " << tick;
    ASSERT_EQ(single.stateChecksum(), threaded.stateChecksum()) << "tick " << tick;
  }
  EXPECT_GT(placed, 2);
  EXPECT_GT(single.planningStats().obstacleRepairs, 0U) << "some route really was repaired";
  EXPECT_TRUE(single.planningStats() == threaded.planningStats());
}

TEST(PlanningThreadsTest, TheNumberOfThreadsCanChangeInTheMiddleOfARunAndNothingNoticesIt) {
  sim::Simulation steady = fleet("compact", 16, 1, 7, 0);
  sim::Simulation changing = fleet("compact", 16, 1, 7, 0);
  const std::size_t plan[] = {0, 2, 3, 0, 1, 2, 0};
  for (const std::size_t workers : plan) {
    changing.usePlanningThreads(workers, 1);
    EXPECT_EQ(changing.planningThreads(), workers);
    for (int tick = 0; tick < 40; ++tick) {
      steady.step();
      changing.step();
    }
    ASSERT_TRUE(steady.snapshot() == changing.snapshot()) << "after the switch to " << workers;
    ASSERT_EQ(steady.stateChecksum(), changing.stateChecksum());
  }
}

TEST(PlanningThreadsTest, ASimulationThatIsMovedKeepsItsThreadsAndItsRun) {
  sim::Simulation single = fleet("compact", 12, 1, 2, 0);
  sim::Simulation first = fleet("compact", 12, 1, 2, 2);
  single.run(30);
  first.run(30);
  sim::Simulation moved = std::move(first);
  EXPECT_EQ(moved.planningThreads(), 2U);
  single.run(30);
  moved.run(30);
  EXPECT_TRUE(single.snapshot() == moved.snapshot());
  EXPECT_EQ(single.stateChecksum(), moved.stateChecksum());
}

// ---- routes repaired after the layout changes ---------------------------------------------------

// Two robots whose routes both run through (3, 1). When that cell becomes a wall one of them has a
// way round, along the bottom row, and the other, bound for the pocket above (3, 1), has none: its
// task goes back to the queue. Each robot gets the answer to its own question.
TEST(PlanningThreadsTest, RepairedRoutesGoToTheRobotsTheyWereAskedFor) {
  const auto hall = *warehousesim::world::Warehouse::fromRows({"###P###", ".......", "......."});
  for (const bool pocketRobotFirst : {false, true}) {
    for (const std::size_t workers : {0U, 2U}) {
      // The task that comes first goes to the robot on the middle row, the other to the one below.
      const std::vector<Point> starts = pocketRobotFirst
                                            ? std::vector<Point>{Point{0, 2}, Point{0, 1}}
                                            : std::vector<Point>{Point{0, 1}, Point{0, 2}};
      sim::Simulation sim(hall, starts);
      sim.usePlanningThreads(workers, 1);
      const auto along = sim.submitTask(Point{6, 1}, Point{6, 2});
      const auto pocket = sim.submitTask(Point{3, 0}, Point{6, 2});
      sim.step();
      const std::size_t alongRobot = pocketRobotFirst ? 1 : 0;
      const std::size_t pocketRobot = 1 - alongRobot;
      ASSERT_TRUE(sim.robots()[alongRobot].task() == along);
      ASSERT_TRUE(sim.robots()[pocketRobot].task() == pocket);

      ASSERT_TRUE(sim.placeObstacle(Point{3, 1}));
      EXPECT_EQ(sim.planningStats().obstacleRepairs, 2U);
      const auto& rerouted = sim.robots()[alongRobot];
      EXPECT_EQ(rerouted.state(), warehousesim::robots::RobotState::Navigating);
      EXPECT_TRUE(rerouted.task() == along);
      EXPECT_EQ(rerouted.path().back(), (Point{6, 1}));
      EXPECT_TRUE(std::find(rerouted.path().begin(), rerouted.path().end(), Point{3, 1}) ==
                  rerouted.path().end());
      const auto& stranded = sim.robots()[pocketRobot];
      EXPECT_EQ(stranded.state(), warehousesim::robots::RobotState::Idle)
          << "pocket robot first: " << pocketRobotFirst << ", " << workers << " threads";
      EXPECT_FALSE(stranded.task().has_value());
      EXPECT_EQ(sim.tasks().find(pocket)->status, warehousesim::tasks::TaskStatus::Pending);
      EXPECT_EQ(sim.tasks().find(along)->status, warehousesim::tasks::TaskStatus::Assigned);
    }
  }
}

// ---- what is refused ----------------------------------------------------------------------------

TEST(PlanningThreadsTest, ASimulationWithAPlannerOfItsOwnCannotPlanOnOtherThreads) {
  sim::SimulationConfig config;
  sim::Simulation custom(warehousesim::world::makeCompactWarehouse(), {Point{1, 1}}, config,
                         std::make_unique<warehousesim::planning::AStarPlanner>());
  ASSERT_FALSE(custom.usesDefaultComponents());
  EXPECT_NO_THROW(custom.usePlanningThreads(0));
  EXPECT_THROW(custom.usePlanningThreads(2), std::invalid_argument);
  EXPECT_EQ(custom.planningThreads(), 0U);
  EXPECT_FALSE(custom.batchStats().has_value());
}

TEST(PlanningThreadsTest, TooManyThreadsAreRefusedAndTheRunKeepsWhatItHad) {
  sim::Simulation simulation = fleet("compact", 8, 2, 1, 2);
  EXPECT_THROW(simulation.usePlanningThreads(65), std::invalid_argument);
  EXPECT_THROW(simulation.usePlanningThreads(2, 0), std::invalid_argument);
  EXPECT_EQ(simulation.planningThreads(), 2U);
  simulation.run(20);  // and it still works
  EXPECT_GT(simulation.batchStats()->batches, 0U);
  simulation.usePlanningThreads(0);
  EXPECT_EQ(simulation.planningThreads(), 0U);
  EXPECT_FALSE(simulation.batchStats().has_value());
  simulation.run(20);
}

TEST(PlanningThreadsTest, ASimulationBuiltWithoutThreadsHasNoneAndNoStats) {
  const sim::Simulation simulation = *sim::makeLayoutSimulation("compact", 1);
  EXPECT_EQ(simulation.planningThreads(), 0U);
  EXPECT_FALSE(simulation.batchStats().has_value());
}

// ---- recordings do not know how many threads made or play them ----------------------------------

TEST(PlanningThreadsTest, ARecordingReplaysToTheSameEndOnAnyNumberOfPlanningThreads) {
  const sim::Recording recording = warehousesim_test::busyRecording("fulfilment", 3, 160);
  const sim::ReplayResult base = sim::replay(recording);
  ASSERT_TRUE(base.ok()) << base.divergence->what;
  EXPECT_FALSE(base.planningBatches.has_value()) << "by default a replay plans on its own thread";
  for (const std::size_t workers : {1U, 2U, 3U}) {
    sim::ReplayOptions options;
    options.planningThreads = workers;
    const sim::ReplayResult threaded = sim::replay(recording, options);
    ASSERT_TRUE(threaded.ok()) << workers << " threads: " << threaded.divergence->what;
    ASSERT_TRUE(threaded.planningBatches.has_value()) << "the replay really planned on threads";
    EXPECT_GT(threaded.planningBatches->batches, 0U);
    EXPECT_EQ(threaded.checksum, base.checksum) << workers << " threads";
    EXPECT_TRUE(threaded.snapshot == base.snapshot);
    EXPECT_EQ(threaded.checkpointsVerified, base.checkpointsVerified);
    EXPECT_EQ(warehousesim_test::metricsText(threaded.metrics),
              warehousesim_test::metricsText(base.metrics));
  }
}

TEST(PlanningThreadsTest, ARecordingMadeWithPlanningThreadsReplaysOnOne) {
  sim::Simulation simulation = *sim::makeLayoutSimulation("fulfilment", 4);
  simulation.usePlanningThreads(2, 1);
  sim::Recorder recorder(std::move(simulation), 25);
  for (int tick = 0; tick < 150; ++tick) {
    recorder.step();
  }
  const sim::ReplayResult replayed = sim::replay(recorder.recording());
  ASSERT_TRUE(replayed.ok()) << replayed.divergence->what;
  EXPECT_EQ(replayed.checksum, recorder.simulation().stateChecksum());
  EXPECT_GE(replayed.checkpointsVerified, 6U);
  EXPECT_TRUE(replayed.endVerified);
}

// ---- the runner ---------------------------------------------------------------------------------

bool waitFor(const std::function<bool()>& condition) {
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::yield();
  }
  return true;
}

TEST(PlanningThreadsRunnerTest, AWakeUpWithNoStepToTakeLeavesTheStateAsItWas) {
  // A step takes two seconds here, so changing the speed wakes the worker with nothing to do.
  SimulationRunner runner(sim::makeDemoSimulation(1), milliseconds(2000));
  runner.start();
  const std::size_t robots = runner.snapshot().robots.size();
  ASSERT_GT(robots, 0U);
  for (int i = 0; i < 40; ++i) {
    runner.setSpeed(i % 2 == 0 ? 2.0 : 1.0);
    const auto until = std::chrono::steady_clock::now() + milliseconds(2);
    while (std::chrono::steady_clock::now() < until) {
      ASSERT_EQ(runner.snapshot().robots.size(), robots) << "after change " << i;
      ASSERT_EQ(runner.frame().latest.robots.size(), robots);
    }
  }
}

TEST(PlanningThreadsRunnerTest, ARunnerPlanningOnThreadsComputesWhatADirectRunDoes) {
  sim::ScenarioOptions options;
  options.robots = 16;
  options.taskSpawnInterval = 1;
  const auto scenario = *sim::layoutScenario("compact", options);
  sim::SimulationRunner runner(scenario(4), milliseconds(1), scenario);
  runner.usePlanningThreads(2);
  runner.start(150);
  ASSERT_TRUE(runner.waitForTick(150, kTimeout));
  runner.stop();

  sim::Simulation reference = scenario(4);
  reference.run(150);
  EXPECT_TRUE(runner.snapshot() == reference.snapshot());
  ASSERT_TRUE(runner.planningBatchStats().has_value());
  EXPECT_GT(runner.planningBatchStats()->batches, 0U);
}

TEST(PlanningThreadsRunnerTest, ARestartKeepsPlanningOnThreadsAndStaysOnTheSameRun) {
  sim::ScenarioOptions options;
  options.robots = 16;
  options.taskSpawnInterval = 1;
  const auto scenario = *sim::layoutScenario("compact", options);
  sim::SimulationRunner runner(scenario(4), milliseconds(1), scenario);
  runner.usePlanningThreads(2);
  runner.start();
  ASSERT_TRUE(runner.waitForTick(30, kTimeout));
  ASSERT_TRUE(runner.post(sim::ResetScenario{9}));
  ASSERT_TRUE(waitFor([&] { return runner.frame().epoch >= 1; }));
  ASSERT_TRUE(runner.waitForTick(60, kTimeout));
  runner.stop();

  const std::uint64_t at = runner.snapshot().tick;
  sim::Simulation reference = scenario(9);
  reference.run(at);
  EXPECT_TRUE(runner.snapshot() == reference.snapshot()) << "tick " << at;
  ASSERT_TRUE(runner.planningBatchStats().has_value()) << "the new run plans on threads too";
  EXPECT_GT(runner.planningBatchStats()->batches, 0U);
}

TEST(PlanningThreadsRunnerTest, RunnersWithPlanningThreadsStartAndStopOverAndOverWithoutHanging) {
  sim::ScenarioOptions options;
  options.robots = 12;
  options.taskSpawnInterval = 1;
  const auto scenario = *sim::layoutScenario("compact", options);
  for (int i = 0; i < 25; ++i) {
    sim::SimulationRunner runner(scenario(static_cast<std::uint64_t>(i)), milliseconds(1),
                                 scenario);
    runner.usePlanningThreads(static_cast<std::size_t>(1 + i % 3));
    runner.start();
    if (i % 2 == 0) {
      ASSERT_TRUE(runner.waitForTick(3, kTimeout));
    }
    if (i % 5 == 0) {
      runner.stop();  // explicitly, and then again from the destructor
    }
  }
}

TEST(PlanningThreadsRunnerTest, ChangingTheThreadsWhileTheRunnerRunsIsSafeAndChangesNothing) {
  sim::ScenarioOptions options;
  options.robots = 16;
  options.taskSpawnInterval = 1;
  const auto scenario = *sim::layoutScenario("compact", options);
  sim::SimulationRunner runner(scenario(2), milliseconds(1), scenario);
  runner.start(300);
  const std::size_t plan[] = {2, 0, 3, 1, 0, 2};
  for (const std::size_t workers : plan) {
    runner.usePlanningThreads(workers);
    std::this_thread::yield();
  }
  ASSERT_TRUE(runner.waitForTick(300, kTimeout));
  runner.stop();
  sim::Simulation reference = scenario(2);
  reference.run(300);
  EXPECT_TRUE(runner.snapshot() == reference.snapshot());
}

}  // namespace
