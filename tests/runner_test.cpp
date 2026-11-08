#include "warehousesim/simulation/runner.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <thread>

#include "warehousesim/simulation/scenarios.hpp"

namespace {

using std::chrono::milliseconds;
using warehousesim::simulation::makeDemoSimulation;
using warehousesim::simulation::SimulationRunner;

constexpr milliseconds kTimeout{10000};

TEST(RunnerTest, ReachesRequestedTickWithSameStateAsDirectRun) {
  SimulationRunner runner(makeDemoSimulation(3), milliseconds(1));
  runner.start(50);
  ASSERT_TRUE(runner.waitForTick(50, kTimeout));

  auto reference = makeDemoSimulation(3);
  reference.run(50);
  EXPECT_TRUE(runner.snapshot() == reference.snapshot());
  runner.stop();
}

TEST(RunnerTest, DoesNotStepBeyondTheLimit) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
  runner.start(5);
  ASSERT_TRUE(runner.waitForTick(5, kTimeout));
  std::this_thread::sleep_for(milliseconds(30));
  EXPECT_EQ(runner.snapshot().tick, 5U);
}

TEST(RunnerTest, StopIsIdempotentAndFreezesState) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
  runner.start();
  ASSERT_TRUE(runner.waitForTick(3, kTimeout));
  runner.stop();
  runner.stop();
  const auto frozen = runner.snapshot();
  std::this_thread::sleep_for(milliseconds(20));
  EXPECT_TRUE(runner.snapshot() == frozen);
}

TEST(RunnerTest, PauseHaltsAndResumeContinues) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
  runner.start();
  ASSERT_TRUE(runner.waitForTick(2, kTimeout));
  runner.setPaused(true);
  const auto paused = runner.snapshot();
  std::this_thread::sleep_for(milliseconds(30));
  EXPECT_EQ(runner.snapshot().tick, paused.tick);

  runner.setPaused(false);
  EXPECT_TRUE(runner.waitForTick(paused.tick + 2, kTimeout));
}

TEST(RunnerTest, StartingTwiceThrows) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
  runner.start();
  EXPECT_THROW(runner.start(), std::logic_error);
}

TEST(RunnerTest, DestructorStopsRunningWorker) {
  {
    SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
    runner.start();
    ASSERT_TRUE(runner.waitForTick(1, kTimeout));
  }
  SUCCEED();
}

TEST(RunnerTest, ConcurrentReadersSeeConsistentSnapshots) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(0));
  runner.start(300);
  std::thread reader([&runner] {
    for (int i = 0; i < 200; ++i) {
      const auto snapshot = runner.snapshot();
      EXPECT_EQ(snapshot.robots.size(), 4U);
    }
  });
  ASSERT_TRUE(runner.waitForTick(300, kTimeout));
  reader.join();
}

TEST(RunnerTest, NeverStartedRunnerExposesInitialState) {
  const SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
  EXPECT_EQ(runner.snapshot().tick, 0U);
  EXPECT_EQ(runner.warehouse().width(), 20);
}

}  // namespace
