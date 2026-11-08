#include "warehousesim/simulation/runner.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <thread>

#include "warehousesim/core/fixed_timestep.hpp"
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

TEST(RunnerTest, SpeedMultiplierScalesTheStepRate) {
  // 640 ms per step would need 12.8 s for 20 ticks at speed 1; at the maximum speed of 64 the
  // same ticks take 0.2 s. Reaching tick 20 well inside the timeout proves the speed applies.
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(640));
  runner.setSpeed(1000.0);  // clamped to the maximum
  EXPECT_DOUBLE_EQ(runner.speed(), warehousesim::core::FixedTimestep::kMaxSpeed);
  runner.start(20);
  EXPECT_TRUE(runner.waitForTick(20, kTimeout));
}

TEST(RunnerTest, SpeedChangeInterruptsTheCurrentSleep) {
  // The first step is 6.4 s away at speed 1. Raising the speed to 64 while the worker is asleep
  // must wake it and shorten that wait to 0.1 s; otherwise tick 3 would arrive after 6.4 s and
  // the 3 s timeout would expire.
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(6400));
  runner.start(3);
  // Give the worker time to reach its sleep; if it is slower than this the test still passes,
  // it just no longer exercises the interruption.
  std::this_thread::sleep_for(milliseconds(100));
  runner.setSpeed(warehousesim::core::FixedTimestep::kMaxSpeed);
  EXPECT_TRUE(runner.waitForTick(3, milliseconds(3000)));
}

TEST(RunnerTest, InvalidSpeedAndStepDurationAreRejected) {
  EXPECT_THROW(SimulationRunner(makeDemoSimulation(1), milliseconds(0)), std::invalid_argument);
  EXPECT_THROW(SimulationRunner(makeDemoSimulation(1), milliseconds(-5)), std::invalid_argument);
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
  EXPECT_THROW(runner.setSpeed(0.0), std::invalid_argument);
  EXPECT_THROW(runner.setSpeed(-1.0), std::invalid_argument);
  EXPECT_DOUBLE_EQ(runner.speed(), 1.0);
}

TEST(RunnerTest, ConcurrentReadersSeeConsistentSnapshots) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(1));
  runner.setSpeed(warehousesim::core::FixedTimestep::kMaxSpeed);
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
