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
using warehousesim::simulation::makeLayoutSimulation;
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

// ---- the frame to draw ------------------------------------------------------------------

TEST(RunnerFrameTest, BeforeAnyStepThereIsNothingToInterpolate) {
  SimulationRunner runner(makeDemoSimulation(2), milliseconds(1));
  const auto frame = runner.frame();
  EXPECT_TRUE(frame.previous == frame.latest);
  EXPECT_TRUE(frame.latest == runner.snapshot());
  EXPECT_EQ(frame.latest.tick, 0U);
  EXPECT_GE(frame.alpha, 0.0);
  EXPECT_LE(frame.alpha, 1.0);
}

TEST(RunnerFrameTest, TheFrameHoldsTheLastTwoStatesAndSettlesOnTheFinalOneWhenTheRunEnds) {
  SimulationRunner runner(makeLayoutSimulation("compact", 4).value(), milliseconds(1));
  runner.start(30);
  ASSERT_TRUE(runner.waitForTick(30, kTimeout));
  // Wait for the worker to notice it has nothing left to do and settle the display on the end.
  warehousesim::simulation::FrameState frame = runner.frame();
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (frame.alpha < 1.0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(milliseconds(1));
    frame = runner.frame();
  }

  auto reference = makeLayoutSimulation("compact", 4).value();
  reference.run(29);
  const auto atTick29 = reference.snapshot();
  reference.step();
  EXPECT_EQ(frame.latest.tick, 30U);
  EXPECT_EQ(frame.previous.tick, 29U);
  EXPECT_TRUE(frame.previous == atTick29);
  EXPECT_TRUE(frame.latest == reference.snapshot());
  EXPECT_DOUBLE_EQ(frame.alpha, 1.0);  // the display shows the final state completely
}

// The stepping is slow enough here (2 s per step) that no step happens while this test watches,
// so the only thing that can change is how far through the step the display is.
TEST(RunnerFrameTest, AlphaGrowsWithRealTimeAndStaysBetweenZeroAndOne) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(2000));
  runner.start();
  double last = -1.0;
  bool grew = false;
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto frame = runner.frame();
    ASSERT_GE(frame.alpha, 0.0);
    ASSERT_LE(frame.alpha, 1.0);
    if (frame.latest.tick == 0) {
      ASSERT_GE(frame.alpha, last);  // never backwards within a step
      last = frame.alpha;
    }
    if (frame.alpha > 0.1) {
      grew = true;
      break;
    }
    std::this_thread::sleep_for(milliseconds(2));
  }
  EXPECT_TRUE(grew);
}

TEST(RunnerFrameTest, PausingHoldsThePictureStillAndResumingCarriesOnFromThere) {
  SimulationRunner runner(makeDemoSimulation(1), milliseconds(2000));
  runner.start();
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (runner.frame().alpha < 0.15 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(milliseconds(2));
  }
  runner.setPaused(true);
  const double held = runner.frame().alpha;
  ASSERT_GE(held, 0.15);
  std::this_thread::sleep_for(milliseconds(60));  // nothing may move while it is paused
  EXPECT_DOUBLE_EQ(runner.frame().alpha, held);

  runner.setPaused(false);
  // It carries on from where it stood, never from the start of the step: every sample taken while
  // the worker wakes up and starts ticking again is at or beyond the held position...
  double alpha = runner.frame().alpha;
  while (alpha < held + 0.05 && std::chrono::steady_clock::now() < deadline) {
    ASSERT_GE(alpha, held - 1e-9);
    std::this_thread::sleep_for(milliseconds(1));
    alpha = runner.frame().alpha;
  }
  // ...and then it grows again.
  EXPECT_GE(alpha, held + 0.05);
}

// However often the renderer asks for frames, the simulation computes the same thing.
TEST(RunnerFrameTest, HowOftenTheRendererPollsDoesNotChangeWhatIsSimulated) {
  auto reference = makeLayoutSimulation("compact", 5).value();
  reference.run(250);
  for (const int pollEveryMs : {0, 1, 7}) {
    SimulationRunner runner(makeLayoutSimulation("compact", 5).value(), milliseconds(1));
    runner.start(250);
    const auto deadline = std::chrono::steady_clock::now() + kTimeout;
    while (runner.snapshot().tick < 250 && std::chrono::steady_clock::now() < deadline) {
      const auto frame = runner.frame();
      ASSERT_LE(frame.previous.tick, frame.latest.tick);
      ASSERT_LE(frame.latest.tick - frame.previous.tick, 200U);
      if (pollEveryMs > 0) {
        std::this_thread::sleep_for(milliseconds(pollEveryMs));
      }
    }
    ASSERT_TRUE(runner.waitForTick(250, kTimeout));
    EXPECT_TRUE(runner.snapshot() == reference.snapshot()) << "polling every " << pollEveryMs;
    runner.stop();
  }
}

}  // namespace
