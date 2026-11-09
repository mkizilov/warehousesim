#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/world/cell_type.hpp"

namespace {

using namespace warehousesim_test;
using std::chrono::milliseconds;
using warehousesim::simulation::apply;
using warehousesim::simulation::Command;
using warehousesim::simulation::CommandRecord;
using warehousesim::simulation::ResetScenario;
using warehousesim::simulation::ScenarioFactory;
using warehousesim::simulation::SetTaskPriority;
using warehousesim::simulation::SimulationRunner;
using warehousesim::simulation::SubmitTask;

constexpr milliseconds kTimeout{15000};

ScenarioFactory demo() { return *warehousesim::simulation::layoutScenario("demo"); }

std::string json(const warehousesim::simulation::Snapshot& snapshot) {
  return warehousesim::simulation::toJson(snapshot).dump();
}

// The simulation the commands in `log` describe: the scenario from its seed, each command applied
// at the tick the log says, before the step that follows it. This is the whole claim about
// determinism: the runner's state at any tick is a function of the scenario, the seed and this
// record, whatever the timing of the clicks that made it.
Simulation replay(const ScenarioFactory& scenario, std::uint64_t seed,
                  const std::vector<CommandRecord>& log, warehousesim::core::Tick until) {
  Simulation sim = scenario(seed);
  std::size_t next = 0;
  while (true) {
    while (next < log.size() && log[next].tick == sim.tick()) {
      const Command& command = log[next].command;
      if (const auto* submit = std::get_if<SubmitTask>(&command)) {
        (void)apply(sim, *submit);
      } else if (const auto* priority = std::get_if<SetTaskPriority>(&command)) {
        (void)apply(sim, *priority);
      }
      ++next;
    }
    if (sim.tick() >= until) {
      return sim;
    }
    sim.step();
  }
}

// ---- posting and applying -----------------------------------------------------------------------

TEST(RunnerCommandsTest, ACommandIsAppliedBetweenStepsAndRecordedWithItsTickAndOutcome) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  runner.setPaused(true);
  runner.start();
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11), TaskPriority::Urgent}));

  std::vector<CommandRecord> log;
  for (int i = 0; i < 2000 && log.empty(); ++i) {
    log = runner.commandLog();
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_EQ(log.size(), 1U);
  EXPECT_EQ(log[0].tick, 0U);
  EXPECT_TRUE(log[0].outcome.accepted);
  EXPECT_TRUE(log[0].outcome.task.has_value());
  EXPECT_TRUE((log[0].command == Command{SubmitTask{P(0, 5), P(0, 11), TaskPriority::Urgent}}));
  const auto snapshot = runner.snapshot();
  ASSERT_EQ(snapshot.activeTasks.size(), 1U);
  EXPECT_EQ(snapshot.activeTasks[0].priority, TaskPriority::Urgent);
  EXPECT_EQ(snapshot.tick, 0U);  // still paused: the command did not need a step
  runner.stop();
}

TEST(RunnerCommandsTest, CommandsAreAppliedInTheOrderTheyWerePosted) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  runner.setPaused(true);
  runner.start();
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(i, 11), TaskPriority::Low}));
  }
  // Task numbers are given out in the order applied, so the dropoffs must come out in order.
  std::vector<CommandRecord> log;
  for (int i = 0; i < 4000 && log.size() < 10; ++i) {
    log = runner.commandLog();
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_EQ(log.size(), 10U);
  for (std::size_t i = 0; i < log.size(); ++i) {
    ASSERT_TRUE(log[i].outcome.task.has_value()) << i;
    EXPECT_EQ(log[i].outcome.task->value, i + 1);
    EXPECT_EQ(std::get<SubmitTask>(log[i].command).dropoff.x, static_cast<int>(i));
  }
  runner.stop();
}

TEST(RunnerCommandsTest, ARefusedCommandIsRecordedWithItsReasonAndChangesNothing) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  runner.setPaused(true);
  runner.start();
  const std::string before = json(runner.snapshot());
  ASSERT_TRUE(runner.post(SubmitTask{P(-1, 0), P(0, 11)}));
  ASSERT_TRUE(runner.post(SetTaskPriority{TaskId{42}, TaskPriority::High}));
  std::vector<CommandRecord> log;
  for (int i = 0; i < 4000 && log.size() < 2; ++i) {
    log = runner.commandLog();
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_EQ(log.size(), 2U);
  EXPECT_FALSE(log[0].outcome.accepted);
  EXPECT_FALSE(log[1].outcome.accepted);
  EXPECT_TRUE(log[0].outcome.message.find("outside the map") != std::string::npos);
  EXPECT_EQ(json(runner.snapshot()), before);
  runner.stop();
}

TEST(RunnerCommandsTest, CommandLogCanBeReadFromAnIndexOn) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  runner.setPaused(true);
  runner.start();
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11)}));
  }
  for (int i = 0; i < 4000 && runner.commandLog().size() < 4; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  EXPECT_EQ(runner.commandLog(0).size(), 4U);
  EXPECT_EQ(runner.commandLog(3).size(), 1U);
  EXPECT_EQ(runner.commandLog(4).size(), 0U);
  EXPECT_EQ(runner.commandLog(1000).size(), 0U);
  runner.stop();
}

TEST(RunnerCommandsTest, TheQueueIsBoundedAndRefusesTheOverflowWithoutLosingWhatIsQueued) {
  // Not started: nothing takes commands off the queue.
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  for (std::size_t i = 0; i < SimulationRunner::kMaxQueuedCommands; ++i) {
    ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11)})) << i;
  }
  EXPECT_FALSE(runner.post(SubmitTask{P(0, 5), P(0, 11)}));
  EXPECT_FALSE(runner.post(ResetScenario{}));
  runner.setPaused(true);
  runner.start();
  for (int i = 0; i < 4000 && runner.commandLog().size() < SimulationRunner::kMaxQueuedCommands;
       ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  EXPECT_EQ(runner.commandLog().size(), SimulationRunner::kMaxQueuedCommands);
  EXPECT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11)}));  // room again
  runner.stop();
}

TEST(RunnerCommandsTest, CommandsPostedBeforeTheRunnerStartsWaitForIt) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11)}));
  EXPECT_TRUE(runner.commandLog().empty());
  EXPECT_EQ(runner.snapshot().activeTasks.size(), 0U);
  runner.setPaused(true);
  runner.start();
  for (int i = 0; i < 4000 && runner.commandLog().empty(); ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  EXPECT_EQ(runner.commandLog().size(), 1U);
  runner.stop();
}

TEST(RunnerCommandsTest, PostingFromManyThreadsLosesNothingAndKeepsEachThreadsOrder) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  runner.start();
  constexpr int kThreads = 4;
  constexpr int kEach = 40;
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&runner, t] {
      for (int i = 0; i < kEach; ++i) {
        // The dropoff column says whose command this is and the pickup row how far along it is.
        while (!runner.post(SubmitTask{P(0, 3 + (i % 3) * 3), P(t * 3, 11), TaskPriority::Low})) {
          std::this_thread::yield();
        }
      }
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  for (int i = 0; i < 8000 && runner.commandLog().size() < kThreads * kEach; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const auto log = runner.commandLog();
  runner.stop();
  ASSERT_EQ(log.size(), static_cast<std::size_t>(kThreads * kEach));
  std::vector<int> seen(kThreads, 0);
  for (const CommandRecord& record : log) {
    const int thread = std::get<SubmitTask>(record.command).dropoff.x / 3;
    ++seen[static_cast<std::size_t>(thread)];
  }
  for (const int count : seen) {
    EXPECT_EQ(count, kEach);
  }
  // Applied ticks never go backwards.
  warehousesim::core::Tick previous = 0;
  for (const CommandRecord& record : log) {
    EXPECT_GE(record.tick, previous);
    previous = record.tick;
  }
}

// ---- resetting ----------------------------------------------------------------------------------

TEST(RunnerCommandsTest, ResetStartsTheScenarioOverFromTheSameSeedAndClearsTheLog) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(5), milliseconds(1), demo());
  runner.start();
  ASSERT_TRUE(runner.waitForTick(30, kTimeout));
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11)}));
  for (int i = 0; i < 4000 && runner.commandLog().empty(); ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  runner.setPaused(true);
  ASSERT_TRUE(runner.post(ResetScenario{}));
  std::vector<CommandRecord> log;
  for (int i = 0; i < 4000; ++i) {
    log = runner.commandLog();
    if (log.size() == 1 && std::holds_alternative<ResetScenario>(log[0].command)) {
      break;
    }
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_EQ(log.size(), 1U);
  EXPECT_TRUE(log[0].outcome.accepted);
  EXPECT_EQ(log[0].tick, 0U);
  EXPECT_TRUE(log[0].outcome.message.find("seed 5") != std::string::npos) << log[0].outcome.message;
  EXPECT_EQ(runner.seed(), 5U);

  const auto after = runner.snapshot();
  EXPECT_EQ(after.tick, 0U);
  EXPECT_EQ(json(after), json(warehousesim::simulation::makeDemoSimulation(5).snapshot()));
  EXPECT_EQ(runner.frame().epoch, 1U);
  EXPECT_TRUE(json(runner.frame().previous) == json(after));
  runner.stop();
}

TEST(RunnerCommandsTest, AResetRunLeadsToTheSameStateAsAFreshRunOfTheSameSeed) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(5), milliseconds(1), demo());
  runner.start(40);
  ASSERT_TRUE(runner.waitForTick(40, kTimeout));
  // A hand-made task in the first run must leave nothing behind in the second.
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11), TaskPriority::Urgent}));
  ASSERT_TRUE(runner.post(ResetScenario{}));
  for (int i = 0; i < 4000 && runner.frame().epoch == 0; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_EQ(runner.frame().epoch, 1U);
  // The stop at tick 40 still holds, so the new run goes to 40 on its own.
  ASSERT_TRUE(runner.waitForTick(40, kTimeout));
  auto reference = warehousesim::simulation::makeDemoSimulation(5);
  reference.run(40);
  EXPECT_EQ(json(runner.snapshot()), json(reference.snapshot()));
  runner.stop();
}

TEST(RunnerCommandsTest, ResetWithASeedUsesItAndThatSeedStaysForLaterResets) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(5), milliseconds(1), demo());
  runner.setPaused(true);
  runner.start();
  ASSERT_TRUE(runner.post(ResetScenario{9}));
  for (int i = 0; i < 4000 && runner.commandLog().empty(); ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  EXPECT_EQ(runner.seed(), 9U);
  EXPECT_EQ(json(runner.snapshot()),
            json(warehousesim::simulation::makeDemoSimulation(9).snapshot()));
  ASSERT_TRUE(runner.post(ResetScenario{}));
  for (int i = 0; i < 4000 && runner.frame().epoch < 2; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  EXPECT_EQ(runner.seed(), 9U);
  runner.stop();
}

TEST(RunnerCommandsTest, ResetWithoutAScenarioIsRefusedAndNothingChanges) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(5), milliseconds(1));
  runner.setPaused(true);
  runner.start();
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11)}));
  ASSERT_TRUE(runner.post(ResetScenario{}));
  for (int i = 0; i < 4000 && runner.commandLog().size() < 2; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const auto log = runner.commandLog();
  ASSERT_EQ(log.size(), 2U);
  EXPECT_FALSE(log[1].outcome.accepted);
  EXPECT_TRUE(log[1].outcome.message.find("cannot be restarted") != std::string::npos);
  EXPECT_EQ(runner.snapshot().activeTasks.size(), 1U);  // the task is still there
  EXPECT_EQ(runner.frame().epoch, 0U);
  runner.stop();
}

TEST(RunnerCommandsTest, AScenarioThatBuildsAnotherLayoutIsNotUsed) {
  ScenarioFactory wrong = [](std::uint64_t seed) {
    return warehousesim::simulation::makeLayoutSimulation("compact", seed).value();
  };
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(5), milliseconds(1), wrong);
  runner.setPaused(true);
  runner.start();
  ASSERT_TRUE(runner.post(ResetScenario{}));
  for (int i = 0; i < 4000 && runner.commandLog().empty(); ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const auto log = runner.commandLog();
  ASSERT_EQ(log.size(), 1U);
  EXPECT_FALSE(log[0].outcome.accepted);
  EXPECT_TRUE(log[0].outcome.message.find("another layout") != std::string::npos);
  EXPECT_EQ(runner.warehouse().width(), 20);
  runner.stop();
}

// PlaceObstacle and ClearObstacle change the layout of the simulation, so the layout the runner
// reports has to follow: whatever draws from it would otherwise show robots driving through a wall.
TEST(RunnerCommandsTest, TheRunnersLayoutFollowsObstaclesPutUpAndTakenDownAndARestartUndoesThem) {
  using warehousesim::core::Point;
  using warehousesim::world::CellType;
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(1), milliseconds(1), demo());
  const Point cell{10, 6};
  ASSERT_EQ(runner.warehouse().at(cell), CellType::Free);
  runner.setPaused(true);  // nothing moves, so nothing holds the cell when the command comes
  runner.start();

  const auto applied = [&](std::size_t count) {
    for (int i = 0; i < 4000 && runner.commandLog().size() < count; ++i) {
      std::this_thread::sleep_for(milliseconds(1));
    }
    return runner.commandLog();
  };
  ASSERT_TRUE(runner.post(warehousesim::simulation::PlaceObstacle{cell, CellType::Wall}));
  auto log = applied(1);
  ASSERT_EQ(log.size(), 1U);
  ASSERT_TRUE(log[0].outcome.accepted) << log[0].outcome.message;
  EXPECT_EQ(runner.warehouse().at(cell), CellType::Wall);

  ASSERT_TRUE(runner.post(warehousesim::simulation::ClearObstacle{cell}));
  log = applied(2);
  ASSERT_EQ(log.size(), 2U);
  ASSERT_TRUE(log[1].outcome.accepted) << log[1].outcome.message;
  EXPECT_EQ(runner.warehouse().at(cell), CellType::Free);

  ASSERT_TRUE(runner.post(warehousesim::simulation::PlaceObstacle{cell, CellType::Shelf}));
  ASSERT_EQ(applied(3).size(), 3U);
  ASSERT_EQ(runner.warehouse().at(cell), CellType::Shelf);
  ASSERT_TRUE(runner.post(ResetScenario{}));
  for (int i = 0; i < 4000 && runner.frame().epoch == 0; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  log = runner.commandLog();
  ASSERT_EQ(log.size(), 1U) << "a restart starts the record over";
  EXPECT_TRUE(log[0].outcome.accepted) << "the original layout is what a restart builds";
  EXPECT_EQ(runner.warehouse().at(cell), CellType::Free);
  runner.stop();
}

TEST(RunnerCommandsTest, AScenarioThatThrowsLeavesTheRunGoing) {
  ScenarioFactory throws = [](std::uint64_t) -> Simulation { throw std::runtime_error("boom"); };
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(5), milliseconds(1), throws);
  runner.start();
  ASSERT_TRUE(runner.waitForTick(3, kTimeout));
  ASSERT_TRUE(runner.post(ResetScenario{}));
  for (int i = 0; i < 4000 && runner.commandLog().empty(); ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const auto log = runner.commandLog();
  ASSERT_EQ(log.size(), 1U);
  EXPECT_FALSE(log[0].outcome.accepted);
  EXPECT_TRUE(log[0].outcome.message.find("boom") != std::string::npos);
  const auto tick = runner.snapshot().tick;
  EXPECT_TRUE(runner.waitForTick(tick + 3, kTimeout));  // still stepping
  runner.stop();
}

TEST(RunnerCommandsTest, ResetClearsTheCommandsPostedAfterItToo) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(5), milliseconds(1), demo());
  runner.setPaused(true);
  runner.start();
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11)}));
  ASSERT_TRUE(runner.post(ResetScenario{}));
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(1, 11)}));
  for (int i = 0; i < 4000 && runner.commandLog().size() < 2; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const auto log = runner.commandLog();
  ASSERT_EQ(log.size(), 2U);
  EXPECT_TRUE(std::holds_alternative<ResetScenario>(log[0].command));
  EXPECT_EQ(std::get<SubmitTask>(log[1].command).dropoff, P(1, 11));
  EXPECT_EQ(log[1].outcome.task->value, 1U);  // numbering started over
  runner.stop();
}

// ---- the same run, however it is driven ---------------------------------------------------------

TEST(RunnerDeterminismTest, PausingResumingAndChangingSpeedDoNotChangeWhatTheSimulationComputes) {
  warehousesim::core::Rng rng(11);
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(4), milliseconds(1), demo());
  runner.start(120);
  while (runner.snapshot().tick < 120) {
    switch (rng.below(4)) {
      case 0:
        runner.setPaused(true);
        break;
      case 1:
        runner.setPaused(false);
        break;
      case 2:
        runner.setSpeed(0.25 * static_cast<double>(1 + rng.below(64)));
        break;
      default:
        (void)runner.frame();
        (void)runner.snapshot();
        break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  runner.setPaused(false);
  ASSERT_TRUE(runner.waitForTick(120, kTimeout));
  auto reference = warehousesim::simulation::makeDemoSimulation(4);
  reference.run(120);
  EXPECT_EQ(json(runner.snapshot()), json(reference.snapshot()));
  runner.stop();
}

TEST(RunnerDeterminismTest, AnyTimingOfTheSameCommandsGivesTheStateTheirRecordDescribes) {
  // Whatever the timing, the state at the end is the replay of the record: so two operators who
  // click the same things at different moments get states that each match their own record, and
  // runs with the same record get the same state.
  for (std::uint64_t round = 1; round <= 6; ++round) {
    warehousesim::core::Rng rng(round);
    SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(2), milliseconds(1),
                            demo());
    runner.start(150);
    while (runner.snapshot().tick < 150) {
      switch (rng.below(4)) {
        case 0:
          (void)runner.post(
              SubmitTask{P(static_cast<int>(rng.below(20)), static_cast<int>(rng.below(12))),
                         P(static_cast<int>(rng.below(20)), static_cast<int>(rng.below(12))),
                         static_cast<TaskPriority>(rng.below(4))});
          break;
        case 1:
          (void)runner.post(SetTaskPriority{TaskId{static_cast<std::uint32_t>(1 + rng.below(40))},
                                            static_cast<TaskPriority>(rng.below(4))});
          break;
        case 2:
          runner.setSpeed(static_cast<double>(1 + rng.below(32)));
          break;
        default:
          break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(300));
    }
    ASSERT_TRUE(runner.waitForTick(150, kTimeout));
    for (int i = 0; i < 100; ++i) {  // let queued commands land
      std::this_thread::sleep_for(milliseconds(1));
    }
    const auto log = runner.commandLog();
    const auto finalSnapshot = runner.snapshot();
    runner.stop();

    ASSERT_EQ(finalSnapshot.tick, 150U);
    const Simulation replayed = replay(demo(), 2, log, 150);
    EXPECT_EQ(json(finalSnapshot), json(replayed.snapshot())) << "round " << round;
    // A second replay is the same again.
    EXPECT_EQ(json(replay(demo(), 2, log, 150).snapshot()), json(replayed.snapshot()));
  }
}

TEST(RunnerDeterminismTest, TheRecordIsEnoughToRebuildTheStateAtEveryEarlierTickToo) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(8), milliseconds(1), demo());
  runner.setPaused(true);
  runner.start(60);
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 5), P(0, 11), TaskPriority::Urgent}));
  runner.setPaused(false);
  ASSERT_TRUE(runner.waitForTick(20, kTimeout));
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 8), P(3, 11), TaskPriority::High}));
  ASSERT_TRUE(runner.waitForTick(60, kTimeout));
  for (int i = 0; i < 100 && runner.commandLog().size() < 2; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const auto log = runner.commandLog();
  ASSERT_EQ(log.size(), 2U);
  const auto finalSnapshot = runner.snapshot();
  runner.stop();
  EXPECT_EQ(json(replay(demo(), 8, log, 60).snapshot()), json(finalSnapshot));
  // And a replay of a run that only gets as far as the second command has not seen it yet.
  const Simulation partway = replay(demo(), 8, log, log[1].tick);
  EXPECT_EQ(partway.snapshot().tick, log[1].tick);
}

TEST(RunnerDeterminismTest, ARunWithNoCommandsIsTheSameAsADirectRunWhateverIsReadAndDrawn) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(6), milliseconds(1), demo());
  runner.start(80);
  while (runner.snapshot().tick < 80) {
    (void)runner.frame();
    std::this_thread::yield();
  }
  ASSERT_TRUE(runner.waitForTick(80, kTimeout));
  auto reference = warehousesim::simulation::makeDemoSimulation(6);
  reference.run(80);
  EXPECT_EQ(json(runner.snapshot()), json(reference.snapshot()));
  EXPECT_TRUE(runner.commandLog().empty());
  runner.stop();
}

}  // namespace
