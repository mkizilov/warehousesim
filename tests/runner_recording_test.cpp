#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "recording_support.hpp"
#include "warehousesim/simulation/replay.hpp"
#include "warehousesim/simulation/runner.hpp"

namespace {

using namespace warehousesim_test;
using std::chrono::milliseconds;
using warehousesim::simulation::replay;
using warehousesim::simulation::ReplayResult;
using warehousesim::simulation::ResetScenario;
using warehousesim::simulation::ScenarioFactory;
using warehousesim::simulation::SimulationRunner;

constexpr milliseconds kTimeout{30000};

ScenarioFactory compact() { return *warehousesim::simulation::layoutScenario("compact"); }

SimulationRunner makeRunner(std::uint64_t seed = 3) {
  return SimulationRunner(*warehousesim::simulation::makeLayoutSimulation("compact", seed),
                          milliseconds(1), compact());
}

// Waits until the runner's command log has `count` entries.
void waitForCommands(const SimulationRunner& runner, std::size_t count) {
  for (int i = 0; i < 20000 && runner.commandLog().size() < count; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_GE(runner.commandLog().size(), count);
}

// Waits until a restart has been applied: the log starts over with it as its first entry.
void waitForReset(const SimulationRunner& runner) {
  const auto restarted = [&runner] {
    const auto log = runner.commandLog();
    return !log.empty() && std::holds_alternative<ResetScenario>(log.front().command);
  };
  for (int i = 0; i < 20000 && !restarted(); ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_TRUE(restarted());
}

TEST(RunnerRecordingTest, NothingIsRecordedUnlessAskedFor) {
  SimulationRunner runner = makeRunner();
  EXPECT_FALSE(runner.recording().has_value());
  runner.start(20);
  ASSERT_TRUE(runner.waitForTick(20, kTimeout));
  runner.stop();
  EXPECT_FALSE(runner.recording().has_value());
}

TEST(RunnerRecordingTest, RecordingCanOnlyBeginAtTheBeginning) {
  SimulationRunner late = makeRunner();
  late.start(10);
  ASSERT_TRUE(late.waitForTick(10, kTimeout));
  late.stop();
  EXPECT_THROW(late.startRecording(), std::invalid_argument);

  Simulation withATask = *warehousesim::simulation::makeLayoutSimulation("compact", 3);
  (void)withATask.submitTask(P(2, 3), P(4, 3));
  SimulationRunner busy(std::move(withATask), milliseconds(1), compact());
  EXPECT_THROW(busy.startRecording(), std::invalid_argument);

  SimulationRunner fine = makeRunner();
  EXPECT_NO_THROW(fine.startRecording(25));
  ASSERT_TRUE(fine.recording().has_value());
  EXPECT_EQ(fine.recording()->ticks, 0U);
  EXPECT_EQ(fine.recording()->checkpointInterval, 25U);
}

TEST(RunnerRecordingTest, TheRecordingOfARunReplaysToTheStateAndTheFiguresTheRunnerEndedIn) {
  SimulationRunner runner = makeRunner(5);
  runner.startRecording(20);
  runner.setSpeed(8);
  runner.start(150);  // the first half: the worker waits at tick 150 until it is started again
  // Commands of every kind, posted while it runs, so that each lands on whatever tick it lands on.
  // (A worker held back by a limit applies them too, at the tick it stopped at.)
  // The robots are taken out of service first, so that the tasks stay waiting and can be changed.
  ASSERT_TRUE(runner.post(SetRobotService{RobotId{1}, false}));
  ASSERT_TRUE(runner.post(SetRobotService{RobotId{2}, false}));
  waitForCommands(runner, 2);
  ASSERT_TRUE(runner.post(SubmitTask{P(3, 3), P(9, 1), TaskPriority::High}));
  ASSERT_TRUE(runner.post(SubmitTask{P(4, 3), P(9, 1), TaskPriority::Low}));
  ASSERT_TRUE(runner.post(SubmitTask{P(-1, 0), P(9, 1)}));  // refused
  waitForCommands(runner, 5);
  const auto log = runner.commandLog();
  ASSERT_TRUE(log[2].outcome.task.has_value());
  ASSERT_TRUE(log[3].outcome.task.has_value());
  ASSERT_TRUE(runner.post(SetTaskPriority{*log[2].outcome.task, TaskPriority::Urgent}));
  ASSERT_TRUE(runner.post(CancelTask{*log[3].outcome.task}));
  ASSERT_TRUE(runner.post(PlaceObstacle{P(5, 3), CellType::Wall}));
  waitForCommands(runner, 8);
  ASSERT_TRUE(runner.post(ClearObstacle{P(5, 3)}));
  ASSERT_TRUE(runner.post(SetRobotService{RobotId{1}, true}));
  ASSERT_TRUE(runner.post(SetRobotService{RobotId{2}, true}));
  waitForCommands(runner, 11);
  ASSERT_TRUE(runner.waitForTick(150, kTimeout));
  runner.stop();

  // Recordings of a run that is not over are each a run in themselves, as of the tick they were
  // taken at. The first is taken with the worker held at its limit, so it is certain to be one of
  // a run half way; how many more are taken while the run goes on depends on timing (the whole run
  // can be over before a slow machine gets to look), so only the first is counted on.
  std::vector<Recording> midway;
  const std::optional<Recording> half = runner.recording();
  ASSERT_TRUE(half.has_value());
  midway.push_back(*half);
  runner.start(300);
  while (runner.snapshot().tick < 300) {
    if (auto recording = runner.recording()) {
      midway.push_back(std::move(*recording));
    }
    std::this_thread::sleep_for(milliseconds(5));
  }
  ASSERT_TRUE(runner.waitForTick(300, kTimeout));
  runner.stop();

  ASSERT_FALSE(midway.empty());
  EXPECT_EQ(midway.front().ticks, 150U);
  EXPECT_EQ(midway.front().commands.size(), 11U)
      << "every command was applied by the half way mark";
  for (const Recording& partial : midway) {
    const ReplayResult result = replay(partial);
    EXPECT_TRUE(result.ok()) << "a recording taken at tick " << partial.ticks << ": "
                             << (result.divergence ? result.divergence->what : "");
    EXPECT_TRUE(result.endVerified);
  }

  const std::optional<Recording> recording = runner.recording();
  ASSERT_TRUE(recording.has_value());
  EXPECT_EQ(recording->ticks, 300U);
  const ReplayResult result = replay(*recording);
  ASSERT_TRUE(result.ok()) << result.divergence->what;
  EXPECT_TRUE(result.snapshot == runner.snapshot());
  EXPECT_EQ(metricsText(result.metrics), metricsText(runner.metrics()));
  EXPECT_EQ(result.checkpointsVerified, 16U);  // ticks 0, 20 ... 300

  // The runner's own log of commands and the recording say the same thing.
  const std::vector<warehousesim::simulation::CommandRecord> finalLog = runner.commandLog();
  ASSERT_EQ(finalLog.size(), recording->commands.size());
  for (std::size_t i = 0; i < finalLog.size(); ++i) {
    EXPECT_EQ(finalLog[i].tick, recording->commands[i].tick) << i;
    EXPECT_TRUE(finalLog[i].command == recording->commands[i].command) << i;
    EXPECT_EQ(finalLog[i].outcome.accepted, recording->commands[i].accepted) << i;
  }
  const std::vector<std::size_t> accepted = acceptedByKind(*recording);
  for (std::size_t kind = 0; kind + 1 < accepted.size(); ++kind) {
    EXPECT_GT(accepted[kind], 0U) << "no accepted command of kind " << kind;
  }
}

TEST(RunnerRecordingTest, TheSameCommandsAtTheSameTicksGiveTheRecordingWhateverTheTimingWas) {
  // Two runners, two different timings; replaying the log of the first on the second's recording
  // shows that only the ticks of the commands matter.
  SimulationRunner first = makeRunner(6);
  first.startRecording(50);
  first.start(150);
  ASSERT_TRUE(first.post(SubmitTask{P(3, 3), P(9, 1)}));
  ASSERT_TRUE(first.waitForTick(150, kTimeout));
  first.stop();
  Recording recording = *first.recording();
  ASSERT_EQ(recording.commands.size(), 1U);

  // Played against a simulation that nobody timed: the same state at the same ticks.
  Recorder byHand(*warehousesim::simulation::makeLayoutSimulation("compact", 6), 50);
  byHand.run(recording.commands[0].tick);
  (void)byHand.apply(recording.commands[0].command);
  byHand.run(150 - recording.commands[0].tick);
  EXPECT_EQ(byHand.simulation().stateChecksum(), recording.end->checksum);
  EXPECT_EQ(byHand.recording().checkpoints.size(), recording.checkpoints.size());
  for (std::size_t i = 0; i < recording.checkpoints.size(); ++i) {
    EXPECT_EQ(byHand.recording().checkpoints[i].checksum, recording.checkpoints[i].checksum) << i;
  }
}

TEST(RunnerRecordingTest, PausingAndChangingTheSpeedAreNotInTheRecordingAndChangeNothingInIt) {
  const auto run = [](bool fiddle) {
    SimulationRunner runner = makeRunner(4);
    runner.startRecording(25);
    runner.start(100);
    if (fiddle) {
      for (int i = 0; i < 5; ++i) {
        runner.setPaused(true);
        std::this_thread::sleep_for(milliseconds(2));
        runner.setSpeed(i % 2 == 0 ? 16.0 : 0.5);
        runner.setPaused(false);
        std::this_thread::sleep_for(milliseconds(2));
      }
      runner.setSpeed(16.0);
    }
    EXPECT_TRUE(runner.waitForTick(100, kTimeout));
    runner.stop();
    return *runner.recording();
  };
  const Recording calm = run(false);
  const Recording fiddled = run(true);
  EXPECT_TRUE(calm.commands.empty());
  EXPECT_TRUE(fiddled.commands.empty());
  EXPECT_EQ(calm.end->checksum, fiddled.end->checksum);
  EXPECT_TRUE(calm.checkpoints == fiddled.checkpoints);
  EXPECT_TRUE(calm.taskEvents == fiddled.taskEvents);
}

TEST(RunnerRecordingTest, ARestartBeginsANewRecordingOnTheNewRun) {
  SimulationRunner runner = makeRunner(3);
  runner.startRecording(10);
  runner.start(60);
  ASSERT_TRUE(runner.post(SubmitTask{P(3, 3), P(9, 1)}));
  ASSERT_TRUE(runner.waitForTick(60, kTimeout));
  ASSERT_EQ(runner.recording()->commands.size(), 1U);

  runner.setPaused(true);  // so that the new run stays where it begins while it is looked at
  ASSERT_TRUE(runner.post(ResetScenario{9}));
  waitForReset(runner);
  const std::optional<Recording> after = runner.recording();
  ASSERT_TRUE(after.has_value());
  EXPECT_EQ(after->scenario.config.seed, 9U);
  EXPECT_EQ(after->ticks, 0U);
  EXPECT_TRUE(after->commands.empty()) << "a reset is not part of the run it starts";
  EXPECT_EQ(after->checkpoints.size(), 1U);

  runner.setPaused(false);
  ASSERT_TRUE(runner.waitForTick(60, kTimeout));
  runner.stop();
  const Recording finished = *runner.recording();
  EXPECT_EQ(finished.scenario.config.seed, 9U);
  EXPECT_EQ(finished.ticks, 60U);
  EXPECT_EQ(finished.checkpointInterval, 10U) << "the new run is recorded as the old one was";
  EXPECT_EQ(finished.checkpoints.size(), 7U);  // ticks 0, 10 ... 60
  const ReplayResult result = replay(finished);
  EXPECT_TRUE(result.ok()) << (result.divergence ? result.divergence->what : "");
  EXPECT_TRUE(result.snapshot == runner.snapshot());
}

TEST(RunnerRecordingTest, ARestartIntoARunThatCannotBeRecordedEndsTheRecording) {
  // The first run is an ordinary one; what the factory builds after a reset has a dispatcher of its
  // own, which a replay could not rebuild.
  const std::vector<std::string> rows =
      warehousesim::simulation::makeLayoutSimulation("compact", 1)->warehouse().toRows();
  const ScenarioFactory odd = [rows](std::uint64_t seed) {
    SimulationConfig config;
    config.seed = seed;
    config.taskSpawnInterval = 5;
    return Simulation(
        *warehousesim::world::Warehouse::fromRows(rows), {P(1, 1)}, config, nullptr,
        std::make_unique<ScriptedDispatcher>(std::vector<std::pair<TaskId, RobotId>>{}));
  };
  SimulationRunner runner(*warehousesim::simulation::makeLayoutSimulation("compact", 1),
                          milliseconds(1), odd);
  runner.startRecording(10);
  ASSERT_TRUE(runner.recording().has_value());
  runner.start(30);
  ASSERT_TRUE(runner.post(ResetScenario{}));
  waitForReset(runner);
  EXPECT_FALSE(runner.recording().has_value());
  runner.stop();
}

}  // namespace
