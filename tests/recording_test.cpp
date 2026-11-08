#include "warehousesim/simulation/recording.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "recording_support.hpp"

namespace {

using namespace warehousesim_test;
using warehousesim::simulation::Checkpoint;
using warehousesim::simulation::EventSource;
using warehousesim::simulation::kEngineRevision;
using warehousesim::simulation::RecordedCommand;
using warehousesim::simulation::ResetScenario;
using warehousesim::simulation::RunJournal;
using warehousesim::simulation::TaskEvent;
using warehousesim::simulation::TaskEventKind;
using warehousesim::simulation::validate;

std::vector<TaskEvent> eventsOf(const Recording& recording, TaskEventKind kind) {
  std::vector<TaskEvent> found;
  for (const TaskEvent& event : recording.taskEvents) {
    if (event.kind == kind) {
      found.push_back(event);
    }
  }
  return found;
}

// ---- what a recording starts from ---------------------------------------------------------------

TEST(RunJournalTest, ARecordingStartsAtTickZero) {
  Simulation sim = quietSimulation();
  sim.step();
  EXPECT_THROW(RunJournal(sim, 10), std::invalid_argument);
  try {
    RunJournal journal(sim, 10);
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("tick 1"), std::string::npos) << error.what();
  }
}

// Why a journal refuses a simulation, or an empty string if it does not.
std::string refusalOf(const Simulation& sim) {
  try {
    RunJournal journal(sim, 10);
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return {};
}

TEST(RunJournalTest, ARecordingStartsBeforeTheFirstTask) {
  Simulation sim = quietSimulation();
  (void)sim.submitTask(P(0, 0), P(6, 0));
  EXPECT_THROW(RunJournal(sim, 10), std::invalid_argument);
  EXPECT_EQ(refusalOf(sim), "a recording starts before the first task is made");
}

TEST(RunJournalTest, NothingMayHaveBeenDoneToTheSimulationBeforehand) {
  // Not even something the journal could not see: the simulation must be what its origin builds.
  const std::string changed = "the simulation was changed before the recording began";
  Simulation walled = quietSimulation();
  ASSERT_TRUE(walled.placeObstacle(P(3, 1)));
  EXPECT_THROW(RunJournal(walled, 10), std::invalid_argument);
  EXPECT_EQ(refusalOf(walled), changed);
  Simulation disabled = quietSimulation();
  ASSERT_TRUE(disabled.disableRobot(RobotId{1}));
  EXPECT_EQ(refusalOf(disabled), changed);
  EXPECT_NO_THROW(RunJournal(quietSimulation(), 10));
  EXPECT_EQ(refusalOf(quietSimulation()), "");
}

TEST(RunJournalTest, ASimulationWithItsOwnPlannerOrDispatcherCannotBeRecorded) {
  Scenario scenario(openFloor(5, 1), {Job{P(0, 0), P(1, 0), P(2, 0)}}, ideal());
  EXPECT_THROW(RunJournal(scenario.sim, 10), std::invalid_argument);
  EXPECT_NE(refusalOf(scenario.sim).find("its own planner or dispatcher"), std::string::npos)
      << refusalOf(scenario.sim);
}

TEST(RunJournalTest, AMissedStepIsAnErrorNotASilentGap) {
  Simulation sim = quietSimulation();
  RunJournal journal(sim, 10);
  sim.step();
  sim.step();
  EXPECT_THROW(journal.stepTaken(sim), std::logic_error);  // it was told of neither
  EXPECT_THROW(journal.commandApplied(sim, SubmitTask{P(0, 0), P(6, 0)}, true), std::logic_error);
  EXPECT_THROW((void)journal.recording(sim), std::logic_error);
}

TEST(RunJournalTest, AResetIsNotPartOfARun) {
  Simulation sim = quietSimulation();
  RunJournal journal(sim, 10);
  EXPECT_THROW(journal.commandApplied(sim, ResetScenario{}, true), std::invalid_argument);
}

// ---- what a recorded run contains ---------------------------------------------------------------

TEST(RecorderTest, ARecordingHoldsWhatTheRunWasBuiltFromAndHowLongItWent) {
  Recorder recorder(*warehousesim::simulation::makeLayoutSimulation("compact", 3), 50);
  recorder.run(120);
  const Recording recording = recorder.recording();
  EXPECT_EQ(recording.engineRevision, kEngineRevision);
  EXPECT_EQ(recording.ticks, 120U);
  EXPECT_EQ(recording.checkpointInterval, 50U);
  EXPECT_EQ(recording.scenario.warehouse.toRows(),
            recorder.simulation().origin().warehouse.toRows());
  EXPECT_EQ(recording.scenario.robotStarts, recorder.simulation().origin().robotStarts);
  EXPECT_EQ(recording.scenario.config.seed, 3U);
  EXPECT_EQ(recording.scenario.config.taskSpawnInterval, 5U);
  EXPECT_TRUE(recording.commands.empty());
  EXPECT_FALSE(validate(recording).has_value());
}

TEST(RecorderTest, CheckpointsAreTheSimulationsOwnChecksumAtTheTicksAskedFor) {
  Recorder recorder(*warehousesim::simulation::makeLayoutSimulation("demo", 5), 40);
  recorder.run(130);
  const Recording recording = recorder.recording();
  std::vector<warehousesim::core::Tick> ticks;
  for (const Checkpoint& checkpoint : recording.checkpoints) {
    ticks.push_back(checkpoint.tick);
  }
  EXPECT_EQ(ticks, (std::vector<warehousesim::core::Tick>{0, 40, 80, 120}));

  Simulation alone = *warehousesim::simulation::makeLayoutSimulation("demo", 5);
  std::vector<std::uint64_t> expected;
  for (warehousesim::core::Tick tick = 0; tick <= 130; ++tick) {
    if (tick % 40 == 0) {
      expected.push_back(alone.stateChecksum());
    }
    if (tick < 130) {
      alone.step();
    }
  }
  ASSERT_EQ(recording.checkpoints.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(recording.checkpoints[i].checksum, expected[i])
        << "tick " << recording.checkpoints[i].tick;
  }
  ASSERT_TRUE(recording.end.has_value());
  EXPECT_EQ(recording.end->tick, 130U);
  EXPECT_EQ(recording.end->checksum, alone.stateChecksum());
}

TEST(RecorderTest, AnIntervalOfZeroKeepsOnlyTheStartAndTheEnd) {
  Recorder recorder(*warehousesim::simulation::makeLayoutSimulation("demo", 5), 0);
  recorder.run(75);
  const Recording recording = recorder.recording();
  ASSERT_EQ(recording.checkpoints.size(), 1U);
  EXPECT_EQ(recording.checkpoints[0].tick, 0U);
  ASSERT_TRUE(recording.end.has_value());
  EXPECT_EQ(recording.end->tick, 75U);
}

TEST(RecorderTest, ARecordingIsACopyAsOfNowAndAskingChangesNothing) {
  Recorder recorder(*warehousesim::simulation::makeLayoutSimulation("compact", 2), 25);
  recorder.run(30);
  const Recording early = recorder.recording();
  const std::uint64_t checksum = recorder.simulation().stateChecksum();
  (void)recorder.recording();
  EXPECT_EQ(recorder.simulation().stateChecksum(), checksum);
  recorder.run(30);
  EXPECT_EQ(early.ticks, 30U);
  EXPECT_EQ(early.end->tick, 30U);
  EXPECT_EQ(recorder.recording().ticks, 60U);
  EXPECT_EQ(recorder.recording().checkpoints.size(), 3U);  // 0, 25, 50
}

TEST(RecorderTest, ARecordedRunIsTheSameRunAsOneThatWasNotRecorded) {
  for (const std::string& name : bundledLayouts()) {
    Simulation plain = *warehousesim::simulation::makeLayoutSimulation(name, 6);
    Recorder recorder(*warehousesim::simulation::makeLayoutSimulation(name, 6), 25);
    for (int tick = 0; tick < 200; ++tick) {
      plain.step();
      recorder.step();
      ASSERT_TRUE(plain.snapshot() == recorder.simulation().snapshot()) << name << " tick " << tick;
    }
  }
}

// ---- commands -----------------------------------------------------------------------------------

TEST(RecorderTest, CommandsAreKeptWithTheirTickTheirOutcomeAndInTheOrderTheyWereGiven) {
  Recorder recorder(quietSimulation(), 10);
  const auto made = recorder.apply(SubmitTask{P(0, 0), P(6, 0)});
  EXPECT_TRUE(made.accepted);
  EXPECT_FALSE(recorder.apply(CancelTask{TaskId{9}}).accepted);  // refused, and still kept
  recorder.run(4);
  (void)recorder.apply(SetTaskPriority{*made.task, TaskPriority::Urgent});  // kept as it came out
  EXPECT_TRUE(recorder.apply(SetRobotService{RobotId{2}, false}).accepted);
  recorder.run(3);
  const Recording recording = recorder.recording();
  ASSERT_EQ(recording.commands.size(), 4U);
  EXPECT_EQ(recording.commands[0].tick, 0U);
  EXPECT_TRUE(recording.commands[0].accepted);
  EXPECT_TRUE((recording.commands[0].command == Command{SubmitTask{P(0, 0), P(6, 0)}}));
  EXPECT_EQ(recording.commands[1].tick, 0U);
  EXPECT_FALSE(recording.commands[1].accepted);
  EXPECT_TRUE(recording.commands[1].command == Command{CancelTask{TaskId{9}}});
  EXPECT_EQ(recording.commands[2].tick, 4U);
  EXPECT_EQ(recording.commands[3].tick, 4U);
  EXPECT_TRUE((recording.commands[3].command == Command{SetRobotService{RobotId{2}, false}}));
  EXPECT_EQ(recording.ticks, 7U);
  EXPECT_FALSE(validate(recording).has_value());
}

TEST(RecorderTest, AResetIsRefusedAndNotKept) {
  Recorder recorder(quietSimulation(), 10);
  const Command reset = ResetScenario{};
  const auto outcome = recorder.apply(reset);
  EXPECT_FALSE(outcome.accepted);
  EXPECT_TRUE(recorder.recording().commands.empty());
}

TEST(RecorderTest, EveryKindOfCommandCanBeKept) {
  const Recording recording = busyRecording("compact", 4, 100);
  const std::vector<std::size_t> accepted = acceptedByKind(recording);
  for (std::size_t kind = 0; kind + 1 < accepted.size(); ++kind) {  // all but ResetScenario
    EXPECT_GT(accepted[kind], 0U) << "no accepted command of kind " << kind;
  }
  EXPECT_EQ(accepted.back(), 0U);
  const auto refused = std::count_if(recording.commands.begin(), recording.commands.end(),
                                     [](const RecordedCommand& entry) { return !entry.accepted; });
  EXPECT_EQ(refused, 2);
  EXPECT_FALSE(validate(recording).has_value());
}

// ---- tasks made and cancelled -------------------------------------------------------------------

TEST(RecorderTest, EveryTaskTheGeneratorMakesIsAnEngineEventAtTheTickItWasMade) {
  Recorder recorder(*warehousesim::simulation::makeLayoutSimulation("compact", 8), 50);
  recorder.run(61);
  const Recording recording = recorder.recording();
  const std::vector<TaskEvent> created = eventsOf(recording, TaskEventKind::Created);
  ASSERT_EQ(created.size(), recorder.simulation().tasks().all().size());
  ASSERT_EQ(created.size(), 13U);  // a task at tick 0, 5, 10 ... 60
  for (std::size_t i = 0; i < created.size(); ++i) {
    const auto& task = recorder.simulation().tasks().all()[i];
    EXPECT_EQ(created[i].tick, i * 5) << i;
    EXPECT_EQ(created[i].tick, task.createdAt);
    EXPECT_EQ(created[i].task, task.id);
    EXPECT_EQ(created[i].source, EventSource::Engine);
    EXPECT_EQ(created[i].pickup, task.pickup);
    EXPECT_EQ(created[i].dropoff, task.dropoff);
    EXPECT_EQ(created[i].priority, task.priority);
  }
  EXPECT_TRUE(eventsOf(recording, TaskEventKind::Cancelled).empty());
}

TEST(RecorderTest, ATaskMadeByACommandIsACommandEventAndComesBeforeTheStepsAtThatTick) {
  Recorder recorder(*warehousesim::simulation::makeLayoutSimulation("compact", 8), 50);
  recorder.run(10);
  const auto made = recorder.apply(SubmitTask{P(3, 3), P(9, 1), TaskPriority::High});
  recorder.run(1);  // the step that starts at tick 10 makes a task of its own
  const Recording recording = recorder.recording();
  std::vector<TaskEvent> atTen;
  for (const TaskEvent& event : recording.taskEvents) {
    if (event.tick == 10) {
      atTen.push_back(event);
    }
  }
  ASSERT_EQ(atTen.size(), 2U);
  EXPECT_EQ(atTen[0].source, EventSource::Command);
  EXPECT_EQ(atTen[0].task, *made.task);
  EXPECT_EQ(atTen[0].pickup, P(3, 3));
  EXPECT_EQ(atTen[0].priority, TaskPriority::High);
  EXPECT_EQ(atTen[1].source, EventSource::Engine);
  EXPECT_EQ(atTen[1].task.value, atTen[0].task.value + 1);
  EXPECT_FALSE(validate(recording).has_value());
}

TEST(RecorderTest, ACancellationIsAnEventAtTheTickOfTheCommand) {
  Recorder recorder(quietSimulation(), 10);
  const auto made = recorder.apply(SubmitTask{P(0, 0), P(6, 0)});
  recorder.run(3);
  ASSERT_TRUE(recorder.apply(CancelTask{*made.task}).accepted);
  recorder.run(2);
  const std::vector<TaskEvent> cancelled = eventsOf(recorder.recording(), TaskEventKind::Cancelled);
  ASSERT_EQ(cancelled.size(), 1U);
  EXPECT_EQ(cancelled[0].tick, 3U);
  EXPECT_EQ(cancelled[0].task, *made.task);
  EXPECT_EQ(cancelled[0].source, EventSource::Command);
}

TEST(RecorderTest, ACancellationThatFollowsFromAWallIsReportedToo) {
  Recorder recorder(quietSimulation(), 10);
  const auto onTheCell = recorder.apply(SubmitTask{P(3, 1), P(6, 0)});
  const auto elsewhere = recorder.apply(SubmitTask{P(1, 1), P(6, 0)});
  recorder.run(2);
  ASSERT_TRUE(recorder.apply(PlaceObstacle{P(3, 1), CellType::Wall}).accepted);
  const Recording recording = recorder.recording();
  const std::vector<TaskEvent> cancelled = eventsOf(recording, TaskEventKind::Cancelled);
  ASSERT_EQ(cancelled.size(), 1U);
  EXPECT_EQ(cancelled[0].task, *onTheCell.task);
  EXPECT_EQ(cancelled[0].tick, 2U);
  EXPECT_EQ(cancelled[0].source, EventSource::Command);
  EXPECT_NE(cancelled[0].task, *elsewhere.task);
}

TEST(RecorderTest, ThoseCancelledAtOnceComeOutInOrderOfId) {
  Recorder recorder(quietSimulation(), 10);
  for (int i = 0; i < 4; ++i) {
    (void)recorder.apply(SubmitTask{P(3, 1), P(6, 0)});
  }
  ASSERT_TRUE(recorder.apply(PlaceObstacle{P(3, 1), CellType::Wall}).accepted);
  const std::vector<TaskEvent> cancelled = eventsOf(recorder.recording(), TaskEventKind::Cancelled);
  ASSERT_EQ(cancelled.size(), 4U);
  for (std::uint32_t i = 0; i < 4; ++i) {
    EXPECT_EQ(cancelled[i].task.value, i + 1);
  }
}

TEST(RecorderTest, FinishedTasksAreNotReportedAsCancelled) {
  Recorder recorder(quietSimulation(), 10);
  const auto made = recorder.apply(SubmitTask{P(0, 0), P(6, 0)});
  recorder.run(60);
  ASSERT_EQ(recorder.simulation().tasks().find(*made.task)->status, TaskStatus::Completed);
  EXPECT_TRUE(eventsOf(recorder.recording(), TaskEventKind::Cancelled).empty());
}

// ---- what validate() demands --------------------------------------------------------------------

Recording valid() {
  Recorder recorder(quietSimulation(), 10);
  const auto made = recorder.apply(SubmitTask{P(0, 0), P(6, 0)});
  recorder.run(3);  // the task is still open: a robot is on its way
  (void)recorder.apply(CancelTask{*made.task});
  recorder.run(22);
  Recording recording = recorder.recording();
  EXPECT_FALSE(validate(recording).has_value());
  return recording;
}

std::string problemOf(const Recording& recording) {
  const auto found = validate(recording);
  return found ? found->text() : std::string();
}

TEST(ValidateTest, ARecordedRunIsValid) {
  EXPECT_FALSE(validate(valid()).has_value());
  EXPECT_FALSE(validate(busyRecording("fulfilment", 2, 120)).has_value());
}

TEST(ValidateTest, CommandsMustStayWithinTheRunAndInOrder) {
  Recording late = valid();
  late.commands[0].tick = 26;
  EXPECT_EQ(problemOf(late), "/commands/0/tick: tick 26 is after the end of the run (tick 25)");

  Recording unordered = valid();
  ASSERT_GE(unordered.commands.size(), 2U);
  std::swap(unordered.commands[0], unordered.commands[1]);
  EXPECT_NE(problemOf(unordered).find("/commands/1/tick: tick 0 is before the command before it"),
            std::string::npos)
      << problemOf(unordered);

  Recording reset = valid();
  reset.commands.push_back({25, ResetScenario{}, true});
  EXPECT_NE(problemOf(reset).find("/commands/2/command: a reset"), std::string::npos)
      << problemOf(reset);
}

TEST(ValidateTest, EventsMustBeInOrderNumberedFromOneAndCancelOnlyWhatExists) {
  Recording late = valid();
  late.taskEvents.back().tick = 99;
  EXPECT_NE(problemOf(late).find("/taskEvents/1/tick: tick 99 is after the end"), std::string::npos)
      << problemOf(late);

  Recording skipped = valid();
  skipped.taskEvents[0].task = TaskId{2};
  EXPECT_NE(problemOf(skipped).find("/taskEvents/0/task: tasks are numbered 1, 2, 3"),
            std::string::npos)
      << problemOf(skipped);

  Recording unknown = valid();
  unknown.taskEvents[1].task = TaskId{5};
  EXPECT_NE(problemOf(unknown).find("/taskEvents/1/task: task 5 has not been made"),
            std::string::npos)
      << problemOf(unknown);

  Recording twice = valid();
  twice.taskEvents.push_back(twice.taskEvents.back());
  EXPECT_NE(problemOf(twice).find("was cancelled before"), std::string::npos) << problemOf(twice);

  Recording backwards = valid();
  backwards.taskEvents[0].tick = 20;
  backwards.taskEvents[1].tick = 3;
  EXPECT_NE(problemOf(backwards).find("/taskEvents/1: out of order"), std::string::npos)
      << problemOf(backwards);

  Recording stepBeforeCommand = valid();  // at one tick, a command's events come first
  stepBeforeCommand.taskEvents[0].source = EventSource::Engine;
  stepBeforeCommand.taskEvents[1].source = EventSource::Command;
  stepBeforeCommand.taskEvents[1].tick = 0;
  EXPECT_NE(problemOf(stepBeforeCommand).find("out of order"), std::string::npos)
      << problemOf(stepBeforeCommand);
}

TEST(ValidateTest, CheckpointsMustIncreaseAndStayWithinTheRun) {
  Recording beyond = valid();
  beyond.checkpoints.push_back({26, 1});
  EXPECT_NE(problemOf(beyond).find("/checkpoints/3/tick: tick 26 is after the end"),
            std::string::npos)
      << problemOf(beyond);

  Recording same = valid();
  same.checkpoints[1].tick = same.checkpoints[0].tick;
  EXPECT_NE(problemOf(same).find("/checkpoints/1/tick: checkpoints are listed by increasing tick"),
            std::string::npos)
      << problemOf(same);

  Recording shortEnd = valid();
  shortEnd.end->tick = 24;
  EXPECT_NE(problemOf(shortEnd).find("/end/tick: the end checkpoint is at tick 24"),
            std::string::npos)
      << problemOf(shortEnd);

  Recording none = valid();
  none.checkpoints.clear();
  none.end.reset();
  none.commands.clear();
  none.taskEvents.clear();
  EXPECT_FALSE(validate(none).has_value()) << "a scenario script has nothing to check";
}

TEST(ValidateTest, TheLimitsAreInclusive) {
  Recording most = valid();
  most.ticks = 1'000'000'000;
  most.end->tick = most.ticks;
  EXPECT_FALSE(validate(most).has_value()) << problemOf(most);
  Recording crowd = valid();
  crowd.scenario.robotStarts.assign(4096, P(0, 0));
  EXPECT_FALSE(validate(crowd).has_value()) << problemOf(crowd);
  Recording side = valid();
  side.scenario.warehouse = warehousesim::world::Warehouse(512, 1);
  EXPECT_FALSE(validate(side).has_value()) << problemOf(side);
  side.scenario.warehouse = warehousesim::world::Warehouse(1, 512);
  EXPECT_FALSE(validate(side).has_value()) << problemOf(side);
  side.scenario.warehouse = warehousesim::world::Warehouse(1, 513);
  EXPECT_TRUE(validate(side).has_value());
}

TEST(ValidateTest, TheScenarioMustBeOfAReasonableSize) {
  Recording huge = valid();
  huge.ticks = 2'000'000'000;
  EXPECT_EQ(problemOf(huge), "/ticks: at most 1000000000 steps");

  Recording crowded = valid();
  crowded.scenario.robotStarts.assign(4097, P(0, 0));
  EXPECT_EQ(problemOf(crowded), "/scenario/robots: at most 4096 robots");

  Recording wide = valid();
  wide.scenario.warehouse = warehousesim::world::Warehouse(513, 2);
  EXPECT_EQ(problemOf(wide), "/scenario/layout: the layout must be 1 to 512 cells on a side");
}

}  // namespace
