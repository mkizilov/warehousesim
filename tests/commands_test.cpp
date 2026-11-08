#include "warehousesim/simulation/commands.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"

namespace {

using namespace warehousesim_test;
using warehousesim::simulation::apply;
using warehousesim::simulation::CommandOutcome;
using warehousesim::simulation::SetTaskPriority;
using warehousesim::simulation::SubmitTask;

// Idle robots, no random tasks: a task exists only if a command made it.
Simulation quiet() {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  return Simulation(layout({"P.....D", ".......", "S.SSS.S", "C......"}), {P(0, 3), P(2, 3)},
                    config);
}

std::string json(const Simulation& sim) {
  return warehousesim::simulation::toJson(sim.snapshot()).dump();
}

bool mentions(const CommandOutcome& outcome, const std::string& part) {
  return outcome.message.find(part) != std::string::npos;
}

// ---- adding a task ------------------------------------------------------------------------------

TEST(CommandsTest, ASubmittedTaskIsAddedWithItsPriorityAndReportedByNumber) {
  Simulation sim = quiet();
  const CommandOutcome outcome = apply(sim, SubmitTask{P(0, 0), P(6, 0), TaskPriority::High});
  EXPECT_TRUE(outcome.accepted);
  ASSERT_TRUE(outcome.task.has_value());
  EXPECT_EQ(outcome.task->value, 1U);
  EXPECT_TRUE(mentions(outcome, "task #1 added")) << outcome.message;
  EXPECT_TRUE(mentions(outcome, "(0, 0) to (6, 0)")) << outcome.message;
  EXPECT_TRUE(mentions(outcome, "high")) << outcome.message;
  const auto* task = sim.tasks().find(*outcome.task);
  ASSERT_TRUE(task != nullptr);
  EXPECT_EQ(task->pickup, P(0, 0));
  EXPECT_EQ(task->dropoff, P(6, 0));
  EXPECT_EQ(task->priority, TaskPriority::High);
  EXPECT_EQ(task->status, TaskStatus::Pending);
  EXPECT_EQ(task->createdAt, sim.tick());
}

TEST(CommandsTest, TaskNumbersFollowOnAndTheDefaultPriorityIsNormal) {
  Simulation sim = quiet();
  EXPECT_EQ(apply(sim, SubmitTask{P(0, 0), P(6, 0)}).task->value, 1U);
  EXPECT_EQ(apply(sim, SubmitTask{P(1, 0), P(5, 0)}).task->value, 2U);
  EXPECT_EQ(sim.tasks().find(TaskId{2})->priority, TaskPriority::Normal);
}

TEST(CommandsTest, ACellOutsideTheMapIsRefusedWithoutChangingAnything) {
  Simulation sim = quiet();
  const std::string before = json(sim);
  for (const Point bad : {P(-1, 0), P(0, -1), P(7, 0), P(0, 4), P(100, 100), P(-2147483647 - 1, 0),
                          P(2147483647, 2147483647)}) {
    const CommandOutcome asPickup = apply(sim, SubmitTask{bad, P(6, 0)});
    EXPECT_FALSE(asPickup.accepted);
    EXPECT_TRUE(mentions(asPickup, "pickup")) << asPickup.message;
    EXPECT_TRUE(mentions(asPickup, "outside the map")) << asPickup.message;
    const CommandOutcome asDropoff = apply(sim, SubmitTask{P(0, 0), bad});
    EXPECT_FALSE(asDropoff.accepted);
    EXPECT_TRUE(mentions(asDropoff, "dropoff")) << asDropoff.message;
    EXPECT_FALSE(asDropoff.task.has_value());
  }
  EXPECT_EQ(json(sim), before);
}

TEST(CommandsTest, AnObstacleCellIsRefusedAndNamedForWhatItIs) {
  Simulation sim = quiet();
  const CommandOutcome shelf = apply(sim, SubmitTask{P(0, 2), P(6, 0)});
  EXPECT_FALSE(shelf.accepted);
  EXPECT_TRUE(mentions(shelf, "pickup (0, 2) is a shelf, not floor")) << shelf.message;
  EXPECT_EQ(sim.tasks().all().size(), 0U);

  Simulation walled(layout({"P#D", "..."}), {P(0, 1)}, ideal());
  const CommandOutcome wall = apply(walled, SubmitTask{P(0, 0), P(1, 0)});
  EXPECT_TRUE(mentions(wall, "dropoff (1, 0) is a wall, not floor")) << wall.message;
}

TEST(CommandsTest, APickupAndDropoffThatAreTheSameCellAreRefused) {
  Simulation sim = quiet();
  const CommandOutcome outcome = apply(sim, SubmitTask{P(3, 0), P(3, 0)});
  EXPECT_FALSE(outcome.accepted);
  EXPECT_TRUE(mentions(outcome, "same cell")) << outcome.message;
  EXPECT_EQ(sim.tasks().all().size(), 0U);
}

TEST(CommandsTest, AnyCellOfWalkableTypeIsAcceptedIncludingChargersAndOccupiedCells) {
  Simulation sim = quiet();
  EXPECT_TRUE(apply(sim, SubmitTask{P(0, 3), P(2, 3)}).accepted);  // chargers, with robots on them
  EXPECT_TRUE(apply(sim, SubmitTask{P(0, 0), P(6, 0)}).accepted);  // pickup and dropoff cells
}

TEST(CommandsTest, ATaskTheRobotsCannotReachIsAcceptedAndThenFailsAsTheDispatcherFindsIt) {
  Simulation sim(layout({"P.#.D", "..#.."}), {P(0, 1)}, ideal());
  const CommandOutcome outcome = apply(sim, SubmitTask{P(0, 0), P(4, 0)});
  EXPECT_TRUE(outcome.accepted);
  sim.run(3);
  EXPECT_EQ(sim.tasks().find(*outcome.task)->status, TaskStatus::Failed);
}

// ---- changing a priority ------------------------------------------------------------------------

TEST(CommandsTest, AWaitingTaskChangesPriorityAndTheDispatcherThenTakesItFirst) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P...D", ".....", "....."}), {P(0, 2)}, config);
  const TaskId first = *apply(sim, SubmitTask{P(0, 0), P(4, 0)}).task;
  const TaskId second = *apply(sim, SubmitTask{P(0, 1), P(4, 1)}).task;
  const CommandOutcome outcome = apply(sim, SetTaskPriority{second, TaskPriority::Urgent});
  EXPECT_TRUE(outcome.accepted) << outcome.message;
  EXPECT_TRUE(mentions(outcome, "task #2 is now urgent")) << outcome.message;
  EXPECT_EQ(outcome.task, second);
  sim.step();
  EXPECT_EQ(sim.tasks().find(second)->status, TaskStatus::Assigned);
  EXPECT_EQ(sim.tasks().find(first)->status, TaskStatus::Pending);
}

TEST(CommandsTest, ATaskARobotHasClaimedIsRefusedWithItsStateInTheReason) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P...D", "....."}), {P(0, 1)}, config);
  const TaskId id = *apply(sim, SubmitTask{P(0, 0), P(4, 0)}).task;
  sim.step();
  ASSERT_EQ(sim.tasks().find(id)->status, TaskStatus::Assigned);
  const CommandOutcome outcome = apply(sim, SetTaskPriority{id, TaskPriority::Urgent});
  EXPECT_FALSE(outcome.accepted);
  EXPECT_TRUE(mentions(outcome, "task #1 is assigned")) << outcome.message;
  EXPECT_TRUE(mentions(outcome, "only a task that is still waiting")) << outcome.message;
  EXPECT_EQ(sim.tasks().find(id)->priority, TaskPriority::Normal);
}

TEST(CommandsTest, AFinishedTaskCannotChangePriority) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P.D"}), {P(1, 0)}, config);
  const TaskId id = *apply(sim, SubmitTask{P(0, 0), P(2, 0)}).task;
  sim.run(40);
  ASSERT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  const CommandOutcome outcome = apply(sim, SetTaskPriority{id, TaskPriority::Urgent});
  EXPECT_FALSE(outcome.accepted);
  EXPECT_TRUE(mentions(outcome, "completed")) << outcome.message;
}

TEST(CommandsTest, AnUnknownTaskIsRefusedNotThrown) {
  Simulation sim = quiet();
  const std::string before = json(sim);
  for (const std::uint32_t number : {0U, 1U, 99U, 4294967295U}) {
    const CommandOutcome outcome = apply(sim, SetTaskPriority{TaskId{number}, TaskPriority::High});
    EXPECT_FALSE(outcome.accepted);
    EXPECT_TRUE(mentions(outcome, "there is no task #" + std::to_string(number)))
        << outcome.message;
  }
  EXPECT_EQ(json(sim), before);
}

TEST(CommandsTest, TheSamePriorityIsAcceptedAsNoChange) {
  Simulation sim = quiet();
  const TaskId id = *apply(sim, SubmitTask{P(0, 0), P(6, 0), TaskPriority::High}).task;
  EXPECT_TRUE(apply(sim, SetTaskPriority{id, TaskPriority::High}).accepted);
}

TEST(CommandsTest, EveryPriorityCanBeChosenForANewTaskAndForAnOldOne) {
  Simulation sim = quiet();
  for (const TaskPriority priority :
       {TaskPriority::Low, TaskPriority::Normal, TaskPriority::High, TaskPriority::Urgent}) {
    const TaskId id = *apply(sim, SubmitTask{P(0, 0), P(6, 0), priority}).task;
    EXPECT_EQ(sim.tasks().find(id)->priority, priority);
    for (const TaskPriority next :
         {TaskPriority::Urgent, TaskPriority::Low, TaskPriority::High, TaskPriority::Normal}) {
      EXPECT_TRUE(apply(sim, SetTaskPriority{id, next}).accepted);
      EXPECT_EQ(sim.tasks().find(id)->priority, next);
    }
  }
}

// ---- the simulation's own part ------------------------------------------------------------------

TEST(SimulationPriorityTest, AnUnknownTaskThrowsAndAKnownOneReportsWhetherItChanged) {
  Simulation sim = quiet();
  EXPECT_THROW((void)sim.setTaskPriority(TaskId{1}, TaskPriority::High), std::out_of_range);
  const TaskId id = sim.submitTask(P(0, 0), P(6, 0));
  EXPECT_TRUE(sim.setTaskPriority(id, TaskPriority::High));
  EXPECT_TRUE(sim.cancelTask(id));
  EXPECT_FALSE(sim.setTaskPriority(id, TaskPriority::Low));
  EXPECT_EQ(sim.tasks().find(id)->priority, TaskPriority::High);
}

// ---- finished tasks in the snapshot -------------------------------------------------------------

TEST(SnapshotRecentTasksTest, ThereAreNoneUntilATaskFinishesAndOpenTasksAreNeverInThem) {
  Simulation sim = quiet();
  EXPECT_TRUE(sim.snapshot().recentTasks.empty());
  (void)sim.submitTask(P(0, 0), P(6, 0));
  EXPECT_TRUE(sim.snapshot().recentTasks.empty());
  EXPECT_EQ(sim.snapshot().activeTasks.size(), 1U);
}

TEST(SnapshotRecentTasksTest,
     FinishedTasksOfEveryKindAreListedNewestFirstAndOnlyTheLastFewAreKept) {
  Simulation sim = quiet();
  std::vector<TaskId> ids;
  for (int i = 0; i < 20; ++i) {
    ids.push_back(sim.submitTask(P(0, 0), P(6, 0)));
  }
  const TaskId stillOpen = sim.submitTask(P(1, 0), P(5, 0));
  for (const TaskId id : ids) {
    ASSERT_TRUE(sim.cancelTask(id));
  }
  const auto snapshot = sim.snapshot();
  ASSERT_EQ(snapshot.recentTasks.size(), warehousesim::simulation::kRecentTaskCount);
  for (std::size_t i = 0; i < snapshot.recentTasks.size(); ++i) {
    EXPECT_EQ(snapshot.recentTasks[i].id, ids[ids.size() - 1 - i]) << i;
    EXPECT_EQ(snapshot.recentTasks[i].status, TaskStatus::Cancelled);
  }
  EXPECT_EQ(snapshot.cancelledTasks, 20U);  // the counters still count them all
  ASSERT_EQ(snapshot.activeTasks.size(), 1U);
  EXPECT_EQ(snapshot.activeTasks[0].id, stillOpen);
}

TEST(SnapshotRecentTasksTest, CompletedAndFailedTasksAppearToo) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P.D#.", "...#."}), {P(1, 1)}, config);
  const TaskId done = sim.submitTask(P(0, 0), P(2, 0));
  const TaskId impossible = sim.submitTask(P(0, 0), P(4, 0));  // behind the wall
  sim.run(80);
  const auto snapshot = sim.snapshot();
  ASSERT_EQ(snapshot.recentTasks.size(), 2U);
  std::set<std::uint32_t> ids{snapshot.recentTasks[0].id.value, snapshot.recentTasks[1].id.value};
  EXPECT_EQ(ids, (std::set<std::uint32_t>{done.value, impossible.value}));
  EXPECT_EQ(sim.tasks().find(done)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.tasks().find(impossible)->status, TaskStatus::Failed);
  EXPECT_TRUE(snapshot.activeTasks.empty());
}

TEST(SnapshotRecentTasksTest, TheJsonReportKeepsToTheCounters) {
  Simulation sim = quiet();
  const TaskId id = sim.submitTask(P(0, 0), P(6, 0));
  ASSERT_TRUE(sim.cancelTask(id));
  const auto report = warehousesim::simulation::toJson(sim.snapshot());
  EXPECT_FALSE(report.contains("recentTasks"));
  EXPECT_EQ(report["cancelledTasks"], 1);
}

TEST(SnapshotRecentTasksTest, TwoEqualSimulationsHaveEqualSnapshotsIncludingTheRecentTasks) {
  Simulation a = quiet();
  Simulation b = quiet();
  for (Simulation* sim : {&a, &b}) {
    const TaskId id = sim->submitTask(P(0, 0), P(6, 0));
    (void)sim->cancelTask(id);
    sim->run(5);
  }
  EXPECT_TRUE(a.snapshot() == b.snapshot());
}

// ---- values -------------------------------------------------------------------------------------

TEST(CommandsTest, CommandsAreComparableValues) {
  EXPECT_TRUE((SubmitTask{P(1, 2), P(3, 4), TaskPriority::Low}) ==
              (SubmitTask{P(1, 2), P(3, 4), TaskPriority::Low}));
  EXPECT_FALSE((SubmitTask{P(1, 2), P(3, 4), TaskPriority::Low}) ==
               (SubmitTask{P(1, 2), P(3, 4), TaskPriority::High}));
  EXPECT_TRUE((SetTaskPriority{TaskId{1}, TaskPriority::Low}) ==
              (SetTaskPriority{TaskId{1}, TaskPriority::Low}));
  EXPECT_FALSE((SetTaskPriority{TaskId{1}, TaskPriority::Low}) ==
               (SetTaskPriority{TaskId{2}, TaskPriority::Low}));
}

// ---- a task made by a command is an ordinary task -----------------------------------------------

TEST(CommandsTest, ACommandTaskIsDeliveredLikeAnyOther) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P...D", "....."}), {P(2, 1)}, config);
  const TaskId id = *apply(sim, SubmitTask{P(0, 0), P(4, 0), TaskPriority::Urgent}).task;
  sim.run(60);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.snapshot().completedTasks, 1U);
}

TEST(CommandsTest, TaskCommandsDoNotDisturbTheRandomTaskStream) {
  // Tasks the generator makes are the same with or without a task added by hand: the command takes
  // no draw from the generator's random numbers.
  const auto generated = [](bool withCommand) {
    Simulation sim = warehousesim::simulation::makeDemoSimulation(7);
    if (withCommand) {
      EXPECT_TRUE(apply(sim, SubmitTask{P(0, 5), P(0, 11)}).accepted);
    }
    sim.run(100);
    std::vector<std::string> made;
    for (const auto& task : sim.tasks().all()) {
      if (withCommand && task.id.value == 1) {
        continue;  // the one added by hand
      }
      made.push_back(std::to_string(task.pickup.x) + "," + std::to_string(task.pickup.y) + ">" +
                     std::to_string(task.dropoff.x) + "," + std::to_string(task.dropoff.y) + " " +
                     warehousesim::tasks::toString(task.priority));
    }
    return made;
  };
  EXPECT_EQ(generated(false), generated(true));
}

}  // namespace
