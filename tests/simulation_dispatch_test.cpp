#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/tasks/dispatcher.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::NavigationGoal;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotState;
using warehousesim::simulation::makeLayoutSimulation;
using warehousesim::simulation::Simulation;
using warehousesim::simulation::SimulationConfig;
using warehousesim::tasks::Assignment;
using warehousesim::tasks::Dispatcher;
using warehousesim::tasks::DispatchPlan;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskPriority;
using warehousesim::tasks::TaskQueue;
using warehousesim::tasks::TaskStatus;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }
TaskId T(std::uint32_t id) { return TaskId{id}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// A dispatcher that ignores the world and proposes whatever it was told to, every tick.
class FixedPlanDispatcher final : public Dispatcher {
 public:
  explicit FixedPlanDispatcher(DispatchPlan plan) : plan_(std::move(plan)) {}

  [[nodiscard]] DispatchPlan plan(const Warehouse&, std::span<const Robot>,
                                  const TaskQueue&) override {
    return plan_;
  }

 private:
  DispatchPlan plan_;
};

bool isLive(TaskStatus status) {
  return status == TaskStatus::Assigned || status == TaskStatus::InProgress;
}

// Every task a robot holds is held by exactly that robot, and every live task is held by someone.
void expectExclusiveClaims(const Simulation& sim) {
  std::set<std::uint32_t> held;
  for (const Robot& robot : sim.robots()) {
    if (!robot.task()) {
      continue;
    }
    const Task* task = sim.tasks().find(*robot.task());
    ASSERT_NE(task, nullptr);
    EXPECT_TRUE(isLive(task->status)) << "robot " << robot.id().value << " holds a dead task";
    EXPECT_TRUE(task->assignee == robot.id());
    EXPECT_TRUE(held.insert(task->id.value).second)
        << "task " << task->id.value << " is held by two robots";
  }
  for (const Task& task : sim.tasks().all()) {
    EXPECT_EQ(isLive(task.status), held.contains(task.id.value))
        << "task " << task.id.value << " is " << toString(task.status);
  }
}

// ---- assignment order in a running simulation -------------------------------------------

TEST(SimulationDispatchTest, AnUrgentTaskIsAssignedBeforeAnOlderNearerLowPriorityOne) {
  Simulation sim(layout({"..........."}), {P(0, 0)});
  const TaskId low = sim.submitTask(P(1, 0), P(2, 0), TaskPriority::Low);
  const TaskId urgent = sim.submitTask(P(8, 0), P(9, 0), TaskPriority::Urgent);
  sim.step();
  EXPECT_EQ(sim.tasks().find(urgent)->status, TaskStatus::Assigned);
  EXPECT_EQ(sim.tasks().find(low)->status, TaskStatus::Pending);
  EXPECT_TRUE(sim.robots()[0].task() == urgent);
}

TEST(SimulationDispatchTest, OneRobotWorksThroughTasksByPriorityThenAgeThenId) {
  Simulation sim(layout({"..."}), {P(0, 0)});
  const std::vector<TaskPriority> priorities = {TaskPriority::Low, TaskPriority::Normal,
                                                TaskPriority::High, TaskPriority::Urgent,
                                                TaskPriority::Normal};
  for (const TaskPriority priority : priorities) {
    (void)sim.submitTask(P(1, 0), P(2, 0), priority);
  }

  std::vector<std::uint32_t> finishedInOrder;
  std::set<std::uint32_t> seen;
  for (int i = 0; i < 80 && finishedInOrder.size() < priorities.size(); ++i) {
    sim.step();
    for (const Task& task : sim.tasks().all()) {
      if (task.status == TaskStatus::Completed && seen.insert(task.id.value).second) {
        finishedInOrder.push_back(task.id.value);
      }
    }
  }
  // Urgent (4), High (3), then the two Normal tasks oldest-id first (2, 5), and Low (1) last.
  EXPECT_EQ(finishedInOrder, (std::vector<std::uint32_t>{4, 3, 2, 5, 1}));
}

TEST(SimulationDispatchTest, SpawnedTaskPrioritiesVaryOnlyWhenAskedTo) {
  for (const bool vary : {false, true}) {
    SimulationConfig config;
    config.taskSpawnInterval = 1;
    config.varyTaskPriority = vary;
    config.seed = 11;
    Simulation sim(layout({"S.....D", "..P...."}), {P(3, 1)}, config);
    sim.run(80);
    std::set<TaskPriority> priorities;
    for (const Task& task : sim.tasks().all()) {
      priorities.insert(task.priority);
    }
    if (vary) {
      EXPECT_GT(priorities.size(), 1U);
    } else {
      EXPECT_EQ(priorities, (std::set<TaskPriority>{TaskPriority::Normal}));
    }
  }
}

// ---- unavailable robots -----------------------------------------------------------------

TEST(SimulationDispatchTest, ADisabledRobotIsNeverGivenWorkUntilItIsRestored) {
  Simulation sim(layout({"........", "........"}), {P(0, 0), P(7, 0)});
  ASSERT_TRUE(sim.disableRobot(R(1)));
  const TaskId first = sim.submitTask(P(1, 0), P(2, 1));  // robot 1 would be the nearest
  for (int i = 0; i < 30 && sim.tasks().find(first)->status != TaskStatus::Completed; ++i) {
    sim.step();
    EXPECT_FALSE(sim.robots()[0].task().has_value());
    EXPECT_EQ(sim.robots()[0].state(), RobotState::Unavailable);
  }
  ASSERT_EQ(sim.tasks().find(first)->status, TaskStatus::Completed);
  EXPECT_TRUE(sim.tasks().find(first)->assignee == R(2));

  ASSERT_TRUE(sim.restoreRobot(R(1)));
  const TaskId second = sim.submitTask(P(0, 1), P(1, 1));  // right next to robot 1
  sim.step();
  EXPECT_TRUE(sim.tasks().find(second)->assignee == R(1));
}

TEST(SimulationDispatchTest, TaskWaitsWhileEveryRobotIsUnavailableAndRunsOnceOneReturns) {
  Simulation sim(layout({"....."}), {P(0, 0), P(4, 0)});
  ASSERT_TRUE(sim.disableRobot(R(1)));
  ASSERT_TRUE(sim.disableRobot(R(2)));
  const TaskId id = sim.submitTask(P(1, 0), P(2, 0));
  sim.run(20);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Pending);
  EXPECT_EQ(sim.snapshot().failedTasks, 0U);

  ASSERT_TRUE(sim.restoreRobot(R(1)));
  sim.run(20);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

TEST(SimulationDispatchTest, ADisabledRobotReturnsItsUncollectedTaskToTheQueue) {
  Simulation sim(layout({"........", "........"}), {P(0, 0), P(7, 1)});
  const TaskId id = sim.submitTask(P(2, 0), P(3, 0));
  sim.step();
  ASSERT_TRUE(sim.tasks().find(id)->assignee == R(1));
  ASSERT_EQ(sim.tasks().find(id)->status, TaskStatus::Assigned);
  ASSERT_TRUE(sim.disableRobot(R(1)));
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Pending);
  sim.step();  // another robot may now take it
  EXPECT_TRUE(sim.tasks().find(id)->assignee == R(2));
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Assigned);
}

// ---- unreachable destinations -----------------------------------------------------------

TEST(SimulationDispatchTest, AnUnreachableUrgentTaskIsRejectedWithoutBlockingTheRest) {
  Simulation sim(layout({"...#...", "...#..."}), {P(0, 0)});
  const TaskId impossible = sim.submitTask(P(5, 0), P(6, 0), TaskPriority::Urgent);
  const TaskId possible = sim.submitTask(P(1, 0), P(0, 1), TaskPriority::Normal);
  sim.step();
  EXPECT_EQ(sim.tasks().find(impossible)->status, TaskStatus::Failed);
  EXPECT_FALSE(sim.tasks().find(impossible)->assignee.has_value());
  EXPECT_EQ(sim.tasks().find(possible)->status, TaskStatus::Assigned);
  sim.run(10);
  EXPECT_EQ(sim.tasks().find(possible)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.snapshot().failedTasks, 1U);
  EXPECT_EQ(sim.snapshot().completedTasks, 1U);
}

TEST(SimulationDispatchTest, ATaskWhoseDropoffIsWalledOffFromItsPickupIsRejectedBeforeAnyoneMoves) {
  Simulation sim(layout({"...#...", "...#..."}), {P(0, 0), P(6, 1)});
  const TaskId id = sim.submitTask(P(1, 0), P(5, 1));
  sim.step();
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Failed);
  EXPECT_FALSE(sim.tasks().find(id)->assignee.has_value());
  for (const Robot& robot : sim.robots()) {
    EXPECT_EQ(robot.state(), RobotState::Idle);
    EXPECT_EQ(robot.cellsTravelled(), 0U);
  }
}

// ---- cancellations ----------------------------------------------------------------------

TEST(SimulationDispatchTest, CancellingAPendingTaskKeepsItFromEverBeingAssigned) {
  Simulation sim(layout({"....."}), {P(0, 0)});
  const TaskId id = sim.submitTask(P(2, 0), P(4, 0));
  EXPECT_TRUE(sim.cancelTask(id));
  sim.run(10);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Cancelled);
  EXPECT_FALSE(sim.tasks().find(id)->assignee.has_value());
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
  EXPECT_EQ(sim.robots()[0].cellsTravelled(), 0U);
  const auto snapshot = sim.snapshot();
  EXPECT_EQ(snapshot.cancelledTasks, 1U);
  EXPECT_EQ(snapshot.completedTasks, 0U);
  EXPECT_TRUE(snapshot.activeTasks.empty());
}

TEST(SimulationDispatchTest, CancelReportsFalseTwiceAndThrowsForUnknownIds) {
  Simulation sim(layout({"..."}), {P(0, 0)});
  const TaskId id = sim.submitTask(P(1, 0), P(2, 0));
  EXPECT_TRUE(sim.cancelTask(id));
  EXPECT_FALSE(sim.cancelTask(id));
  EXPECT_THROW((void)sim.cancelTask(T(99)), std::out_of_range);
  EXPECT_THROW((void)sim.cancelTask(TaskId{}), std::out_of_range);
  EXPECT_EQ(sim.snapshot().cancelledTasks, 1U);
}

TEST(SimulationDispatchTest, ACompletedTaskCannotBeCancelled) {
  Simulation sim(layout({"..."}), {P(0, 0)});
  const TaskId id = sim.submitTask(P(1, 0), P(2, 0));
  sim.run(10);
  ASSERT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_FALSE(sim.cancelTask(id));
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.snapshot().completedTasks, 1U);
  EXPECT_EQ(sim.snapshot().cancelledTasks, 0U);
}

TEST(SimulationDispatchTest, CancellingAnAssignedTaskStopsTheRobotAndFreesItForTheNextOne) {
  Simulation sim(layout({"......"}), {P(0, 0)});
  const TaskId first = sim.submitTask(P(4, 0), P(5, 0));
  sim.step();
  ASSERT_EQ(sim.robots()[0].state(), RobotState::Navigating);
  ASSERT_EQ(sim.robots()[0].goal(), NavigationGoal::Pickup);

  EXPECT_TRUE(sim.cancelTask(first));
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
  EXPECT_FALSE(sim.robots()[0].task().has_value());
  EXPECT_EQ(sim.tasks().find(first)->status, TaskStatus::Cancelled);
  EXPECT_TRUE(sim.tasks().find(first)->assignee == R(1));  // kept as a record of who held it

  const TaskId second = sim.submitTask(P(1, 0), P(0, 0));
  sim.run(12);
  EXPECT_EQ(sim.tasks().find(second)->status, TaskStatus::Completed);
  EXPECT_EQ(sim.tasks().find(first)->status, TaskStatus::Cancelled);  // never revived
  expectExclusiveClaims(sim);
}

TEST(SimulationDispatchTest, CancellingWhileCarryingDropsTheCargoWithoutCompletingTheTask) {
  SimulationConfig config;
  config.robot.loadTicks = 2;
  Simulation sim(layout({"......"}), {P(0, 0)}, config);
  const TaskId id = sim.submitTask(P(1, 0), P(5, 0));
  for (int i = 0; i < 20 && sim.robots()[0].goal() != NavigationGoal::Dropoff; ++i) {
    sim.step();
  }
  ASSERT_EQ(sim.robots()[0].goal(), NavigationGoal::Dropoff);
  ASSERT_EQ(sim.tasks().find(id)->status, TaskStatus::InProgress);

  EXPECT_TRUE(sim.cancelTask(id));
  EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle);
  sim.run(10);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Cancelled);
  EXPECT_EQ(sim.snapshot().completedTasks, 0U);
  EXPECT_EQ(sim.snapshot().failedTasks, 0U);
}

TEST(SimulationDispatchTest, CancellingAtEveryMomentOfATaskLeavesEverythingConsistent) {
  SimulationConfig config;
  config.robot.loadTicks = 2;
  config.robot.unloadTicks = 2;
  std::set<RobotState> cancelledFrom;
  int cancelled = 0;
  int tooLate = 0;
  for (int moment = 0; moment <= 14; ++moment) {
    Simulation sim(layout({"......"}), {P(0, 0)}, config);
    const TaskId id = sim.submitTask(P(2, 0), P(5, 0));
    sim.run(static_cast<warehousesim::core::Tick>(moment));
    const RobotState before = sim.robots()[0].state();

    const bool wasCancelled = sim.cancelTask(id);
    if (wasCancelled) {
      ++cancelled;
      cancelledFrom.insert(before);
      EXPECT_EQ(sim.robots()[0].state(), RobotState::Idle) << "moment " << moment;
      EXPECT_FALSE(sim.robots()[0].task().has_value());
      EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Cancelled);
    } else {
      ++tooLate;
      EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed) << "moment " << moment;
    }
    expectExclusiveClaims(sim);

    // The robot is fully usable afterwards.
    const TaskId next = sim.submitTask(P(1, 0), P(0, 0));
    sim.run(15);
    EXPECT_EQ(sim.tasks().find(next)->status, TaskStatus::Completed) << "moment " << moment;
    EXPECT_FALSE(sim.robots()[0].inconsistency().has_value()) << "moment " << moment;
    EXPECT_EQ(sim.snapshot().completedTasks, wasCancelled ? 1U : 2U) << "moment " << moment;
    EXPECT_EQ(sim.snapshot().cancelledTasks, wasCancelled ? 1U : 0U) << "moment " << moment;
    EXPECT_EQ(sim.snapshot().failedTasks, 0U) << "moment " << moment;
  }
  // The sweep reached every stage in which a task can be withdrawn, and the end of the task.
  EXPECT_TRUE(cancelledFrom.contains(RobotState::Idle));
  EXPECT_TRUE(cancelledFrom.contains(RobotState::Navigating));
  EXPECT_TRUE(cancelledFrom.contains(RobotState::Loading));
  EXPECT_TRUE(cancelledFrom.contains(RobotState::Unloading));
  EXPECT_GT(cancelled, 8);
  EXPECT_GT(tooLate, 0);
}

// ---- exclusive claims -------------------------------------------------------------------

TEST(SimulationDispatchTest, AFaultyDispatcherCannotGiveOneTaskToTwoRobotsOrTwoTasksToOne) {
  DispatchPlan plan;
  plan.assignments = {Assignment{R(1), T(1), 0}, Assignment{R(2), T(1), 0},
                      Assignment{R(1), T(2), 0}};
  Simulation sim(layout({"......"}), {P(0, 0), P(5, 0)}, {}, nullptr,
                 std::make_unique<FixedPlanDispatcher>(plan));
  const TaskId first = sim.submitTask(P(2, 0), P(3, 0));
  const TaskId second = sim.submitTask(P(4, 0), P(3, 0));
  ASSERT_EQ(first, T(1));
  ASSERT_EQ(second, T(2));

  sim.step();
  EXPECT_TRUE(sim.tasks().find(first)->assignee == R(1));
  EXPECT_TRUE(sim.robots()[0].task() == first);
  EXPECT_FALSE(sim.robots()[1].task().has_value());  // the duplicate claim on task 1 was refused
  EXPECT_EQ(sim.robots()[1].state(), RobotState::Idle);
  EXPECT_EQ(sim.tasks().find(second)->status, TaskStatus::Pending);  // robot 1 was already busy
  EXPECT_FALSE(sim.tasks().find(second)->assignee.has_value());
  expectExclusiveClaims(sim);

  sim.run(25);  // the same plan is offered every tick; the guarantees keep holding
  expectExclusiveClaims(sim);
  EXPECT_EQ(sim.tasks().find(first)->status, TaskStatus::Completed);
}

TEST(SimulationDispatchTest, AFaultyDispatcherCannotUseAnUnavailableRobotOrACancelledTask) {
  DispatchPlan plan;
  plan.assignments = {Assignment{R(1), T(1), 0}, Assignment{R(2), T(2), 0}};
  Simulation sim(layout({"......"}), {P(0, 0), P(5, 0)}, {}, nullptr,
                 std::make_unique<FixedPlanDispatcher>(plan));
  const TaskId cancelled = sim.submitTask(P(1, 0), P(2, 0));
  const TaskId open = sim.submitTask(P(4, 0), P(3, 0));
  ASSERT_TRUE(sim.cancelTask(cancelled));
  ASSERT_TRUE(sim.disableRobot(R(2)));

  sim.run(10);
  EXPECT_EQ(sim.tasks().find(cancelled)->status, TaskStatus::Cancelled);
  EXPECT_FALSE(sim.robots()[0].task().has_value());
  EXPECT_EQ(sim.robots()[1].state(), RobotState::Unavailable);
  EXPECT_EQ(sim.tasks().find(open)->status, TaskStatus::Pending);
  expectExclusiveClaims(sim);
}

TEST(SimulationDispatchTest, ExclusiveClaimsHoldEveryTickWithManyRobotsTasksAndCancellations) {
  for (const char* name : {"compact", "fulfilment"}) {
    auto sim = makeLayoutSimulation(name, 7);
    ASSERT_TRUE(sim.has_value());
    for (int tick = 0; tick < 700; ++tick) {
      sim->step();
      if (tick % 13 == 0 && !sim->tasks().all().empty()) {
        const auto count = static_cast<std::uint32_t>(sim->tasks().all().size());
        (void)sim->cancelTask(T(static_cast<std::uint32_t>(tick / 13) % count + 1));
      }
      expectExclusiveClaims(*sim);
      if (::testing::Test::HasFailure()) {
        FAIL() << name << " at tick " << tick;
      }
    }
    const auto snapshot = sim->snapshot();
    EXPECT_EQ(snapshot.completedTasks + snapshot.failedTasks + snapshot.cancelledTasks +
                  snapshot.activeTasks.size(),
              sim->tasks().all().size());
    EXPECT_GT(snapshot.completedTasks, 10U);
    EXPECT_GT(snapshot.cancelledTasks, 5U);
  }
}

TEST(SimulationDispatchTest, TheSameOperationsGiveTheSameStateEveryTime) {
  const auto run = [] {
    auto sim = makeLayoutSimulation("fulfilment", 21);
    for (int tick = 0; tick < 400; ++tick) {
      sim->step();
      if (tick % 17 == 0) {
        const auto count = static_cast<std::uint32_t>(sim->tasks().all().size());
        if (count > 0) {
          (void)sim->cancelTask(T(static_cast<std::uint32_t>(tick) % count + 1));
        }
      }
    }
    return sim->snapshot();
  };
  EXPECT_TRUE(run() == run());
}

}  // namespace
