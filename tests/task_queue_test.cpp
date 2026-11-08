#include "warehousesim/tasks/task_queue.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/tasks/task.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::tasks::PendingKey;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskPriority;
using warehousesim::tasks::TaskQueue;
using warehousesim::tasks::TaskStatus;

Point P(int x, int y) { return Point{x, y}; }

// The pending tasks in dispatch order, as ids.
std::vector<std::uint32_t> pendingOrder(const TaskQueue& queue) {
  std::vector<std::uint32_t> ids;
  for (const PendingKey& key : queue.pending()) {
    ids.push_back(key.id.value);
  }
  return ids;
}

// ---- changing the priority of a waiting task ---------------------------------------------

TEST(TaskQueueTest, ChangingThePriorityOfAWaitingTaskMovesItInTheDispatchOrder) {
  TaskQueue queue;
  const TaskId a = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::Normal);
  const TaskId b = queue.submit(P(0, 0), P(1, 1), 1, TaskPriority::Normal);
  const TaskId c = queue.submit(P(0, 0), P(1, 1), 2, TaskPriority::Normal);
  ASSERT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{a.value, b.value, c.value}));

  EXPECT_TRUE(queue.setPriority(c, TaskPriority::Urgent));
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{c.value, a.value, b.value}));
  EXPECT_EQ(queue.find(c)->priority, TaskPriority::Urgent);

  EXPECT_TRUE(queue.setPriority(a, TaskPriority::Low));
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{c.value, b.value, a.value}));
}

TEST(TaskQueueTest, ANewPriorityKeepsTheTasksAgeSoItQueuesBehindOlderTasksOfTheSamePriority) {
  TaskQueue queue;
  const TaskId old = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::High);
  const TaskId young = queue.submit(P(0, 0), P(1, 1), 50, TaskPriority::Low);
  EXPECT_TRUE(queue.setPriority(young, TaskPriority::High));
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{old.value, young.value}));
  EXPECT_EQ(queue.find(young)->createdAt, 50U);
}

TEST(TaskQueueTest, SettingThePriorityItAlreadyHasSucceedsAndChangesNothing) {
  TaskQueue queue;
  const TaskId a = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::High);
  const TaskId b = queue.submit(P(0, 0), P(1, 1), 1, TaskPriority::High);
  EXPECT_TRUE(queue.setPriority(b, TaskPriority::High));
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{a.value, b.value}));
  EXPECT_EQ(queue.pending().size(), 2U);
}

TEST(TaskQueueTest, OnlyAWaitingTaskCanChangePriorityAndARefusalChangesNothing) {
  TaskQueue queue;
  const TaskId assigned = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::Normal);
  const TaskId progress = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::Normal);
  const TaskId done = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::Normal);
  const TaskId failed = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::Normal);
  const TaskId cancelled = queue.submit(P(0, 0), P(1, 1), 0, TaskPriority::Normal);
  queue.assign(assigned, RobotId{1});
  queue.assign(progress, RobotId{2});
  queue.markInProgress(progress);
  queue.assign(done, RobotId{3});
  queue.markInProgress(done);
  queue.complete(done);
  queue.reject(failed);
  ASSERT_TRUE(queue.cancel(cancelled));

  for (const TaskId id : {assigned, progress, done, failed, cancelled}) {
    EXPECT_FALSE(queue.setPriority(id, TaskPriority::Urgent)) << id.value;
    EXPECT_EQ(queue.find(id)->priority, TaskPriority::Normal) << id.value;
  }
  EXPECT_TRUE(queue.pending().empty());
}

TEST(TaskQueueTest, ATaskThatWasGivenBackCanChangePriorityAgain) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 1), 0);
  queue.assign(id, RobotId{1});
  EXPECT_FALSE(queue.setPriority(id, TaskPriority::Urgent));
  queue.release(id);
  EXPECT_TRUE(queue.setPriority(id, TaskPriority::Urgent));
  EXPECT_EQ(queue.pending().begin()->priority, TaskPriority::Urgent);
}

TEST(TaskQueueTest, ChangingPriorityOfAnUnknownTaskThrows) {
  TaskQueue queue;
  EXPECT_THROW((void)queue.setPriority(TaskId{1}, TaskPriority::High), std::out_of_range);
  EXPECT_THROW((void)queue.setPriority(TaskId{}, TaskPriority::High), std::out_of_range);
}

TEST(TaskQueueTest, CountsAndTheQueueStayConsistentWhenPrioritiesChange) {
  TaskQueue queue;
  Rng rng(5);
  std::vector<TaskId> ids;
  for (int i = 0; i < 30; ++i) {
    ids.push_back(queue.submit(P(0, 0), P(1, 1), static_cast<std::uint64_t>(rng.below(5)),
                               static_cast<TaskPriority>(rng.below(4))));
  }
  for (int step = 0; step < 300; ++step) {
    const TaskId id = ids[rng.below(ids.size())];
    (void)queue.setPriority(id, static_cast<TaskPriority>(rng.below(4)));
    ASSERT_EQ(queue.pending().size(), queue.count(TaskStatus::Pending));
    ASSERT_EQ(queue.pending().size(), 30U);
  }
  PendingKey previous = *queue.pending().begin();
  for (const PendingKey& key : queue.pending()) {
    const Task* task = queue.find(key.id);
    ASSERT_TRUE(task != nullptr);
    EXPECT_EQ(key.priority, task->priority);
    EXPECT_EQ(key.createdAt, task->createdAt);
    EXPECT_TRUE(!(warehousesim::tasks::PendingOrder{}(key, previous)) ||
                &key == &*queue.pending().begin());
    previous = key;
  }
}

// ---- adding tasks -----------------------------------------------------------------------

TEST(TaskQueueTest, SubmitRecordsEveryFieldOfTheTask) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(2, 3), P(4, 5), 17, TaskPriority::High);
  const Task* task = queue.find(id);
  ASSERT_TRUE(task != nullptr);
  EXPECT_EQ(task->id, id);
  EXPECT_EQ(task->pickup, P(2, 3));
  EXPECT_EQ(task->dropoff, P(4, 5));
  EXPECT_EQ(task->createdAt, 17U);
  EXPECT_EQ(task->priority, TaskPriority::High);
  EXPECT_EQ(task->status, TaskStatus::Pending);
  EXPECT_FALSE(task->assignee.has_value());
  EXPECT_TRUE(task->isOpen());
  EXPECT_FALSE(task->isAssigned());
  EXPECT_FALSE(task->isFinished());
}

TEST(TaskQueueTest, IdsAreUniqueAndSequentialStartingAtOne) {
  TaskQueue queue;
  std::set<std::uint32_t> ids;
  for (int i = 0; i < 50; ++i) {
    const TaskId id = queue.submit(P(0, 0), P(1, 1), static_cast<std::uint64_t>(i));
    EXPECT_EQ(id.value, static_cast<std::uint32_t>(i + 1));
    EXPECT_TRUE(ids.insert(id.value).second);
  }
  EXPECT_EQ(queue.all().size(), 50U);
}

TEST(TaskQueueTest, PriorityDefaultsToNormal) {
  TaskQueue queue;
  EXPECT_EQ(queue.find(queue.submit(P(0, 0), P(1, 0), 0))->priority, TaskPriority::Normal);
}

TEST(TaskQueueTest, FindReturnsNullForUnknownIds) {
  TaskQueue queue;
  EXPECT_TRUE(queue.find(TaskId{}) == nullptr);
  EXPECT_TRUE(queue.find(TaskId{1}) == nullptr);
  queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_TRUE(queue.find(TaskId{2}) == nullptr);
}

// ---- dispatch order ---------------------------------------------------------------------

TEST(TaskQueueTest, PendingTasksAreOrderedByPriorityThenAgeThenId) {
  TaskQueue queue;
  queue.submit(P(0, 0), P(1, 0), 10, TaskPriority::Normal);  // 1
  queue.submit(P(0, 0), P(1, 0), 11, TaskPriority::Urgent);  // 2
  queue.submit(P(0, 0), P(1, 0), 5, TaskPriority::Normal);   // 3: older than 1
  queue.submit(P(0, 0), P(1, 0), 12, TaskPriority::Low);     // 4
  queue.submit(P(0, 0), P(1, 0), 10, TaskPriority::Normal);  // 5: same age as 1, later id
  queue.submit(P(0, 0), P(1, 0), 99, TaskPriority::High);    // 6: newest but still above Normal
  queue.submit(P(0, 0), P(1, 0), 1, TaskPriority::Low);      // 7: oldest Low
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{2, 6, 3, 1, 5, 7, 4}));
}

TEST(TaskQueueTest, OrderDoesNotDependOnSubmissionHistory) {
  // The same set of (priority, createdAt) pairs submitted in any order gives the same ranking.
  Rng rng(8);
  struct Spec {
    TaskPriority priority;
    std::uint64_t createdAt;
  };
  std::vector<Spec> specs;
  for (int i = 0; i < 30; ++i) {
    specs.push_back({static_cast<TaskPriority>(rng.below(4)), rng.below(6)});
  }
  const auto ranked = [&](const std::vector<Spec>& order) {
    TaskQueue queue;
    for (const Spec& s : order) {
      queue.submit(P(0, 0), P(1, 0), s.createdAt, s.priority);
    }
    std::vector<std::pair<int, std::uint64_t>> keys;
    for (const PendingKey& key : queue.pending()) {
      keys.emplace_back(static_cast<int>(key.priority), key.createdAt);
    }
    return keys;
  };
  std::vector<Spec> shuffled = specs;
  for (std::size_t i = shuffled.size(); i > 1; --i) {
    std::swap(shuffled[i - 1], shuffled[rng.below(i)]);
  }
  EXPECT_EQ(ranked(specs), ranked(shuffled));
}

TEST(TaskQueueTest, PendingSetTracksEveryTransition) {
  TaskQueue queue;
  const TaskId a = queue.submit(P(0, 0), P(1, 0), 0);
  const TaskId b = queue.submit(P(0, 0), P(1, 0), 1);
  const TaskId c = queue.submit(P(0, 0), P(1, 0), 2);
  const TaskId d = queue.submit(P(0, 0), P(1, 0), 3);
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{1, 2, 3, 4}));
  queue.assign(a, RobotId{1});
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{2, 3, 4}));
  EXPECT_TRUE(queue.cancel(b));
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{3, 4}));
  queue.reject(c);
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{4}));
  queue.release(a);  // back into the queue at its original place
  EXPECT_EQ(pendingOrder(queue), (std::vector<std::uint32_t>{1, 4}));
  queue.assign(d, RobotId{2});
  queue.assign(a, RobotId{1});
  EXPECT_TRUE(queue.pending().empty());
}

// ---- exclusive assignment ---------------------------------------------------------------

TEST(TaskQueueTest, ATaskCanBeClaimedByOnlyOneRobot) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_TRUE(queue.tryAssign(id, RobotId{1}));
  EXPECT_FALSE(queue.tryAssign(id, RobotId{2}));  // too late
  EXPECT_FALSE(queue.tryAssign(id, RobotId{1}));  // not even twice by the same robot
  EXPECT_EQ(queue.find(id)->assignee, RobotId{1});
  EXPECT_EQ(queue.find(id)->status, TaskStatus::Assigned);
  EXPECT_TRUE(queue.find(id)->isAssigned());
}

TEST(TaskQueueTest, AssignThrowsWhenTheTaskCannotBeClaimed) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(id, RobotId{1});
  EXPECT_THROW(queue.assign(id, RobotId{2}), std::logic_error);
  EXPECT_EQ(queue.find(id)->assignee, RobotId{1});
}

TEST(TaskQueueTest, TasksBeingWorkedOnOrFinishedCannotBeClaimed) {
  TaskQueue queue;
  const TaskId inProgress = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(inProgress, RobotId{1});
  queue.markInProgress(inProgress);
  const TaskId done = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(done, RobotId{1});
  queue.markInProgress(done);
  queue.complete(done);
  const TaskId cancelled = queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_TRUE(queue.cancel(cancelled));
  const TaskId rejected = queue.submit(P(0, 0), P(1, 0), 0);
  queue.reject(rejected);
  for (const TaskId id : {inProgress, done, cancelled, rejected}) {
    EXPECT_FALSE(queue.tryAssign(id, RobotId{9}));
  }
  EXPECT_EQ(queue.find(inProgress)->assignee, RobotId{1});
  EXPECT_FALSE(queue.find(cancelled)->assignee.has_value());
}

TEST(TaskQueueTest, AReleasedTaskCanBeClaimedByAnotherRobot) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(id, RobotId{1});
  queue.release(id);
  EXPECT_FALSE(queue.find(id)->assignee.has_value());
  EXPECT_TRUE(queue.tryAssign(id, RobotId{2}));
  EXPECT_EQ(queue.find(id)->assignee, RobotId{2});
}

TEST(TaskQueueTest, EveryTaskIsHeldByAtMostOneRobotUnderRandomClaims) {
  Rng rng(21);
  TaskQueue queue;
  for (int i = 0; i < 40; ++i) {
    queue.submit(P(0, 0), P(1, 0), 0);
  }
  std::vector<int> winners(41, 0);
  for (int attempt = 0; attempt < 2000; ++attempt) {
    const TaskId id{static_cast<std::uint32_t>(1 + rng.below(40))};
    if (queue.tryAssign(id, RobotId{static_cast<std::uint32_t>(1 + rng.below(8))})) {
      ++winners[id.value];
    }
  }
  for (std::uint32_t id = 1; id <= 40; ++id) {
    EXPECT_LE(winners[id], 1);
  }
}

// ---- completing -------------------------------------------------------------------------

TEST(TaskQueueTest, HappyPathLifecycle) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(id, RobotId{3});
  EXPECT_EQ(queue.find(id)->assignee, RobotId{3});
  queue.markInProgress(id);
  EXPECT_EQ(queue.find(id)->status, TaskStatus::InProgress);
  EXPECT_TRUE(queue.find(id)->isAssigned());
  queue.complete(id);
  const Task* task = queue.find(id);
  EXPECT_EQ(task->status, TaskStatus::Completed);
  EXPECT_TRUE(task->isFinished());
  EXPECT_FALSE(task->isOpen());
  EXPECT_EQ(task->assignee, RobotId{3});  // who did it stays on record
  EXPECT_EQ(queue.count(TaskStatus::Completed), 1U);
  EXPECT_EQ(queue.count(TaskStatus::Pending), 0U);
}

TEST(TaskQueueTest, CompletionRequiresADeliveryInProgress) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_THROW(queue.complete(id), std::logic_error);  // never started
  queue.assign(id, RobotId{1});
  EXPECT_THROW(queue.complete(id), std::logic_error);  // not collected yet
  queue.markInProgress(id);
  queue.complete(id);
  EXPECT_THROW(queue.complete(id), std::logic_error);  // only once
  EXPECT_THROW(queue.markInProgress(id), std::logic_error);
  EXPECT_EQ(queue.find(id)->status, TaskStatus::Completed);
}

// ---- cancelling -------------------------------------------------------------------------

TEST(TaskQueueTest, CancelWithdrawsAPendingTask) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_TRUE(queue.cancel(id));
  EXPECT_EQ(queue.find(id)->status, TaskStatus::Cancelled);
  EXPECT_TRUE(queue.find(id)->isFinished());
  EXPECT_TRUE(queue.pending().empty());
  EXPECT_EQ(queue.count(TaskStatus::Cancelled), 1U);
  EXPECT_EQ(queue.count(TaskStatus::Pending), 0U);
}

TEST(TaskQueueTest, CancelWorksFromEveryOpenState) {
  TaskQueue queue;
  const TaskId assigned = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(assigned, RobotId{1});
  const TaskId carried = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(carried, RobotId{2});
  queue.markInProgress(carried);
  EXPECT_TRUE(queue.cancel(assigned));
  EXPECT_TRUE(queue.cancel(carried));
  EXPECT_EQ(queue.find(assigned)->status, TaskStatus::Cancelled);
  EXPECT_EQ(queue.find(assigned)->assignee, RobotId{1});  // who held it stays on record
  EXPECT_EQ(queue.find(carried)->status, TaskStatus::Cancelled);
  EXPECT_EQ(queue.count(TaskStatus::Cancelled), 2U);
}

TEST(TaskQueueTest, CancelIsRefusedOnceATaskHasFinished) {
  TaskQueue queue;
  const TaskId done = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(done, RobotId{1});
  queue.markInProgress(done);
  queue.complete(done);
  const TaskId failed = queue.submit(P(0, 0), P(1, 0), 0);
  queue.reject(failed);
  const TaskId cancelled = queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_TRUE(queue.cancel(cancelled));

  EXPECT_FALSE(queue.cancel(done));
  EXPECT_FALSE(queue.cancel(failed));
  EXPECT_FALSE(queue.cancel(cancelled));  // not twice
  EXPECT_EQ(queue.find(done)->status, TaskStatus::Completed);
  EXPECT_EQ(queue.find(failed)->status, TaskStatus::Failed);
  EXPECT_EQ(queue.find(cancelled)->status, TaskStatus::Cancelled);
}

TEST(TaskQueueTest, ACancelledTaskCannotBeRevived) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(id, RobotId{1});
  ASSERT_TRUE(queue.cancel(id));
  EXPECT_THROW(queue.release(id), std::logic_error);
  EXPECT_THROW(queue.markInProgress(id), std::logic_error);
  EXPECT_THROW(queue.complete(id), std::logic_error);
  EXPECT_THROW(queue.fail(id), std::logic_error);
  EXPECT_THROW(queue.reject(id), std::logic_error);
  EXPECT_FALSE(queue.tryAssign(id, RobotId{2}));
  EXPECT_EQ(queue.find(id)->status, TaskStatus::Cancelled);
}

// ---- failing and rejecting --------------------------------------------------------------

TEST(TaskQueueTest, RejectFailsOnlyPendingTasks) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  queue.reject(id);
  EXPECT_EQ(queue.find(id)->status, TaskStatus::Failed);
  EXPECT_THROW(queue.reject(id), std::logic_error);
  const TaskId assigned = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(assigned, RobotId{1});
  EXPECT_THROW(queue.reject(assigned), std::logic_error);
}

TEST(TaskQueueTest, FailAppliesToAssignedAndInProgressTasksOnly) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_THROW(queue.fail(id), std::logic_error);  // pending: use reject()
  queue.assign(id, RobotId{1});
  queue.fail(id);
  EXPECT_EQ(queue.find(id)->status, TaskStatus::Failed);
  EXPECT_THROW(queue.fail(id), std::logic_error);
}

// ---- release and bookkeeping ------------------------------------------------------------

TEST(TaskQueueTest, ReleaseReturnsAnAssignedTaskToPending) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  queue.assign(id, RobotId{2});
  queue.release(id);
  EXPECT_EQ(queue.find(id)->status, TaskStatus::Pending);
  EXPECT_FALSE(queue.find(id)->assignee.has_value());
}

TEST(TaskQueueTest, OnlyAssignedTasksCanBeReleased) {
  TaskQueue queue;
  const TaskId id = queue.submit(P(0, 0), P(1, 0), 0);
  EXPECT_THROW(queue.release(id), std::logic_error);  // still pending
  queue.assign(id, RobotId{1});
  queue.markInProgress(id);
  EXPECT_THROW(queue.release(id), std::logic_error);  // already collected
}

TEST(TaskQueueTest, UnknownIdsThrowOutOfRange) {
  TaskQueue queue;
  EXPECT_THROW(queue.assign(TaskId{1}, RobotId{1}), std::out_of_range);
  EXPECT_THROW((void)queue.tryAssign(TaskId{}, RobotId{1}), std::out_of_range);
  EXPECT_THROW((void)queue.cancel(TaskId{5}), std::out_of_range);
  EXPECT_THROW(queue.complete(TaskId{}), std::out_of_range);
  EXPECT_THROW(queue.release(TaskId{2}), std::out_of_range);
  EXPECT_THROW(queue.reject(TaskId{2}), std::out_of_range);
}

TEST(TaskQueueTest, StatusCountsAlwaysAddUpToTheNumberOfTasks) {
  Rng rng(4);
  TaskQueue queue;
  for (int step = 0; step < 600; ++step) {
    const TaskId id{static_cast<std::uint32_t>(1 + rng.below(30))};
    if (queue.find(id) == nullptr) {
      queue.submit(P(0, 0), P(1, 0), static_cast<std::uint64_t>(step),
                   static_cast<TaskPriority>(rng.below(4)));
      continue;
    }
    try {
      switch (rng.below(7)) {
        case 0:
          (void)queue.tryAssign(id, RobotId{1});
          break;
        case 1:
          queue.release(id);
          break;
        case 2:
          queue.markInProgress(id);
          break;
        case 3:
          queue.complete(id);
          break;
        case 4:
          queue.fail(id);
          break;
        case 5:
          queue.reject(id);
          break;
        default:
          (void)queue.cancel(id);
          break;
      }
    } catch (const std::logic_error&) {
      // illegal for the task's current state: expected, and must leave it untouched
    }
    std::size_t total = 0;
    for (const warehousesim::tasks::TaskStatus status :
         {TaskStatus::Pending, TaskStatus::Assigned, TaskStatus::InProgress, TaskStatus::Completed,
          TaskStatus::Failed, TaskStatus::Cancelled}) {
      std::size_t counted = 0;
      for (const Task& task : queue.all()) {
        counted += task.status == status ? 1U : 0U;
      }
      ASSERT_EQ(queue.count(status), counted);
      total += counted;
    }
    ASSERT_EQ(total, queue.all().size());
    ASSERT_EQ(queue.pending().size(), queue.count(TaskStatus::Pending));
  }
}

TEST(TaskTypesTest, NamesAreDistinct) {
  std::set<std::string> statuses;
  for (const TaskStatus status :
       {TaskStatus::Pending, TaskStatus::Assigned, TaskStatus::InProgress, TaskStatus::Completed,
        TaskStatus::Failed, TaskStatus::Cancelled}) {
    statuses.insert(warehousesim::tasks::toString(status));
  }
  EXPECT_EQ(statuses.size(), 6U);
  std::set<std::string> priorities;
  for (const TaskPriority priority :
       {TaskPriority::Low, TaskPriority::Normal, TaskPriority::High, TaskPriority::Urgent}) {
    priorities.insert(warehousesim::tasks::toString(priority));
  }
  EXPECT_EQ(priorities.size(), 4U);
}

}  // namespace
