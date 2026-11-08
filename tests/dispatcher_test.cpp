#include "warehousesim/tasks/dispatcher.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/robots/robot_spec.hpp"
#include "warehousesim/tasks/distance_estimator.hpp"
#include "warehousesim/tasks/task_queue.hpp"
#include "warehousesim/testing/map_generators.hpp"
#include "warehousesim/testing/reference_search.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::DirectionSet;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotPath;
using warehousesim::robots::RobotSpec;
using warehousesim::tasks::Assignment;
using warehousesim::tasks::DispatchPlan;
using warehousesim::tasks::GreedyDispatcher;
using warehousesim::tasks::ManhattanDistance;
using warehousesim::tasks::PathDistance;
using warehousesim::tasks::Rejection;
using warehousesim::tasks::RejectionReason;
using warehousesim::tasks::TaskPriority;
using warehousesim::tasks::TaskQueue;
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

GreedyDispatcher straightLine() { return GreedyDispatcher(std::make_unique<ManhattanDistance>()); }

// (robot, task) pairs of a plan, in plan order.
using Pairs = std::vector<std::pair<std::uint32_t, std::uint32_t>>;
Pairs pairs(const DispatchPlan& plan) {
  Pairs result;
  for (const Assignment& a : plan.assignments) {
    result.emplace_back(a.robot.value, a.task.value);
  }
  return result;
}

// Robots that are not available, each in a different way.
Robot navigatingRobot(std::uint32_t id, Point at) {
  Robot robot(R(id), at);
  robot.beginTask(T(900 + id), P(at.x + 1, at.y), RobotPath{P(at.x + 1, at.y)});
  return robot;
}
Robot loadingRobot(std::uint32_t id, Point at) {
  Robot robot(R(id), at);
  robot.beginTask(T(900 + id), at, {});
  robot.beginLoading();
  return robot;
}
Robot chargingRobot(std::uint32_t id, Point at) {
  Robot robot(R(id), at);
  robot.beginCharging();
  return robot;
}
Robot brokenRobot(std::uint32_t id, Point at) {
  Robot robot(R(id), at);
  robot.makeUnavailable();
  return robot;
}
Robot lowBatteryRobot(std::uint32_t id, Point at) {
  RobotSpec spec;
  spec.lowBatteryLevel = 5000;
  spec.initialCharge = 4000;
  return Robot(R(id), at, spec);
}
Robot flatBatteryRobot(std::uint32_t id, Point at) {
  RobotSpec spec;
  spec.initialCharge = 0;  // idle, but with nothing left to move with
  return Robot(R(id), at, spec);
}

// ---- assignment order -------------------------------------------------------------------

TEST(DispatcherOrderTest, TheMostUrgentTaskIsServedFirstEvenIfItIsNewerAndFarther) {
  const Warehouse w = layout({"............"});
  TaskQueue queue;
  queue.submit(P(1, 0), P(0, 0), 0, TaskPriority::Low);     // 1: oldest and nearest
  queue.submit(P(9, 0), P(0, 0), 5, TaskPriority::Urgent);  // 2
  queue.submit(P(3, 0), P(0, 0), 2, TaskPriority::Normal);  // 3
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0))};
  auto dispatcher = straightLine();
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_EQ(pairs(plan), (Pairs{{1, 2}}));
  EXPECT_EQ(plan.assignments[0].estimatedDistance, 9);
  EXPECT_TRUE(plan.rejections.empty());
}

TEST(DispatcherOrderTest, TasksAreHandedOutInPriorityOrderThenToTheNearestRobot) {
  const Warehouse w = layout({"............"});
  TaskQueue queue;
  queue.submit(P(1, 0), P(0, 0), 0, TaskPriority::Low);     // 1
  queue.submit(P(9, 0), P(0, 0), 5, TaskPriority::Urgent);  // 2
  queue.submit(P(3, 0), P(0, 0), 2, TaskPriority::Normal);  // 3
  queue.submit(P(8, 0), P(0, 0), 9, TaskPriority::High);    // 4
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0)), Robot(R(2), P(11, 0))};
  auto dispatcher = straightLine();
  // Urgent (pickup 9) first: robot 2 is nearer. Then High (pickup 8): only robot 1 is left.
  EXPECT_EQ(pairs(dispatcher.plan(w, fleet, queue)), (Pairs{{2, 2}, {1, 4}}));
}

TEST(DispatcherOrderTest, WithinAPriorityTheOldestTaskComesFirst) {
  const Warehouse w = layout({"........"});
  TaskQueue queue;
  queue.submit(P(1, 0), P(0, 0), 5);  // 1
  queue.submit(P(2, 0), P(0, 0), 3);  // 2: oldest
  queue.submit(P(3, 0), P(0, 0), 9);  // 3
  const std::vector<Robot> fleet = {Robot(R(1), P(7, 0))};
  auto dispatcher = straightLine();
  EXPECT_EQ(pairs(dispatcher.plan(w, fleet, queue)), (Pairs{{1, 2}}));
}

TEST(DispatcherOrderTest, EqualPriorityAndAgeFallBackToTheLowestTaskId) {
  const Warehouse w = layout({"........"});
  TaskQueue queue;
  queue.submit(P(5, 0), P(0, 0), 4);
  queue.submit(P(6, 0), P(0, 0), 4);
  queue.submit(P(7, 0), P(0, 0), 4);
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0)), Robot(R(2), P(1, 0))};
  auto dispatcher = straightLine();
  const Pairs result = pairs(dispatcher.plan(w, fleet, queue));
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].second, 1U);
  EXPECT_EQ(result[1].second, 2U);
}

// ---- choosing the robot -----------------------------------------------------------------

TEST(DispatcherRobotChoiceTest, TheNearestAvailableRobotGetsTheTask) {
  const Warehouse w = layout({"............"});
  TaskQueue queue;
  queue.submit(P(6, 0), P(11, 0), 0);
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0)), Robot(R(2), P(5, 0)),
                                    Robot(R(3), P(9, 0))};
  auto dispatcher = straightLine();
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_EQ(pairs(plan), (Pairs{{2, 1}}));
  EXPECT_EQ(plan.assignments[0].estimatedDistance, 1);
}

TEST(DispatcherRobotChoiceTest, TiesGoToTheLowestRobotIdWhateverTheFleetOrder) {
  const Warehouse w = layout({"....", "....", "...."});
  TaskQueue queue;
  queue.submit(P(0, 0), P(3, 2), 0);
  // Robots 3 and 1 are both two steps away; robot 2 is farther.
  const std::vector<Robot> fleet = {Robot(R(3), P(2, 0)), Robot(R(2), P(3, 2)),
                                    Robot(R(1), P(0, 2))};
  auto dispatcher = straightLine();
  EXPECT_EQ(pairs(dispatcher.plan(w, fleet, queue)), (Pairs{{1, 1}}));
}

TEST(DispatcherRobotChoiceTest, RealPathLengthBeatsStraightLineDistance) {
  // A wall stands between robot 1 and the pickup: it is closer as the crow flies (2 vs 3) but
  // has to walk 6 steps around, while robot 2 walks 3.
  const Warehouse w = layout({"...#....", "...#....", "........"});
  TaskQueue queue;
  queue.submit(P(4, 0), P(7, 2), 0);
  const std::vector<Robot> fleet = {Robot(R(1), P(2, 0)), Robot(R(2), P(7, 0))};

  auto straight = straightLine();
  EXPECT_EQ(pairs(straight.plan(w, fleet, queue)), (Pairs{{1, 1}}));

  GreedyDispatcher byPath;  // default: PathDistance
  const DispatchPlan plan = byPath.plan(w, fleet, queue);
  EXPECT_EQ(pairs(plan), (Pairs{{2, 1}}));
  EXPECT_EQ(plan.assignments[0].estimatedDistance, 3);
}

TEST(DispatcherRobotChoiceTest, EachRobotAndEachTaskIsUsedAtMostOnce) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(10, 10);
  TaskQueue queue;
  for (int i = 0; i < 7; ++i) {
    queue.submit(P(i, 5), P(i, 9), static_cast<std::uint64_t>(i));
  }
  std::vector<Robot> fleet;
  for (std::uint32_t id = 1; id <= 4; ++id) {
    fleet.emplace_back(R(id), P(static_cast<int>(id), 0));
  }
  GreedyDispatcher dispatcher;
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_EQ(plan.assignments.size(), 4U);  // more tasks than robots: every robot is used once
  std::set<std::uint32_t> robots;
  std::set<std::uint32_t> tasks;
  for (const Assignment& a : plan.assignments) {
    EXPECT_TRUE(robots.insert(a.robot.value).second);
    EXPECT_TRUE(tasks.insert(a.task.value).second);
  }
}

TEST(DispatcherRobotChoiceTest, FewerTasksThanRobotsLeavesRobotsFree) {
  const Warehouse w = layout({"......"});
  TaskQueue queue;
  queue.submit(P(1, 0), P(5, 0), 0);
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0)), Robot(R(2), P(3, 0)),
                                    Robot(R(3), P(5, 0))};
  auto dispatcher = straightLine();
  EXPECT_EQ(pairs(dispatcher.plan(w, fleet, queue)), (Pairs{{1, 1}}));
}

// ---- unavailable robots -----------------------------------------------------------------

TEST(DispatcherAvailabilityTest, OnlyIdleChargedRobotsAreEligible) {
  const Warehouse w = layout({"............"});
  TaskQueue queue;
  queue.submit(P(1, 0), P(0, 0), 0);
  // Every robot below is closer to the pickup than robot 7, yet none may be used.
  const std::vector<Robot> fleet = {navigatingRobot(1, P(2, 0)), loadingRobot(2, P(2, 0)),
                                    chargingRobot(3, P(1, 0)),   brokenRobot(4, P(0, 0)),
                                    lowBatteryRobot(5, P(1, 0)), flatBatteryRobot(6, P(0, 0)),
                                    Robot(R(7), P(11, 0))};
  auto dispatcher = straightLine();
  EXPECT_EQ(pairs(dispatcher.plan(w, fleet, queue)), (Pairs{{7, 1}}));
}

TEST(DispatcherAvailabilityTest, WithNoAvailableRobotNothingIsAssignedOrRejected) {
  const Warehouse w = layout({"....#...."});
  TaskQueue queue;
  queue.submit(P(1, 0), P(2, 0), 0);
  queue.submit(P(1, 0), P(7, 0), 0);  // would be rejected if anyone were free to look at it
  const std::vector<Robot> fleet = {navigatingRobot(1, P(2, 0)), brokenRobot(2, P(3, 0))};
  GreedyDispatcher dispatcher;
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_TRUE(plan.assignments.empty());
  EXPECT_TRUE(plan.rejections.empty());
}

TEST(DispatcherAvailabilityTest, EmptyFleetOrEmptyQueueGivesAnEmptyPlan) {
  const Warehouse w = layout({"..."});
  TaskQueue queue;
  auto dispatcher = straightLine();
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0))};
  EXPECT_TRUE(dispatcher.plan(w, fleet, queue).assignments.empty());
  queue.submit(P(1, 0), P(2, 0), 0);
  EXPECT_TRUE(dispatcher.plan(w, std::vector<Robot>{}, queue).assignments.empty());
}

TEST(DispatcherAvailabilityTest, NonPendingTasksAreNeverOffered) {
  const Warehouse w = layout({"......"});
  TaskQueue queue;
  const TaskId held = queue.submit(P(1, 0), P(5, 0), 0);
  const TaskId cancelled = queue.submit(P(2, 0), P(5, 0), 0);
  const TaskId open = queue.submit(P(3, 0), P(5, 0), 0);
  queue.assign(held, R(9));
  ASSERT_TRUE(queue.cancel(cancelled));
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0))};
  auto dispatcher = straightLine();
  EXPECT_EQ(pairs(dispatcher.plan(w, fleet, queue)), (Pairs{{1, open.value}}));
}

// ---- unreachable destinations -----------------------------------------------------------

// Two rooms joined by nothing:   x = 0..2 | wall at x = 3 | x = 4..6
const std::vector<std::string> kTwoRooms = {"...#...", "...#...", "...#..."};

TEST(DispatcherUnreachableTest, ATaskWhoseDropoffCannotBeReachedFromItsPickupIsRejected) {
  const Warehouse w = layout(kTwoRooms);
  TaskQueue queue;
  queue.submit(P(1, 0), P(5, 0), 0);  // 1: across the wall
  queue.submit(P(2, 1), P(0, 1), 1);  // 2: fine
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0))};
  GreedyDispatcher dispatcher;
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_EQ(plan.rejections, (std::vector<Rejection>{{T(1), RejectionReason::DropoffUnreachable}}));
  // The rejected task did not consume the robot: it moves on to the next task.
  EXPECT_EQ(pairs(plan), (Pairs{{1, 2}}));
}

TEST(DispatcherUnreachableTest, ATaskNoRobotCanEverReachIsRejected) {
  const Warehouse w = layout(kTwoRooms);
  TaskQueue queue;
  queue.submit(P(5, 0), P(6, 2), 0);  // right room; every robot is in the left room
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0)), Robot(R(2), P(2, 2))};
  GreedyDispatcher dispatcher;
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_TRUE(plan.assignments.empty());
  EXPECT_EQ(plan.rejections, (std::vector<Rejection>{{T(1), RejectionReason::PickupUnreachable}}));
}

TEST(DispatcherUnreachableTest, ATaskOnlyABusyRobotCanReachWaitsInsteadOfBeingRejected) {
  const Warehouse w = layout(kTwoRooms);
  TaskQueue queue;
  queue.submit(P(5, 0), P(6, 2), 0);
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0)), navigatingRobot(2, P(4, 1))};
  GreedyDispatcher dispatcher;
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_TRUE(plan.assignments.empty());  // robot 1 cannot get there, robot 2 is busy
  EXPECT_TRUE(plan.rejections.empty());   // ...but robot 2 will be free again
}

TEST(DispatcherUnreachableTest, RobotsThatCannotReachAPickupAreSkippedNotChosen) {
  const Warehouse w = layout(kTwoRooms);
  TaskQueue queue;
  queue.submit(P(5, 0), P(6, 2), 0);
  // Robot 1 is right next to the wall (distance 3 by the crow) but in the wrong room.
  const std::vector<Robot> fleet = {Robot(R(1), P(2, 0)), Robot(R(2), P(6, 1))};
  GreedyDispatcher dispatcher;
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_EQ(pairs(plan), (Pairs{{2, 1}}));
  EXPECT_TRUE(plan.rejections.empty());
}

TEST(DispatcherUnreachableTest, WalledInAndOutOfGridLocationsAreRejected) {
  const Warehouse w = layout({"...", ".#.", "..."});
  TaskQueue queue;
  queue.submit(P(1, 1), P(0, 0), 0);  // pickup inside a wall
  queue.submit(P(0, 0), P(9, 9), 1);  // dropoff outside the grid
  const std::vector<Robot> fleet = {Robot(R(1), P(2, 2))};
  GreedyDispatcher dispatcher;
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_TRUE(plan.assignments.empty());
  EXPECT_EQ(plan.rejections.size(), 2U);
}

TEST(DispatcherUnreachableTest, StraightLineEstimatesCannotSeeWalls) {
  // Documented limitation of the cheap estimator: it never rejects.
  const Warehouse w = layout(kTwoRooms);
  TaskQueue queue;
  queue.submit(P(1, 0), P(5, 0), 0);
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0))};
  auto dispatcher = straightLine();
  const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
  EXPECT_TRUE(plan.rejections.empty());
  EXPECT_EQ(pairs(plan), (Pairs{{1, 1}}));
}

TEST(DispatcherUnreachableTest, RejectionReasonsHaveDistinctNames) {
  EXPECT_NE(std::string(warehousesim::tasks::toString(RejectionReason::DropoffUnreachable)),
            std::string(warehousesim::tasks::toString(RejectionReason::PickupUnreachable)));
}

// ---- purity and determinism -------------------------------------------------------------

TEST(DispatcherDeterminismTest, PlanningChangesNeitherTheQueueNorTheFleet) {
  const Warehouse w = layout(kTwoRooms);
  TaskQueue queue;
  queue.submit(P(1, 0), P(5, 0), 0);
  queue.submit(P(2, 1), P(0, 1), 1, TaskPriority::High);
  const std::vector<Robot> fleet = {Robot(R(1), P(0, 0)), Robot(R(2), P(2, 2))};
  GreedyDispatcher dispatcher;
  const std::vector<warehousesim::tasks::Task> before = queue.all();
  const std::size_t pendingBefore = queue.pending().size();
  (void)dispatcher.plan(w, fleet, queue);
  EXPECT_TRUE(queue.all() == before);
  EXPECT_EQ(queue.pending().size(), pendingBefore);
  for (const Robot& robot : fleet) {
    EXPECT_EQ(robot.state(), warehousesim::robots::RobotState::Idle);
    EXPECT_FALSE(robot.task().has_value());
  }
}

TEST(DispatcherDeterminismTest, SameInputsGiveTheSamePlanEveryTime) {
  Rng rng(33);
  const Warehouse w = warehousesim::testing::makeRandomObstacleMap(20, 20, 200, rng);
  TaskQueue queue;
  for (int i = 0; i < 12; ++i) {
    queue.submit(P(static_cast<int>(rng.below(20)), static_cast<int>(rng.below(20))),
                 P(static_cast<int>(rng.below(20)), static_cast<int>(rng.below(20))), rng.below(5),
                 static_cast<TaskPriority>(rng.below(4)));
  }
  std::vector<Robot> fleet;
  for (std::uint32_t id = 1; id <= 5; ++id) {
    fleet.emplace_back(R(id), P(static_cast<int>(rng.below(20)), static_cast<int>(rng.below(20))));
  }
  GreedyDispatcher a;
  GreedyDispatcher b;
  const DispatchPlan first = a.plan(w, fleet, queue);
  EXPECT_TRUE(first.assignments == a.plan(w, fleet, queue).assignments);  // warm cache, same answer
  EXPECT_TRUE(first.assignments == b.plan(w, fleet, queue).assignments);  // cold, same answer
  EXPECT_TRUE(first.rejections == b.plan(w, fleet, queue).rejections);
}

TEST(DispatcherDeterminismTest, TheOrderOfTheFleetVectorDoesNotMatter) {
  Rng rng(41);
  const Warehouse w = warehousesim::testing::makeOpenGrid(15, 15);
  TaskQueue queue;
  for (int i = 0; i < 9; ++i) {
    queue.submit(P(static_cast<int>(rng.below(15)), static_cast<int>(rng.below(15))), P(0, 0),
                 rng.below(4), static_cast<TaskPriority>(rng.below(4)));
  }
  std::vector<Robot> fleet;
  for (std::uint32_t id = 1; id <= 6; ++id) {
    fleet.emplace_back(R(id), P(static_cast<int>(rng.below(15)), static_cast<int>(rng.below(15))));
  }
  GreedyDispatcher dispatcher;
  const Pairs expected = pairs(dispatcher.plan(w, fleet, queue));
  for (int shuffle = 0; shuffle < 10; ++shuffle) {
    for (std::size_t i = fleet.size(); i > 1; --i) {
      std::swap(fleet[i - 1], fleet[rng.below(i)]);
    }
    EXPECT_EQ(pairs(dispatcher.plan(w, fleet, queue)), expected);
  }
}

// ---- cross-check against an independent implementation of the same rule -----------------

struct ReferencePlan {
  Pairs assignments;
  std::vector<Rejection> rejections;
};

// The documented greedy rule, written independently: sort the pending tasks explicitly instead of
// relying on the queue's ordering, and use breadth-first search for every distance.
ReferencePlan referenceGreedy(const Warehouse& w, const std::vector<Robot>& fleet,
                              const TaskQueue& queue) {
  using warehousesim::testing::shortestDistance;
  std::vector<const warehousesim::tasks::Task*> pending;
  for (const auto& task : queue.all()) {
    if (task.status == warehousesim::tasks::TaskStatus::Pending) {
      pending.push_back(&task);
    }
  }
  std::ranges::sort(pending, [](const auto* a, const auto* b) {
    if (a->priority != b->priority) return a->priority > b->priority;
    if (a->createdAt != b->createdAt) return a->createdAt < b->createdAt;
    return a->id < b->id;
  });
  std::vector<const Robot*> available;
  for (const Robot& robot : fleet) {
    if (robot.isAvailable()) available.push_back(&robot);
  }
  ReferencePlan plan;
  for (const auto* task : pending) {
    if (available.empty()) break;
    if (!shortestDistance(w, task->pickup, task->dropoff, {}, DirectionSet::all())) {
      plan.rejections.push_back({task->id, RejectionReason::DropoffUnreachable});
      continue;
    }
    const Robot* best = nullptr;
    int bestDistance = 0;
    for (const Robot* robot : available) {
      const auto d = shortestDistance(w, robot->position(), task->pickup, {}, DirectionSet::all());
      if (d && (best == nullptr || *d < bestDistance ||
                (*d == bestDistance && robot->id() < best->id()))) {
        best = robot;
        bestDistance = *d;
      }
    }
    if (best != nullptr) {
      plan.assignments.emplace_back(best->id().value, task->id.value);
      std::erase(available, best);
      continue;
    }
    bool anyone = false;
    for (const Robot& robot : fleet) {
      anyone =
          anyone ||
          shortestDistance(w, robot.position(), task->pickup, {}, DirectionSet::all()).has_value();
    }
    if (!anyone) plan.rejections.push_back({task->id, RejectionReason::PickupUnreachable});
  }
  return plan;
}

TEST(DispatcherReferenceTest, MatchesAnIndependentGreedyOnRandomWarehouses) {
  Rng rng(0xD15C0);
  int assignedTotal = 0;
  int rejectedTotal = 0;
  int waitingTotal = 0;
  for (int round = 0; round < 400; ++round) {
    const int width = 4 + static_cast<int>(rng.below(10));
    const int height = 4 + static_cast<int>(rng.below(10));
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        width, height, 150 + static_cast<std::uint32_t>(rng.below(250)), rng);
    const auto randomWalkable = [&] {
      for (int attempt = 0; attempt < 40; ++attempt) {
        const Point p = P(static_cast<int>(rng.below(static_cast<std::uint64_t>(width))),
                          static_cast<int>(rng.below(static_cast<std::uint64_t>(height))));
        if (w.isWalkable(p)) return p;
      }
      return P(-1, -1);
    };
    std::vector<Robot> fleet;
    const std::uint32_t robotCount = 1 + static_cast<std::uint32_t>(rng.below(6));
    for (std::uint32_t id = 1; id <= robotCount; ++id) {
      const Point at = randomWalkable();
      if (!w.isWalkable(at)) continue;
      switch (rng.below(8)) {
        case 0:
          fleet.push_back(brokenRobot(id, at));
          break;
        case 1:
          fleet.push_back(chargingRobot(id, at));
          break;
        case 2:
          fleet.push_back(lowBatteryRobot(id, at));
          break;
        default:
          fleet.emplace_back(R(id), at);
          break;
      }
    }
    TaskQueue queue;
    const int taskCount = 1 + static_cast<int>(rng.below(14));
    for (int i = 0; i < taskCount; ++i) {
      const Point pickup = randomWalkable();
      const Point dropoff = randomWalkable();
      if (w.isWalkable(pickup) && w.isWalkable(dropoff)) {
        queue.submit(pickup, dropoff, rng.below(6), static_cast<TaskPriority>(rng.below(4)));
      }
    }
    GreedyDispatcher dispatcher;
    const DispatchPlan plan = dispatcher.plan(w, fleet, queue);
    const ReferencePlan expected = referenceGreedy(w, fleet, queue);
    ASSERT_EQ(pairs(plan), expected.assignments);
    ASSERT_EQ(plan.rejections, expected.rejections);
    assignedTotal += static_cast<int>(plan.assignments.size());
    rejectedTotal += static_cast<int>(plan.rejections.size());
    waitingTotal += static_cast<int>(queue.pending().size()) -
                    static_cast<int>(plan.assignments.size() + plan.rejections.size());
  }
  EXPECT_GT(assignedTotal, 400);  // the comparison exercised assignments, rejections and waiting
  EXPECT_GT(rejectedTotal, 30);
  EXPECT_GT(waitingTotal, 100);
}

}  // namespace
