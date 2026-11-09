#include "warehousesim/simulation/audit.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::core::Tick;
using warehousesim::simulation::AuditViolation;
using warehousesim::simulation::InvariantAuditor;
using warehousesim::simulation::RobotPose;
using warehousesim::simulation::RobotSnapshot;
using warehousesim::simulation::Simulation;
using warehousesim::simulation::Snapshot;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskStatus;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

// Eight by three, with a wall at (5, 2).
Warehouse hall() { return *Warehouse::fromRows({"........", "........", ".....#.."}); }

RobotSnapshot robotAt(std::uint32_t id, Point at) {
  RobotSnapshot r;
  r.id = RobotId{id};
  r.position = at;
  r.holds = {at};
  r.batteryLevel = 50;
  r.batteryCapacity = 100;
  return r;
}

Snapshot state(Tick tick, std::vector<RobotSnapshot> robots) {
  Snapshot s;
  s.tick = tick;
  s.robots = std::move(robots);
  return s;
}

Task makeTask(std::uint32_t id, TaskStatus status, std::optional<std::uint32_t> robot = {},
              Tick createdAt = 0) {
  Task t;
  t.id = TaskId{id};
  t.pickup = P(0, 0);
  t.dropoff = P(1, 0);
  t.status = status;
  if (robot) {
    t.assignee = RobotId{*robot};
  }
  t.createdAt = createdAt;
  return t;
}

bool mentions(const InvariantAuditor& auditor, const std::string& text) {
  return std::any_of(
      auditor.violations().begin(), auditor.violations().end(),
      [&](const AuditViolation& v) { return v.what.find(text) != std::string::npos; });
}

// The first state is clean and the second is `bad`, which must be noticed with a message that
// contains `text` (and, when `only` is set, with nothing else wrong with it).
void expectNoticed(const Snapshot& first, const Snapshot& bad, const std::string& text) {
  InvariantAuditor auditor(hall());
  auditor.observe(first);
  ASSERT_TRUE(auditor.ok()) << auditor.summary();
  auditor.observe(bad);
  EXPECT_FALSE(auditor.ok()) << "nothing was noticed; wanted '" << text << "'";
  EXPECT_TRUE(mentions(auditor, text)) << "wanted '" << text << "' in:\n" << auditor.summary();
}

// ---- a clean state is clean --------------------------------------------------------------------

TEST(InvariantAuditorTest, ARunOfTidyStatesHasNoViolations) {
  InvariantAuditor auditor(hall());
  RobotSnapshot a = robotAt(1, P(0, 0));
  RobotSnapshot b = robotAt(2, P(7, 0));
  for (Tick tick = 0; tick < 5; ++tick) {
    auditor.observe(state(tick, {a, b}));
    a.position = P(static_cast<int>(tick) + 1, 0);
    a.holds = {a.position};
  }
  EXPECT_TRUE(auditor.ok()) << auditor.summary();
  EXPECT_EQ(auditor.statesObserved(), 5U);
  EXPECT_EQ(auditor.violationCount(), 0U);
  EXPECT_EQ(auditor.summary(), "audited 5 states: no violations");
}

TEST(InvariantAuditorTest, TwoRobotsExactlyACellApartDoNotOverlap) {
  InvariantAuditor auditor(hall());
  auditor.observe(state(0, {robotAt(1, P(2, 1)), robotAt(2, P(3, 1))}));
  EXPECT_TRUE(auditor.ok()) << auditor.summary();
}

// ---- collision safety --------------------------------------------------------------------------

TEST(InvariantAuditorTest, ACellHeldByTwoRobotsIsNoticed) {
  RobotSnapshot a = robotAt(1, P(1, 1));
  RobotSnapshot b = robotAt(2, P(3, 1));
  const Snapshot first = state(0, {a, b});
  b.holds = {P(3, 1), P(2, 1), P(1, 1)};  // reaches into robot 1's cell
  expectNoticed(first, state(1, {a, b}), "is held by robot 1 and by robot 2");
}

TEST(InvariantAuditorTest, ARobotThatDoesNotHoldItsOwnCellIsNoticed) {
  RobotSnapshot a = robotAt(1, P(1, 1));
  const Snapshot first = state(0, {a});
  a.holds = {P(2, 1)};
  expectNoticed(first, state(1, {a}), "does not hold the cell it is on");
  a.holds = {};
  expectNoticed(first, state(1, {a}), "does not hold the cell it is on");
}

TEST(InvariantAuditorTest, ARobotOnAWallOrOffTheMapIsNoticed) {
  const Snapshot first = state(0, {robotAt(1, P(4, 2))});
  expectNoticed(first, state(1, {robotAt(1, P(5, 2))}), "which is a wall or off the map");
  expectNoticed(first, state(1, {robotAt(1, P(-1, 0))}), "which is a wall or off the map");
}

TEST(InvariantAuditorTest, AHeldWallOrCellsThatAreNotNeighboursAreNoticed) {
  RobotSnapshot a = robotAt(1, P(4, 2));
  const Snapshot first = state(0, {a});
  a.holds = {P(4, 2), P(5, 2)};
  expectNoticed(first, state(1, {a}), "holds (5,2), which is a wall");
  a.holds = {P(4, 2), P(4, 0)};
  expectNoticed(first, state(1, {a}), "which are not next to each other");
}

TEST(InvariantAuditorTest, TwoRobotsWhoseCentresAreCloserThanACellOverlap) {
  RobotSnapshot a = robotAt(1, P(2, 1));
  RobotSnapshot b = robotAt(2, P(3, 1));
  const Snapshot first = state(0, {a, b});
  b.pose = RobotPose{{3.5 - 0.6, 1.5}, 0.0};  // 0.4 cells from robot 1's centre
  expectNoticed(first, state(1, {a, b}), "robot 1 and robot 2 overlap");
}

TEST(InvariantAuditorTest, TwoRobotsThatExchangeCellsInOneTickAreNoticed) {
  RobotSnapshot a = robotAt(1, P(2, 1));
  RobotSnapshot b = robotAt(2, P(3, 1));
  const Snapshot first = state(0, {a, b});
  a.position = P(3, 1);
  a.holds = {P(3, 1)};
  b.position = P(2, 1);
  b.holds = {P(2, 1)};
  expectNoticed(first, state(1, {a, b}), "swapped cells");
}

TEST(InvariantAuditorTest, ARobotThatMovesMoreThanACellInATickIsNoticed) {
  const Snapshot first = state(0, {robotAt(1, P(1, 1))});
  expectNoticed(first, state(1, {robotAt(1, P(3, 1))}), "in one tick");
  expectNoticed(first, state(1, {robotAt(1, P(3, 1))}), "jumped");
}

TEST(InvariantAuditorTest, APoseFurtherThanACellFromItsCellOrOffTheMapIsNoticed) {
  RobotSnapshot a = robotAt(1, P(1, 1));
  const Snapshot first = state(0, {a});
  a.pose = RobotPose{{3.0, 1.5}, 0.0};
  expectNoticed(first, state(1, {a}), "more than a cell away");
  RobotSnapshot edge = robotAt(1, P(0, 0));
  edge.pose = RobotPose{{-0.5, 0.5}, 0.0};
  expectNoticed(first, state(1, {edge}), "outside the map");
}

// ---- numerical sanity --------------------------------------------------------------------------

TEST(InvariantAuditorTest, NotANumberAndInfinityAreNoticedWhereverTheyAppear) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const Snapshot first = state(0, {robotAt(1, P(1, 1))});
  RobotSnapshot a = robotAt(1, P(1, 1));
  a.pose = RobotPose{{nan, 1.5}, 0.0};
  expectNoticed(first, state(1, {a}), "not a number");
  a.pose = RobotPose{{1.5, 1.5}, inf};
  expectNoticed(first, state(1, {a}), "not a number");
  a.pose.reset();
  a.speed = nan;
  expectNoticed(first, state(1, {a}), "not a number");
}

TEST(InvariantAuditorTest, ASpeedOutsideZeroToOneIsNoticed) {
  const Snapshot first = state(0, {robotAt(1, P(1, 1))});
  RobotSnapshot a = robotAt(1, P(1, 1));
  a.speed = 1.5;
  expectNoticed(first, state(1, {a}), "outside 0 to 1 cells a tick");
  a.speed = -0.1;
  expectNoticed(first, state(1, {a}), "outside 0 to 1 cells a tick");
}

TEST(InvariantAuditorTest, ABatteryAboveItsCapacityOrCargoForNoTaskIsNoticed) {
  const Snapshot first = state(0, {robotAt(1, P(1, 1))});
  RobotSnapshot a = robotAt(1, P(1, 1));
  a.batteryLevel = 101;
  expectNoticed(first, state(1, {a}), "has a battery of 101 in 100");
  RobotSnapshot b = robotAt(1, P(1, 1));
  b.carrying = true;
  expectNoticed(first, state(1, {b}), "carries cargo for no task");
}

// ---- ownership and accounting ------------------------------------------------------------------

TEST(InvariantAuditorTest, ATaskAndItsRobotMustAgreeWhoOwnsIt) {
  RobotSnapshot a = robotAt(1, P(1, 1));
  Snapshot first = state(0, {a});
  Snapshot good = state(1, {a});
  a.task = TaskId{1};
  good.robots[0] = a;
  good.activeTasks = {makeTask(1, TaskStatus::Assigned, 1)};
  InvariantAuditor clean(hall());
  clean.observe(first);
  clean.observe(good);
  EXPECT_TRUE(clean.ok()) << clean.summary();

  Snapshot ownerless = good;
  ownerless.robots[0].task.reset();
  expectNoticed(first, ownerless, "which works on something else");

  Snapshot orphan = good;
  orphan.activeTasks.clear();
  expectNoticed(first, orphan, "which is not an open task");

  Snapshot stranger = good;
  stranger.activeTasks[0] = makeTask(1, TaskStatus::Assigned, 2);
  stranger.robots.push_back(robotAt(2, P(6, 0)));
  Snapshot firstOfTwo = state(0, {robotAt(1, P(1, 1)), robotAt(2, P(6, 0))});
  expectNoticed(firstOfTwo, stranger, "which is not its task");

  Snapshot noOwner = good;
  noOwner.activeTasks[0] = makeTask(1, TaskStatus::InProgress);
  expectNoticed(first, noOwner, "belongs to no robot");
}

TEST(InvariantAuditorTest, ATaskWorkedOnByTwoRobotsIsNoticed) {
  RobotSnapshot a = robotAt(1, P(1, 1));
  RobotSnapshot b = robotAt(2, P(6, 0));
  const Snapshot first = state(0, {a, b});
  a.task = TaskId{1};
  b.task = TaskId{1};
  Snapshot bad = state(1, {a, b});
  bad.activeTasks = {makeTask(1, TaskStatus::Assigned, 1)};
  expectNoticed(first, bad, "is worked on by robot 1 and by robot 2");
}

TEST(InvariantAuditorTest, EveryTaskMustBeAccountedFor) {
  const Snapshot first = state(0, {robotAt(1, P(1, 1))});
  Snapshot bad = state(1, {robotAt(1, P(1, 1))});
  bad.activeTasks = {makeTask(3, TaskStatus::Pending)};
  bad.completedTasks = 1;  // 1 + 1 open = 2, but task 3 exists
  expectNoticed(first, bad, "3 tasks were made but 2 are accounted for");

  Snapshot fine = state(1, {robotAt(1, P(1, 1))});
  fine.activeTasks = {makeTask(3, TaskStatus::Pending)};
  fine.completedTasks = 1;
  fine.cancelledTasks = 1;
  InvariantAuditor auditor(hall());
  auditor.observe(first);
  auditor.observe(fine);
  EXPECT_TRUE(auditor.ok()) << auditor.summary();
}

TEST(InvariantAuditorTest, TasksFromTheFutureOrOutOfOrderOrListedTwiceOrFinishedAreNoticed) {
  const Snapshot first = state(0, {robotAt(1, P(1, 1))});
  Snapshot future = state(1, {robotAt(1, P(1, 1))});
  future.activeTasks = {makeTask(1, TaskStatus::Pending, {}, 9)};
  expectNoticed(first, future, "in the future");

  Snapshot disorder = state(1, {robotAt(1, P(1, 1))});
  disorder.activeTasks = {makeTask(2, TaskStatus::Pending), makeTask(1, TaskStatus::Pending)};
  expectNoticed(first, disorder, "is listed out of order");

  Snapshot twice = state(1, {robotAt(1, P(1, 1))});
  twice.activeTasks = {makeTask(1, TaskStatus::Pending), makeTask(1, TaskStatus::Pending)};
  expectNoticed(first, twice, "is listed twice");

  Snapshot finished = state(1, {robotAt(1, P(1, 1))});
  finished.activeTasks = {makeTask(1, TaskStatus::Completed)};
  expectNoticed(first, finished, "but listed as open");
}

// ---- event ordering ----------------------------------------------------------------------------

TEST(InvariantAuditorTest, TheTickMustAdvanceByExactlyOne) {
  const Snapshot first = state(4, {robotAt(1, P(1, 1))});
  expectNoticed(first, state(6, {robotAt(1, P(1, 1))}), "tick 6 follows tick 4");
  expectNoticed(first, state(4, {robotAt(1, P(1, 1))}), "tick 4 follows tick 4");
  expectNoticed(first, state(3, {robotAt(1, P(1, 1))}), "tick 3 follows tick 4");
}

TEST(InvariantAuditorTest, CountersNeverGoDown) {
  Snapshot first = state(0, {robotAt(1, P(1, 1))});
  first.completedTasks = 2;
  first.failedTasks = 1;
  first.cancelledTasks = 1;
  first.robots[0].deliveries = 2;
  for (int which = 0; which < 4; ++which) {
    Snapshot bad = state(1, {robotAt(1, P(1, 1))});
    bad.completedTasks = 2;
    bad.failedTasks = 1;
    bad.cancelledTasks = 1;
    bad.robots[0].deliveries = 2;
    if (which == 0) {
      bad.completedTasks = 1;
    } else if (which == 1) {
      bad.failedTasks = 0;
    } else if (which == 2) {
      bad.cancelledTasks = 0;
    } else {
      bad.robots[0].deliveries = 1;
    }
    InvariantAuditor auditor(hall());
    auditor.observe(first);
    auditor.observe(bad);
    EXPECT_FALSE(auditor.ok()) << "counter " << which << " went down unnoticed";
  }
}

TEST(InvariantAuditorTest, TheRobotsAreTheSameFromOneStateToTheNext) {
  const Snapshot first = state(0, {robotAt(1, P(1, 1)), robotAt(2, P(6, 0))});
  expectNoticed(first, state(1, {robotAt(1, P(1, 1))}), "the number of robots changed from 2 to 1");
  expectNoticed(state(0, {robotAt(1, P(1, 1))}), state(1, {robotAt(2, P(1, 1))}), "has id 2");
}

// ---- the auditor itself ------------------------------------------------------------------------

TEST(InvariantAuditorTest, ViolationsAreCountedInFullButOnlyTheFirstAreKept) {
  InvariantAuditor auditor(hall());
  for (Tick tick = 0; tick < 300; ++tick) {
    // On the wall every time, which breaks two rules: it stands on a wall and holds one.
    auditor.observe(state(tick, {robotAt(1, P(5, 2))}));
  }
  EXPECT_EQ(auditor.violationCount(), 600U);
  EXPECT_EQ(auditor.violations().size(), InvariantAuditor::kMaxKept);
  EXPECT_EQ(auditor.violations().front().tick, 0U);
  EXPECT_EQ(auditor.violations().back().tick, InvariantAuditor::kMaxKept / 2 - 1);
  EXPECT_NE(auditor.summary().find("600 violations"), std::string::npos);
  EXPECT_NE(auditor.summary().find("First 5:"), std::string::npos);
}

TEST(InvariantAuditorTest, AnEmptyFleetAndAnEmptyQueueAreFine) {
  InvariantAuditor auditor(hall());
  auditor.observe(state(0, {}));
  auditor.observe(state(1, {}));
  EXPECT_TRUE(auditor.ok()) << auditor.summary();
}

// ---- real runs ---------------------------------------------------------------------------------

TEST(InvariantAuditorTest, EveryBundledLayoutRunsCleanForThousandsOfTicks) {
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    auto sim = warehousesim::simulation::makeLayoutSimulation(std::string(info.name), 3);
    ASSERT_TRUE(sim.has_value()) << info.name;
    InvariantAuditor auditor(sim->warehouse());
    auditor.observe(*sim);
    for (int tick = 0; tick < 2000; ++tick) {
      sim->step();
      auditor.observe(*sim);
    }
    EXPECT_TRUE(auditor.ok()) << info.name << ": " << auditor.summary();
    EXPECT_EQ(auditor.statesObserved(), 2001U);
    EXPECT_GT(sim->snapshot().completedTasks, 0U) << info.name << " did no work at all";
  }
}

TEST(InvariantAuditorTest, FollowsALayoutThatIsEditedWhileTheSimulationRuns) {
  auto sim = warehousesim::simulation::makeLayoutSimulation("fulfilment", 2);
  ASSERT_TRUE(sim.has_value());
  InvariantAuditor auditor(sim->warehouse());
  const std::vector<Point> floor =
      sim->warehouse().cellsOfType(warehousesim::world::CellType::Free);
  int placed = 0;
  for (int tick = 0; tick < 600; ++tick) {
    if (tick % 50 == 20 && placed < 8) {
      placed += sim->placeObstacle(floor[floor.size() / 2 + static_cast<std::size_t>(placed) * 7])
                    ? 1
                    : 0;
    }
    sim->step();
    auditor.observe(*sim);
  }
  EXPECT_GT(placed, 3) << "walls really were put up";
  EXPECT_TRUE(auditor.ok()) << auditor.summary();
}

TEST(InvariantAuditorTest, AuditingNeverChangesARun) {
  auto plain = warehousesim::simulation::makeLayoutSimulation("compact", 5);
  auto audited = warehousesim::simulation::makeLayoutSimulation("compact", 5);
  ASSERT_TRUE(plain.has_value() && audited.has_value());
  InvariantAuditor auditor(audited->warehouse());
  for (int tick = 0; tick < 500; ++tick) {
    plain->step();
    audited->step();
    auditor.observe(*audited);
  }
  EXPECT_EQ(plain->stateChecksum(), audited->stateChecksum());
  EXPECT_TRUE(plain->snapshot() == audited->snapshot());
}

}  // namespace
