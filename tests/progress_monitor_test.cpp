#include "warehousesim/traffic/progress_monitor.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::traffic::ProgressMonitor;

RobotId R(std::uint32_t id) { return RobotId{id}; }
Point P(int x, int y) { return Point{x, y}; }

// Robot `robot` wants to move, is where it always is (its own column), and waits for `other`.
void stuckBehind(ProgressMonitor& monitor, std::uint32_t robot, std::uint32_t other) {
  monitor.record(R(robot), true, P(static_cast<int>(robot) * 10, 0), R(other));
}

TEST(ProgressMonitorTest,
     ARobotThatWantsToMoveAndStaysWhereItIsCountsUpAndStartsOverWhenItGetsAway) {
  ProgressMonitor monitor(3);
  for (int i = 0; i < 5; ++i) {
    stuckBehind(monitor, 1, 2);
  }
  // The first step only marks where it began to wait.
  EXPECT_EQ(monitor.stuckTicks(R(1)), 4U);
  EXPECT_TRUE(monitor.waitingFor(R(1)) == R(2));
  monitor.record(R(1), true, P(14, 0), std::nullopt);  // four cells away: it has got somewhere
  EXPECT_EQ(monitor.stuckTicks(R(1)), 0U);
  EXPECT_FALSE(monitor.waitingFor(R(1)).has_value());
}

TEST(ProgressMonitorTest, ARobotMovingBackAndForthWithinTheNeighbourhoodIsStillGoingNowhere) {
  ProgressMonitor monitor(2);
  for (int i = 0; i < 12; ++i) {
    // It steps aside and back every other step, never more than a cell from where it began.
    monitor.record(R(1), true, P(i % 2, 0), R(2));
  }
  EXPECT_EQ(monitor.stuckTicks(R(1)), 11U);
}

TEST(ProgressMonitorTest, ARobotNeedsToGetMoreThanTheRadiusAwayToCountAsHavingMoved) {
  ProgressMonitor monitor(1, 2);
  monitor.record(R(1), true, P(0, 0), std::nullopt);
  monitor.record(R(1), true, P(2, 0), std::nullopt);  // two cells: still the neighbourhood
  EXPECT_EQ(monitor.stuckTicks(R(1)), 1U);
  monitor.record(R(1), true, P(3, 0), std::nullopt);  // three: away
  EXPECT_EQ(monitor.stuckTicks(R(1)), 0U);
  ProgressMonitor tight(1, 0);  // radius 0: any move at all counts
  tight.record(R(1), true, P(0, 0), std::nullopt);
  tight.record(R(1), true, P(0, 0), std::nullopt);
  EXPECT_EQ(tight.stuckTicks(R(1)), 1U);
  tight.record(R(1), true, P(1, 0), std::nullopt);
  EXPECT_EQ(tight.stuckTicks(R(1)), 0U);
}

TEST(ProgressMonitorTest, ARobotThatDoesNotWantToMoveIsNeverStuck) {
  ProgressMonitor monitor(2);
  for (int i = 0; i < 10; ++i) {
    monitor.record(R(2), false, P(5, 5), std::nullopt);  // loading, charging, idle
  }
  EXPECT_EQ(monitor.stuckTicks(R(2)), 0U);
  EXPECT_TRUE(monitor.stuck(1).empty());
}

TEST(ProgressMonitorTest, AWaitStartsWhenTheRobotWantsToMoveNotWhenItWasMerelyStandingThere) {
  ProgressMonitor monitor(1);
  monitor.record(R(1), false, P(3, 3), std::nullopt);  // idle on this very cell
  monitor.record(R(1), true, P(3, 3), std::nullopt);   // now it has a route and has to wait
  EXPECT_EQ(monitor.stuckTicks(R(1)), 0U);
  monitor.record(R(1), true, P(3, 3), std::nullopt);
  EXPECT_EQ(monitor.stuckTicks(R(1)), 1U);
}

TEST(ProgressMonitorTest, ResetForgetsWhatWasCountedAndWhereTheRobotBeganToWait) {
  ProgressMonitor monitor(2);
  for (int i = 0; i < 6; ++i) {
    stuckBehind(monitor, 1, 2);
  }
  monitor.reset(R(1));
  EXPECT_EQ(monitor.stuckTicks(R(1)), 0U);
  EXPECT_FALSE(monitor.waitingFor(R(1)).has_value());
  stuckBehind(monitor, 1, 2);
  EXPECT_EQ(monitor.stuckTicks(R(1)), 0U);  // it begins again: this step only marks the start
  stuckBehind(monitor, 1, 2);
  EXPECT_EQ(monitor.stuckTicks(R(1)), 1U);
}

TEST(ProgressMonitorTest, StuckListsTheRobotsAtOrOverTheThresholdInIdOrder) {
  ProgressMonitor monitor(4);
  for (int i = 0; i < 6; ++i) {
    stuckBehind(monitor, 4, 1);  // 5 counted
  }
  for (int i = 0; i < 4; ++i) {
    stuckBehind(monitor, 2, 1);  // 3
  }
  stuckBehind(monitor, 3, 1);
  stuckBehind(monitor, 3, 1);  // 1
  EXPECT_EQ(monitor.stuck(3), (std::vector<RobotId>{R(2), R(4)}));
  EXPECT_EQ(monitor.stuck(5), (std::vector<RobotId>{R(4)}));
  EXPECT_EQ(monitor.stuck(1), (std::vector<RobotId>{R(2), R(3), R(4)}));
  EXPECT_TRUE(monitor.stuck(6).empty());
}

// Records the same waiting `steps` times (the first only marks where the wait began).
void waitFor(ProgressMonitor& monitor, std::uint32_t robot, std::uint32_t other, int steps) {
  for (int i = 0; i < steps; ++i) {
    stuckBehind(monitor, robot, other);
  }
}

TEST(ProgressMonitorTest, TwoRobotsWaitingForEachOthersCellAreADeadlock) {
  ProgressMonitor monitor(3);
  waitFor(monitor, 1, 3, 5);
  waitFor(monitor, 3, 1, 5);
  const auto rings = monitor.deadlocks(3);
  ASSERT_EQ(rings.size(), 1U);
  EXPECT_EQ(rings[0], (std::vector<RobotId>{R(1), R(3)}));
  EXPECT_TRUE(monitor.deadlocks(5).empty());  // not yet stuck that long
}

TEST(ProgressMonitorTest, AChainIsNotADeadlockUntilItCloses) {
  ProgressMonitor monitor(4);
  waitFor(monitor, 1, 2, 5);
  waitFor(monitor, 2, 3, 5);
  waitFor(monitor, 3, 4, 5);  // robot 4 is free to move
  EXPECT_TRUE(monitor.deadlocks(1).empty());
  waitFor(monitor, 4, 1, 5);  // now it is stuck behind the head of the chain
  const auto rings = monitor.deadlocks(1);
  ASSERT_EQ(rings.size(), 1U);
  EXPECT_EQ(rings[0], (std::vector<RobotId>{R(1), R(2), R(3), R(4)}));
}

TEST(ProgressMonitorTest, ARingListsItsRobotsInWaitingOrderStartingWithTheLowestId) {
  ProgressMonitor monitor(5);
  waitFor(monitor, 4, 2, 3);
  waitFor(monitor, 2, 5, 3);
  waitFor(monitor, 5, 4, 3);
  const auto rings = monitor.deadlocks(1);
  ASSERT_EQ(rings.size(), 1U);
  EXPECT_EQ(rings[0], (std::vector<RobotId>{R(2), R(5), R(4)}));
}

TEST(ProgressMonitorTest, ARingThatIsEnteredAtItsHighestRobotStillStartsWithItsLowest) {
  ProgressMonitor monitor(5);
  waitFor(monitor, 1, 4, 3);  // robot 1 queues behind the ring and is looked at first
  waitFor(monitor, 4, 3, 3);
  waitFor(monitor, 3, 5, 3);
  waitFor(monitor, 5, 4, 3);
  const auto rings = monitor.deadlocks(1);
  ASSERT_EQ(rings.size(), 1U);
  EXPECT_EQ(rings[0], (std::vector<RobotId>{R(3), R(5), R(4)}));
}

TEST(ProgressMonitorTest, RobotsOutsideTheRingThatWaitForItAreNotPartOfIt) {
  ProgressMonitor monitor(4);
  waitFor(monitor, 1, 2, 3);
  waitFor(monitor, 2, 1, 3);
  waitFor(monitor, 3, 1, 3);  // queues behind the ring
  waitFor(monitor, 4, 3, 3);
  const auto rings = monitor.deadlocks(1);
  ASSERT_EQ(rings.size(), 1U);
  EXPECT_EQ(rings[0], (std::vector<RobotId>{R(1), R(2)}));
}

TEST(ProgressMonitorTest, SeveralRingsAreReportedInOrderOfTheirLowestRobot) {
  ProgressMonitor monitor(6);
  waitFor(monitor, 5, 6, 3);
  waitFor(monitor, 6, 5, 3);
  waitFor(monitor, 2, 3, 3);
  waitFor(monitor, 3, 2, 3);
  const auto rings = monitor.deadlocks(1);
  ASSERT_EQ(rings.size(), 2U);
  EXPECT_EQ(rings[0], (std::vector<RobotId>{R(2), R(3)}));
  EXPECT_EQ(rings[1], (std::vector<RobotId>{R(5), R(6)}));
}

TEST(ProgressMonitorTest, ARingBreaksWhenOneOfItsRobotsGetsAwayOrIsNotStuckLongEnough) {
  ProgressMonitor monitor(2);
  waitFor(monitor, 1, 2, 7);
  waitFor(monitor, 2, 1, 2);  // only one step counted
  EXPECT_TRUE(monitor.deadlocks(3).empty());
  waitFor(monitor, 2, 1, 4);
  EXPECT_EQ(monitor.deadlocks(3).size(), 1U);
  monitor.record(R(2), true, P(60, 60), std::nullopt);
  EXPECT_TRUE(monitor.deadlocks(1).empty());
}

TEST(ProgressMonitorTest, AnUnknownRobotIsRefusedAndAnUnknownTargetIsIgnored) {
  ProgressMonitor monitor(2);
  EXPECT_THROW(monitor.record(R(3), true, P(0, 0), std::nullopt), std::out_of_range);
  EXPECT_THROW(monitor.record(RobotId{}, true, P(0, 0), std::nullopt), std::out_of_range);
  EXPECT_THROW((void)monitor.stuckTicks(R(9)), std::out_of_range);
  EXPECT_THROW(monitor.reset(R(9)), std::out_of_range);
  waitFor(monitor, 1, 9, 4);  // waits for a robot that does not exist: no ring
  EXPECT_TRUE(monitor.deadlocks(1).empty());
}

}  // namespace
