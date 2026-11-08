#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotStatus;

Point P(int x, int y) { return Point{x, y}; }

TEST(RobotTest, StartsIdleAtItsPosition) {
  const Robot robot(RobotId{1}, P(3, 4));
  EXPECT_EQ(robot.id(), RobotId{1});
  EXPECT_EQ(robot.position(), P(3, 4));
  EXPECT_EQ(robot.status(), RobotStatus::Idle);
  EXPECT_FALSE(robot.task().has_value());
  EXPECT_FALSE(robot.hasPath());
  EXPECT_EQ(robot.distanceTravelled(), 0U);
}

TEST(RobotTest, FollowsPathOneCellPerAdvance) {
  Robot robot(RobotId{1}, P(0, 0));
  robot.setPath({P(1, 0), P(1, 1)});
  EXPECT_EQ(robot.nextStep(), P(1, 0));
  robot.advance();
  EXPECT_EQ(robot.position(), P(1, 0));
  robot.advance();
  EXPECT_EQ(robot.position(), P(1, 1));
  EXPECT_FALSE(robot.hasPath());
  EXPECT_EQ(robot.distanceTravelled(), 2U);
}

TEST(RobotTest, AdvanceWithoutPathThrows) {
  Robot robot(RobotId{1}, P(0, 0));
  EXPECT_THROW(robot.advance(), std::logic_error);
}

TEST(RobotTest, BlockedCounterResetsOnMoveAndNewPath) {
  Robot robot(RobotId{1}, P(0, 0));
  robot.setPath({P(1, 0), P(2, 0)});
  robot.noteBlocked();
  robot.noteBlocked();
  EXPECT_EQ(robot.blockedTicks(), 2U);
  robot.advance();
  EXPECT_EQ(robot.blockedTicks(), 0U);
  robot.noteBlocked();
  robot.setPath({P(1, 1)});
  EXPECT_EQ(robot.blockedTicks(), 0U);
  robot.noteBlocked();
  robot.clearBlocked();
  EXPECT_EQ(robot.blockedTicks(), 0U);
}

TEST(RobotTest, TaskLifecycleTransitions) {
  Robot robot(RobotId{1}, P(0, 0));
  robot.beginTask(TaskId{7});
  EXPECT_EQ(robot.status(), RobotStatus::ToPickup);
  EXPECT_EQ(robot.task(), TaskId{7});
  robot.beginDelivery();
  EXPECT_EQ(robot.status(), RobotStatus::ToDropoff);
  robot.setPath({P(1, 0)});
  robot.finishTask();
  EXPECT_EQ(robot.status(), RobotStatus::Idle);
  EXPECT_FALSE(robot.task().has_value());
  EXPECT_FALSE(robot.hasPath());
}

TEST(RobotTest, InvalidTransitionsThrow) {
  Robot robot(RobotId{1}, P(0, 0));
  EXPECT_THROW(robot.beginDelivery(), std::logic_error);
  robot.beginTask(TaskId{1});
  EXPECT_THROW(robot.beginTask(TaskId{2}), std::logic_error);
}

TEST(RobotTest, SidestepMovesToAdjacentCellOnly) {
  Robot robot(RobotId{1}, P(2, 2));
  EXPECT_THROW(robot.sidestep(P(4, 2)), std::invalid_argument);
  EXPECT_THROW(robot.sidestep(P(3, 3)), std::invalid_argument);
  robot.sidestep(P(2, 3));
  EXPECT_EQ(robot.position(), P(2, 3));
  EXPECT_EQ(robot.distanceTravelled(), 1U);
}

TEST(RobotTest, SidestepDropsPathButKeepsTask) {
  Robot robot(RobotId{1}, P(0, 0));
  robot.beginTask(TaskId{4});
  robot.setPath({P(1, 0), P(2, 0)});
  robot.noteBlocked();
  robot.sidestep(P(0, 1));
  EXPECT_FALSE(robot.hasPath());
  EXPECT_EQ(robot.blockedTicks(), 0U);
  EXPECT_EQ(robot.status(), RobotStatus::ToPickup);
  EXPECT_EQ(robot.task(), TaskId{4});
}

}  // namespace
