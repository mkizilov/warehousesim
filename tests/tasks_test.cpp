#include <gtest/gtest.h>

#include <set>
#include <stdexcept>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/tasks/assigner.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/tasks/task_generator.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Robot;
using warehousesim::tasks::Assignment;
using warehousesim::tasks::NearestRobotAssigner;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskBoard;
using warehousesim::tasks::TaskGenerator;
using warehousesim::tasks::TaskStatus;

Point P(int x, int y) { return Point{x, y}; }

TEST(TaskBoardTest, SubmitAssignsSequentialIdsStartingAtOne) {
  TaskBoard board;
  EXPECT_EQ(board.submit(P(0, 0), P(1, 1), 5), TaskId{1});
  EXPECT_EQ(board.submit(P(2, 2), P(3, 3), 6), TaskId{2});
  const Task* task = board.find(TaskId{2});
  ASSERT_TRUE(task != nullptr);
  EXPECT_EQ(task->pickup, P(2, 2));
  EXPECT_EQ(task->dropoff, P(3, 3));
  EXPECT_EQ(task->createdAt, 6U);
  EXPECT_EQ(task->status, TaskStatus::Pending);
}

TEST(TaskBoardTest, FindReturnsNullForUnknownIds) {
  TaskBoard board;
  EXPECT_TRUE(board.find(TaskId{}) == nullptr);
  EXPECT_TRUE(board.find(TaskId{1}) == nullptr);
}

TEST(TaskBoardTest, HappyPathLifecycle) {
  TaskBoard board;
  const TaskId id = board.submit(P(0, 0), P(1, 0), 0);
  board.assign(id, RobotId{3});
  EXPECT_EQ(board.find(id)->assignee, RobotId{3});
  board.markInProgress(id);
  board.complete(id);
  EXPECT_EQ(board.find(id)->status, TaskStatus::Completed);
  EXPECT_EQ(board.count(TaskStatus::Completed), 1U);
  EXPECT_EQ(board.count(TaskStatus::Pending), 0U);
}

TEST(TaskBoardTest, InvalidTransitionsThrow) {
  TaskBoard board;
  const TaskId id = board.submit(P(0, 0), P(1, 0), 0);
  EXPECT_THROW(board.markInProgress(id), std::logic_error);
  EXPECT_THROW(board.complete(id), std::logic_error);
  EXPECT_THROW(board.fail(id), std::logic_error);
  board.assign(id, RobotId{1});
  EXPECT_THROW(board.assign(id, RobotId{2}), std::logic_error);
  board.fail(id);
  EXPECT_THROW(board.complete(id), std::logic_error);
}

TEST(TaskBoardTest, UnknownIdThrowsOutOfRange) {
  TaskBoard board;
  EXPECT_THROW(board.assign(TaskId{1}, RobotId{1}), std::out_of_range);
  EXPECT_THROW(board.fail(TaskId{}), std::out_of_range);
}

TEST(NearestRobotAssignerTest, PicksClosestIdleRobot) {
  const std::vector<Robot> robots = {Robot(RobotId{1}, P(0, 0)), Robot(RobotId{2}, P(9, 9))};
  TaskBoard board;
  board.submit(P(8, 8), P(0, 0), 0);
  const auto result = NearestRobotAssigner{}.assign(robots, board.all());
  const std::vector<Assignment> expected = {{RobotId{2}, TaskId{1}}};
  EXPECT_EQ(result, expected);
}

TEST(NearestRobotAssignerTest, TiesGoToLowestRobotId) {
  const std::vector<Robot> robots = {Robot(RobotId{2}, P(0, 2)), Robot(RobotId{1}, P(2, 0))};
  TaskBoard board;
  board.submit(P(0, 0), P(5, 5), 0);
  const auto result = NearestRobotAssigner{}.assign(robots, board.all());
  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result[0].robot, RobotId{1});
}

TEST(NearestRobotAssignerTest, SkipsBusyRobotsAndNonPendingTasks) {
  std::vector<Robot> robots = {Robot(RobotId{1}, P(0, 0)), Robot(RobotId{2}, P(1, 0))};
  robots[0].beginTask(TaskId{1});
  TaskBoard board;
  const TaskId assigned = board.submit(P(0, 0), P(1, 1), 0);
  board.assign(assigned, RobotId{1});
  board.submit(P(2, 0), P(1, 1), 0);
  const auto result = NearestRobotAssigner{}.assign(robots, board.all());
  const std::vector<Assignment> expected = {{RobotId{2}, TaskId{2}}};
  EXPECT_EQ(result, expected);
}

TEST(NearestRobotAssignerTest, AssignsEachRobotAndTaskAtMostOnce) {
  const std::vector<Robot> robots = {Robot(RobotId{1}, P(0, 0)), Robot(RobotId{2}, P(5, 0))};
  TaskBoard board;
  board.submit(P(1, 0), P(0, 0), 0);
  board.submit(P(2, 0), P(0, 0), 0);
  board.submit(P(3, 0), P(0, 0), 0);
  const auto result = NearestRobotAssigner{}.assign(robots, board.all());
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0], (Assignment{RobotId{1}, TaskId{1}}));
  EXPECT_EQ(result[1], (Assignment{RobotId{2}, TaskId{2}}));
}

TEST(NearestRobotAssignerTest, NoRobotsOrNoTasksYieldsNothing) {
  TaskBoard board;
  board.submit(P(1, 0), P(0, 0), 0);
  EXPECT_TRUE(NearestRobotAssigner{}.assign(std::vector<Robot>{}, board.all()).empty());
  const std::vector<Robot> robots = {Robot(RobotId{1}, P(0, 0))};
  EXPECT_TRUE(NearestRobotAssigner{}.assign(robots, std::vector<Task>{}).empty());
}

TEST(TaskGeneratorTest, RejectsEmptyLocationLists) {
  EXPECT_THROW(TaskGenerator({}, {P(0, 0)}, 1), std::invalid_argument);
  EXPECT_THROW(TaskGenerator({P(0, 0)}, {}, 1), std::invalid_argument);
}

TEST(TaskGeneratorTest, SameSeedReproducesRequestsAndStaysWithinLocations) {
  const std::vector<Point> pickups = {P(1, 0), P(2, 0), P(3, 0)};
  const std::vector<Point> dropoffs = {P(0, 5), P(1, 5)};
  TaskGenerator a(pickups, dropoffs, 99);
  TaskGenerator b(pickups, dropoffs, 99);
  const std::set<Point> allowedPickups(pickups.begin(), pickups.end());
  const std::set<Point> allowedDropoffs(dropoffs.begin(), dropoffs.end());
  for (int i = 0; i < 50; ++i) {
    const auto left = a.next();
    const auto right = b.next();
    EXPECT_EQ(left.pickup, right.pickup);
    EXPECT_EQ(left.dropoff, right.dropoff);
    EXPECT_TRUE(allowedPickups.contains(left.pickup));
    EXPECT_TRUE(allowedDropoffs.contains(left.dropoff));
  }
}

}  // namespace
