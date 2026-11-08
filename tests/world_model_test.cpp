#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/world.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::world::CellEdit;
using warehousesim::world::CellType;
using warehousesim::world::MoveStatus;
using warehousesim::world::PlaceStatus;
using warehousesim::world::Warehouse;
using warehousesim::world::World;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }

World worldFrom(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return World(*warehouse);
}

// ---- static vs dynamic separation -------------------------------------------------------

TEST(WorldTest, OccupancyNeverChangesTheStaticLayout) {
  World world = worldFrom({"...", ".S.", "..D"});
  const std::vector<std::string> before = world.layout().toRows();
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(world.addRobot(R(2), P(2, 2)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(world.removeRobot(R(2)));
  EXPECT_EQ(world.layout().toRows(), before);
}

TEST(WorldTest, AnOccupiedCellIsStillWalkableButNotFree) {
  World world = worldFrom({"..."});
  ASSERT_TRUE(world.addRobot(R(1), P(1, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.isWalkable(P(1, 0)));
  EXPECT_TRUE(world.isOccupied(P(1, 0)));
  EXPECT_FALSE(world.isFree(P(1, 0)));
  EXPECT_TRUE(world.isFree(P(0, 0)));
  EXPECT_EQ(world.occupantAt(P(1, 0)), R(1));
}

TEST(WorldTest, ObstaclesAreNeverFreeAndNeverOccupied) {
  const World world = worldFrom({"#S."});
  EXPECT_TRUE(world.isObstacle(P(0, 0)));
  EXPECT_TRUE(world.isObstacle(P(1, 0)));
  EXPECT_FALSE(world.isFree(P(0, 0)));
  EXPECT_FALSE(world.isFree(P(1, 0)));
  EXPECT_FALSE(world.isOccupied(P(0, 0)));
  EXPECT_FALSE(world.isFree(P(5, 5)));  // outside the grid
}

TEST(WorldTest, FreeNeighborsExcludeObstaclesAndRobots) {
  World world = worldFrom({".#.", "...", ".S."});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 1)) == PlaceStatus::Ok);
  // Centre (1,1): north wall, south shelf, west robot, east free.
  EXPECT_EQ(world.freeNeighbors(P(1, 1)), (std::vector<Point>{P(2, 1)}));
  // Walkable neighbours ignore robots; plain neighbours ignore obstacles as well.
  EXPECT_EQ(world.walkableNeighbors(P(1, 1)), (std::vector<Point>{P(2, 1), P(0, 1)}));
  EXPECT_EQ(world.neighbors(P(1, 1)).size(), 4U);
}

// ---- adding robots ----------------------------------------------------------------------

TEST(WorldTest, AddRobotAcceptsEveryWalkableCellType) {
  World world = worldFrom({".PDC"});
  std::uint32_t id = 1;
  for (int x = 0; x < 4; ++x) {
    EXPECT_TRUE(world.addRobot(R(id++), P(x, 0)) == PlaceStatus::Ok);
  }
  EXPECT_EQ(world.occupancy().robotCount(), 4U);
}

TEST(WorldTest, AddRobotRejectsObstaclesBoundsDuplicatesAndInvalidIds) {
  World world = worldFrom({".#S."});
  EXPECT_TRUE(world.addRobot(R(1), P(1, 0)) == PlaceStatus::BlockedByObstacle);
  EXPECT_TRUE(world.addRobot(R(1), P(2, 0)) == PlaceStatus::BlockedByObstacle);
  EXPECT_TRUE(world.addRobot(R(1), P(4, 0)) == PlaceStatus::OutOfBounds);
  EXPECT_TRUE(world.addRobot(R(1), P(0, -1)) == PlaceStatus::OutOfBounds);
  EXPECT_TRUE(world.addRobot(RobotId{}, P(0, 0)) == PlaceStatus::InvalidRobot);
  EXPECT_TRUE(world.addRobot(RobotId{}, P(1, 0)) == PlaceStatus::InvalidRobot);
  EXPECT_EQ(world.occupancy().robotCount(), 0U);

  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.addRobot(R(1), P(3, 0)) == PlaceStatus::DuplicateRobot);
  EXPECT_TRUE(world.addRobot(R(1), P(1, 0)) == PlaceStatus::DuplicateRobot);
  EXPECT_TRUE(world.addRobot(R(2), P(0, 0)) == PlaceStatus::CellOccupied);
  EXPECT_EQ(world.occupancy().robotCount(), 1U);
}

// ---- valid movement ---------------------------------------------------------------------

TEST(WorldTest, MovesOneOrthogonalStepInEveryDirection) {
  World world = worldFrom({"...", "...", "..."});
  ASSERT_TRUE(world.addRobot(R(1), P(1, 1)) == PlaceStatus::Ok);
  for (const Point target : {P(1, 0), P(1, 1), P(2, 1), P(1, 1), P(1, 2), P(1, 1), P(0, 1)}) {
    EXPECT_TRUE(world.moveRobot(R(1), target) == MoveStatus::Ok);
    EXPECT_EQ(world.occupancy().positionOf(R(1)), target);
  }
}

TEST(WorldTest, MoveFreesTheOldCell) {
  World world = worldFrom({".."});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(world.moveRobot(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(world.isFree(P(0, 0)));
  EXPECT_FALSE(world.isFree(P(1, 0)));
  // A second robot can now take the vacated cell.
  EXPECT_TRUE(world.addRobot(R(2), P(0, 0)) == PlaceStatus::Ok);
}

TEST(WorldTest, MoveRejectsNonAdjacentTargets) {
  World world = worldFrom({"...", "...", "..."});
  ASSERT_TRUE(world.addRobot(R(1), P(1, 1)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(1), P(0, 0)) == MoveStatus::NotAdjacent);  // diagonal
  EXPECT_TRUE(world.moveRobot(R(1), P(2, 2)) == MoveStatus::NotAdjacent);
  EXPECT_TRUE(world.moveRobot(R(1), P(1, 1)) == MoveStatus::NotAdjacent);  // stay put
  World wide = worldFrom({"....."});
  ASSERT_TRUE(wide.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(wide.moveRobot(R(1), P(2, 0)) == MoveStatus::NotAdjacent);  // two steps
  EXPECT_EQ(wide.occupancy().positionOf(R(1)), P(0, 0));
}

TEST(WorldTest, MoveRejectsObstaclesBoundsAndRobots) {
  World world = worldFrom({"#.S", "..."});
  ASSERT_TRUE(world.addRobot(R(1), P(1, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(world.addRobot(R(2), P(1, 1)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(1), P(0, 0)) == MoveStatus::BlockedByObstacle);  // wall
  EXPECT_TRUE(world.moveRobot(R(1), P(2, 0)) == MoveStatus::BlockedByObstacle);  // shelf
  EXPECT_TRUE(world.moveRobot(R(1), P(1, 1)) == MoveStatus::CellOccupied);
  EXPECT_TRUE(world.moveRobot(R(1), P(1, -1)) == MoveStatus::OutOfBounds);
  EXPECT_TRUE(world.moveRobot(R(9), P(0, 1)) == MoveStatus::UnknownRobot);
  EXPECT_EQ(world.occupancy().positionOf(R(1)), P(1, 0));
  EXPECT_EQ(world.occupancy().positionOf(R(2)), P(1, 1));
}

TEST(WorldTest, MovingOffTheEdgeIsOutOfBoundsNotAnObstacle) {
  World world = worldFrom({".."});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(1), P(-1, 0)) == MoveStatus::OutOfBounds);
  ASSERT_TRUE(world.moveRobot(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(1), P(2, 0)) == MoveStatus::OutOfBounds);
}

TEST(WorldTest, CheckMoveReportsTheSameVerdictWithoutMoving) {
  World world = worldFrom({"..#"});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.checkMove(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_EQ(world.occupancy().positionOf(R(1)), P(0, 0));
  EXPECT_TRUE(world.checkMove(R(1), P(2, 0)) == MoveStatus::NotAdjacent);
  EXPECT_TRUE(world.checkMove(R(2), P(1, 0)) == MoveStatus::UnknownRobot);
}

TEST(WorldTest, RobotsCanFollowEachOtherNoseToTail) {
  World world = worldFrom({"...."});
  ASSERT_TRUE(world.addRobot(R(1), P(1, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(world.addRobot(R(2), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(2), P(1, 0)) == MoveStatus::CellOccupied);  // leader hasn't moved
  EXPECT_TRUE(world.moveRobot(R(1), P(2, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(2), P(1, 0)) == MoveStatus::Ok);
}

// ---- obstacle placement and cell modification -------------------------------------------

TEST(WorldTest, PlaceObstacleTurnsAFreeCellIntoABarrierRobotsRespect) {
  World world = worldFrom({"..."});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.placeObstacle(P(1, 0), CellType::Wall) == CellEdit::Ok);
  EXPECT_TRUE(world.isObstacle(P(1, 0)));
  EXPECT_TRUE(world.moveRobot(R(1), P(1, 0)) == MoveStatus::BlockedByObstacle);
  EXPECT_TRUE(world.placeObstacle(P(2, 0), CellType::Shelf) == CellEdit::Ok);
  EXPECT_EQ(world.layout().at(P(2, 0)), CellType::Shelf);
}

TEST(WorldTest, PlaceObstacleIsRefusedWhereARobotStands) {
  World world = worldFrom({".."});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.placeObstacle(P(0, 0), CellType::Wall) == CellEdit::OccupiedByRobot);
  EXPECT_TRUE(world.setCell(P(0, 0), CellType::Shelf) == CellEdit::OccupiedByRobot);
  EXPECT_EQ(world.layout().at(P(0, 0)), CellType::Free);
  // Once the robot leaves, the same edit succeeds.
  ASSERT_TRUE(world.moveRobot(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(world.placeObstacle(P(0, 0), CellType::Wall) == CellEdit::Ok);
}

TEST(WorldTest, PlaceObstacleRejectsWalkableTypesAndOutOfBounds) {
  World world = worldFrom({".."});
  for (const CellType type :
       {CellType::Free, CellType::Pickup, CellType::Dropoff, CellType::Charging}) {
    EXPECT_TRUE(world.placeObstacle(P(0, 0), type) == CellEdit::NotAnObstacle);
  }
  EXPECT_TRUE(world.placeObstacle(P(2, 0), CellType::Wall) == CellEdit::OutOfBounds);
  EXPECT_TRUE(world.placeObstacle(P(0, -1), CellType::Shelf) == CellEdit::OutOfBounds);
  EXPECT_EQ(world.layout().toRows(), (std::vector<std::string>{".."}));
}

TEST(WorldTest, SetCellAllowsWalkableTypesUnderARobot) {
  World world = worldFrom({"."});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.setCell(P(0, 0), CellType::Charging) == CellEdit::Ok);
  EXPECT_EQ(world.layout().at(P(0, 0)), CellType::Charging);
  EXPECT_EQ(world.occupancy().positionOf(R(1)), P(0, 0));
  EXPECT_TRUE(world.setCell(P(1, 0), CellType::Pickup) == CellEdit::OutOfBounds);
}

TEST(WorldTest, ClearCellRemovesObstaclesAndOpensTheCell) {
  World world = worldFrom({".#."});
  ASSERT_TRUE(world.addRobot(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(world.moveRobot(R(1), P(1, 0)) == MoveStatus::BlockedByObstacle);
  EXPECT_TRUE(world.clearCell(P(1, 0)) == CellEdit::Ok);
  EXPECT_TRUE(world.moveRobot(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(world.clearCell(P(9, 9)) == CellEdit::OutOfBounds);
}

// ---- consistency under random use -------------------------------------------------------

// Whatever sequence of robot moves and layout edits is attempted, no robot may ever end up on
// an obstacle, two robots may never share a cell, and the layout and occupancy must agree.
TEST(WorldTest, RandomMovesAndEditsNeverBreakTheInvariants) {
  World world = worldFrom({"......", "......", "......", "......"});
  Rng rng(77);
  const std::vector<RobotId> robots = {R(1), R(2), R(3), R(4), R(5)};
  for (const RobotId robot : robots) {
    PlaceStatus status = PlaceStatus::CellOccupied;
    while (status != PlaceStatus::Ok) {
      status =
          world.addRobot(robot, P(static_cast<int>(rng.below(6)), static_cast<int>(rng.below(4))));
    }
  }

  const std::vector<CellType> types = {CellType::Free,   CellType::Wall,    CellType::Shelf,
                                       CellType::Pickup, CellType::Dropoff, CellType::Charging};
  for (int step = 0; step < 4000; ++step) {
    if (rng.below(4) == 0) {
      const Point p = P(static_cast<int>(rng.below(7)) - 1, static_cast<int>(rng.below(5)) - 1);
      const CellType type = types[rng.below(types.size())];
      const CellEdit result = world.setCell(p, type);
      const bool obstacle = type == CellType::Wall || type == CellType::Shelf;
      if (!world.inBounds(p)) {
        EXPECT_TRUE(result == CellEdit::OutOfBounds);
      } else if (obstacle && world.isOccupied(p)) {
        EXPECT_TRUE(result == CellEdit::OccupiedByRobot);
      } else {
        EXPECT_TRUE(result == CellEdit::Ok);
      }
    } else {
      const RobotId robot = robots[rng.below(robots.size())];
      const Point from = *world.occupancy().positionOf(robot);
      const Point to = from + warehousesim::core::kCardinalDirections[rng.below(4)];
      const MoveStatus expected = !world.inBounds(to)    ? MoveStatus::OutOfBounds
                                  : world.isObstacle(to) ? MoveStatus::BlockedByObstacle
                                  : world.isOccupied(to) ? MoveStatus::CellOccupied
                                                         : MoveStatus::Ok;
      EXPECT_TRUE(world.moveRobot(robot, to) == expected);
    }

    std::vector<Point> seen;
    for (const RobotId robot : robots) {
      const auto position = world.occupancy().positionOf(robot);
      ASSERT_TRUE(position.has_value());
      ASSERT_TRUE(world.isWalkable(*position));
      ASSERT_EQ(world.occupantAt(*position), robot);
      for (const Point other : seen) {
        ASSERT_NE(other, *position);
      }
      seen.push_back(*position);
    }
    ASSERT_EQ(world.occupancy().robotCount(), robots.size());
  }
}

}  // namespace
