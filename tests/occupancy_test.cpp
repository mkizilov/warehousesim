#include "warehousesim/world/occupancy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/grid_size.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::world::GridSize;
using warehousesim::world::MoveStatus;
using warehousesim::world::Occupancy;
using warehousesim::world::PlaceStatus;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }

TEST(OccupancyTest, StartsEmpty) {
  const Occupancy occupancy(GridSize(3, 3));
  EXPECT_EQ(occupancy.robotCount(), 0U);
  EXPECT_FALSE(occupancy.isOccupied(P(1, 1)));
  EXPECT_FALSE(occupancy.occupantAt(P(1, 1)).has_value());
  EXPECT_FALSE(occupancy.positionOf(R(1)).has_value());
  EXPECT_TRUE(occupancy.occupiedCells().empty());
}

TEST(OccupancyTest, PlaceRecordsBothDirectionsOfTheMapping) {
  Occupancy occupancy(GridSize(3, 3));
  EXPECT_TRUE(occupancy.place(R(7), P(2, 1)) == PlaceStatus::Ok);
  EXPECT_TRUE(occupancy.isOccupied(P(2, 1)));
  EXPECT_EQ(occupancy.occupantAt(P(2, 1)), R(7));
  EXPECT_EQ(occupancy.positionOf(R(7)), P(2, 1));
  EXPECT_EQ(occupancy.robotCount(), 1U);
  EXPECT_FALSE(occupancy.isOccupied(P(1, 2)));  // x/y are not swapped
}

TEST(OccupancyTest, PlaceRejectsInvalidRequestsWithoutChangingState) {
  Occupancy occupancy(GridSize(3, 3));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);

  EXPECT_TRUE(occupancy.place(RobotId{}, P(1, 1)) == PlaceStatus::InvalidRobot);
  EXPECT_TRUE(occupancy.place(R(1), P(1, 1)) == PlaceStatus::DuplicateRobot);
  EXPECT_TRUE(occupancy.place(R(2), P(0, 0)) == PlaceStatus::CellOccupied);
  EXPECT_TRUE(occupancy.place(R(2), P(3, 0)) == PlaceStatus::OutOfBounds);
  EXPECT_TRUE(occupancy.place(R(2), P(0, -1)) == PlaceStatus::OutOfBounds);

  EXPECT_EQ(occupancy.robotCount(), 1U);
  EXPECT_EQ(occupancy.positionOf(R(1)), P(0, 0));
  EXPECT_FALSE(occupancy.positionOf(R(2)).has_value());
  EXPECT_FALSE(occupancy.isOccupied(P(1, 1)));
}

TEST(OccupancyTest, QueriesOutsideTheGridAreSafe) {
  Occupancy occupancy(GridSize(2, 2));
  ASSERT_TRUE(occupancy.place(R(1), P(1, 1)) == PlaceStatus::Ok);
  EXPECT_FALSE(occupancy.isOccupied(P(2, 1)));
  EXPECT_FALSE(occupancy.isOccupied(P(-1, 0)));
  EXPECT_FALSE(occupancy.isOccupied(P(std::numeric_limits<std::int32_t>::max(), 1)));
  EXPECT_FALSE(occupancy.occupantAt(P(0, -5)).has_value());
}

TEST(OccupancyTest, MoveFreesTheOldCellAndTakesTheNewOne) {
  Occupancy occupancy(GridSize(3, 3));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);
  EXPECT_TRUE(occupancy.move(R(1), P(2, 2)) == MoveStatus::Ok);
  EXPECT_FALSE(occupancy.isOccupied(P(0, 0)));
  EXPECT_EQ(occupancy.occupantAt(P(2, 2)), R(1));
  EXPECT_EQ(occupancy.positionOf(R(1)), P(2, 2));
  EXPECT_EQ(occupancy.robotCount(), 1U);
}

TEST(OccupancyTest, MoveRejectsInvalidRequestsWithoutChangingState) {
  Occupancy occupancy(GridSize(3, 3));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(occupancy.place(R(2), P(1, 0)) == PlaceStatus::Ok);

  EXPECT_TRUE(occupancy.move(R(9), P(2, 2)) == MoveStatus::UnknownRobot);
  EXPECT_TRUE(occupancy.move(R(1), P(3, 3)) == MoveStatus::OutOfBounds);
  EXPECT_TRUE(occupancy.move(R(1), P(1, 0)) == MoveStatus::CellOccupied);
  EXPECT_TRUE(occupancy.move(R(1), P(0, 0)) == MoveStatus::CellOccupied);  // own cell

  EXPECT_EQ(occupancy.positionOf(R(1)), P(0, 0));
  EXPECT_EQ(occupancy.positionOf(R(2)), P(1, 0));
  EXPECT_EQ(occupancy.occupantAt(P(0, 0)), R(1));
  EXPECT_EQ(occupancy.occupantAt(P(1, 0)), R(2));
}

TEST(OccupancyTest, RemoveFreesTheCellAndForgetsTheRobot) {
  Occupancy occupancy(GridSize(2, 2));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 1)) == PlaceStatus::Ok);
  EXPECT_TRUE(occupancy.remove(R(1)));
  EXPECT_FALSE(occupancy.isOccupied(P(0, 1)));
  EXPECT_FALSE(occupancy.positionOf(R(1)).has_value());
  EXPECT_EQ(occupancy.robotCount(), 0U);
  EXPECT_FALSE(occupancy.remove(R(1)));
  // The robot id and the cell can both be reused.
  EXPECT_TRUE(occupancy.place(R(1), P(0, 1)) == PlaceStatus::Ok);
}

TEST(OccupancyTest, OccupiedCellsAreOrderedByRobotId) {
  Occupancy occupancy(GridSize(4, 1));
  ASSERT_TRUE(occupancy.place(R(3), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(occupancy.place(R(1), P(3, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(occupancy.place(R(2), P(1, 0)) == PlaceStatus::Ok);
  const std::vector<Point> expected = {P(3, 0), P(1, 0), P(0, 0)};
  EXPECT_EQ(occupancy.occupiedCells(), expected);
}

TEST(OccupancyTest, EveryCellOfTheGridCanBeOccupiedExactlyOnce) {
  const GridSize size(4, 3);
  Occupancy occupancy(size);
  for (std::size_t i = 0; i < size.cellCount(); ++i) {
    ASSERT_TRUE(occupancy.place(R(static_cast<std::uint32_t>(i + 1)), size.pointAt(i)) ==
                PlaceStatus::Ok);
  }
  EXPECT_EQ(occupancy.robotCount(), size.cellCount());
  for (std::size_t i = 0; i < size.cellCount(); ++i) {
    EXPECT_EQ(occupancy.occupantAt(size.pointAt(i)), R(static_cast<std::uint32_t>(i + 1)));
  }
  EXPECT_TRUE(occupancy.place(R(99), P(0, 0)) == PlaceStatus::CellOccupied);
}

// Replays a long deterministic sequence of random operations against a trivial reference
// model (robot -> cell) and checks after every operation that both indexes agree with it and
// with each other.
TEST(OccupancyTest, RandomOperationsKeepBothIndexesConsistent) {
  const GridSize size(5, 4);
  Occupancy occupancy(size);
  std::map<RobotId, Point> model;
  Rng rng(2024);

  const auto randomPoint = [&] {
    // Slightly larger than the grid so out-of-bounds requests are exercised too.
    return P(static_cast<int>(rng.below(7)) - 1, static_cast<int>(rng.below(6)) - 1);
  };
  const auto modelOccupied = [&](Point p) {
    for (const auto& [robot, position] : model) {
      if (position == p) {
        return true;
      }
    }
    return false;
  };

  for (int step = 0; step < 3000; ++step) {
    const RobotId robot = R(static_cast<std::uint32_t>(rng.below(8)));  // id 0 is invalid
    const Point p = randomPoint();
    switch (rng.below(3)) {
      case 0: {
        const bool shouldSucceed =
            robot.valid() && !model.contains(robot) && size.contains(p) && !modelOccupied(p);
        const PlaceStatus status = occupancy.place(robot, p);
        EXPECT_EQ(status == PlaceStatus::Ok, shouldSucceed);
        if (shouldSucceed) {
          model[robot] = p;
        }
        break;
      }
      case 1: {
        const bool shouldSucceed = model.contains(robot) && size.contains(p) && !modelOccupied(p);
        const MoveStatus status = occupancy.move(robot, p);
        EXPECT_EQ(status == MoveStatus::Ok, shouldSucceed);
        if (shouldSucceed) {
          model[robot] = p;
        }
        break;
      }
      default: {
        const bool shouldSucceed = model.contains(robot);
        EXPECT_EQ(occupancy.remove(robot), shouldSucceed);
        model.erase(robot);
        break;
      }
    }

    ASSERT_EQ(occupancy.robotCount(), model.size());
    std::size_t occupiedCells = 0;
    for (std::size_t i = 0; i < size.cellCount(); ++i) {
      const Point cell = size.pointAt(i);
      const auto occupant = occupancy.occupantAt(cell);
      ASSERT_EQ(occupant.has_value(), occupancy.isOccupied(cell));
      if (occupant) {
        ++occupiedCells;
        ASSERT_TRUE(model.contains(*occupant));
        ASSERT_EQ(model.at(*occupant), cell);
        ASSERT_EQ(occupancy.positionOf(*occupant), cell);
      }
    }
    ASSERT_EQ(occupiedCells, model.size());
    ASSERT_EQ(occupancy.occupiedCells().size(), model.size());
    for (const auto& [id, position] : model) {
      ASSERT_EQ(occupancy.positionOf(id), position);
      ASSERT_EQ(occupancy.occupantAt(position), id);
    }
  }
}

// ---- cells claimed ahead of a robot ------------------------------------------------------

TEST(OccupancyClaimTest, AClaimedCellBelongsToTheRobotAndKeepsEveryoneElseOut) {
  Occupancy occupancy(GridSize(4, 2));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(occupancy.place(R(2), P(3, 1)) == PlaceStatus::Ok);
  EXPECT_TRUE(occupancy.claim(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(occupancy.claim(R(1), P(2, 0)) == MoveStatus::Ok);

  EXPECT_EQ(occupancy.holdsOf(R(1)), (std::vector<Point>{P(0, 0), P(1, 0), P(2, 0)}));
  EXPECT_EQ(occupancy.positionOf(R(1)), P(0, 0));  // the position is still the first cell
  EXPECT_EQ(occupancy.occupantAt(P(2, 0)), R(1));
  EXPECT_TRUE(occupancy.isOccupied(P(1, 0)));
  EXPECT_EQ(occupancy.robotCount(), 2U);  // robots, not cells
  EXPECT_EQ(occupancy.occupiedCells(),
            (std::vector<Point>{P(0, 0), P(1, 0), P(2, 0), P(3, 1)}));  // by robot id, in order

  EXPECT_TRUE(occupancy.claim(R(2), P(2, 0)) == MoveStatus::CellOccupied);
  EXPECT_TRUE(occupancy.claim(R(2), P(1, 0)) == MoveStatus::CellOccupied);
  EXPECT_TRUE(occupancy.claim(R(1), P(1, 0)) == MoveStatus::CellOccupied);  // not twice
  EXPECT_TRUE(occupancy.claim(R(1), P(4, 0)) == MoveStatus::OutOfBounds);
  EXPECT_TRUE(occupancy.claim(R(9), P(2, 1)) == MoveStatus::UnknownRobot);
  EXPECT_TRUE(occupancy.place(R(3), P(1, 0)) == PlaceStatus::CellOccupied);
  EXPECT_EQ(occupancy.holdsOf(R(2)), (std::vector<Point>{P(3, 1)}));  // refusals changed nothing
  EXPECT_TRUE(occupancy.holdsOf(R(9)).empty());
}

TEST(OccupancyClaimTest, AdvancingPassesTheFirstClaimedCellAndFreesTheOneLeftBehind) {
  Occupancy occupancy(GridSize(4, 1));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(occupancy.claim(R(1), P(1, 0)) == MoveStatus::Ok);
  ASSERT_TRUE(occupancy.claim(R(1), P(2, 0)) == MoveStatus::Ok);

  EXPECT_TRUE(occupancy.advance(R(1)));
  EXPECT_EQ(occupancy.positionOf(R(1)), P(1, 0));
  EXPECT_EQ(occupancy.holdsOf(R(1)), (std::vector<Point>{P(1, 0), P(2, 0)}));
  EXPECT_FALSE(occupancy.isOccupied(P(0, 0)));  // free for someone else now
  EXPECT_TRUE(occupancy.place(R(2), P(0, 0)) == PlaceStatus::Ok);

  EXPECT_TRUE(occupancy.advance(R(1)));
  EXPECT_EQ(occupancy.holdsOf(R(1)), (std::vector<Point>{P(2, 0)}));
  EXPECT_FALSE(occupancy.advance(R(1)));  // nothing claimed: it cannot pass a cell it does not hold
  EXPECT_FALSE(occupancy.advance(R(7)));
  EXPECT_EQ(occupancy.positionOf(R(1)), P(2, 0));
}

TEST(OccupancyClaimTest, ReleasingGivesBackTheNewestClaimsAndNeverThePosition) {
  Occupancy occupancy(GridSize(5, 1));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);
  for (int x = 1; x <= 3; ++x) {
    ASSERT_TRUE(occupancy.claim(R(1), P(x, 0)) == MoveStatus::Ok);
  }
  EXPECT_EQ(occupancy.releaseClaims(R(1), 5), 0U);  // already within the limit
  EXPECT_EQ(occupancy.releaseClaims(R(1), 1), 2U);
  EXPECT_EQ(occupancy.holdsOf(R(1)), (std::vector<Point>{P(0, 0), P(1, 0)}));
  EXPECT_FALSE(occupancy.isOccupied(P(3, 0)));
  EXPECT_FALSE(occupancy.isOccupied(P(2, 0)));
  EXPECT_EQ(occupancy.releaseClaims(R(1), 0), 1U);
  EXPECT_EQ(occupancy.holdsOf(R(1)), (std::vector<Point>{P(0, 0)}));
  EXPECT_EQ(occupancy.releaseClaims(R(1), 0), 0U);  // the position stays
  EXPECT_EQ(occupancy.releaseClaims(R(9), 0), 0U);
  EXPECT_TRUE(occupancy.isOccupied(P(0, 0)));
}

TEST(OccupancyClaimTest, ARobotThatHoldsClaimedCellsCannotBeTeleported) {
  Occupancy occupancy(GridSize(4, 1));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(occupancy.claim(R(1), P(1, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(occupancy.move(R(1), P(3, 0)) == MoveStatus::HasClaims);
  EXPECT_EQ(occupancy.holdsOf(R(1)), (std::vector<Point>{P(0, 0), P(1, 0)}));
  EXPECT_FALSE(occupancy.isOccupied(P(3, 0)));
  EXPECT_EQ(occupancy.releaseClaims(R(1), 0), 1U);
  EXPECT_TRUE(occupancy.move(R(1), P(3, 0)) == MoveStatus::Ok);
}

TEST(OccupancyClaimTest, RemovingARobotFreesEverythingItHeld) {
  Occupancy occupancy(GridSize(4, 1));
  ASSERT_TRUE(occupancy.place(R(1), P(0, 0)) == PlaceStatus::Ok);
  ASSERT_TRUE(occupancy.claim(R(1), P(1, 0)) == MoveStatus::Ok);
  ASSERT_TRUE(occupancy.claim(R(1), P(2, 0)) == MoveStatus::Ok);
  EXPECT_TRUE(occupancy.remove(R(1)));
  for (int x = 0; x < 4; ++x) {
    EXPECT_FALSE(occupancy.isOccupied(P(x, 0)));
  }
  EXPECT_TRUE(occupancy.occupiedCells().empty());
  EXPECT_EQ(occupancy.robotCount(), 0U);
}

// The two indexes (cell -> robot and robot -> held cells) never disagree, whatever happens.
TEST(OccupancyClaimTest, RandomOperationsKeepBothIndexesInStep) {
  Rng rng(99);
  const GridSize size(5, 4);
  Occupancy occupancy(size);
  std::map<RobotId, std::vector<Point>> model;  // robot -> position then claims
  const auto heldByAnyone = [&](Point p) {
    for (const auto& [id, cells] : model) {
      for (const Point cell : cells) {
        if (cell == p) {
          return true;
        }
      }
    }
    return false;
  };
  int claims = 0;
  int advances = 0;
  for (int step = 0; step < 4000; ++step) {
    const RobotId robot = R(static_cast<std::uint32_t>(1 + rng.below(4)));
    const Point cell = P(static_cast<int>(rng.below(6)) - 1, static_cast<int>(rng.below(5)) - 1);
    switch (rng.below(6)) {
      case 0: {
        const bool fits = size.contains(cell) && !heldByAnyone(cell) && !model.contains(robot);
        EXPECT_EQ(occupancy.place(robot, cell) == PlaceStatus::Ok, fits);
        if (fits) {
          model[robot] = {cell};
        }
        break;
      }
      case 1:
      case 2: {
        const bool fits = model.contains(robot) && size.contains(cell) && !heldByAnyone(cell);
        EXPECT_EQ(occupancy.claim(robot, cell) == MoveStatus::Ok, fits);
        if (fits) {
          model[robot].push_back(cell);
          ++claims;
        }
        break;
      }
      case 3: {
        const bool can = model.contains(robot) && model[robot].size() > 1;
        EXPECT_EQ(occupancy.advance(robot), can);
        if (can) {
          model[robot].erase(model[robot].begin());
          ++advances;
        }
        break;
      }
      case 4: {
        const std::size_t keep = rng.below(3);
        if (model.contains(robot)) {
          auto& cells = model[robot];
          const std::size_t before = cells.size();
          if (cells.size() > 1 + keep) {
            cells.resize(1 + keep);
          }
          EXPECT_EQ(occupancy.releaseClaims(robot, keep), before - cells.size());
        } else {
          EXPECT_EQ(occupancy.releaseClaims(robot, keep), 0U);
        }
        break;
      }
      default: {
        EXPECT_EQ(occupancy.remove(robot), model.contains(robot));
        model.erase(robot);
        break;
      }
    }

    std::size_t totalHeld = 0;
    for (const auto& [id, cells] : model) {
      ASSERT_EQ(occupancy.holdsOf(id), cells);
      ASSERT_EQ(occupancy.positionOf(id), cells.front());
      for (const Point held : cells) {
        ASSERT_EQ(occupancy.occupantAt(held), id);
      }
      totalHeld += cells.size();
    }
    ASSERT_EQ(occupancy.robotCount(), model.size());
    ASSERT_EQ(occupancy.occupiedCells().size(), totalHeld);
    std::size_t occupiedCells = 0;
    for (std::size_t i = 0; i < size.cellCount(); ++i) {
      occupiedCells += occupancy.isOccupied(size.pointAt(i)) ? 1U : 0U;
    }
    ASSERT_EQ(occupiedCells, totalHeld);  // one holder per cell, nothing left behind
  }
  EXPECT_GT(claims, 200);
  EXPECT_GT(advances, 100);
}

}  // namespace
