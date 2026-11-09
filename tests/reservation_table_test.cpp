#include "warehousesim/traffic/reservation_table.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::traffic::Conflict;
using warehousesim::traffic::ConflictKind;
using warehousesim::traffic::Occupy;
using warehousesim::traffic::Request;
using warehousesim::traffic::ReservationTable;
using warehousesim::traffic::Step;
using warehousesim::traffic::StepRange;
using warehousesim::traffic::Traverse;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// A cell's worth of time: [from, to).
Occupy occupy(Point cell, Step from, Step to) { return Occupy{cell, StepRange{from, to}}; }

// ---- steps ------------------------------------------------------------------------------------

TEST(StepRangeTest, IsHalfOpenSoTouchingRangesDoNotOverlap) {
  const StepRange range{3, 6};
  EXPECT_FALSE(range.empty());
  EXPECT_TRUE(range.contains(3));
  EXPECT_TRUE(range.contains(5));
  EXPECT_FALSE(range.contains(6));
  EXPECT_FALSE(range.contains(2));
  EXPECT_TRUE(range.overlaps({5, 9}));
  EXPECT_TRUE(range.overlaps({0, 4}));
  EXPECT_TRUE(range.overlaps({4, 5}));
  EXPECT_FALSE(range.overlaps({6, 9}));
  EXPECT_FALSE(range.overlaps({0, 3}));
  EXPECT_TRUE((StepRange{4, 4}).empty());
  EXPECT_TRUE((StepRange{5, 2}).empty());
}

// ---- reserving and looking up -----------------------------------------------------------------

TEST(ReservationTableTest, AFreeCellCanBeReservedAndAnyoneCanSeeWhoHasIt) {
  ReservationTable table(layout({"....", "...."}));
  EXPECT_FALSE(table.reserve(R(1), occupy(P(2, 1), 3, 6)).has_value());

  EXPECT_FALSE(table.holder(P(2, 1), 2).has_value());
  EXPECT_EQ(table.holder(P(2, 1), 3), R(1));
  EXPECT_EQ(table.holder(P(2, 1), 5), R(1));
  EXPECT_FALSE(table.holder(P(2, 1), 6).has_value());
  EXPECT_FALSE(table.holder(P(1, 1), 4).has_value());
  EXPECT_FALSE(table.holder(P(9, 9), 4).has_value());  // off the grid: nobody
  EXPECT_EQ(table.occupancyCount(), 1U);
  EXPECT_EQ(table.reservationsOf(R(1)),
            (std::vector<warehousesim::traffic::Reservation>{{R(1), P(2, 1), {3, 6}}}));
  EXPECT_TRUE(table.reservationsOf(R(2)).empty());
}

TEST(ReservationTableTest, OccupiedAtListsEveryHeldCellInRowMajorOrder) {
  ReservationTable table(layout({"....", "...."}));
  ASSERT_FALSE(table.reserve(R(2), occupy(P(0, 1), 0, 5)).has_value());
  ASSERT_FALSE(table.reserve(R(1), occupy(P(3, 0), 2, 4)).has_value());
  ASSERT_FALSE(table.reserve(R(3), occupy(P(1, 0), 0, 9)).has_value());
  using Held = std::vector<std::pair<Point, RobotId>>;
  EXPECT_EQ(table.occupiedAt(3), (Held{{P(1, 0), R(3)}, {P(3, 0), R(1)}, {P(0, 1), R(2)}}));
  EXPECT_EQ(table.occupiedAt(6), (Held{{P(1, 0), R(3)}}));
  EXPECT_TRUE(table.occupiedAt(20).empty());
}

// ---- two robots entering the same cell -------------------------------------------------------

TEST(ReservationTableTest, TwoRobotsCannotHaveTheSameCellInTheSameStep) {
  ReservationTable table(layout({"..."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 3, 5)).has_value());

  const auto clash = table.reserve(R(2), occupy(P(1, 0), 4, 7));
  ASSERT_TRUE(clash.has_value());
  EXPECT_EQ(clash->kind, ConflictKind::Vertex);
  EXPECT_EQ(clash->cell, P(1, 0));
  EXPECT_EQ(clash->step, 4U);  // the first step at which both would be there
  EXPECT_EQ(clash->other, R(1));
  EXPECT_FALSE(table.holder(P(1, 0), 6).has_value());  // the refused request left no trace

  // Arriving the step after the other has gone, or leaving the step before it comes, is fine.
  EXPECT_FALSE(table.reserve(R(2), occupy(P(1, 0), 5, 7)).has_value());
  EXPECT_FALSE(table.reserve(R(3), occupy(P(1, 0), 0, 3)).has_value());
}

TEST(ReservationTableTest, SimultaneousArrivalsAreSettledByWhoReservedFirst) {
  // Both robots want the same cell for the same steps; the table has no opinion of its own: the
  // order of reservation is the priority.
  for (const bool firstIsOne : {true, false}) {
    ReservationTable table(layout({"..."}));
    const RobotId first = firstIsOne ? R(1) : R(2);
    const RobotId second = firstIsOne ? R(2) : R(1);
    ASSERT_FALSE(table.reserve(first, occupy(P(1, 0), 5, 6)).has_value());
    const auto clash = table.reserve(second, occupy(P(1, 0), 5, 6));
    ASSERT_TRUE(clash.has_value());
    EXPECT_EQ(clash->kind, ConflictKind::Vertex);
    EXPECT_EQ(clash->other, first);
    EXPECT_EQ(table.holder(P(1, 0), 5), first);
  }
}

// ---- head-on swaps -----------------------------------------------------------------------------

TEST(ReservationTableTest, TwoRobotsCannotSwapPlacesAlongTheSameEdge) {
  ReservationTable table(layout({"...."}));
  ASSERT_FALSE(table.reserve(R(1), Traverse{P(1, 0), P(2, 0), 4}).has_value());

  const auto swap = table.reserve(R(2), Traverse{P(2, 0), P(1, 0), 4});
  ASSERT_TRUE(swap.has_value());
  EXPECT_EQ(swap->kind, ConflictKind::Swap);
  EXPECT_EQ(swap->cell, P(1, 0));  // the cell the second robot was moving into
  EXPECT_EQ(swap->step, 4U);
  EXPECT_EQ(swap->other, R(1));
  EXPECT_EQ(table.traversalCount(), 1U);

  EXPECT_FALSE(table.reserve(R(2), Traverse{P(2, 0), P(1, 0), 5}).has_value());  // a step later
  EXPECT_FALSE(
      table.reserve(R(3), Traverse{P(1, 0), P(2, 0), 4}).has_value());  // same way: no swap
  EXPECT_FALSE(table.reserve(R(3), Traverse{P(2, 0), P(3, 0), 4}).has_value());  // another edge
}

TEST(ReservationTableTest, ASwapIsToldApartFromAPlainClashByItsEdgesBeingChecked) {
  // The same two robots exchanging cells also clash on the cells; reserving the edges first is what
  // makes the report say "swap".
  ReservationTable table(layout({"...."}));
  ASSERT_FALSE(
      table
          .reserveAll(R(1), std::vector<Request>{Traverse{P(1, 0), P(2, 0), 3},
                                                 occupy(P(1, 0), 3, 4), occupy(P(2, 0), 3, 4)})
          .has_value());
  const auto edgesFirst =
      table.checkAll(R(2), std::vector<Request>{Traverse{P(2, 0), P(1, 0), 3},
                                                occupy(P(2, 0), 3, 4), occupy(P(1, 0), 3, 4)});
  ASSERT_TRUE(edgesFirst.has_value());
  EXPECT_EQ(edgesFirst->kind, ConflictKind::Swap);
  const auto cellsFirst = table.checkAll(
      R(2), std::vector<Request>{occupy(P(2, 0), 3, 4), Traverse{P(2, 0), P(1, 0), 3}});
  ASSERT_TRUE(cellsFirst.has_value());
  EXPECT_EQ(cellsFirst->kind, ConflictKind::Vertex);
}

// ---- intersections ----------------------------------------------------------------------------

//   . N .
//   W + E       a crossing: one robot goes west to east, the other north to south
//   . S .
TEST(ReservationTableTest, PathsCrossingAtAnIntersectionClashOnlyIfTheyAreThereTogether) {
  ReservationTable table(layout({"...", "...", "..."}));
  // West to east: (0,1) at step 2, the centre at 3, (2,1) at 4.
  ASSERT_FALSE(
      table
          .reserveAll(R(1), std::vector<Request>{occupy(P(0, 1), 2, 4), occupy(P(1, 1), 3, 5),
                                                 occupy(P(2, 1), 4, 6)})
          .has_value());
  // North to south, reaching the centre at the same time: they would collide there.
  const std::vector<Request> together = {occupy(P(1, 0), 2, 4), occupy(P(1, 1), 3, 5),
                                         occupy(P(1, 2), 4, 6)};
  const auto clash = table.checkAll(R(2), together);
  ASSERT_TRUE(clash.has_value());
  EXPECT_EQ(clash->kind, ConflictKind::Vertex);
  EXPECT_EQ(clash->cell, P(1, 1));  // the intersection itself
  EXPECT_EQ(clash->step, 3U);
  EXPECT_EQ(clash->other, R(1));

  // Two steps later the first robot is gone and the second can cross.
  const std::vector<Request> later = {occupy(P(1, 0), 4, 6), occupy(P(1, 1), 5, 7),
                                      occupy(P(1, 2), 6, 8)};
  EXPECT_FALSE(table.checkAll(R(2), later).has_value());
  // And a robot that is through the intersection before the other arrives never meets it.
  const std::vector<Request> earlier = {occupy(P(1, 0), 0, 2), occupy(P(1, 1), 1, 3),
                                        occupy(P(1, 2), 2, 4)};
  EXPECT_FALSE(table.checkAll(R(2), earlier).has_value());
}

// ---- conflicting reservations -------------------------------------------------------------------

TEST(ReservationTableTest, ARequestIsAllOrNothing) {
  ReservationTable table(layout({"..."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(2, 0), 5, 8)).has_value());
  const std::vector<Request> plan = {occupy(P(0, 0), 3, 4), occupy(P(1, 0), 4, 5),
                                     occupy(P(2, 0), 5, 6)};  // the last one clashes
  const auto clash = table.reserveAll(R(2), plan);
  ASSERT_TRUE(clash.has_value());
  EXPECT_EQ(clash->cell, P(2, 0));
  EXPECT_TRUE(table.reservationsOf(R(2)).empty());  // not even the first two stayed
  EXPECT_EQ(table.occupancyCount(), 1U);
  EXPECT_FALSE(table.holder(P(0, 0), 3).has_value());
}

TEST(ReservationTableTest, TheFirstConflictInTheOrderGivenIsTheOneReported) {
  ReservationTable table(layout({"...."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 0, 9)).has_value());
  ASSERT_FALSE(table.reserve(R(1), occupy(P(3, 0), 0, 9)).has_value());
  const auto a =
      table.checkAll(R(2), std::vector<Request>{occupy(P(3, 0), 2, 3), occupy(P(1, 0), 2, 3)});
  const auto b =
      table.checkAll(R(2), std::vector<Request>{occupy(P(1, 0), 2, 3), occupy(P(3, 0), 2, 3)});
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_EQ(a->cell, P(3, 0));
  EXPECT_EQ(b->cell, P(1, 0));
}

TEST(ReservationTableTest, TheSoonestConflictIsReportedNotTheFirstInTheOrderGiven) {
  ReservationTable table(layout({"...."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 5, 6)).has_value());
  ASSERT_FALSE(table.reserve(R(1), occupy(P(3, 0), 2, 3)).has_value());
  // A long stay on the first cell is listed ahead of the short one, but clashes later.
  const std::vector<Request> asked = {occupy(P(1, 0), 0, 10), occupy(P(3, 0), 0, 10)};
  EXPECT_EQ(table.checkAll(R(2), asked)->cell, P(1, 0));
  const auto soonest = table.earliestConflict(R(2), asked);
  ASSERT_TRUE(soonest.has_value());
  EXPECT_EQ(soonest->cell, P(3, 0));
  EXPECT_EQ(soonest->step, 2U);
  EXPECT_FALSE(
      table.earliestConflict(R(2), std::vector<Request>{occupy(P(0, 0), 0, 10)}).has_value());
  EXPECT_FALSE(table.earliestConflict(R(2), {}).has_value());
}

TEST(ReservationTableTest, AtTheSameStepAWallBeatsASwapAndASwapBeatsAPlainClash) {
  ReservationTable table(layout({".#.."}));
  ASSERT_FALSE(
      table
          .reserveAll(R(1), std::vector<Request>{Traverse{P(2, 0), P(3, 0), 3},
                                                 occupy(P(2, 0), 3, 4), occupy(P(3, 0), 3, 4)})
          .has_value());
  // Robot 2 moves back the other way, on the cells robot 1 holds: it clashes on the cell (listed
  // first) and on the edge at step 3. The report says swap.
  const std::vector<Request> head_on = {occupy(P(3, 0), 3, 4), Traverse{P(3, 0), P(2, 0), 3}};
  EXPECT_EQ(table.checkAll(R(2), head_on)->kind, ConflictKind::Vertex);
  EXPECT_EQ(table.earliestConflict(R(2), head_on)->kind, ConflictKind::Swap);
  // A wall at that very step is the better answer still.
  const std::vector<Request> wall = {occupy(P(3, 0), 3, 4), Traverse{P(3, 0), P(2, 0), 3},
                                     occupy(P(1, 0), 3, 4)};
  EXPECT_EQ(table.earliestConflict(R(2), wall)->kind, ConflictKind::Blocked);
  // But a clash at an earlier step still wins over any of them.
  const std::vector<Request> sooner = {occupy(P(1, 0), 9, 10), occupy(P(2, 0), 2, 4)};
  EXPECT_EQ(table.earliestConflict(R(2), sooner)->step, 3U);
  EXPECT_EQ(table.earliestConflict(R(2), sooner)->kind, ConflictKind::Vertex);
}

TEST(ReservationTableTest, WhenSeveralReservationsClashTheEarliestIsReported) {
  ReservationTable table(layout({"..."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 8, 10)).has_value());  // stored first, but later
  ASSERT_FALSE(table.reserve(R(2), occupy(P(1, 0), 2, 4)).has_value());
  const auto clash = table.check(R(3), occupy(P(1, 0), 0, 20));
  ASSERT_TRUE(clash.has_value());
  EXPECT_EQ(clash->step, 2U);
  EXPECT_EQ(clash->other, R(2));
}

TEST(ReservationTableTest, EqualClashesAreBrokenByTheLowestRobotIdWhateverTheStorageOrder) {
  for (const bool lowFirst : {true, false}) {
    ReservationTable table(layout({"...."}));
    const Traverse east{P(2, 0), P(1, 0), 4};
    if (lowFirst) {
      ASSERT_FALSE(table.reserve(R(2), east).has_value());
      ASSERT_FALSE(table.reserve(R(3), east).has_value());
    } else {
      ASSERT_FALSE(table.reserve(R(3), east).has_value());
      ASSERT_FALSE(table.reserve(R(2), east).has_value());
    }
    const auto clash = table.check(R(5), Traverse{P(1, 0), P(2, 0), 4});
    ASSERT_TRUE(clash.has_value());
    EXPECT_EQ(clash->other, R(2));
  }
}

TEST(ReservationTableTest, ARobotNeverClashesWithItselfAndItsOwnReservationsMerge) {
  ReservationTable table(layout({"..."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 2, 4)).has_value());
  EXPECT_FALSE(table.reserve(R(1), occupy(P(1, 0), 3, 6)).has_value());    // overlaps its own
  EXPECT_FALSE(table.reserve(R(1), occupy(P(1, 0), 6, 8)).has_value());    // touches its own
  EXPECT_FALSE(table.reserve(R(1), occupy(P(1, 0), 20, 21)).has_value());  // a separate one
  EXPECT_EQ(table.reservationsOf(R(1)), (std::vector<warehousesim::traffic::Reservation>{
                                            {R(1), P(1, 0), {2, 8}}, {R(1), P(1, 0), {20, 21}}}));
  ASSERT_FALSE(table.reserve(R(1), Traverse{P(1, 0), P(2, 0), 3}).has_value());
  EXPECT_FALSE(table.reserve(R(1), Traverse{P(2, 0), P(1, 0), 3}).has_value());  // even back again
  EXPECT_FALSE(table.reserve(R(1), Traverse{P(1, 0), P(2, 0), 3}).has_value());  // and repeated
  EXPECT_EQ(table.traversalCount(), 2U);
}

// ---- blocked cells ---------------------------------------------------------------------------

TEST(ReservationTableTest, WallsShelvesAndTheEdgeOfTheGridCannotBeReserved) {
  ReservationTable table(layout({".#S.", "...."}));
  for (const Point blocked : {P(1, 0), P(2, 0), P(-1, 0), P(4, 0), P(0, 2), P(0, -1)}) {
    const auto clash = table.reserve(R(1), occupy(blocked, 4, 6));
    ASSERT_TRUE(clash.has_value()) << blocked.x << "," << blocked.y;
    EXPECT_EQ(clash->kind, ConflictKind::Blocked);
    EXPECT_EQ(clash->cell, blocked);
    EXPECT_EQ(clash->step, 4U);
    EXPECT_FALSE(clash->other.has_value());  // nobody to blame: it is just not floor
  }
  EXPECT_EQ(table.occupancyCount(), 0U);
  EXPECT_FALSE(table.reserve(R(1), occupy(P(0, 0), 4, 6)).has_value());  // while floor is fine
}

TEST(ReservationTableTest, MovingIntoABlockedCellIsRefusedWhoeverElseIsAbout) {
  ReservationTable table(layout({".#.", "..."}));
  const auto wall = table.reserve(R(1), Traverse{P(0, 0), P(1, 0), 2});
  ASSERT_TRUE(wall.has_value());
  EXPECT_EQ(wall->kind, ConflictKind::Blocked);
  EXPECT_EQ(wall->cell, P(1, 0));
  const auto edge = table.reserve(R(1), Traverse{P(0, 0), P(-1, 0), 2});
  ASSERT_TRUE(edge.has_value());
  EXPECT_EQ(edge->kind, ConflictKind::Blocked);
  EXPECT_EQ(table.traversalCount(), 0U);
}

TEST(ReservationTableTest, ARobotStandingStillBlocksTheCellForEveryoneElse) {
  ReservationTable table(layout({"...."}));
  ASSERT_FALSE(table.reserve(R(4), occupy(P(2, 0), 10, 58)).has_value());  // parked for the horizon
  for (const Step step : {10U, 30U, 57U}) {
    const auto clash = table.check(R(1), occupy(P(2, 0), step, step + 1));
    ASSERT_TRUE(clash.has_value()) << step;
    EXPECT_EQ(clash->other, R(4));
  }
  EXPECT_FALSE(table.check(R(1), occupy(P(2, 0), 58, 60)).has_value());
}

// ---- housekeeping ------------------------------------------------------------------------------

TEST(ReservationTableTest, ReleaseGivesEverythingBackAndClearForgetsAll) {
  ReservationTable table(layout({"..."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(0, 0), 0, 5)).has_value());
  ASSERT_FALSE(table.reserve(R(1), Traverse{P(0, 0), P(1, 0), 1}).has_value());
  ASSERT_FALSE(table.reserve(R(2), occupy(P(1, 0), 0, 5)).has_value());
  table.release(R(1));
  EXPECT_TRUE(table.reservationsOf(R(1)).empty());
  EXPECT_EQ(table.traversalCount(), 0U);
  EXPECT_FALSE(table.reserve(R(2), Traverse{P(1, 0), P(0, 0), 1}).has_value());  // no swap left
  EXPECT_FALSE(table.reserve(R(3), occupy(P(0, 0), 0, 5)).has_value());  // cell is free again
  EXPECT_EQ(table.holder(P(1, 0), 2), R(2));                             // others untouched

  table.clear();
  EXPECT_EQ(table.occupancyCount(), 0U);
  EXPECT_EQ(table.traversalCount(), 0U);
  EXPECT_FALSE(table.holder(P(1, 0), 2).has_value());
  EXPECT_FALSE(table.reserve(R(1), occupy(P(1, 0), 0, 5)).has_value());  // and it still works
}

TEST(ReservationTableTest, ReleasingAndReservingAgainDoesNotInflateTheCounts) {
  ReservationTable table(layout({"..."}));
  for (int round = 0; round < 5; ++round) {
    ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 0, 3)).has_value());
    table.release(R(1));
  }
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 0, 3)).has_value());
  EXPECT_EQ(table.occupancyCount(), 1U);
  EXPECT_EQ(table.occupiedAt(1).size(), 1U);
}

TEST(ReservationTableTest, BadRequestsAreProgrammingErrors) {
  ReservationTable table(layout({"..."}));
  EXPECT_THROW((void)table.reserve(RobotId{}, occupy(P(0, 0), 0, 1)), std::invalid_argument);
  EXPECT_THROW((void)table.reserve(R(1), occupy(P(0, 0), 3, 3)), std::invalid_argument);
  EXPECT_THROW((void)table.reserve(R(1), occupy(P(0, 0), 5, 2)), std::invalid_argument);
  EXPECT_THROW((void)table.reserve(R(1), Traverse{P(0, 0), P(2, 0), 1}), std::invalid_argument);
  EXPECT_THROW((void)table.reserve(R(1), Traverse{P(0, 0), P(0, 0), 1}), std::invalid_argument);
  EXPECT_EQ(table.occupancyCount(), 0U);
}

// ---- against a brute-force reference ------------------------------------------------------------

// Everything the table promises, decided the slow obvious way.
TEST(ReservationTablePropertyTest, RandomRequestsAreAcceptedOrRefusedExactlyAsAReferenceSays) {
  const Warehouse warehouse = layout({"....#", ".S...", "....."});
  ReservationTable table(warehouse);
  std::map<std::pair<Point, Step>, RobotId> cells;
  std::set<std::tuple<Point, Point, Step, RobotId>> edges;

  Rng rng(0xBEEF);
  int accepted = 0;
  int vertex = 0;
  int swaps = 0;
  int blocked = 0;
  const Point directions[4] = {P(1, 0), P(-1, 0), P(0, 1), P(0, -1)};
  for (int i = 0; i < 6000; ++i) {
    const RobotId robot = R(static_cast<std::uint32_t>(1 + rng.below(4)));
    std::optional<Conflict> expected;
    Request request;
    if (rng.below(3) != 0) {
      const Point cell = P(static_cast<int>(rng.below(7)) - 1, static_cast<int>(rng.below(5)) - 1);
      const Step from = rng.below(12);
      const Step to = from + 1 + rng.below(4);
      request = occupy(cell, from, to);
      if (!warehouse.isWalkable(cell)) {
        expected = Conflict{ConflictKind::Blocked, cell, from, std::nullopt};
      } else {
        for (Step s = from; s < to && !expected; ++s) {
          const auto held = cells.find({cell, s});
          if (held != cells.end() && held->second != robot) {
            expected = Conflict{ConflictKind::Vertex, cell, s, held->second};
          }
        }
      }
    } else {
      const Point from = P(static_cast<int>(rng.below(5)), static_cast<int>(rng.below(3)));
      const Point to = from + directions[rng.below(4)];
      const Step step = rng.below(12);
      request = Traverse{from, to, step};
      if (!warehouse.isWalkable(to)) {
        expected = Conflict{ConflictKind::Blocked, to, step, std::nullopt};
      } else {
        for (const auto& [a, b, s, owner] : edges) {
          if (a == to && b == from && s == step && owner != robot &&
              (!expected || owner < *expected->other)) {
            expected = Conflict{ConflictKind::Swap, to, step, owner};
          }
        }
      }
    }

    const std::optional<Conflict> actual = table.reserve(robot, request);
    ASSERT_EQ(actual.has_value(), expected.has_value()) << "request " << i;
    if (expected) {
      ASSERT_TRUE(*actual == *expected) << "request " << i;
      vertex += expected->kind == ConflictKind::Vertex;
      swaps += expected->kind == ConflictKind::Swap;
      blocked += expected->kind == ConflictKind::Blocked;
      continue;
    }
    ++accepted;
    if (const auto* o = std::get_if<Occupy>(&request)) {
      for (Step s = o->steps.from; s < o->steps.to; ++s) {
        cells[{o->cell, s}] = robot;
      }
    } else {
      const auto& t = std::get<Traverse>(request);
      edges.emplace(t.from, t.to, t.step, robot);
    }
  }

  // The table agrees with the model about who has what, and no cell has two holders.
  for (const auto& [key, owner] : cells) {
    ASSERT_EQ(table.holder(key.first, key.second), owner);
  }
  std::size_t reservedCellSteps = 0;
  for (std::uint32_t id = 1; id <= 4; ++id) {
    for (const auto& reservation : table.reservationsOf(R(id))) {
      reservedCellSteps += reservation.steps.to - reservation.steps.from;
    }
  }
  EXPECT_EQ(reservedCellSteps, cells.size());
  EXPECT_EQ(table.traversalCount(), edges.size());
  EXPECT_GT(accepted, 500);  // the run really exercised every kind of answer
  EXPECT_GT(vertex, 500);
  EXPECT_GT(swaps, 50);
  EXPECT_GT(blocked, 500);
}

// ---- the yes-or-no questions a search asks by the thousand --------------------------------------

TEST(ReservationTableFastPathTest, IsFreeAndIsTraversableAgreeWithCheckOnRandomTables) {
  const Warehouse warehouse = layout({"....#", ".S...", "....."});
  Rng rng(0xFA57);
  const Point directions[4] = {P(1, 0), P(-1, 0), P(0, 1), P(0, -1)};
  int free = 0;
  int taken = 0;
  int passable = 0;
  int refused = 0;
  for (int round = 0; round < 40; ++round) {
    ReservationTable table(warehouse);
    for (int i = 0; i < 70; ++i) {
      const RobotId robot = R(static_cast<std::uint32_t>(1 + rng.below(4)));
      if (rng.below(3) != 0) {
        const Step from = rng.below(12);
        (void)table.reserve(
            robot, occupy(P(static_cast<int>(rng.below(5)), static_cast<int>(rng.below(3))), from,
                          from + 1 + rng.below(3)));
      } else {
        const Point from = P(static_cast<int>(rng.below(5)), static_cast<int>(rng.below(3)));
        (void)table.reserve(robot, Traverse{from, from + directions[rng.below(4)], rng.below(12)});
      }
    }
    for (int q = 0; q < 300; ++q) {
      const RobotId robot = R(static_cast<std::uint32_t>(1 + rng.below(5)));
      const Step step = rng.below(14);
      const Point cell = P(static_cast<int>(rng.below(7)) - 1, static_cast<int>(rng.below(5)) - 1);
      const bool isFree = table.isFree(robot, cell, step);
      ASSERT_EQ(isFree, !table.check(robot, occupy(cell, step, step + 1)).has_value())
          << cell.x << "," << cell.y << " at " << step;
      (isFree ? free : taken) += 1;

      const Point from = P(static_cast<int>(rng.below(7)) - 1, static_cast<int>(rng.below(5)) - 1);
      const Point to = from + directions[rng.below(4)];
      const bool canGo = table.isTraversable(robot, from, to, step);
      ASSERT_EQ(canGo, !table.check(robot, Traverse{from, to, step}).has_value())
          << from.x << "," << from.y << " to " << to.x << "," << to.y << " at " << step;
      (canGo ? passable : refused) += 1;
    }
  }
  EXPECT_GT(free, 1000);  // every kind of answer was given
  EXPECT_GT(taken, 1000);
  EXPECT_GT(passable, 500);
  EXPECT_GT(refused, 300);
}

TEST(ReservationTableFastPathTest, ARobotNeverBlocksItself) {
  ReservationTable table(layout({"..."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 3, 6)).has_value());
  ASSERT_FALSE(table.reserve(R(1), Traverse{P(0, 0), P(1, 0), 2}).has_value());
  EXPECT_TRUE(table.isFree(R(1), P(1, 0), 4));
  EXPECT_FALSE(table.isFree(R(2), P(1, 0), 4));
  EXPECT_TRUE(table.isFree(R(2), P(1, 0), 6)) << "the range is half open";
  EXPECT_TRUE(table.isTraversable(R(1), P(1, 0), P(0, 0), 2));
  EXPECT_FALSE(table.isTraversable(R(2), P(1, 0), P(0, 0), 2)) << "head on with robot 1's move";
  EXPECT_TRUE(table.isTraversable(R(2), P(1, 0), P(0, 0), 3));
  EXPECT_TRUE(table.isTraversable(R(2), P(1, 0), P(2, 0), 2)) << "the same way is not head on";
}

TEST(ReservationTableFastPathTest, AWallAndTheEdgeOfTheMapAreNeverFree) {
  ReservationTable table(layout({".#."}));
  EXPECT_FALSE(table.isFree(R(1), P(1, 0), 0));
  EXPECT_FALSE(table.isFree(R(1), P(-1, 0), 0));
  EXPECT_FALSE(table.isFree(R(1), P(0, 5), 0));
  EXPECT_TRUE(table.isFree(R(1), P(0, 0), 0));
  EXPECT_FALSE(table.isTraversable(R(1), P(0, 0), P(1, 0), 0));
  EXPECT_FALSE(table.isTraversable(R(1), P(2, 0), P(3, 0), 0));
  EXPECT_FALSE(table.isTraversable(R(1), P(-1, 0), P(0, 0), 0)) << "it cannot start off the map";
}

TEST(ReservationTableFastPathTest, ATraversalFromOffTheMapIsRefusedAsBlockedNotRecorded) {
  ReservationTable table(layout({"..."}));
  const auto clash = table.reserve(R(1), Traverse{P(-1, 0), P(0, 0), 2});
  ASSERT_TRUE(clash.has_value());
  EXPECT_EQ(clash->kind, ConflictKind::Blocked);
  EXPECT_EQ(table.traversalCount(), 0U);
}

// ---- giving back what a robot holds -------------------------------------------------------------

TEST(ReservationTableReleaseTest, ReleaseGivesBackEverythingARobotHoldsWhereverItIsAndNothingElse) {
  ReservationTable table(layout({"....", "...."}));
  ASSERT_FALSE(table.reserve(R(1), occupy(P(0, 0), 2, 4)).has_value());
  ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), 3, 5)).has_value());
  ASSERT_FALSE(table.reserve(R(1), occupy(P(2, 1), 9, 10)).has_value());
  ASSERT_FALSE(table.reserve(R(1), Traverse{P(0, 0), P(1, 0), 2}).has_value());
  ASSERT_FALSE(table.reserve(R(1), Traverse{P(1, 0), P(2, 0), 4}).has_value());
  ASSERT_FALSE(table.reserve(R(1), Traverse{P(2, 1), P(3, 1), 9}).has_value());
  ASSERT_FALSE(table.reserve(R(2), occupy(P(1, 0), 0, 2)).has_value());
  ASSERT_FALSE(table.reserve(R(2), Traverse{P(3, 0), P(2, 0), 7}).has_value());
  ASSERT_EQ(table.occupancyCount(), 4U);
  ASSERT_EQ(table.traversalCount(), 4U);

  table.release(R(1));
  EXPECT_EQ(table.occupancyCount(), 1U);
  EXPECT_EQ(table.traversalCount(), 1U);
  EXPECT_TRUE(table.reservationsOf(R(1)).empty());
  EXPECT_EQ(table.holder(P(1, 0), 1), R(2));
  EXPECT_FALSE(table.holder(P(0, 0), 3).has_value());

  table.release(R(1));  // nothing left to give back
  table.release(R(9));  // never held anything
  EXPECT_EQ(table.occupancyCount(), 1U);
  EXPECT_EQ(table.traversalCount(), 1U);

  // The cells and edges are free for others again, and robot 2's move still stands in the way.
  EXPECT_FALSE(table.reserve(R(3), occupy(P(0, 0), 2, 4)).has_value());
  EXPECT_TRUE(table.reserve(R(3), Traverse{P(2, 0), P(3, 0), 7}).has_value());
}

TEST(ReservationTableReleaseTest, ARobotCanReserveAndReleaseOverAndOverWithoutTheTableGrowing) {
  ReservationTable table(layout({"......", "......"}));
  ASSERT_FALSE(table.reserve(R(2), occupy(P(5, 1), 0, 100)).has_value());
  for (int i = 0; i < 300; ++i) {
    const Step at = static_cast<Step>(i % 7);
    ASSERT_FALSE(table.reserve(R(1), occupy(P(0, 0), at, at + 3)).has_value());
    ASSERT_FALSE(table.reserve(R(1), occupy(P(1, 0), at, at + 3)).has_value());
    ASSERT_FALSE(table.reserve(R(1), Traverse{P(0, 0), P(1, 0), at}).has_value());
    ASSERT_EQ(table.occupancyCount(), 3U);
    ASSERT_EQ(table.traversalCount(), 1U);
    table.release(R(1));
    ASSERT_EQ(table.occupancyCount(), 1U);
    ASSERT_EQ(table.traversalCount(), 0U);
  }
  table.clear();
  EXPECT_EQ(table.occupancyCount(), 0U);
  ASSERT_FALSE(table.reserve(R(1), occupy(P(0, 0), 0, 2)).has_value());
  table.release(R(1));
  EXPECT_EQ(table.occupancyCount(), 0U) << "release after clear still finds what was reserved";
}

TEST(ReservationTablePropertyTest, TheSameRequestsInTheSameOrderGiveTheSameTableEveryTime) {
  const auto run = [] {
    ReservationTable table(layout({"......", "......", "......"}));
    Rng rng(77);
    std::vector<std::optional<Conflict>> answers;
    for (int i = 0; i < 800; ++i) {
      const RobotId robot = R(static_cast<std::uint32_t>(1 + rng.below(5)));
      const Point cell = P(static_cast<int>(rng.below(6)), static_cast<int>(rng.below(3)));
      const Step from = rng.below(20);
      answers.push_back(table.reserve(robot, occupy(cell, from, from + 1 + rng.below(5))));
    }
    return std::pair{answers, table.occupiedAt(7)};
  };
  const auto first = run();
  const auto second = run();
  EXPECT_TRUE(first.first == second.first);
  EXPECT_TRUE(first.second == second.second);
}

}  // namespace
