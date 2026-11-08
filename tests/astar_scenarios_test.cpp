#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/testing/map_generators.hpp"
#include "warehousesim/testing/reference_search.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Direction;
using warehousesim::core::DirectionSet;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::planning::AStarOptions;
using warehousesim::planning::AStarPlanner;
using warehousesim::planning::BlockedSet;
using warehousesim::planning::Path;
using warehousesim::planning::SearchResult;
using warehousesim::planning::SearchStatus;
using warehousesim::testing::kUnreachable;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// Plans and checks the answer against the independent reference: same reachability, same length,
// and a legal route. Returns the status for further assertions.
SearchResult planAndVerify(AStarPlanner& planner, const Warehouse& w, Point start, Point goal,
                           const BlockedSet& blocked = {}, Path* pathOut = nullptr) {
  Path path;
  const SearchResult result = planner.search(w, start, goal, blocked, path);
  const auto expected = warehousesim::testing::shortestDistance(w, start, goal, blocked,
                                                                planner.options().directions);
  EXPECT_EQ(result.status == SearchStatus::Found, expected.has_value());
  if (result.status == SearchStatus::Found && expected) {
    EXPECT_EQ(result.cost, *expected);
    EXPECT_EQ(path.size(), static_cast<std::size_t>(*expected));
    EXPECT_EQ(warehousesim::testing::pathProblem(w, start, goal, path, blocked,
                                                 planner.options().directions)
                  .value_or("ok"),
              "ok");
  } else {
    EXPECT_TRUE(path.empty());
  }
  if (pathOut != nullptr) {
    *pathOut = path;
  }
  return result;
}

// ---- open grids ----------------------------------------------------------------------

TEST(AStarOpenGridTest, CostIsTheManhattanDistanceForEveryPairOnASmallGrid) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(6, 5);
  AStarPlanner planner;
  for (int sy = 0; sy < 5; ++sy) {
    for (int sx = 0; sx < 6; ++sx) {
      for (int gy = 0; gy < 5; ++gy) {
        for (int gx = 0; gx < 6; ++gx) {
          const SearchResult r = planAndVerify(planner, w, P(sx, sy), P(gx, gy));
          ASSERT_TRUE(r.status == SearchStatus::Found);
          EXPECT_EQ(r.cost, std::abs(sx - gx) + std::abs(sy - gy));
        }
      }
    }
  }
}

TEST(AStarOpenGridTest, StraightLinesAreExpandedWithoutDetours) {
  // With a perfect heuristic on an open grid A* walks straight to the goal: it expands exactly the
  // cells on its path (plus none that are off it) when the goal is on the start's row.
  const Warehouse w = warehousesim::testing::makeOpenGrid(50, 1);
  AStarPlanner planner;
  Path path;
  const SearchResult r = planner.search(w, P(0, 0), P(49, 0), {}, path);
  ASSERT_TRUE(r.status == SearchStatus::Found);
  EXPECT_EQ(r.cost, 49);
  EXPECT_EQ(r.expansions, 50U);
}

TEST(AStarOpenGridTest, TheHeuristicKeepsTheSearchFarSmallerThanTheMap) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(100, 100);
  AStarPlanner planner;
  Path path;
  const SearchResult r = planner.search(w, P(0, 0), P(99, 99), {}, path);
  ASSERT_TRUE(r.status == SearchStatus::Found);
  EXPECT_EQ(r.cost, 198);
  // Breadth-first search would expand all 10,000 cells; A* with ties broken towards the goal stays
  // on a narrow band.
  EXPECT_LT(r.expansions, 1000U);
}

TEST(AStarOpenGridTest, StartEqualToGoalIsAFreeEmptyPath) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(3, 3);
  AStarPlanner planner;
  Path path{P(9, 9)};  // stale content must be cleared
  const SearchResult r = planner.search(w, P(1, 1), P(1, 1), {}, path);
  EXPECT_TRUE(r.status == SearchStatus::Found);
  EXPECT_EQ(r.cost, 0);
  EXPECT_EQ(r.expansions, 0U);
  EXPECT_TRUE(path.empty());
}

TEST(AStarOpenGridTest, NeighbouringCellsAreOneStepApart) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(3, 3);
  AStarPlanner planner;
  Path path;
  EXPECT_TRUE(planAndVerify(planner, w, P(1, 1), P(2, 1), {}, &path).status == SearchStatus::Found);
  EXPECT_EQ(path, (Path{P(2, 1)}));
}

TEST(AStarOpenGridTest, DegenerateOneCellAndOneColumnGrids) {
  AStarPlanner planner;
  const Warehouse single = warehousesim::testing::makeOpenGrid(1, 1);
  EXPECT_TRUE(planAndVerify(planner, single, P(0, 0), P(0, 0)).status == SearchStatus::Found);
  const Warehouse column = warehousesim::testing::makeOpenGrid(1, 7);
  Path path;
  EXPECT_TRUE(planAndVerify(planner, column, P(0, 6), P(0, 0), {}, &path).status ==
              SearchStatus::Found);
  EXPECT_EQ(path.size(), 6U);
  EXPECT_EQ(path.front(), P(0, 5));
}

// ---- deterministic tie-breaking --------------------------------------------------------

TEST(AStarTieBreakTest, OpenGridPathsFollowTheDocumentedOrder) {
  // Ties on f go to the smaller h, then the smaller row-major index, so the search runs along the
  // top row to the goal's column and then down.
  const Warehouse w = warehousesim::testing::makeOpenGrid(3, 3);
  AStarPlanner planner;
  Path path;
  ASSERT_TRUE(planner.search(w, P(0, 0), P(2, 2), {}, path).status == SearchStatus::Found);
  EXPECT_EQ(path, (Path{P(1, 0), P(2, 0), P(2, 1), P(2, 2)}));

  ASSERT_TRUE(planner.search(w, P(2, 2), P(0, 0), {}, path).status == SearchStatus::Found);
  EXPECT_EQ(path, (Path{P(2, 1), P(2, 0), P(1, 0), P(0, 0)}));
}

TEST(AStarTieBreakTest, RepeatedAndReusedSearchesGiveIdenticalPaths) {
  Rng rng(5);
  const Warehouse w = warehousesim::testing::makeRandomObstacleMap(30, 30, 200, rng);
  AStarPlanner reused;
  for (int i = 0; i < 40; ++i) {
    const Point start = P(static_cast<int>(rng.below(30)), static_cast<int>(rng.below(30)));
    const Point goal = P(static_cast<int>(rng.below(30)), static_cast<int>(rng.below(30)));
    AStarPlanner fresh;
    const auto a = reused.findPath(w, start, goal, {});
    const auto b = fresh.findPath(w, start, goal, {});
    const auto c = reused.findPath(w, start, goal, {});
    EXPECT_EQ(a, b);
    EXPECT_EQ(a, c);
  }
}

TEST(AStarTieBreakTest, EqualLengthRoutesAreChosenReproducibly) {
  // Two equally short routes around a single pillar; the same one every time.
  const Warehouse w = layout({"...", ".#.", "..."});
  AStarPlanner planner;
  Path first;
  ASSERT_TRUE(planner.search(w, P(0, 1), P(2, 1), {}, first).status == SearchStatus::Found);
  EXPECT_EQ(first.size(), 4U);
  EXPECT_EQ(first, (Path{P(0, 0), P(1, 0), P(2, 0), P(2, 1)}));  // over the top, not underneath
  for (int i = 0; i < 5; ++i) {
    Path again;
    ASSERT_TRUE(planner.search(w, P(0, 1), P(2, 1), {}, again).status == SearchStatus::Found);
    EXPECT_EQ(again, first);
  }
}

// ---- mazes -----------------------------------------------------------------------------

TEST(AStarMazeTest, APerfectMazeHasExactlyOneRouteAndAStarFindsIt) {
  for (std::uint64_t seed = 1; seed <= 8; ++seed) {
    Rng rng(seed);
    const Warehouse maze = warehousesim::testing::makeMaze(12, 9, rng);
    ASSERT_EQ(maze.width(), 25);
    ASSERT_EQ(maze.height(), 19);
    AStarPlanner planner;
    // From the top-left room to the bottom-right room, and between random rooms.
    std::vector<std::pair<Point, Point>> queries = {{P(1, 1), P(23, 17)}, {P(23, 17), P(1, 1)}};
    for (int i = 0; i < 12; ++i) {
      queries.emplace_back(
          P(1 + 2 * static_cast<int>(rng.below(12)), 1 + 2 * static_cast<int>(rng.below(9))),
          P(1 + 2 * static_cast<int>(rng.below(12)), 1 + 2 * static_cast<int>(rng.below(9))));
    }
    for (const auto& [start, goal] : queries) {
      Path path;
      const SearchResult r = planAndVerify(planner, maze, start, goal, {}, &path);
      ASSERT_TRUE(r.status == SearchStatus::Found);
      // Shortest and only route: no cell is visited twice.
      std::set<Point> cells(path.begin(), path.end());
      EXPECT_EQ(cells.size(), path.size());
    }
  }
}

TEST(AStarMazeTest, EveryRoomOfAMazeIsReachableFromTheFirst) {
  Rng rng(77);
  const Warehouse maze = warehousesim::testing::makeMaze(8, 8, rng);
  AStarPlanner planner;
  for (int cy = 0; cy < 8; ++cy) {
    for (int cx = 0; cx < 8; ++cx) {
      EXPECT_TRUE(planAndVerify(planner, maze, P(1, 1), P(2 * cx + 1, 2 * cy + 1)).status ==
                  SearchStatus::Found);
    }
  }
}

TEST(AStarMazeTest, WallingOffAPassageMakesTheFarSideUnreachable) {
  Rng rng(3);
  Warehouse maze = warehousesim::testing::makeMaze(6, 6, rng);
  AStarPlanner planner;
  Path path;
  ASSERT_TRUE(planner.search(maze, P(1, 1), P(11, 11), {}, path).status == SearchStatus::Found);
  // A perfect maze has one route: blocking any cell on it (other than the ends) disconnects them.
  ASSERT_GT(path.size(), 2U);
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const BlockedSet blocked{path[i]};
    EXPECT_TRUE(planAndVerify(planner, maze, P(1, 1), P(11, 11), blocked).status ==
                SearchStatus::Unreachable);
  }
}

// ---- narrow corridors ------------------------------------------------------------------

TEST(AStarCorridorTest, ASerpentineCorridorIsWalkedEndToEnd) {
  const std::int32_t width = 9;
  const std::int32_t corridors = 6;
  const Warehouse w = warehousesim::testing::makeSerpentine(width, corridors);
  ASSERT_EQ(w.height(), 11);
  const Point end = P(corridors % 2 == 1 ? width - 1 : 0, w.height() - 1);
  AStarPlanner planner;
  Path path;
  const SearchResult r = planAndVerify(planner, w, P(0, 0), end, {}, &path);
  ASSERT_TRUE(r.status == SearchStatus::Found);
  EXPECT_EQ(r.cost, corridors * (width - 1) + 2 * (corridors - 1));
  // Nothing branches, so every step is forced and nothing off the route is expanded.
  EXPECT_EQ(r.expansions, path.size() + 1);
}

TEST(AStarCorridorTest, AOneCellDoorJoinsTwoRooms) {
  const Warehouse w = layout({"......#......", "......#......",
                              "............."  // the only gap, in the bottom row
                              ,
                              "......#......"});
  AStarPlanner planner;
  Path path;
  const SearchResult r = planAndVerify(planner, w, P(0, 0), P(12, 0), {}, &path);
  ASSERT_TRUE(r.status == SearchStatus::Found);
  EXPECT_EQ(r.cost, 12 + 2 + 2);  // down to the gap row, across, back up
  EXPECT_TRUE(std::find(path.begin(), path.end(), P(6, 2)) != path.end());  // through the door
}

TEST(AStarCorridorTest, ARobotStandingInTheDoorwayBlocksTheRoute) {
  const Warehouse w = layout({"..#..", "..#..", ".....", "..#..", "..#.."});
  AStarPlanner planner;
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(4, 4)).status == SearchStatus::Found);
  const BlockedSet doorway{P(2, 2)};
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(4, 4), doorway).status ==
              SearchStatus::Unreachable);
}

TEST(AStarCorridorTest, DeadEndsAreNotPartOfTheRoute) {
  const Warehouse w = layout(
      {"#########", "#.......#", "#.#####.#", "#.#...#.#", "#.#.#.#.#", "#...#...#", "#########"});
  AStarPlanner planner;
  Path path;
  const SearchResult r = planAndVerify(planner, w, P(1, 1), P(7, 5), {}, &path);
  ASSERT_TRUE(r.status == SearchStatus::Found);
  EXPECT_EQ(r.cost, 10);
  for (const Point p : path) {
    EXPECT_FALSE(p.y == 3 && p.x == 3);  // the dead-end alcove inside
  }
}

TEST(AStarCorridorTest, OneStepWideGapsBetweenShelvesAreUsable) {
  const Warehouse w = layout({"S.S.S", "S.S.S", "S.S.S", ".....", "S.S.S"});
  AStarPlanner planner;
  Path path;
  const SearchResult r = planAndVerify(planner, w, P(1, 0), P(3, 4), {}, &path);
  ASSERT_TRUE(r.status == SearchStatus::Found);
  EXPECT_EQ(r.cost, 6);
}

// ---- blocked and invalid endpoints -----------------------------------------------------

TEST(AStarEndpointTest, ImpossibleEndpointsAreReportedByKind) {
  const Warehouse w = layout({".#S.", "...."});
  AStarPlanner planner;
  Path path;
  EXPECT_TRUE(planner.search(w, P(0, 0), P(1, 0), {}, path).status ==
              SearchStatus::InvalidGoal);  // wall
  EXPECT_TRUE(planner.search(w, P(0, 0), P(2, 0), {}, path).status ==
              SearchStatus::InvalidGoal);  // shelf
  EXPECT_TRUE(planner.search(w, P(0, 0), P(9, 9), {}, path).status ==
              SearchStatus::InvalidGoal);  // outside
  EXPECT_TRUE(planner.search(w, P(0, 0), P(-1, 0), {}, path).status == SearchStatus::InvalidGoal);
  EXPECT_TRUE(planner.search(w, P(1, 0), P(0, 0), {}, path).status == SearchStatus::InvalidStart);
  EXPECT_TRUE(planner.search(w, P(-3, 0), P(0, 0), {}, path).status == SearchStatus::InvalidStart);
  EXPECT_TRUE(path.empty());
}

TEST(AStarEndpointTest, ABlockedGoalIsRejectedButABlockedStartIsFine) {
  const Warehouse w = layout({"....."});
  AStarPlanner planner;
  Path path;
  EXPECT_TRUE(planner.search(w, P(0, 0), P(4, 0), BlockedSet{P(4, 0)}, path).status ==
              SearchStatus::GoalBlocked);
  EXPECT_TRUE(planner.search(w, P(0, 0), P(4, 0), BlockedSet{P(0, 0)}, path).status ==
              SearchStatus::Found);  // the robot's own cell
  EXPECT_EQ(path.size(), 4U);
}

TEST(AStarEndpointTest, BlockedCellsOutsideTheGridAreIgnored) {
  const Warehouse w = layout({"...."});
  AStarPlanner planner;
  Path path;
  EXPECT_TRUE(planner.search(w, P(0, 0), P(3, 0), BlockedSet{P(-1, 0), P(7, 7)}, path).status ==
              SearchStatus::Found);
}

TEST(AStarEndpointTest, SpecialWalkableCellsAreOrdinaryTerrain) {
  const Warehouse w = layout({"PDC"});
  AStarPlanner planner;
  Path path;
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(2, 0), {}, &path).status == SearchStatus::Found);
  EXPECT_EQ(path.size(), 2U);
}

// ---- unreachable targets ---------------------------------------------------------------

TEST(AStarUnreachableTest, ASealedGoalIsRejectedWithoutSearching) {
  Warehouse w = warehousesim::testing::makeOpenGrid(40, 40);
  for (const Point wall : {P(19, 20), P(21, 20), P(20, 19), P(20, 21)}) {
    w.set(wall, CellType::Wall);
  }
  AStarPlanner planner;
  Path path;
  const SearchResult r = planner.search(w, P(0, 0), P(20, 20), {}, path);
  EXPECT_TRUE(r.status == SearchStatus::Unreachable);
  EXPECT_EQ(r.expansions, 0U);  // no flood fill of the 1,600-cell map
}

TEST(AStarUnreachableTest, ASealedStartIsRejectedWithoutSearching) {
  Warehouse w = warehousesim::testing::makeOpenGrid(10, 10);
  w.set(P(1, 0), CellType::Wall);
  w.set(P(0, 1), CellType::Shelf);
  AStarPlanner planner;
  Path path;
  const SearchResult r = planner.search(w, P(0, 0), P(9, 9), {}, path);
  EXPECT_TRUE(r.status == SearchStatus::Unreachable);
  EXPECT_EQ(r.expansions, 0U);
}

TEST(AStarUnreachableTest, RobotsAroundTheGoalSealItToo) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(7, 7);
  AStarPlanner planner;
  Path path;
  const BlockedSet ring{P(2, 3), P(4, 3), P(3, 2), P(3, 4)};
  const SearchResult r = planner.search(w, P(0, 0), P(3, 3), ring, path);
  EXPECT_TRUE(r.status == SearchStatus::Unreachable);
  EXPECT_EQ(r.expansions, 0U);
}

TEST(AStarUnreachableTest, AWallAcrossTheMapExhaustsExactlyTheStartsSide) {
  const Warehouse w = warehousesim::testing::makeSplitMap(21, 15);
  AStarPlanner planner;
  Path path;
  const SearchResult r = planner.search(w, P(0, 0), P(20, 14), {}, path);
  EXPECT_TRUE(r.status == SearchStatus::Unreachable);
  EXPECT_TRUE(path.empty());
  // Every cell left of the wall (10 columns x 15 rows) is reachable, and each is expanded once.
  EXPECT_EQ(r.expansions, 150U);
  const auto reference = warehousesim::testing::distancesFrom(w, P(0, 0), {}, DirectionSet::all());
  std::size_t reachable = 0;
  for (const std::int32_t d : reference) {
    reachable += d != kUnreachable ? 1U : 0U;
  }
  EXPECT_EQ(reachable, 150U);
}

TEST(AStarUnreachableTest, AnEnclosedRoomIsUnreachableFromOutside) {
  const Warehouse w = layout({".......", ".#####.", ".#...#.", ".#...#.", ".#####.", "......."});
  AStarPlanner planner;
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(3, 2)).status == SearchStatus::Unreachable);
  EXPECT_TRUE(planAndVerify(planner, w, P(2, 2), P(3, 3)).status == SearchStatus::Found);
  EXPECT_TRUE(planAndVerify(planner, w, P(2, 2), P(0, 0)).status == SearchStatus::Unreachable);
}

TEST(AStarUnreachableTest, ASearchBudgetStopsHugeFruitlessSearches) {
  const Warehouse w = warehousesim::testing::makeSplitMap(201, 201);
  AStarOptions options;
  options.maxExpansions = 500;
  AStarPlanner planner(options);
  Path path;
  const SearchResult r = planner.search(w, P(0, 0), P(200, 200), {}, path);
  EXPECT_TRUE(r.status == SearchStatus::BudgetExceeded);
  EXPECT_EQ(r.expansions, 500U);
  EXPECT_TRUE(path.empty());
  EXPECT_FALSE(planner.findPath(w, P(0, 0), P(200, 200), {}).has_value());

  // A route that fits in the budget is still found.
  EXPECT_TRUE(planner.search(w, P(0, 0), P(60, 0), {}, path).status == SearchStatus::Found);
}

// ---- configurable movement -------------------------------------------------------------

TEST(AStarMovementTest, DirectionSetBasics) {
  EXPECT_TRUE(DirectionSet{}.empty());
  EXPECT_EQ(DirectionSet::all().size(), 4U);
  const DirectionSet eastWest{Direction::East, Direction::West};
  EXPECT_EQ(eastWest.size(), 2U);
  EXPECT_TRUE(eastWest.contains(Direction::East));
  EXPECT_FALSE(eastWest.contains(Direction::North));
  EXPECT_TRUE((DirectionSet{Direction::East, Direction::East}) == (DirectionSet{Direction::East}));
}

TEST(AStarMovementTest, NoDirectionsIsAConfigurationError) {
  AStarOptions options;
  options.directions = DirectionSet{};
  EXPECT_THROW(AStarPlanner{options}, std::invalid_argument);
}

TEST(AStarMovementTest, RowOnlyRobotsCannotChangeRows) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(6, 4);
  AStarOptions options;
  options.directions = DirectionSet{Direction::East, Direction::West};
  AStarPlanner planner(options);
  Path path;
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 2), P(5, 2), {}, &path).status == SearchStatus::Found);
  EXPECT_EQ(path.size(), 5U);
  EXPECT_TRUE(planAndVerify(planner, w, P(5, 2), P(0, 2)).status == SearchStatus::Found);
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 2), P(5, 3)).status == SearchStatus::Unreachable);
}

TEST(AStarMovementTest, OneWayMovementIsAsymmetric) {
  const Warehouse w = layout({"....."});
  AStarOptions options;
  options.directions = DirectionSet{Direction::East};
  AStarPlanner planner(options);
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(4, 0)).status == SearchStatus::Found);
  EXPECT_TRUE(planAndVerify(planner, w, P(4, 0), P(0, 0)).status == SearchStatus::Unreachable);
}

TEST(AStarMovementTest, RestrictedMovesStillFindTheShortestLegalRoute) {
  // East and South only: staircase routes to the south-east, impossible towards the north-west.
  const Warehouse w = layout({"....", ".##.", "....", "...."});
  AStarOptions options;
  options.directions = DirectionSet{Direction::East, Direction::South};
  AStarPlanner planner(options);
  Path path;
  EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(3, 3), {}, &path).status == SearchStatus::Found);
  EXPECT_EQ(path.size(), 6U);
  EXPECT_TRUE(planAndVerify(planner, w, P(3, 3), P(0, 0)).status == SearchStatus::Unreachable);

  // The goal row can only be entered by moving east along it, and no south/east route leads back
  // up.
  const Warehouse upper = layout({"..#..", "#.#..", "#...."});
  EXPECT_TRUE(planAndVerify(planner, upper, P(1, 0), P(4, 0)).status == SearchStatus::Unreachable);
  AStarPlanner unrestricted;
  EXPECT_TRUE(planAndVerify(unrestricted, upper, P(1, 0), P(4, 0)).status == SearchStatus::Found);
}

TEST(AStarMovementTest, ExpansionOrderIsFixedWhateverTheSetOrderOfConstruction) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(5, 5);
  AStarOptions a;
  a.directions = DirectionSet{Direction::West, Direction::South, Direction::East, Direction::North};
  AStarOptions b;
  b.directions = DirectionSet::all();
  AStarPlanner planA(a);
  AStarPlanner planB(b);
  EXPECT_EQ(planA.findPath(w, P(0, 0), P(4, 4), {}), planB.findPath(w, P(0, 0), P(4, 4), {}));
}

// ---- results and buffers ---------------------------------------------------------------

TEST(AStarResultTest, StatusNamesAreDistinct) {
  std::set<std::string> names;
  for (const SearchStatus status :
       {SearchStatus::Found, SearchStatus::InvalidStart, SearchStatus::InvalidGoal,
        SearchStatus::GoalBlocked, SearchStatus::Unreachable, SearchStatus::BudgetExceeded}) {
    names.insert(warehousesim::planning::toString(status));
  }
  EXPECT_EQ(names.size(), 6U);
}

TEST(AStarResultTest, SearchReusesTheCallersPathBufferAndClearsItOnFailure) {
  const Warehouse w = layout({"....", "..#.", "...."});
  AStarPlanner planner;
  Path path;
  ASSERT_TRUE(planner.search(w, P(0, 0), P(3, 2), {}, path).status == SearchStatus::Found);
  EXPECT_EQ(path.size(), 5U);
  const auto* storage = path.data();
  ASSERT_TRUE(planner.search(w, P(0, 0), P(3, 0), {}, path).status == SearchStatus::Found);
  EXPECT_EQ(path.size(), 3U);
  EXPECT_TRUE(path.data() == storage);  // shorter route, same buffer
  EXPECT_TRUE(planner.search(w, P(0, 0), P(2, 1), {}, path).status == SearchStatus::InvalidGoal);
  EXPECT_TRUE(path.empty());
}

TEST(AStarResultTest, FindPathAgreesWithSearch) {
  const Warehouse w = layout({"....", ".##.", "...."});
  AStarPlanner planner;
  Path path;
  ASSERT_TRUE(planner.search(w, P(0, 0), P(3, 2), {}, path).status == SearchStatus::Found);
  EXPECT_EQ(planner.findPath(w, P(0, 0), P(3, 2), {}), std::optional<Path>(path));
  EXPECT_FALSE(planner.findPath(w, P(0, 0), P(1, 1), {}).has_value());
  EXPECT_EQ(planner.findPath(w, P(2, 2), P(2, 2), {}), std::optional<Path>(Path{}));
}

TEST(AStarResultTest, OneThousandSearchesInARowStayCorrect) {
  // Exercises the generation counter that stands in for clearing the workspace.
  const Warehouse w = layout({"......", ".####.", "......"});
  AStarPlanner planner;
  for (int i = 0; i < 1000; ++i) {
    EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(5, 2)).status == SearchStatus::Found);
    EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(2, 1)).status == SearchStatus::InvalidGoal);
  }
}

TEST(AStarResultTest, PlannerHandlesGridsOfDifferentSizesBackToBack) {
  AStarPlanner planner;
  for (const std::int32_t size : {5, 40, 3, 90, 7, 40}) {
    const Warehouse w = warehousesim::testing::makeOpenGrid(size, size);
    EXPECT_TRUE(planAndVerify(planner, w, P(0, 0), P(size - 1, size - 1)).status ==
                SearchStatus::Found);
  }
}

}  // namespace
