#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/testing/allocation_counter.hpp"
#include "warehousesim/testing/map_generators.hpp"
#include "warehousesim/world/warehouse.hpp"

// Built into its own executable together with testing/allocation_counter.cpp, which counts calls
// to the global operator new.
namespace {

using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::planning::AStarPlanner;
using warehousesim::planning::BlockedSet;
using warehousesim::planning::Path;
using warehousesim::planning::SearchStatus;
using warehousesim::testing::allocationCount;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }

template <typename F>
std::uint64_t allocationsDuring(F&& action) {
  const std::uint64_t before = allocationCount();
  action();
  return allocationCount() - before;
}

struct Query {
  Point start;
  Point goal;
  BlockedSet blocked;
};

// A mix of everything the planner can be asked: long routes, a sealed goal, a far unreachable
// goal behind a wall, a blocked goal, start == goal, and invalid endpoints.
std::vector<Query> mixedQueries(const Warehouse& w) {
  return {
      {P(0, 0), P(w.width() - 1, w.height() - 1), {}},
      {P(w.width() - 1, 0), P(0, w.height() - 1), {P(1, 1), P(2, 2), P(3, 3)}},
      {P(5, 5), P(5, 5), {}},
      {P(0, 0), P(10, 10), {P(10, 10)}},
      {P(0, 0), P(-4, 3), {}},
      {P(0, 0), P(w.width() / 2, 0), {}},
  };
}

TEST(AStarAllocationTest, TheFirstSearchOnAGridSizeAllocatesItsWorkspace) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(64, 64);
  AStarPlanner planner;
  Path path;
  EXPECT_GT(allocationsDuring([&] { (void)planner.search(w, P(0, 0), P(63, 63), {}, path); }), 0U);
}

TEST(AStarAllocationTest, WarmSearchesWithAReusedPathBufferAllocateNothing) {
  Rng rng(11);
  const Warehouse w = warehousesim::testing::makeRandomObstacleMap(64, 64, 180, rng);
  const std::vector<Query> queries = mixedQueries(w);
  AStarPlanner planner;
  Path path;
  for (const Query& q : queries) {  // warm-up: sizes the workspace, heap and path buffer
    (void)planner.search(w, q.start, q.goal, q.blocked, path);
  }
  int found = 0;
  for (const Query& q : queries) {
    SearchStatus status = SearchStatus::Unreachable;
    const std::uint64_t allocations = allocationsDuring(
        [&] { status = planner.search(w, q.start, q.goal, q.blocked, path).status; });
    EXPECT_EQ(allocations, 0U);
    found += status == SearchStatus::Found ? 1 : 0;
  }
  EXPECT_GE(found, 2);  // the queries really did search, not just bail out
}

TEST(AStarAllocationTest, UnreachableFloodSearchesAllocateNothingWhenWarm) {
  const Warehouse w = warehousesim::testing::makeSplitMap(120, 120);
  AStarPlanner planner;
  Path path;
  (void)planner.search(w, P(0, 0), P(119, 119), {}, path);
  EXPECT_EQ(allocationsDuring([&] {
              EXPECT_TRUE(planner.search(w, P(0, 0), P(119, 119), {}, path).status ==
                          SearchStatus::Unreachable);
            }),
            0U);
}

TEST(AStarAllocationTest, FindPathAllocatesOnlyTheReturnedPath) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(40, 40);
  AStarPlanner planner;
  (void)planner.findPath(w, P(0, 0), P(39, 39), {});  // warm up
  std::optional<Path> path;
  EXPECT_EQ(allocationsDuring([&] { path = planner.findPath(w, P(0, 0), P(39, 39), {}); }), 1U);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 78U);

  EXPECT_EQ(allocationsDuring([&] { path = planner.findPath(w, P(3, 3), P(3, 3), {}); }), 0U);
  EXPECT_EQ(allocationsDuring([&] { path = planner.findPath(w, P(0, 0), P(-1, 0), {}); }), 0U);
}

TEST(AStarAllocationTest, ALargerGridGrowsTheWorkspaceOnceAndThenSettles) {
  AStarPlanner planner;
  Path path;
  const Warehouse small = warehousesim::testing::makeOpenGrid(32, 32);
  const Warehouse large = warehousesim::testing::makeOpenGrid(128, 128);
  (void)planner.search(small, P(0, 0), P(31, 31), {}, path);
  EXPECT_GT(allocationsDuring([&] { (void)planner.search(large, P(0, 0), P(127, 127), {}, path); }),
            0U);
  EXPECT_EQ(allocationsDuring([&] { (void)planner.search(large, P(0, 0), P(127, 127), {}, path); }),
            0U);
  // Going back to the smaller grid reuses the bigger workspace without allocating.
  EXPECT_EQ(allocationsDuring([&] { (void)planner.search(small, P(0, 0), P(31, 31), {}, path); }),
            0U);
}

}  // namespace
