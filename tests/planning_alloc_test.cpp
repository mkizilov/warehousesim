#include <gtest/gtest.h>

#include <cstddef>
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
//
// What is counted is the planner's own allocations, so everything else is made outside the
// measured region: the paths, and the empty set of blocked cells (a PointSet is two vectors). A
// checked standard library allocates for its containers too: MSVC's, with _ITERATOR_DEBUG_LEVEL > 0
// (its Debug configuration), allocates a small proxy each time a vector is made or moved into.
// libc++, libstdc++ and MSVC's Release configuration do not. Where a vector cannot be made outside
// the measured region (the ones findPath makes), the library's cost for it is measured here, not
// assumed.
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

const BlockedSet kNoBlocked{};

template <typename F>
std::uint64_t allocationsDuring(F&& action) {
  const std::uint64_t before = allocationCount();
  action();
  return allocationCount() - before;
}

// What the standard library alone allocates for one vector it makes or moves into: 0 normally, 1
// with checked containers.
std::uint64_t vectorBookkeeping() {
  return allocationsDuring([] {
    const std::vector<Point> made;
    (void)made;
  });
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
  EXPECT_GT(
      allocationsDuring([&] { (void)planner.search(w, P(0, 0), P(63, 63), kNoBlocked, path); }),
      0U);
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
  (void)planner.search(w, P(0, 0), P(119, 119), kNoBlocked, path);
  SearchStatus status = SearchStatus::Found;
  EXPECT_EQ(allocationsDuring(
                [&] { status = planner.search(w, P(0, 0), P(119, 119), kNoBlocked, path).status; }),
            0U);
  EXPECT_TRUE(status == SearchStatus::Unreachable);
}

struct FindPathCost {
  std::uint64_t allocations = 0;
  std::optional<std::size_t> length;  // of the path found, if one was
};

FindPathCost measureFindPath(AStarPlanner& planner, const Warehouse& w, Point from, Point to) {
  FindPathCost cost;
  cost.allocations = allocationsDuring([&] {
    const std::optional<Path> found = planner.findPath(w, from, to, kNoBlocked);
    if (found) {
      cost.length = found->size();
    }
  });
  return cost;
}

TEST(AStarAllocationTest, FindPathAllocatesOnlyTheReturnedPath) {
  const Warehouse w = warehousesim::testing::makeOpenGrid(40, 40);
  AStarPlanner planner;
  (void)planner.findPath(w, P(0, 0), P(39, 39), kNoBlocked);  // warm up

  // findPath makes two vectors: the one the search fills, and the one it hands back in the result
  // (moved from the first). The planner's own allocation is the buffer of the path.
  const std::uint64_t perVector = vectorBookkeeping();
  EXPECT_LE(perVector, 1U) << "a vector should cost the library at most one allocation of its own";

  const FindPathCost route = measureFindPath(planner, w, P(0, 0), P(39, 39));
  EXPECT_EQ(route.allocations, 1 + 2 * perVector);
  ASSERT_TRUE(route.length.has_value());
  EXPECT_EQ(*route.length, 78U);

  const FindPathCost here = measureFindPath(planner, w, P(3, 3), P(3, 3));  // an empty path
  EXPECT_EQ(here.allocations, 2 * perVector);
  ASSERT_TRUE(here.length.has_value());
  EXPECT_EQ(*here.length, 0U);

  const FindPathCost invalid = measureFindPath(planner, w, P(0, 0), P(-1, 0));  // no path at all
  EXPECT_EQ(invalid.allocations, perVector);
  EXPECT_FALSE(invalid.length.has_value());
}

TEST(AStarAllocationTest, ALargerGridGrowsTheWorkspaceOnceAndThenSettles) {
  AStarPlanner planner;
  Path path;
  const Warehouse small = warehousesim::testing::makeOpenGrid(32, 32);
  const Warehouse large = warehousesim::testing::makeOpenGrid(128, 128);
  (void)planner.search(small, P(0, 0), P(31, 31), kNoBlocked, path);
  EXPECT_GT(allocationsDuring(
                [&] { (void)planner.search(large, P(0, 0), P(127, 127), kNoBlocked, path); }),
            0U);
  EXPECT_EQ(allocationsDuring(
                [&] { (void)planner.search(large, P(0, 0), P(127, 127), kNoBlocked, path); }),
            0U);
  // Going back to the smaller grid reuses the bigger workspace without allocating.
  EXPECT_EQ(
      allocationsDuring([&] { (void)planner.search(small, P(0, 0), P(31, 31), kNoBlocked, path); }),
      0U);
}

}  // namespace
