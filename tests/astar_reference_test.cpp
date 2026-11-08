#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <vector>

#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/testing/map_generators.hpp"
#include "warehousesim/testing/reference_search.hpp"
#include "warehousesim/world/warehouse.hpp"

// A* is checked against breadth-first search, an independent shortest-path algorithm that is
// trivially correct for unit costs, on thousands of random maps, queries and move sets.
namespace {

using warehousesim::core::Direction;
using warehousesim::core::DirectionSet;
using warehousesim::core::kAllDirections;
using warehousesim::core::manhattanDistance;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::planning::AStarOptions;
using warehousesim::planning::AStarPlanner;
using warehousesim::planning::BlockedSet;
using warehousesim::planning::Path;
using warehousesim::planning::SearchResult;
using warehousesim::planning::SearchStatus;
using warehousesim::testing::kUnreachable;
using warehousesim::world::Warehouse;

int randomInt(Rng& rng, int bound) {
  return static_cast<int>(rng.below(static_cast<std::uint64_t>(bound)));
}

DirectionSet randomDirections(Rng& rng) {
  if (rng.below(10) < 4) {
    return DirectionSet::all();
  }
  const std::uint64_t mask = 1 + rng.below(15);  // any non-empty subset
  DirectionSet set;
  for (std::size_t i = 0; i < kAllDirections.size(); ++i) {
    if ((mask >> i) & 1U) {
      set = set.with(kAllDirections[i]);
    }
  }
  return set;
}

// A random cell, usually a walkable one (so most queries are real), sometimes anything at all
// including cells outside the grid and obstacles (so rejections are exercised too).
Point randomCell(Rng& rng, const Warehouse& w) {
  const bool wantWalkable = rng.below(10) < 8;
  Point p{};
  for (int attempt = 0; attempt < 16; ++attempt) {
    p = Point{randomInt(rng, w.width() + 2) - 1, randomInt(rng, w.height() + 2) - 1};
    if (!wantWalkable || w.isWalkable(p)) {
      break;
    }
  }
  return p;
}

struct Tally {
  int found = 0;
  int unreachable = 0;
  int invalid = 0;
  int nonTrivial = 0;  // found with at least 3 steps
};

// Runs one query and compares everything observable with the reference.
void checkQuery(AStarPlanner& planner, const Warehouse& w, Point start, Point goal,
                const BlockedSet& blocked, Tally& tally) {
  Path path;
  const SearchResult result = planner.search(w, start, goal, blocked, path);
  const DirectionSet directions = planner.options().directions;
  const std::optional<std::int32_t> expected =
      warehousesim::testing::shortestDistance(w, start, goal, blocked, directions);

  ASSERT_EQ(result.status == SearchStatus::Found, expected.has_value());
  if (!expected) {
    ++(w.isWalkable(start) && w.isWalkable(goal) && !blocked.contains(goal) ? tally.unreachable
                                                                            : tally.invalid);
    EXPECT_TRUE(path.empty());
    return;
  }
  ++tally.found;
  tally.nonTrivial += *expected >= 3 ? 1 : 0;
  EXPECT_EQ(result.cost, *expected);  // optimal
  EXPECT_EQ(path.size(), static_cast<std::size_t>(*expected));
  EXPECT_EQ(warehousesim::testing::pathProblem(w, start, goal, path, blocked, directions)
                .value_or("valid"),
            "valid");                                  // legal
  EXPECT_LE(result.expansions, w.size().cellCount());  // at most once per cell
  if (*expected > 0) {
    EXPECT_GE(result.expansions, static_cast<std::size_t>(*expected) + 1);  // lower bound
  }
}

TEST(AStarReferenceTest, RandomMapsAndQueriesMatchBreadthFirstSearch) {
  Rng rng(0xA57A12);
  Tally tally;
  for (int map = 0; map < 1500; ++map) {
    const int width = 1 + randomInt(rng, 14);
    const int height = 1 + randomInt(rng, 14);
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        width, height, static_cast<std::uint32_t>(rng.below(500)), rng);
    AStarOptions options;
    options.directions = randomDirections(rng);
    AStarPlanner planner(options);  // one planner per map, reused for all its queries
    for (int query = 0; query < 8; ++query) {
      // Starts and goals are sometimes outside the grid or on obstacles: those must be rejected.
      const Point start = randomCell(rng, w);
      const Point goal = randomCell(rng, w);
      BlockedSet blocked;
      for (int i = randomInt(rng, 6); i > 0; --i) {
        blocked.insert(randomCell(rng, w));
      }
      checkQuery(planner, w, start, goal, blocked, tally);
    }
  }
  // Make sure the comparison was not vacuous: a healthy mix of outcomes was exercised.
  EXPECT_GT(tally.found, 3000);
  EXPECT_GT(tally.nonTrivial, 1800);
  EXPECT_GT(tally.unreachable, 3000);
  EXPECT_GT(tally.invalid, 3000);
}

TEST(AStarReferenceTest, LargerMapsAndLongRoutesMatchToo) {
  Rng rng(424242);
  Tally tally;
  for (int map = 0; map < 25; ++map) {
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        60 + randomInt(rng, 40), 60 + randomInt(rng, 40),
        150 + static_cast<std::uint32_t>(rng.below(200)), rng);
    AStarPlanner planner;
    for (int query = 0; query < 12; ++query) {
      const Point start{randomInt(rng, w.width()), randomInt(rng, w.height())};
      const Point goal{randomInt(rng, w.width()), randomInt(rng, w.height())};
      checkQuery(planner, w, start, goal, {}, tally);
    }
  }
  EXPECT_GT(tally.found, 100);
}

TEST(AStarReferenceTest, MazesMatchBreadthFirstSearchAndTheRouteIsUnique) {
  Rng rng(1234);
  for (int round = 0; round < 40; ++round) {
    const int cellsWide = 2 + randomInt(rng, 15);
    const int cellsHigh = 2 + randomInt(rng, 15);
    const Warehouse maze = warehousesim::testing::makeMaze(cellsWide, cellsHigh, rng);
    AStarPlanner planner;
    Tally tally;
    for (int query = 0; query < 10; ++query) {
      const Point start{1 + 2 * randomInt(rng, cellsWide), 1 + 2 * randomInt(rng, cellsHigh)};
      const Point goal{1 + 2 * randomInt(rng, cellsWide), 1 + 2 * randomInt(rng, cellsHigh)};
      checkQuery(planner, maze, start, goal, {}, tally);
      EXPECT_EQ(tally.unreachable, 0);  // every room of a perfect maze is connected
    }
  }
}

// h(cell) = Manhattan distance to the goal must never exceed the true distance (admissible) and
// must never drop by more than one per step (consistent), for the full move set and every subset.
TEST(AStarReferenceTest, ManhattanHeuristicIsAdmissibleAndConsistent) {
  Rng rng(99);
  for (int map = 0; map < 300; ++map) {
    const int width = 2 + randomInt(rng, 12);
    const int height = 2 + randomInt(rng, 12);
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        width, height, static_cast<std::uint32_t>(rng.below(450)), rng);
    const Point goal{randomInt(rng, width), randomInt(rng, height)};
    const DirectionSet directions = randomDirections(rng);
    const std::vector<std::int32_t> toGoal =
        warehousesim::testing::distancesTo(w, goal, directions);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const Point cell{x, y};
        const std::int32_t trueDistance = toGoal[w.size().indexOf(cell)];
        if (trueDistance == kUnreachable) {
          continue;
        }
        const std::int32_t h = manhattanDistance(cell, goal);
        EXPECT_LE(h, trueDistance);  // admissible
        for (const Direction direction : kAllDirections) {
          const Point next = cell + warehousesim::core::offsetOf(direction);
          if (directions.contains(direction) && w.isWalkable(next) && w.isWalkable(cell)) {
            EXPECT_LE(h, 1 + manhattanDistance(next, goal));  // consistent
          }
        }
      }
    }
  }
}

// A planner that has run thousands of searches over grids of many sizes must answer exactly like a
// brand-new one: nothing may leak between searches through the reused workspace.
TEST(AStarReferenceTest, AReusedPlannerAnswersLikeAFreshOne) {
  Rng rng(2718);
  AStarPlanner reused;
  for (int i = 0; i < 1500; ++i) {
    const int width = 1 + randomInt(rng, 25);
    const int height = 1 + randomInt(rng, 25);
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        width, height, static_cast<std::uint32_t>(rng.below(400)), rng);
    const Point start{randomInt(rng, width), randomInt(rng, height)};
    const Point goal{randomInt(rng, width), randomInt(rng, height)};
    BlockedSet blocked;
    for (int b = randomInt(rng, 5); b > 0; --b) {
      blocked.insert({randomInt(rng, width), randomInt(rng, height)});
    }
    AStarPlanner fresh;
    Path reusedPath;
    Path freshPath;
    const SearchResult a = reused.search(w, start, goal, blocked, reusedPath);
    const SearchResult b = fresh.search(w, start, goal, blocked, freshPath);
    ASSERT_TRUE(a.status == b.status);
    ASSERT_EQ(reusedPath, freshPath);
    ASSERT_EQ(a.cost, b.cost);
    ASSERT_EQ(a.expansions, b.expansions);
  }
}

// Paths are bit-for-bit reproducible across runs, compilers and standard libraries: the maps come
// from the platform-independent Rng, and the search order is fully determined by its tie-breaking
// keys. This hash was recorded once; if it changes, the planner's results changed.
TEST(AStarReferenceTest, ResultsAreReproducibleAcrossPlatforms) {
  Rng rng(31415926);
  std::uint64_t hash = 14695981039346656037ULL;  // FNV-1a
  const auto mix = [&hash](std::uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      hash ^= (value >> (8 * byte)) & 0xFFU;
      hash *= 1099511628211ULL;
    }
  };
  AStarPlanner planner;
  for (int i = 0; i < 400; ++i) {
    const int width = 3 + randomInt(rng, 30);
    const int height = 3 + randomInt(rng, 30);
    const Warehouse w = warehousesim::testing::makeRandomObstacleMap(
        width, height, static_cast<std::uint32_t>(rng.below(350)), rng);
    const Point start{randomInt(rng, width), randomInt(rng, height)};
    const Point goal{randomInt(rng, width), randomInt(rng, height)};
    Path path;
    const SearchResult r = planner.search(w, start, goal, {}, path);
    mix(static_cast<std::uint64_t>(r.status));
    mix(r.expansions);
    for (const Point p : path) {
      mix(static_cast<std::uint64_t>(p.x));
      mix(static_cast<std::uint64_t>(p.y));
    }
  }
  EXPECT_EQ(hash, 11782161010464066301ULL);
}

}  // namespace
