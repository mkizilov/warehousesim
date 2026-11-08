// Measures A* planning time on maps of increasing size and compares it with a plain breadth-first
// search. Single-threaded; all maps are generated in memory from fixed seeds, so nothing is written
// to disk and two runs plan exactly the same queries.
//
//   warehousesim_planning_bench [--quick] [--max-size N] [--csv]
//
// Columns: path = route length (steps), expanded = cells A* expanded, warm = a reused planner and
// path buffer, cold = a new planner (and its workspace) for every query, bfs = reference
// breadth-first search, allocs = heap allocations per warm search.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "arguments.hpp"
#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/testing/allocation_counter.hpp"
#include "warehousesim/testing/map_generators.hpp"
#include "warehousesim/testing/reference_search.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::DirectionSet;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::planning::AStarPlanner;
using warehousesim::planning::BlockedSet;
using warehousesim::planning::Path;
using warehousesim::planning::SearchResult;
using warehousesim::planning::SearchStatus;
using warehousesim::world::CellType;
using warehousesim::world::Warehouse;

using Clock = std::chrono::steady_clock;

struct Scenario {
  std::string name;
  Warehouse map;
  Point start;
  Point goal;
};

struct Timing {
  double medianNs = 0.0;
  double minNs = 0.0;
};

struct Settings {
  int samples = 11;
  double sampleMillis = 4.0;  // each sample runs the operation until about this much time passed
};

// Anything the optimiser could otherwise discard because nothing reads it.
volatile std::uint64_t gSink = 0;

double nanosecondsSince(Clock::time_point begin) {
  return std::chrono::duration<double, std::nano>(Clock::now() - begin).count();
}

// Time per call: calibrate how many calls fill one sample, then take the median of several.
template <typename Operation>
Timing measure(const Settings& settings, Operation&& operation) {
  operation();  // warm caches and any lazily sized buffers
  std::uint64_t inner = 1;
  while (inner < (1ULL << 20)) {
    const auto begin = Clock::now();
    for (std::uint64_t i = 0; i < inner; ++i) {
      operation();
    }
    if (nanosecondsSince(begin) >= settings.sampleMillis * 1e6) {
      break;
    }
    inner *= 2;
  }
  std::vector<double> perCall;
  for (int sample = 0; sample < settings.samples; ++sample) {
    const auto begin = Clock::now();
    for (std::uint64_t i = 0; i < inner; ++i) {
      operation();
    }
    perCall.push_back(nanosecondsSince(begin) / static_cast<double>(inner));
  }
  std::ranges::sort(perCall);
  return {perCall[perCall.size() / 2], perCall.front()};
}

std::vector<Scenario> makeScenarios(std::int32_t size) {
  std::vector<Scenario> scenarios;
  const Point corner{size - 1, size - 1};

  scenarios.push_back({"open", warehousesim::testing::makeOpenGrid(size, size), {0, 0}, corner});

  // 25% of the cells are obstacles; take the first seed whose corners are connected.
  for (std::uint64_t seed = 1; seed <= 500; ++seed) {
    Rng rng(seed * 7919 + static_cast<std::uint64_t>(size));
    Warehouse map = warehousesim::testing::makeRandomObstacleMap(size, size, 250, rng);
    map.set({0, 0}, CellType::Free);
    map.set(corner, CellType::Free);
    if (warehousesim::testing::shortestDistance(map, {0, 0}, corner, {}, DirectionSet::all())) {
      scenarios.push_back({"scattered25", std::move(map), {0, 0}, corner});
      break;
    }
  }

  // A perfect maze: one long winding route from the first room to the last.
  {
    Rng rng(static_cast<std::uint64_t>(size) * 104729);
    const std::int32_t rooms = std::max<std::int32_t>(2, size / 2);
    Warehouse maze = warehousesim::testing::makeMaze(rooms, rooms, rng);
    const Point last{2 * rooms - 1, 2 * rooms - 1};
    scenarios.push_back({"maze", std::move(maze), {1, 1}, last});
  }

  // A wall splits the map; the goal is on the far side, so A* has to exhaust the start's side.
  scenarios.push_back(
      {"unreachable", warehousesim::testing::makeSplitMap(size, size), {0, 0}, corner});

  // The goal is walled in: rejected before any searching.
  {
    Warehouse map = warehousesim::testing::makeOpenGrid(size, size);
    const Point goal{size / 2, size / 2};
    for (const Point wall : {Point{goal.x - 1, goal.y}, Point{goal.x + 1, goal.y},
                             Point{goal.x, goal.y - 1}, Point{goal.x, goal.y + 1}}) {
      map.set(wall, CellType::Wall);
    }
    scenarios.push_back({"sealed-goal", std::move(map), {0, 0}, goal});
  }
  return scenarios;
}

struct Row {
  std::string scenario;
  std::int32_t size = 0;
  std::size_t cells = 0;
  std::string path;  // length, or "-" if there is none
  std::size_t expanded = 0;
  Timing warm;
  Timing cold;
  Timing bfs;
  std::uint64_t warmAllocations = 0;
};

// Returns nullopt (and prints why) if A* disagrees with the reference.
std::optional<Row> run(const Scenario& scenario, std::int32_t size, const Settings& settings) {
  const BlockedSet noBlocked;
  AStarPlanner planner;
  Path path;
  const SearchResult first =
      planner.search(scenario.map, scenario.start, scenario.goal, noBlocked, path);
  const auto reference = warehousesim::testing::shortestDistance(
      scenario.map, scenario.start, scenario.goal, noBlocked, DirectionSet::all());
  if ((first.status == SearchStatus::Found) != reference.has_value() ||
      (reference && first.cost != *reference)) {
    std::fprintf(stderr, "MISMATCH: %s %d: A* says %s cost %d, reference says %s\n",
                 scenario.name.c_str(), size, warehousesim::planning::toString(first.status),
                 first.cost, reference ? std::to_string(*reference).c_str() : "unreachable");
    return std::nullopt;
  }

  Row row;
  row.scenario = scenario.name;
  row.size = size;
  row.cells = scenario.map.size().cellCount();
  row.path = first.status == SearchStatus::Found ? std::to_string(first.cost) : "-";
  row.expanded = first.expansions;

  row.warm = measure(settings, [&] {
    gSink = gSink + static_cast<std::uint64_t>(
                        planner.search(scenario.map, scenario.start, scenario.goal, noBlocked, path)
                            .expansions);
  });
  row.cold = measure(settings, [&] {
    AStarPlanner fresh;
    Path freshPath;
    gSink =
        gSink + static_cast<std::uint64_t>(
                    fresh.search(scenario.map, scenario.start, scenario.goal, noBlocked, freshPath)
                        .expansions);
  });
  row.bfs = measure(settings, [&] {
    const auto distance = warehousesim::testing::shortestDistance(
        scenario.map, scenario.start, scenario.goal, noBlocked, DirectionSet::all());
    gSink = gSink + static_cast<std::uint64_t>(distance.value_or(0));
  });

  const std::uint64_t before = warehousesim::testing::allocationCount();
  (void)planner.search(scenario.map, scenario.start, scenario.goal, noBlocked, path);
  row.warmAllocations = warehousesim::testing::allocationCount() - before;
  return row;
}

void printHeader(bool csv) {
  if (csv) {
    std::puts("scenario,size,cells,path,expanded,warm_ns,warm_min_ns,cold_ns,bfs_ns,warm_allocs");
  } else {
    std::printf("%-12s %5s %8s %7s %9s %11s %11s %11s %9s %7s\n", "scenario", "size", "cells",
                "path", "expanded", "warm (us)", "cold (us)", "bfs (us)", "ns/exp", "allocs");
  }
}

void printRow(const Row& row, bool csv) {
  if (csv) {
    std::printf("%s,%d,%zu,%s,%zu,%.0f,%.0f,%.0f,%.0f,%llu\n", row.scenario.c_str(), row.size,
                row.cells, row.path.c_str(), row.expanded, row.warm.medianNs, row.warm.minNs,
                row.cold.medianNs, row.bfs.medianNs,
                static_cast<unsigned long long>(row.warmAllocations));
    return;
  }
  const double nsPerExpansion =
      row.expanded == 0 ? 0.0 : row.warm.medianNs / static_cast<double>(row.expanded);
  std::printf("%-12s %5d %8zu %7s %9zu %11.2f %11.2f %11.2f %9.1f %7llu\n", row.scenario.c_str(),
              row.size, row.cells, row.path.c_str(), row.expanded, row.warm.medianNs / 1e3,
              row.cold.medianNs / 1e3, row.bfs.medianNs / 1e3, nsPerExpansion,
              static_cast<unsigned long long>(row.warmAllocations));
}

void printUsage() {
  std::fputs(
      "usage: warehousesim_planning_bench [--quick] [--max-size N] [--csv]\n"
      "  --quick       tiny maps and few samples: a fast correctness-and-smoke run\n"
      "  --max-size N  largest map side to benchmark (default 512)\n"
      "  --csv         machine-readable output\n",
      stderr);
}

}  // namespace

int main(int argc, char** argv) {
  bool quick = false;
  bool csv = false;
  std::uint64_t maxSize = 512;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (std::strcmp(argv[i], "--quick") == 0) {
      quick = true;
    } else if (std::strcmp(argv[i], "--csv") == 0) {
      csv = true;
    } else if (!warehousesim::apps::readUnsignedOption(argc, argv, i, "--max-size", maxSize,
                                                       failed)) {
      failed = true;
    }
  }
  if (failed || maxSize < 8 || maxSize > 4096) {
    printUsage();
    return 2;
  }

  Settings settings;
  std::vector<std::int32_t> sizes = {16, 32, 64, 128, 256, 512};
  if (quick) {
    settings = {3, 0.1};
    sizes = {16, 32};
  }

#ifdef NDEBUG
  const char* build = "optimised (NDEBUG)";
#else
  const char* build = "unoptimised/debug - timings are NOT representative";
#endif
  if (!csv) {
    std::printf("A* planning benchmark, %s build, single thread, median of %d samples\n", build,
                settings.samples);
  }
  printHeader(csv);
  for (const std::int32_t size : sizes) {
    if (static_cast<std::uint64_t>(size) > maxSize) {
      break;
    }
    for (const Scenario& scenario : makeScenarios(size)) {
      const std::optional<Row> row = run(scenario, size, settings);
      if (!row) {
        return 1;
      }
      printRow(*row, csv);
    }
  }
  return 0;
}
