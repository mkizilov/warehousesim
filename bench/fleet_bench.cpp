// Headless fleet benchmark: simulates every combination of layout, fleet size and seed for a fixed
// number of ticks, measures each run, and prints one table that compares them.
//
//   warehousesim_fleet_bench [--layouts a,b] [--fleets 1,2,4] [--seeds 1,2,3] [--ticks N]
//                            [--spawn N] [--seconds-per-tick X] [--csv FILE] [--quick]
//
// Everything printed is SIMULATED: ticks of a model, cells of a grid and units of an abstract
// energy model, a property of this simulation and its seeds. None of it says how a real robot or a
// real warehouse would perform. The runs are deterministic: the same options always give the same
// numbers (it checks this by running the first case twice), and two robots overlapping or changing
// places in any run fails the benchmark.
//
// Exit status: 0 on success, 1 if a check failed or a file could not be written, 2 for bad options.
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "arguments.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/simulation/benchmark.hpp"

namespace {

using warehousesim::simulation::BenchmarkSpec;

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_fleet_bench [--layouts a,b] [--fleets 1,2,4] [--seeds 1,2,3]\n"
         "                                [--ticks N] [--spawn N] [--seconds-per-tick X]\n"
         "                                [--csv FILE] [--quick]\n"
         "  --layouts   comma-separated layouts: "
      << warehousesim::apps::layoutNames()
      << " (default: all)\n"
         "  --fleets    fleet sizes to compare (default 1,2,4,8)\n"
         "  --seeds     seeds to pool for every layout and fleet size (default 1,2,3)\n"
         "  --ticks     simulated ticks per run (default 3000)\n"
         "  --spawn     a new random task every N ticks (default 5; 0 = none)\n"
         "  --seconds-per-tick  what a tick stands for when turning ticks into hours (default 1)\n"
         "  --csv FILE  also write one row per run (layout, fleet, seed) to FILE\n"
         "  --quick     a tiny sweep for smoke tests: 150 ticks, fleets 1,2, seed 1\n";
}

std::vector<std::string> splitList(const std::string& text) {
  std::vector<std::string> items;
  std::string item;
  for (const char c : text) {
    if (c == ',') {
      items.push_back(item);
      item.clear();
    } else {
      item += c;
    }
  }
  items.push_back(item);
  return items;
}

// Every item must be a whole number; nullopt otherwise (including an empty item).
std::optional<std::vector<std::uint64_t>> parseNumbers(const std::string& text) {
  std::vector<std::uint64_t> numbers;
  for (const std::string& item : splitList(text)) {
    const auto value = warehousesim::apps::parseUnsigned(item);
    if (!value) {
      return std::nullopt;
    }
    numbers.push_back(*value);
  }
  return numbers;
}

}  // namespace

int main(int argc, char** argv) {
  BenchmarkSpec spec;
  std::string layouts;
  std::string fleets;
  std::string seeds;
  std::string csvFile;
  std::uint64_t ticks = 0;
  bool ticksGiven = false;
  std::uint64_t spawn = 0;
  bool spawnGiven = false;
  double secondsPerTick = spec.metrics.secondsPerTick;
  bool quick = false;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (std::string_view(argv[i]) == "--quick") {
      quick = true;
      continue;
    }
    ticksGiven = ticksGiven || std::string_view(argv[i]) == "--ticks";
    spawnGiven = spawnGiven || std::string_view(argv[i]) == "--spawn";
    if (warehousesim::apps::readStringOption(argc, argv, i, "--layouts", layouts, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--fleets", fleets, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--seeds", seeds, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--csv", csvFile, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--ticks", ticks, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--spawn", spawn, failed) ||
        warehousesim::apps::readDoubleOption(argc, argv, i, "--seconds-per-tick", secondsPerTick,
                                             failed)) {
      continue;
    }
    failed = true;
  }
  if (failed) {
    printUsage(std::cerr);
    return 2;
  }

  if (quick) {
    spec.ticks = 150;
    spec.fleetSizes = {1, 2};
    spec.seeds = {1};
  }
  if (ticksGiven) {
    spec.ticks = ticks;
  }
  if (!layouts.empty()) {
    spec.layouts = splitList(layouts);
  }
  if (!fleets.empty()) {
    const auto numbers = parseNumbers(fleets);
    if (!numbers) {
      std::cerr << "error: --fleets must be whole numbers separated by commas\n";
      printUsage(std::cerr);
      return 2;
    }
    spec.fleetSizes.assign(numbers->begin(), numbers->end());
  }
  if (!seeds.empty()) {
    const auto numbers = parseNumbers(seeds);
    if (!numbers) {
      std::cerr << "error: --seeds must be whole numbers separated by commas\n";
      printUsage(std::cerr);
      return 2;
    }
    spec.seeds = *numbers;
  }
  if (spawnGiven) {
    spec.taskSpawnInterval = spawn;
  }
  spec.metrics.secondsPerTick = secondsPerTick;

  try {
    spec.metrics.validate();
    std::cout << "WarehouseSim fleet benchmark - SIMULATED results\n"
              << "  " << warehousesim::metrics::kDisclaimer << "\n"
              << "  " << spec.layouts.size() << " layouts x " << spec.fleetSizes.size()
              << " fleet sizes x " << spec.seeds.size() << " seeds, " << spec.ticks
              << " ticks each, 1 tick = " << spec.metrics.secondsPerTick
              << " s of simulated time\n\n";

    const auto runs = warehousesim::simulation::runBenchmark(spec);
    std::cout << warehousesim::simulation::markdownTable(warehousesim::simulation::pool(runs))
              << "\nbacklog = tasks still open at the end of a run (mean over seeds); mean done = "
                 "from submission to delivery, so it includes the wait in the queue.\n";

    if (!csvFile.empty()) {
      std::vector<std::pair<warehousesim::metrics::RunLabel, warehousesim::metrics::FleetMetrics>>
          rows;
      rows.reserve(runs.size());
      for (const auto& run : runs) {
        rows.emplace_back(run.label, run.metrics);
      }
      std::ofstream out(csvFile, std::ios::binary | std::ios::trunc);
      out << warehousesim::metrics::runsCsv(rows);
      out.flush();
      if (!out) {
        std::cerr << "error: could not write " << csvFile << "\n";
        return 1;
      }
      std::cout << "wrote " << runs.size() << " rows to " << csvFile << "\n";
    }

    std::vector<std::string> problems = warehousesim::simulation::checkRuns(runs);
    const std::vector<std::string> unstable = warehousesim::simulation::checkDeterminism(spec);
    problems.insert(problems.end(), unstable.begin(), unstable.end());
    for (const std::string& problem : problems) {
      std::cerr << "CHECK FAILED: " << problem << "\n";
    }
    if (!problems.empty()) {
      return 1;
    }
    std::cout << "checks passed: " << runs.size()
              << " runs, no robot overlapped or swapped places, every task accounted for, the "
                 "first case reproduced exactly\n";
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
