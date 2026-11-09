// Performance regression benchmark: a fixed set of scenarios (layout, fleet, task load, seed) run
// through the engine and compared with a committed baseline.
//
//   warehousesim_regression_bench [--quick] [--repeats N] [--baseline FILE]
//                                 [--write-baseline FILE]
//                                 [--save-times FILE] [--times FILE] [--max-slowdown X]
//
// What is compared with the baseline file is exact and does not depend on the machine:
//   * behaviour: tasks completed and failed, cooperative-search work and the state checksum must be
//     identical (a change that moves them changed what the simulation computes, which has to be a
//     decision: regenerate the baseline with --write-baseline and say why);
//   * allocations: heap allocations per step may not grow by more than 10% (+8) (counted by
//     replacing operator new, so not available, and not checked, in sanitizer builds, nor with a
//     standard library other than libc++, which the baseline was measured with).
// Wall-clock time is printed but is not in the baseline file, because it is a property of a
// machine. To catch a slowdown, save the times of a build you trust (--save-times) and compare
// later builds on the same machine (--times, failing past --max-slowdown, default 1.5).
//
// Exit status: 0 if all checks pass, 1 if one failed, 2 for bad options.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>
#include <version>

#include "arguments.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/testing/allocation_counter.hpp"

namespace {

namespace sim = warehousesim::simulation;
using Clock = std::chrono::steady_clock;

#ifndef WAREHOUSESIM_REGRESSION_BASELINE
#define WAREHOUSESIM_REGRESSION_BASELINE "regression_baseline.txt"
#endif

// How many times the standard library allocates depends on the library (growth factors, the size of
// a small string), and the baseline was measured with libc++; with another library the column is
// shown but not compared.
#if defined(_LIBCPP_VERSION)
constexpr bool kAllocationsComparable = true;
#else
constexpr bool kAllocationsComparable = false;
#endif

struct Case {
  const char* name;
  const char* layout;
  std::size_t robots;
  std::uint64_t spawn;
  std::uint64_t ticks;
  std::uint64_t seed;
  bool inQuick;
};

// The first three are small enough for every test run; the large hall is where the engine's cost
// concentrates (hundreds of robots planning against each other's reservations).
constexpr Case kCases[] = {
    {"demo-x4", "demo", 4, 5, 3000, 1, true},
    {"compact-x20", "compact", 20, 2, 3500, 5, true},
    {"fulfilment-x24", "fulfilment", 24, 1, 1500, 3, true},
    {"large-x100", "large", 100, 1, 1500, 1, false},
    {"large-x256", "large", 256, 1, 800, 1, false},
};

// Steps run before measuring starts, so that buffers have grown to the size they stay at.
constexpr std::uint64_t kWarmupSteps = 200;

struct Result {
  std::string name;
  std::uint64_t measuredSteps = 0;
  double msPerStep = 0.0;
  double allocationsPerStep = 0.0;
  std::uint64_t completed = 0;
  std::uint64_t failed = 0;
  std::uint64_t searches = 0;
  std::uint64_t work = 0;
  std::uint64_t checksum = 0;
};

Result measure(const Case& c, int repeats) {
  Result result;
  result.name = c.name;
  std::vector<double> times;
  for (int repeat = 0; repeat < repeats; ++repeat) {
    sim::ScenarioOptions options;
    options.robots = c.robots;
    options.taskSpawnInterval = c.spawn;
    sim::Simulation simulation = (*sim::layoutScenario(c.layout, options))(c.seed);
    simulation.run(kWarmupSteps);

    const std::uint64_t before = warehousesim::testing::allocationCount();
    const auto begin = Clock::now();
    simulation.run(c.ticks - kWarmupSteps);
    const double seconds = std::chrono::duration<double>(Clock::now() - begin).count();
    const std::uint64_t allocations = warehousesim::testing::allocationCount() - before;

    result.measuredSteps = c.ticks - kWarmupSteps;
    times.push_back(1000.0 * seconds / static_cast<double>(result.measuredSteps));
    if (repeat == 0) {
      const sim::Snapshot end = simulation.snapshot();
      result.allocationsPerStep =
          static_cast<double>(allocations) / static_cast<double>(result.measuredSteps);
      result.completed = end.completedTasks;
      result.failed = end.failedTasks;
      result.searches = end.planning.searches;
      result.work = end.planning.work;
      result.checksum = simulation.stateChecksum();
    }
  }
  std::sort(times.begin(), times.end());
  result.msPerStep = times[times.size() / 2];
  return result;
}

std::string hex(std::uint64_t value) {
  char text[32];
  std::snprintf(text, sizeof text, "%016llx", static_cast<unsigned long long>(value));
  return text;
}

// ---- files --------------------------------------------------------------------------------------

std::string baselineLine(const Result& r) {
  std::ostringstream line;
  line << r.name << ' ' << r.completed << ' ' << r.failed << ' ' << r.searches << ' ' << r.work
       << ' ' << hex(r.checksum) << ' '
       << static_cast<std::uint64_t>(std::llround(r.allocationsPerStep));
  return line.str();
}

std::map<std::string, Result> readBaseline(const std::string& file, bool& found) {
  std::map<std::string, Result> baseline;
  std::ifstream in(file);
  found = in.good();
  for (std::string line; std::getline(in, line);) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream fields(line);
    Result r;
    std::string checksum;
    std::uint64_t allocations = 0;
    if (fields >> r.name >> r.completed >> r.failed >> r.searches >> r.work >> checksum >>
        allocations) {
      r.checksum = std::stoull(checksum, nullptr, 16);
      r.allocationsPerStep = static_cast<double>(allocations);
      baseline[r.name] = r;
    }
  }
  return baseline;
}

std::map<std::string, double> readTimes(const std::string& file, bool& found) {
  std::map<std::string, double> times;
  std::ifstream in(file);
  found = in.good();
  std::string name;
  double ms = 0.0;
  while (in >> name >> ms) {
    times[name] = ms;
  }
  return times;
}

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_regression_bench [--quick] [--repeats N] [--baseline FILE]\n"
         "                                     [--write-baseline FILE] [--save-times FILE]\n"
         "                                     [--times FILE [--max-slowdown X]]\n"
         "  --quick           only the small scenarios (what the test suite runs)\n"
         "  --repeats N       runs of each scenario to take the median time of (default 3)\n"
         "  --baseline FILE   the file to compare with (default "
      << WAREHOUSESIM_REGRESSION_BASELINE
      << ")\n"
         "  --write-baseline  write the measured behaviour and allocations as the new baseline\n"
         "  --save-times      write the measured ms/step, to compare later builds with\n"
         "  --times FILE      fail if a scenario is slower than in FILE by more than "
         "--max-slowdown\n"
         "  --max-slowdown X  default 1.5\n";
}

}  // namespace

int main(int argc, char** argv) {
  bool quick = false;
  std::uint64_t repeats = 3;
  std::string baselineFile = WAREHOUSESIM_REGRESSION_BASELINE;
  std::string writeBaseline;
  std::string saveTimes;
  std::string timesFile;
  double maxSlowdown = 1.5;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (std::string_view(argv[i]) == "--quick") {
      quick = true;
      continue;
    }
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--repeats", repeats, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--baseline", baselineFile, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--write-baseline", writeBaseline,
                                             failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--save-times", saveTimes, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--times", timesFile, failed) ||
        warehousesim::apps::readDoubleOption(argc, argv, i, "--max-slowdown", maxSlowdown,
                                             failed)) {
      continue;
    }
    failed = true;
  }
  if (failed || repeats == 0 || repeats > 99 || !(maxSlowdown >= 1.0) ||
      (quick && !writeBaseline.empty())) {
    printUsage(std::cerr);
    return 2;
  }

  try {
    std::vector<Result> results;
    for (const Case& c : kCases) {
      if (!quick || c.inQuick) {
        results.push_back(measure(c, static_cast<int>(repeats)));
      }
    }

    std::cout
        << "WarehouseSim performance regression benchmark (" << (quick ? "quick" : "full")
        << ", median of " << repeats << " run" << (repeats == 1 ? "" : "s")
        << ")\n  Times are wall-clock times of this machine and build; the other columns are\n"
           "  exact. Nothing here says how a real robot or warehouse performs.\n\n"
           "| scenario | steps | ms / step | allocs / step | done | failed | work | checksum |\n"
           "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n";
    for (const Result& r : results) {
      char line[256];
      std::snprintf(line, sizeof line, "| %s | %llu | %.3f | %.1f | %llu | %llu | %llu | %s |\n",
                    r.name.c_str(), static_cast<unsigned long long>(r.measuredSteps), r.msPerStep,
                    r.allocationsPerStep, static_cast<unsigned long long>(r.completed),
                    static_cast<unsigned long long>(r.failed),
                    static_cast<unsigned long long>(r.work), hex(r.checksum).c_str());
      std::cout << line;
    }

    if (!writeBaseline.empty()) {
      std::ofstream out(writeBaseline, std::ios::trunc);
      out << "# Regenerate with warehousesim_regression_bench --write-baseline. Behaviour is "
             "exact; "
             "the\n"
             "# last column is heap allocations per step (0 when it could not be counted).\n"
             "# scenario completed failed searches work checksum allocations\n";
      for (const Result& r : results) {
        out << baselineLine(r) << '\n';
      }
      out.flush();
      if (!out) {
        std::cerr << "error: could not write " << writeBaseline << '\n';
        return 1;
      }
      std::cout << "\nwrote the baseline to " << writeBaseline << '\n';
    }
    if (!saveTimes.empty()) {
      std::ofstream out(saveTimes, std::ios::trunc);
      for (const Result& r : results) {
        out << r.name << ' ' << r.msPerStep << '\n';
      }
      out.flush();
      if (!out) {
        std::cerr << "error: could not write " << saveTimes << '\n';
        return 1;
      }
    }

    std::vector<std::string> problems;
    if (writeBaseline.empty()) {
      bool haveBaseline = false;
      const std::map<std::string, Result> baseline = readBaseline(baselineFile, haveBaseline);
      if (!haveBaseline) {
        problems.push_back("cannot read the baseline " + baselineFile);
      }
      for (const Result& r : results) {
        const auto found = baseline.find(r.name);
        if (found == baseline.end()) {
          if (haveBaseline) {
            problems.push_back(r.name + " is not in the baseline: run --write-baseline");
          }
          continue;
        }
        const Result& b = found->second;
        if (r.completed != b.completed || r.failed != b.failed || r.searches != b.searches ||
            r.work != b.work || r.checksum != b.checksum) {
          problems.push_back(r.name + ": behaviour changed (done " + std::to_string(r.completed) +
                             " vs " + std::to_string(b.completed) + ", failed " +
                             std::to_string(r.failed) + " vs " + std::to_string(b.failed) +
                             ", work " + std::to_string(r.work) + " vs " + std::to_string(b.work) +
                             ", checksum " + hex(r.checksum) + " vs " + hex(b.checksum) +
                             "); if that is intended, regenerate the baseline and say why");
        }
        if constexpr (kAllocationsComparable) {
          if (b.allocationsPerStep > 0.0 && r.allocationsPerStep > 0.0 &&
              r.allocationsPerStep > b.allocationsPerStep * 1.10 + 8.0) {
            problems.push_back(r.name + ": " + std::to_string(r.allocationsPerStep) +
                               " allocations a step, the baseline has " +
                               std::to_string(b.allocationsPerStep));
          }
        }
      }
    }
    if (!timesFile.empty()) {
      bool haveTimes = false;
      const std::map<std::string, double> times = readTimes(timesFile, haveTimes);
      if (!haveTimes) {
        problems.push_back("cannot read the saved times " + timesFile);
      }
      for (const Result& r : results) {
        const auto found = times.find(r.name);
        if (found != times.end() && r.msPerStep > found->second * maxSlowdown) {
          problems.push_back(r.name + ": " + std::to_string(r.msPerStep) + " ms a step, but " +
                             std::to_string(found->second) + " when the times were saved (limit " +
                             std::to_string(maxSlowdown) + "x)");
        }
      }
    }

    for (const std::string& problem : problems) {
      std::cerr << "REGRESSION: " << problem << '\n';
    }
    if (!problems.empty()) {
      return 1;
    }
    std::cout << "\nchecks passed: " << results.size() << " scenarios match the baseline"
              << (timesFile.empty() ? "" : " and the saved times") << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
