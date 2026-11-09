// Scaling benchmark: what planning on several threads does to the cost of a simulation step.
//
//   warehousesim_scaling_bench [--layout NAME] [--robots 100,128,256] [--spawn N] [--ticks N]
//                              [--threads 0,1,2] [--samples N] [--seed N] [--min-batch N]
//                              [--storms N] [--quick]
//
// Three measurements, each at every thread count (0 = everything on the calling thread):
//   1. whole steps of a big fleet: milliseconds per step, wall clock;
//   2. a layout change under a big fleet (an obstacle dropped on the busiest cell of the planned
//      routes), which makes every robot whose route it cuts ask for a new one at once;
//   3. raw batches of independent path questions, which is the most the pool can give;
//   4. what drawing a frame costs the main thread before a pixel is drawn (no window needed).
//
// The thread count never changes what is computed, and the benchmark checks that: the state
// checksum after each run, the routes of each batch, must be exactly the single-threaded ones, or
// it fails. Times are wall-clock times of this machine and build, medians of several samples; they
// depend on both, so quote them from an optimised build and say which machine. They are the cost of
// computing the simulation, not simulated results, and say nothing about real robots.
//
// Exit status: 0 on success, 1 if a check failed, 2 for bad options.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "arguments.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/planning/path_batch.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/view/fleet_controller.hpp"
#include "warehousesim/view/fleet_scene.hpp"
#include "warehousesim/view/interpolation.hpp"

namespace {

namespace sim = warehousesim::simulation;
using warehousesim::core::Point;
using Clock = std::chrono::steady_clock;

struct Options {
  std::string layout = "large";
  std::vector<std::uint64_t> robots{100, 128, 256};
  std::uint64_t spawn = 1;
  std::uint64_t ticks = 1500;
  std::vector<std::uint64_t> threads{0, 1, 2};
  std::uint64_t samples = 3;
  std::uint64_t seed = 1;
  std::uint64_t minBatch = 16;
  std::uint64_t storms = 12;
  std::uint64_t warmup = 300;
  std::uint64_t repeats = 200;
};

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_scaling_bench [--layout NAME] [--robots 100,128,256] [--spawn N]\n"
         "                                  [--ticks N] [--threads 0,1,2] [--samples N] [--seed "
         "N]\n"
         "                                  [--min-batch N] [--storms N] [--quick]\n"
         "  --layout     the hall (default large); layouts: "
      << warehousesim::apps::layoutNames()
      << "\n"
         "  --robots     fleet sizes to run (default 100,128,256)\n"
         "  --spawn      a new random task every N ticks (default 1: more work than the fleet "
         "does)\n"
         "  --ticks      steps per sample (default 1500)\n"
         "  --threads    planning threads besides the calling one, to compare (default 0,1,2)\n"
         "  --samples    runs per case; the median is reported (default 3)\n"
         "  --min-batch  batches with fewer questions than this stay on the calling thread (16)\n"
         "  --storms     obstacles dropped per sample in the layout-change measurement (default "
         "12)\n"
         "  --quick      a tiny run for smoke tests\n";
}

double seconds(Clock::time_point from, Clock::time_point to) {
  return std::chrono::duration<double>(to - from).count();
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return n % 2 == 1 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2.0;
}

std::string num(double value, int decimals) {
  char text[64];
  std::snprintf(text, sizeof text, "%.*f", decimals, value);
  return text;
}

void row(const std::vector<std::string>& cells) {
  std::cout << "|";
  for (const std::string& cell : cells) {
    std::cout << " " << cell << " |";
  }
  std::cout << "\n";
}

void header(const std::vector<std::string>& names) {
  row(names);
  std::cout << "|";
  for (std::size_t i = 0; i < names.size(); ++i) {
    std::cout << " ---: |";
  }
  std::cout << "\n";
}

std::string hex(std::uint64_t value) {
  char text[32];
  std::snprintf(text, sizeof text, "%016llx", static_cast<unsigned long long>(value));
  return text;
}

struct Failures {
  std::vector<std::string> messages;
  void add(std::string message) { messages.push_back(std::move(message)); }
};

sim::ScenarioFactory scenarioFor(const Options& options, std::uint64_t robots) {
  sim::ScenarioOptions scenario;
  scenario.robots = robots;
  scenario.taskSpawnInterval = options.spawn;
  auto factory = sim::layoutScenario(options.layout, scenario);
  if (!factory) {
    throw std::invalid_argument("unknown layout \"" + options.layout + "\"");
  }
  return *factory;
}

// ---- 1. whole steps --------------------------------------------------------------------------

struct StepSample {
  double msPerStep = 0.0;
  std::uint64_t checksum = 0;
  std::size_t completed = 0;
  std::optional<warehousesim::planning::PathBatchExecutor::Stats> batches;
};

StepSample timeSteps(const sim::ScenarioFactory& scenario, const Options& options,
                     std::uint64_t workers) {
  sim::Simulation simulation = scenario(options.seed);
  simulation.usePlanningThreads(workers, options.minBatch);
  const auto begin = Clock::now();
  for (std::uint64_t t = 0; t < options.ticks; ++t) {
    simulation.step();
  }
  const double elapsed = seconds(begin, Clock::now());
  return {1000.0 * elapsed / static_cast<double>(options.ticks), simulation.stateChecksum(),
          simulation.snapshot().completedTasks, simulation.batchStats()};
}

void measureSteps(const Options& options, Failures& failures) {
  std::cout << "\n### 1. Whole steps: " << options.layout << ", " << options.ticks
            << " steps per sample, a task every " << options.spawn << " tick(s), seed "
            << options.seed << "\n\n";
  header({"robots", "planning threads", "ms / step", "best ms / step", "speed-up", "batches / step",
          "questions / batch", "tasks done", "state checksum"});
  for (const std::uint64_t robots : options.robots) {
    const sim::ScenarioFactory scenario = scenarioFor(options, robots);
    std::optional<double> baseline;
    std::optional<StepSample> reference;
    for (const std::uint64_t workers : options.threads) {
      std::vector<double> times;
      StepSample last;
      for (std::uint64_t s = 0; s < options.samples; ++s) {
        last = timeSteps(scenario, options, workers);
        times.push_back(last.msPerStep);
        if (reference && last.checksum != reference->checksum) {
          failures.add(options.layout + " x" + std::to_string(robots) + " with " +
                       std::to_string(workers) + " planning threads ended in state " +
                       hex(last.checksum) + ", not " + hex(reference->checksum) +
                       ": the thread count changed the run");
        }
        if (!reference) {
          reference = last;
        }
      }
      const double typical = median(times);
      if (!baseline) {
        baseline = typical;
      }
      const double perStep = last.batches ? static_cast<double>(last.batches->batches) /
                                                static_cast<double>(options.ticks)
                                          : 0.0;
      const double perBatch = last.batches && last.batches->batches > 0
                                  ? static_cast<double>(last.batches->queries) /
                                        static_cast<double>(last.batches->batches)
                                  : 0.0;
      row({std::to_string(robots), std::to_string(workers), num(typical, 3),
           num(*std::min_element(times.begin(), times.end()), 3), num(*baseline / typical, 2) + "x",
           last.batches ? num(perStep, 2) : "-", last.batches ? num(perBatch, 1) : "-",
           std::to_string(last.completed), hex(last.checksum)});
    }
  }
}

// ---- 2. an obstacle under a big fleet --------------------------------------------------------

// The free cells that the most planned routes still have to go through (cells a robot does not hold
// yet, and not the last cell of a route), busiest first, ties in row-major order.
std::vector<Point> busiestCells(const sim::Simulation& simulation, std::size_t limit) {
  const warehousesim::world::GridSize size = simulation.warehouse().size();
  std::vector<std::uint32_t> use(size.cellCount(), 0);
  for (const auto& robot : simulation.robots()) {
    if (robot.state() != warehousesim::robots::RobotState::Navigating) {
      continue;
    }
    for (std::size_t i = robot.authority(); i + 1 < robot.path().size(); ++i) {
      ++use[size.indexOf(robot.path()[i])];
    }
  }
  std::vector<std::size_t> order;
  for (std::size_t i = 0; i < use.size(); ++i) {
    if (use[i] > 0 &&
        simulation.warehouse().at(size.pointAt(i)) == warehousesim::world::CellType::Free) {
      order.push_back(i);
    }
  }
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return use[a] != use[b] ? use[a] > use[b] : a < b;
  });
  std::vector<Point> cells;
  for (std::size_t i = 0; i < std::min(limit, order.size()); ++i) {
    cells.push_back(size.pointAt(order[i]));
  }
  return cells;
}

struct StormSample {
  double msPerStorm = 0.0;
  double routesPerStorm = 0.0;
  std::uint64_t checksum = 0;
};

StormSample timeStorms(const sim::ScenarioFactory& scenario, const Options& options,
                       std::uint64_t workers) {
  sim::Simulation simulation = scenario(options.seed);
  simulation.usePlanningThreads(workers, options.minBatch);
  simulation.run(options.warmup);
  const std::uint64_t repairsBefore = simulation.planningStats().obstacleRepairs;
  double total = 0.0;
  std::uint64_t placed = 0;
  for (std::uint64_t round = 0; round < options.storms; ++round) {
    for (const Point cell : busiestCells(simulation, 24)) {
      const auto begin = Clock::now();
      const bool done = simulation.placeObstacle(cell);
      const auto end = Clock::now();
      if (done) {
        total += seconds(begin, end);
        ++placed;
        simulation.run(2);
        (void)simulation.clearObstacle(cell);
        break;
      }
    }
    simulation.run(4);
  }
  const std::uint64_t repaired = simulation.planningStats().obstacleRepairs - repairsBefore;
  return {placed == 0 ? 0.0 : 1000.0 * total / static_cast<double>(placed),
          placed == 0 ? 0.0 : static_cast<double>(repaired) / static_cast<double>(placed),
          simulation.stateChecksum()};
}

void measureStorms(const Options& options, Failures& failures) {
  std::cout << "\n### 2. A layout change under a big fleet: " << options.layout << ", "
            << options.storms << " obstacles dropped on the busiest cell, after " << options.warmup
            << " steps\n\n";
  header({"robots", "planning threads", "ms / obstacle", "speed-up", "routes repaired / obstacle",
          "state checksum"});
  for (const std::uint64_t robots : options.robots) {
    const sim::ScenarioFactory scenario = scenarioFor(options, robots);
    std::optional<double> baseline;
    std::optional<std::uint64_t> checksum;
    for (const std::uint64_t workers : options.threads) {
      std::vector<double> times;
      StormSample last;
      for (std::uint64_t s = 0; s < options.samples; ++s) {
        last = timeStorms(scenario, options, workers);
        times.push_back(last.msPerStorm);
        if (checksum && last.checksum != *checksum) {
          failures.add(options.layout + " x" + std::to_string(robots) + " with " +
                       std::to_string(workers) +
                       " planning threads gave another state after the obstacles than with " +
                       std::to_string(options.threads.front()));
        }
        checksum = checksum.value_or(last.checksum);
      }
      const double typical = median(times);
      baseline = baseline.value_or(typical);
      row({std::to_string(robots), std::to_string(workers), num(typical, 3),
           typical > 0.0 ? num(*baseline / typical, 2) + "x" : "-", num(last.routesPerStorm, 1),
           hex(last.checksum)});
    }
  }
}

// ---- 3. raw batches --------------------------------------------------------------------------

void measureBatches(const Options& options, Failures& failures) {
  using warehousesim::planning::PathAnswer;
  using warehousesim::planning::PathBatchExecutor;
  using warehousesim::planning::PathQuery;

  auto warehouse = warehousesim::world::makePredefinedWarehouse(options.layout);
  if (!warehouse) {
    return;
  }
  std::vector<Point> walkable;
  for (std::size_t i = 0; i < warehouse->size().cellCount(); ++i) {
    if (warehouse->isWalkable(warehouse->size().pointAt(i))) {
      walkable.push_back(warehouse->size().pointAt(i));
    }
  }
  std::cout << "\n### 3. Raw batches of independent path questions: " << options.layout << " ("
            << warehouse->width() << "x" << warehouse->height() << "), random start and goal, "
            << options.repeats << " batches per case\n\n";
  header({"questions / batch", "planning threads", "us / batch", "us / question", "speed-up",
          "same routes as 0 threads"});
  for (const std::size_t size :
       {std::size_t{16}, std::size_t{64}, std::size_t{256}, std::size_t{1024}}) {
    warehousesim::core::Rng rng(options.seed + size);
    std::vector<PathQuery> queries;
    for (std::size_t i = 0; i < size; ++i) {
      queries.push_back(
          {walkable[rng.below(walkable.size())], walkable[rng.below(walkable.size())]});
    }
    std::vector<PathAnswer> reference;
    std::optional<double> baseline;
    for (const std::uint64_t workers : options.threads) {
      PathBatchExecutor executor({static_cast<std::size_t>(workers), 1, 64});
      std::vector<PathAnswer> answers;
      // Warm-up: allocates the scratch memory, and gives the caches and the clock of the machine
      // time to settle, so that the first case measured is not the slow one.
      for (std::uint64_t r = 0; r < std::max<std::uint64_t>(20, options.repeats / 2); ++r) {
        executor.run(*warehouse, queries, answers, true);
      }
      std::vector<double> times;
      for (std::uint64_t r = 0; r < options.repeats; ++r) {
        const auto begin = Clock::now();
        executor.run(*warehouse, queries, answers, true);
        times.push_back(1e6 * seconds(begin, Clock::now()));
      }
      bool same = true;
      if (workers == options.threads.front()) {
        reference = answers;
      } else {
        for (std::size_t i = 0; i < size; ++i) {
          same = same && answers[i].status == reference[i].status &&
                 answers[i].cost == reference[i].cost && answers[i].path == reference[i].path &&
                 answers[i].expansions == reference[i].expansions;
        }
        if (!same) {
          failures.add("a batch of " + std::to_string(size) + " questions gave other routes with " +
                       std::to_string(workers) + " planning threads");
        }
      }
      const double typical = median(times);
      baseline = baseline.value_or(typical);
      row({std::to_string(size), std::to_string(workers), num(typical, 1),
           num(typical / static_cast<double>(size), 2), num(*baseline / typical, 2) + "x",
           same ? "yes" : "NO"});
    }
  }
}

// ---- 4. drawing a frame ----------------------------------------------------------------------

// What the main thread does for each frame before a pixel is drawn, for a busy fleet: take the last
// two states from the runner, blend them by how far real time has got between them, and turn the
// result into draw commands. Executing the commands is SDL's and cannot be measured without a
// window; the number of commands says how much that will be. The first column is what the stepping
// thread pays for each step to publish its state.
void measureFrames(const Options& options) {
  std::cout << "\n### 4. Drawing a frame: " << options.layout << ", " << options.warmup
            << " steps in, a 1280x720 window, routes and held cells shown, " << options.repeats
            << " frames per case\n\n";
  header({"robots", "snapshot us (per step, stepping thread)", "frame() us", "blend us", "scene us",
          "draw commands", "main thread, % of a 60 fps frame"});
  volatile std::size_t sink = 0;
  for (const std::uint64_t robots : options.robots) {
    sim::Simulation simulation = scenarioFor(options, robots)(options.seed);
    simulation.run(options.warmup);
    std::vector<double> snapshotTimes;
    for (std::uint64_t r = 0; r < options.repeats; ++r) {
      const auto begin = Clock::now();
      const sim::Snapshot snapshot = simulation.snapshot();
      snapshotTimes.push_back(1e6 * seconds(begin, Clock::now()));
      sink = sink + snapshot.robots.size();
    }

    // Not started: the picture is the state as it is, and no worker contends for the lock.
    sim::SimulationRunner runner(std::move(simulation), std::chrono::milliseconds(100));
    warehousesim::view::FleetController controller(warehousesim::view::ScreenSize{1280, 720},
                                                   runner.warehouse());
    std::vector<double> frameTimes;
    std::vector<double> blendTimes;
    std::vector<double> sceneTimes;
    std::size_t commands = 0;
    for (std::uint64_t r = 0; r < options.repeats; ++r) {
      const auto t0 = Clock::now();
      const sim::FrameState frame = runner.frame();
      const auto t1 = Clock::now();
      const sim::Snapshot drawn = warehousesim::view::blend(frame.previous, frame.latest, 0.5);
      const auto t2 = Clock::now();
      const warehousesim::view::DrawList scene =
          warehousesim::view::buildFleetScene(runner.warehouse(), drawn, controller);
      const auto t3 = Clock::now();
      frameTimes.push_back(1e6 * seconds(t0, t1));
      blendTimes.push_back(1e6 * seconds(t1, t2));
      sceneTimes.push_back(1e6 * seconds(t2, t3));
      commands = scene.commands.size();
      sink = sink + commands;
    }
    const double total = median(frameTimes) + median(blendTimes) + median(sceneTimes);
    row({std::to_string(robots), num(median(snapshotTimes), 1), num(median(frameTimes), 1),
         num(median(blendTimes), 1), num(median(sceneTimes), 1), std::to_string(commands),
         num(100.0 * total / (1e6 / 60.0), 2)});
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  bool quick = false;
  bool failed = false;
  std::string robots;
  std::string threads;
  std::uint64_t spawn = options.spawn;
  for (int i = 1; i < argc && !failed; ++i) {
    if (std::string_view(argv[i]) == "--quick") {
      quick = true;
      continue;
    }
    if (warehousesim::apps::readStringOption(argc, argv, i, "--layout", options.layout, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--robots", robots, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--threads", threads, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--spawn", spawn, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--ticks", options.ticks, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--samples", options.samples,
                                               failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--seed", options.seed, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--min-batch", options.minBatch,
                                               failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--storms", options.storms, failed)) {
      continue;
    }
    failed = true;
  }
  if (!robots.empty()) {
    const auto list = warehousesim::apps::parseUnsignedList(robots);
    failed = failed || !list;
    if (list) {
      options.robots = *list;
    }
  }
  if (!threads.empty()) {
    const auto list = warehousesim::apps::parseUnsignedList(threads);
    failed = failed || !list;
    if (list) {
      options.threads = *list;
    }
  }
  options.spawn = spawn;
  if (quick) {
    options.robots = {100};
    options.threads = {0, 2};
    options.ticks = 60;
    options.samples = 1;
    options.storms = 2;
    options.warmup = 60;
    options.repeats = 5;
  }
  failed = failed || options.samples == 0 || options.ticks == 0 || options.robots.empty() ||
           options.threads.empty() || options.minBatch == 0 ||
           *std::max_element(options.threads.begin(), options.threads.end()) > 64;
  if (failed) {
    printUsage(std::cerr);
    return 2;
  }

  Failures failures;
  try {
    std::cout << "WarehouseSim scaling benchmark - wall-clock cost of computing the simulation\n"
              << "  machine: " << std::thread::hardware_concurrency()
              << " hardware threads; build: "
#ifdef NDEBUG
              << "optimised (NDEBUG)"
#else
              << "NOT optimised: do not quote these numbers"
#endif
              << "\n  median of " << options.samples
              << " sample(s) per case; \"planning threads\" are besides the calling one\n";
    measureSteps(options, failures);
    measureStorms(options, failures);
    measureBatches(options, failures);
    measureFrames(options);
  } catch (const std::invalid_argument& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }
  std::cout << "\n";
  for (const std::string& message : failures.messages) {
    std::cerr << "CHECK FAILED: " << message << "\n";
  }
  if (!failures.messages.empty()) {
    return 1;
  }
  std::cout << "checks passed: every thread count gave exactly the single-threaded states and "
               "routes\n";
  return 0;
}
