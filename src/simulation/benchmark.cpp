#include "warehousesim/simulation/benchmark.hpp"

#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/scenarios.hpp"

namespace warehousesim::simulation {

namespace {

std::string fixed(double value, int decimals) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(decimals) << value;
  return out.str();
}

std::string cell(const std::optional<double>& value, int decimals) {
  return value ? fixed(*value, decimals) : std::string("n/a");
}

std::string percent(const std::optional<double>& fraction) {
  return fraction ? fixed(*fraction * 100.0, 1) : std::string("n/a");
}

std::string describe(const BenchmarkRun& run) {
  return run.label.layout + " x" + std::to_string(run.metrics.robots.size()) + " seed " +
         std::to_string(run.label.seed);
}

void requireSpec(const BenchmarkSpec& spec) {
  if (spec.layouts.empty() || spec.fleetSizes.empty() || spec.seeds.empty()) {
    throw std::invalid_argument("a benchmark needs at least one layout, fleet size and seed");
  }
  if (spec.ticks == 0) {
    throw std::invalid_argument("a benchmark needs at least one tick");
  }
}

}  // namespace

BenchmarkRun runBenchmarkCase(const BenchmarkSpec& spec, const std::string& layout,
                              std::size_t robots, std::uint64_t seed) {
  requireSpec(spec);
  ScenarioOptions options;
  options.robots = robots;
  options.taskSpawnInterval = spec.taskSpawnInterval;
  const std::optional<ScenarioFactory> scenario = layoutScenario(layout, options);
  if (!scenario) {
    throw std::invalid_argument("unknown layout \"" + layout + "\"");
  }
  Simulation simulation = (*scenario)(seed);
  MetricsCollector collector(simulation, spec.metrics);
  for (core::Tick tick = 0; tick < spec.ticks; ++tick) {
    simulation.step();
    collector.observe(simulation);
  }
  return {{layout, seed, simulation.config().taskSpawnInterval}, collector.report()};
}

std::vector<BenchmarkRun> runBenchmark(const BenchmarkSpec& spec) {
  requireSpec(spec);
  std::vector<BenchmarkRun> runs;
  for (const std::string& layout : spec.layouts) {
    for (const std::size_t robots : spec.fleetSizes) {
      for (const std::uint64_t seed : spec.seeds) {
        runs.push_back(runBenchmarkCase(spec, layout, robots, seed));
      }
    }
  }
  return runs;
}

std::vector<BenchmarkCell> pool(const std::vector<BenchmarkRun>& runs) {
  std::vector<BenchmarkCell> cells;
  for (const BenchmarkRun& run : runs) {
    const std::size_t robots = run.metrics.robots.size();
    auto found = std::find_if(cells.begin(), cells.end(), [&](const BenchmarkCell& c) {
      return c.layout == run.label.layout && c.robots == robots;
    });
    if (found == cells.end()) {
      cells.push_back({run.label.layout, robots, 1, run.metrics});
    } else {
      found->metrics.merge(run.metrics);
      ++found->runs;
    }
  }
  return cells;
}

std::vector<std::string> benchmarkColumns() {
  return {"layout",        "robots",     "runs",          "tasks/h",   "tasks/robot-h",
          "mean done (s)", "backlog",    "utilization %", "waiting %", "idle %",
          "charging %",    "cells/task", "conflicts/h",   "replans/h", "energy/task"};
}

std::string markdownTable(const std::vector<BenchmarkCell>& cells) {
  const std::vector<std::string> columns = benchmarkColumns();
  std::string text = "|";
  for (const std::string& name : columns) {
    text += " " + name + " |";
  }
  text += "\n|";
  for (std::size_t i = 0; i < columns.size(); ++i) {
    text += i < 2 ? " --- |" : " ---: |";
  }
  text += "\n";
  for (const BenchmarkCell& c : cells) {
    const metrics::FleetMetrics& m = c.metrics;
    // The backlog is the mean number of tasks still open at the end of a run.
    const std::string backlog =
        c.runs == 0 ? std::string("n/a")
                    : fixed(static_cast<double>(m.tasksOpen) / static_cast<double>(c.runs), 1);
    const std::vector<std::string> fields{c.layout,
                                          std::to_string(c.robots),
                                          std::to_string(c.runs),
                                          cell(m.tasksPerHour(), 1),
                                          cell(m.tasksPerRobotHour(), 1),
                                          cell(m.meanCompletionSeconds(), 1),
                                          backlog,
                                          percent(m.utilization()),
                                          percent(m.fraction(metrics::Activity::Waiting)),
                                          percent(m.fraction(metrics::Activity::Idle)),
                                          percent(m.fraction(metrics::Activity::Charging)),
                                          cell(m.cellsPerCompletedTask(), 1),
                                          cell(m.conflictsPreventedPerHour(), 1),
                                          cell(m.replansPerHour(), 1),
                                          cell(m.energyPerCompletedTask(), 1)};
    text += "|";
    for (const std::string& field : fields) {
      text += " " + field + " |";
    }
    text += "\n";
  }
  return text;
}

std::vector<std::string> checkRuns(const std::vector<BenchmarkRun>& runs) {
  std::vector<std::string> problems;
  for (const BenchmarkRun& run : runs) {
    const metrics::FleetMetrics& m = run.metrics;
    if (m.collisionsOccurred != 0) {
      problems.push_back(describe(run) + ": " + std::to_string(m.collisionsOccurred) +
                         " collisions (robots overlapped or changed places)");
    }
    if (m.tasksCompleted + m.tasksFailed + m.tasksCancelled + m.tasksOpen != m.tasksSubmitted) {
      problems.push_back(
          describe(run) + ": tasks do not add up (" + std::to_string(m.tasksCompleted) +
          " completed + " + std::to_string(m.tasksFailed) + " failed + " +
          std::to_string(m.tasksCancelled) + " cancelled + " + std::to_string(m.tasksOpen) +
          " open != " + std::to_string(m.tasksSubmitted) + " submitted)");
    }
    for (const metrics::RobotMetrics& robot : m.robots) {
      if (robot.budget.total() != m.ticks) {
        problems.push_back(describe(run) + ": robot " + std::to_string(robot.id.value) +
                           " was accounted for in " + std::to_string(robot.budget.total()) +
                           " of " + std::to_string(m.ticks) + " ticks");
      }
    }
  }
  return problems;
}

std::vector<std::string> checkDeterminism(const BenchmarkSpec& spec) {
  requireSpec(spec);
  const std::string& layout = spec.layouts.front();
  const std::size_t robots = spec.fleetSizes.front();
  const std::uint64_t seed = spec.seeds.front();
  const BenchmarkRun first = runBenchmarkCase(spec, layout, robots, seed);
  const BenchmarkRun second = runBenchmarkCase(spec, layout, robots, seed);
  std::vector<std::string> problems;
  if (metrics::runsCsvRow(first.label, first.metrics) !=
          metrics::runsCsvRow(second.label, second.metrics) ||
      metrics::robotsCsv(first.metrics) != metrics::robotsCsv(second.metrics) ||
      metrics::seriesCsv(first.metrics) != metrics::seriesCsv(second.metrics)) {
    problems.push_back(describe(first) + ": two runs of the same case gave different results");
  }
  return problems;
}

}  // namespace warehousesim::simulation
