#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/metrics/fleet_metrics.hpp"

namespace warehousesim::simulation {

// A headless benchmark: every combination of layout, fleet size and seed is simulated for the same
// number of ticks with the same task rate, and measured by a MetricsCollector. Nothing in it reads
// a clock or a random source of its own: the same spec always gives the same runs, which is what
// makes the numbers a baseline that later changes can be compared with.
//
// Everything it reports is simulated: ticks of a model, cells of a grid, units of an abstract
// energy model. A baseline is a statement about this simulation at this commit, not about any robot
// or warehouse in the world.
struct BenchmarkSpec {
  std::vector<std::string> layouts{"demo", "compact", "fulfilment"};
  std::vector<std::size_t> fleetSizes{1, 2, 4, 8};
  std::vector<std::uint64_t> seeds{1, 2, 3};
  core::Tick ticks = 3000;
  // A new random task every this many ticks; unset keeps the scenarios' own rate (5).
  std::optional<core::Tick> taskSpawnInterval;
  metrics::MetricsOptions metrics;
};

struct BenchmarkRun {
  metrics::RunLabel label;  // layout, seed, task interval
  metrics::FleetMetrics metrics;
};

// The runs of a spec in the order layout, fleet size, seed (each as listed). Throws
// std::invalid_argument, naming the culprit, for an unknown layout, a fleet the layout has no room
// for, no layouts, fleets or seeds at all, or no ticks.
[[nodiscard]] std::vector<BenchmarkRun> runBenchmark(const BenchmarkSpec& spec);
// One run.
[[nodiscard]] BenchmarkRun runBenchmarkCase(const BenchmarkSpec& spec, const std::string& layout,
                                            std::size_t robots, std::uint64_t seed);

// What the runs of one layout and fleet size come to over all their seeds: the counts added up,
// so that every figure is the pooled one (total over total, not a mean of means).
struct BenchmarkCell {
  std::string layout;
  std::size_t robots = 0;
  std::size_t runs = 0;  // seeds pooled
  metrics::FleetMetrics metrics;
};

// In the order the layout and fleet size first appear in `runs`.
[[nodiscard]] std::vector<BenchmarkCell> pool(const std::vector<BenchmarkRun>& runs);

// The cells as a GitHub-flavoured Markdown table, one row per layout and fleet size, with the
// columns spelled out in kBenchmarkColumns. Undefined figures read "n/a".
[[nodiscard]] std::vector<std::string> benchmarkColumns();
[[nodiscard]] std::string markdownTable(const std::vector<BenchmarkCell>& cells);

// Things that must hold of any set of runs, whatever they measured; empty if they do. Each
// problem names the run. Checked: no robots ever overlapped or changed places, every task is
// accounted for (completed + failed + cancelled + open = submitted), and every robot has exactly
// one activity for every tick.
[[nodiscard]] std::vector<std::string> checkRuns(const std::vector<BenchmarkRun>& runs);
// Runs the first case of the spec twice and says so if the two give different results.
[[nodiscard]] std::vector<std::string> checkDeterminism(const BenchmarkSpec& spec);

}  // namespace warehousesim::simulation
