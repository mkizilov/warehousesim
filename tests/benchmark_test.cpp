#include "warehousesim/simulation/benchmark.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/metrics/export.hpp"
#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/scenarios.hpp"

namespace {

using warehousesim::metrics::Activity;
using warehousesim::metrics::FleetMetrics;
using warehousesim::simulation::BenchmarkCell;
using warehousesim::simulation::benchmarkColumns;
using warehousesim::simulation::BenchmarkRun;
using warehousesim::simulation::BenchmarkSpec;
using warehousesim::simulation::checkDeterminism;
using warehousesim::simulation::checkRuns;
using warehousesim::simulation::markdownTable;
using warehousesim::simulation::pool;
using warehousesim::simulation::runBenchmark;
using warehousesim::simulation::runBenchmarkCase;

BenchmarkSpec small() {
  BenchmarkSpec spec;
  spec.layouts = {"demo", "compact"};
  spec.fleetSizes = {1, 3};
  spec.seeds = {1, 2};
  spec.ticks = 120;
  return spec;
}

std::vector<std::string> rows(const std::string& table) {
  std::vector<std::string> result;
  std::istringstream in(table);
  for (std::string line; std::getline(in, line);) {
    result.push_back(line);
  }
  return result;
}

std::size_t countBars(const std::string& line) {
  std::size_t bars = 0;
  for (const char c : line) {
    bars += c == '|' ? 1 : 0;
  }
  return bars;
}

// ---- running ------------------------------------------------------------------------------------

TEST(BenchmarkTest, TheRunsAreEveryCombinationInTheOrderLayoutThenFleetThenSeed) {
  const auto runs = runBenchmark(small());
  ASSERT_EQ(runs.size(), 8U);
  std::vector<std::string> seen;
  for (const BenchmarkRun& run : runs) {
    seen.push_back(run.label.layout + "/" + std::to_string(run.metrics.robots.size()) + "/" +
                   std::to_string(run.label.seed));
  }
  EXPECT_EQ(seen,
            (std::vector<std::string>{"demo/1/1", "demo/1/2", "demo/3/1", "demo/3/2", "compact/1/1",
                                      "compact/1/2", "compact/3/1", "compact/3/2"}));
}

TEST(BenchmarkTest, EveryRunCoversExactlyTheTicksAskedFor) {
  for (const BenchmarkRun& run : runBenchmark(small())) {
    EXPECT_EQ(run.metrics.ticks, 120U);
    EXPECT_EQ(run.metrics.robotTicks(), 120U * run.metrics.robots.size());
  }
}

TEST(BenchmarkTest, ARunIsWhatAMeasuredSimulationOfThatScenarioGives) {
  BenchmarkSpec spec = small();
  const BenchmarkRun run = runBenchmarkCase(spec, "compact", 3, 2);
  warehousesim::simulation::ScenarioOptions options;
  options.robots = 3;
  auto sim = (*warehousesim::simulation::layoutScenario("compact", options))(2);
  warehousesim::simulation::MetricsCollector collector(sim, spec.metrics);
  for (int i = 0; i < 120; ++i) {
    sim.step();
    collector.observe(sim);
  }
  EXPECT_EQ(warehousesim::metrics::runsCsvRow(run.label, run.metrics),
            warehousesim::metrics::runsCsvRow(run.label, collector.report()));
  EXPECT_EQ(run.label.layout, "compact");
  EXPECT_EQ(run.label.seed, 2U);
  EXPECT_EQ(run.label.taskSpawnInterval, 5U);
}

TEST(BenchmarkTest, TheSameSpecGivesTheSameRunsEveryTime) {
  const auto first = runBenchmark(small());
  const auto second = runBenchmark(small());
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(warehousesim::metrics::runsCsvRow(first[i].label, first[i].metrics),
              warehousesim::metrics::runsCsvRow(second[i].label, second[i].metrics));
  }
}

TEST(BenchmarkTest, TheTaskRateAndTheTimeScaleAreTheSpecsOwn) {
  BenchmarkSpec spec = small();
  spec.layouts = {"demo"};
  spec.fleetSizes = {2};
  spec.seeds = {1};
  spec.taskSpawnInterval = 20;
  spec.metrics.secondsPerTick = 2.0;
  const BenchmarkRun run = runBenchmarkCase(spec, "demo", 2, 1);
  EXPECT_EQ(run.label.taskSpawnInterval, 20U);
  EXPECT_EQ(run.metrics.tasksSubmitted, 6U);  // 120 ticks, one every 20: 0, 20, ..., 100
  EXPECT_EQ(run.metrics.options.secondsPerTick, 2.0);
  spec.taskSpawnInterval = 0;
  EXPECT_EQ(runBenchmarkCase(spec, "demo", 2, 1).metrics.tasksSubmitted, 0U);
}

TEST(BenchmarkTest, MoreRobotsAreOfferedTheSameWorkSoTheComparisonIsFair) {
  BenchmarkSpec spec = small();
  spec.layouts = {"demo"};
  spec.seeds = {5};
  const auto one = runBenchmarkCase(spec, "demo", 1, 5);
  const auto three = runBenchmarkCase(spec, "demo", 3, 5);
  EXPECT_EQ(one.metrics.tasksSubmitted, three.metrics.tasksSubmitted);
}

// ---- refusing a bad spec ------------------------------------------------------------------------

TEST(BenchmarkTest, ASpecWithNothingToRunIsRefused) {
  BenchmarkSpec spec = small();
  spec.layouts.clear();
  EXPECT_THROW((void)runBenchmark(spec), std::invalid_argument);
  spec = small();
  spec.fleetSizes.clear();
  EXPECT_THROW((void)runBenchmark(spec), std::invalid_argument);
  spec = small();
  spec.seeds.clear();
  EXPECT_THROW((void)runBenchmark(spec), std::invalid_argument);
  spec = small();
  spec.ticks = 0;
  EXPECT_THROW((void)runBenchmark(spec), std::invalid_argument);
  EXPECT_THROW((void)checkDeterminism(spec), std::invalid_argument);
}

TEST(BenchmarkTest, AnUnknownLayoutIsNamedInTheError) {
  BenchmarkSpec spec = small();
  spec.layouts = {"demo", "nowhere"};
  try {
    (void)runBenchmark(spec);
    FAIL() << "should have been refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("nowhere"), std::string::npos) << error.what();
  }
}

TEST(BenchmarkTest, AFleetTheLayoutHasNoRoomForIsRefusedWithTheReason) {
  BenchmarkSpec spec = small();
  spec.fleetSizes = {1000};
  EXPECT_THROW((void)runBenchmark(spec), std::invalid_argument);
  spec.fleetSizes = {0};
  EXPECT_THROW((void)runBenchmark(spec), std::invalid_argument);
}

TEST(BenchmarkTest, ABadTimeScaleIsRefused) {
  BenchmarkSpec spec = small();
  spec.metrics.secondsPerTick = 0.0;
  EXPECT_THROW((void)runBenchmark(spec), std::invalid_argument);
}

// ---- pooling and the table ----------------------------------------------------------------------

TEST(BenchmarkPoolTest, OneCellPerLayoutAndFleetInTheOrderTheyFirstAppear) {
  const auto cells = pool(runBenchmark(small()));
  ASSERT_EQ(cells.size(), 4U);
  EXPECT_EQ(cells[0].layout, "demo");
  EXPECT_EQ(cells[0].robots, 1U);
  EXPECT_EQ(cells[1].layout, "demo");
  EXPECT_EQ(cells[1].robots, 3U);
  EXPECT_EQ(cells[2].layout, "compact");
  EXPECT_EQ(cells[3].robots, 3U);
  for (const BenchmarkCell& cell : cells) {
    EXPECT_EQ(cell.runs, 2U);
    EXPECT_EQ(cell.metrics.ticks, 240U);  // two seeds of 120
  }
}

TEST(BenchmarkPoolTest, ACellIsTheSumOfItsRunsSoItsFiguresArePooledNotAveraged) {
  const auto runs = runBenchmark(small());
  const auto cells = pool(runs);
  EXPECT_EQ(cells[0].metrics.tasksCompleted,
            runs[0].metrics.tasksCompleted + runs[1].metrics.tasksCompleted);
  EXPECT_EQ(cells[0].metrics.cellsTravelled(),
            runs[0].metrics.cellsTravelled() + runs[1].metrics.cellsTravelled());
  EXPECT_EQ(cells[0].metrics.completionTicksTotal,
            runs[0].metrics.completionTicksTotal + runs[1].metrics.completionTicksTotal);
  EXPECT_EQ(cells[0].metrics.ticksIn(Activity::Moving),
            runs[0].metrics.ticksIn(Activity::Moving) + runs[1].metrics.ticksIn(Activity::Moving));
}

TEST(BenchmarkPoolTest, NoRunsMakeNoCellsAndALoneRunIsItsOwnCell) {
  EXPECT_TRUE(pool({}).empty());
  const auto runs = runBenchmark([] {
    BenchmarkSpec spec = small();
    spec.layouts = {"demo"};
    spec.fleetSizes = {2};
    spec.seeds = {7};
    return spec;
  }());
  const auto cells = pool(runs);
  ASSERT_EQ(cells.size(), 1U);
  EXPECT_EQ(cells[0].runs, 1U);
  EXPECT_EQ(cells[0].metrics.tasksCompleted, runs[0].metrics.tasksCompleted);
}

TEST(BenchmarkTableTest, ItIsAMarkdownTableWithAHeaderARuleAndARowPerCell) {
  const auto cells = pool(runBenchmark(small()));
  const auto text = rows(markdownTable(cells));
  ASSERT_EQ(text.size(), 2U + cells.size());
  const std::size_t columns = benchmarkColumns().size();
  for (const std::string& line : text) {
    EXPECT_EQ(countBars(line), columns + 1) << line;
    EXPECT_EQ(line.front(), '|');
    EXPECT_EQ(line.back(), '|');
  }
  EXPECT_NE(text[0].find("| layout |"), std::string::npos);
  EXPECT_NE(text[0].find("tasks/h"), std::string::npos);
  EXPECT_NE(text[1].find("---"), std::string::npos);
  EXPECT_NE(text[2].find("| demo | 1 | 2 |"), std::string::npos) << text[2];
}

TEST(BenchmarkTableTest, TextColumnsAreLeftAlignedAndNumbersAreRightAligned) {
  const auto text = rows(markdownTable(pool(runBenchmark(small()))));
  const std::string rule = text[1];
  EXPECT_EQ(rule.find("| --- | --- | ---: |"), 0U) << rule;  // layout, robots, then the numbers
  std::size_t aligned = 0;
  for (std::size_t at = rule.find("---:"); at != std::string::npos;
       at = rule.find("---:", at + 1)) {
    ++aligned;
  }
  EXPECT_EQ(aligned, benchmarkColumns().size() - 2);
}

TEST(BenchmarkTableTest, NoCellsIsJustTheHeader) { EXPECT_EQ(rows(markdownTable({})).size(), 2U); }

TEST(BenchmarkTableTest, AFigureThatIsUndefinedReadsNotAvailable) {
  BenchmarkCell empty;
  empty.layout = "x";
  empty.robots = 0;
  empty.runs = 0;
  const std::string row = rows(markdownTable({empty}))[2];
  EXPECT_NE(row.find("n/a"), std::string::npos) << row;
  EXPECT_EQ(row.find("nan"), std::string::npos);
  EXPECT_EQ(row.find("inf"), std::string::npos);
}

TEST(BenchmarkTableTest, TheNumbersAreTheOnesOfThePooledMetricsToOneDecimal) {
  const auto cells = pool(runBenchmark(small()));
  const FleetMetrics& m = cells[1].metrics;
  std::ostringstream tasksPerHour;
  tasksPerHour.precision(1);
  tasksPerHour << std::fixed << *m.tasksPerHour();
  EXPECT_NE(rows(markdownTable(cells))[3].find("| " + tasksPerHour.str() + " |"), std::string::npos)
      << rows(markdownTable(cells))[3];
}

TEST(BenchmarkTableTest, TheBacklogIsTheMeanNumberOfOpenTasksPerRunNotTheTotal) {
  auto cells = pool(runBenchmark(small()));
  cells[0].runs = 4;
  cells[0].metrics.tasksOpen = 10;
  const auto columns = benchmarkColumns();
  std::size_t backlog = columns.size();
  for (std::size_t i = 0; i < columns.size(); ++i) {
    if (columns[i] == "backlog") {
      backlog = i;
    }
  }
  ASSERT_LT(backlog, columns.size());
  std::vector<std::string> fields;
  std::istringstream in(rows(markdownTable(cells))[2]);
  for (std::string field; std::getline(in, field, '|');) {
    fields.push_back(field);
  }
  ASSERT_GT(fields.size(), backlog + 1);
  EXPECT_EQ(fields[backlog + 1],
            " 2.5 ");  // 10 open tasks over 4 runs; fields[0] is before the first bar
}

// ---- the checks that make it a benchmark and not just a printout --------------------------------

TEST(BenchmarkCheckTest, AnHonestSweepPassesEveryCheck) {
  EXPECT_TRUE(checkRuns(runBenchmark(small())).empty());
  EXPECT_TRUE(checkDeterminism(small()).empty());
}

TEST(BenchmarkCheckTest, ACollisionFailsTheCheckAndNamesTheRun) {
  auto runs = runBenchmark(small());
  runs[2].metrics.collisionsOccurred = 2;
  const auto problems = checkRuns(runs);
  ASSERT_EQ(problems.size(), 1U);
  EXPECT_NE(problems[0].find("demo x3 seed 1"), std::string::npos) << problems[0];
  EXPECT_NE(problems[0].find("2 collisions"), std::string::npos);
}

TEST(BenchmarkCheckTest, ATaskThatIsNowhereFailsTheCheck) {
  auto runs = runBenchmark(small());
  runs[0].metrics.tasksOpen += 1;
  const auto problems = checkRuns(runs);
  ASSERT_EQ(problems.size(), 1U);
  EXPECT_NE(problems[0].find("tasks do not add up"), std::string::npos) << problems[0];
}

TEST(BenchmarkCheckTest, ACancelledTaskIsSomewhereAndSoCountsAsAccountedFor) {
  auto runs = runBenchmark(small());
  runs[0].metrics.tasksSubmitted += 1;
  runs[0].metrics.tasksCancelled += 1;
  EXPECT_TRUE(checkRuns(runs).empty());
  runs[0].metrics.tasksCancelled -= 1;  // now it is nowhere
  const auto problems = checkRuns(runs);
  ASSERT_EQ(problems.size(), 1U);
  EXPECT_NE(problems[0].find("tasks do not add up"), std::string::npos) << problems[0];
}

TEST(BenchmarkCheckTest, ARobotNotAccountedForInEveryTickFailsTheCheck) {
  auto runs = runBenchmark(small());
  runs[1].metrics.robots[0].budget.ticks[0] += 1;
  const auto problems = checkRuns(runs);
  ASSERT_EQ(problems.size(), 1U);
  EXPECT_NE(problems[0].find("robot 1 was accounted for in 121 of 120 ticks"), std::string::npos)
      << problems[0];
}

TEST(BenchmarkCheckTest, SeveralProblemsAreAllReported) {
  auto runs = runBenchmark(small());
  runs[0].metrics.collisionsOccurred = 1;
  runs[3].metrics.collisionsOccurred = 1;
  runs[3].metrics.tasksCompleted += 1;
  EXPECT_EQ(checkRuns(runs).size(), 3U);
}

TEST(BenchmarkCheckTest, NoRunsHaveNoProblems) { EXPECT_TRUE(checkRuns({}).empty()); }

}  // namespace
