#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "scratch_dir.hpp"
#include "warehousesim/metrics/accumulator.hpp"
#include "warehousesim/metrics/export.hpp"

namespace {

using warehousesim_test::readFile;
using warehousesim_test::ScratchDir;

using warehousesim::core::RobotId;
using warehousesim::metrics::Activity;
using warehousesim::metrics::csvField;
using warehousesim::metrics::dashboardLines;
using warehousesim::metrics::FleetMetrics;
using warehousesim::metrics::formatDuration;
using warehousesim::metrics::kDashboardColumns;
using warehousesim::metrics::kDisclaimer;
using warehousesim::metrics::MetricsSample;
using warehousesim::metrics::RobotMetrics;
using warehousesim::metrics::robotsCsv;
using warehousesim::metrics::runColumns;
using warehousesim::metrics::RunLabel;
using warehousesim::metrics::runsCsv;
using warehousesim::metrics::runsCsvHeader;
using warehousesim::metrics::runsCsvRow;
using warehousesim::metrics::seriesCsv;
using warehousesim::metrics::splitCsvLine;

std::vector<std::string> lines(const std::string& text) {
  std::vector<std::string> result;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    result.push_back(line);
  }
  return result;
}

// A run of two robots over one simulated hour at 1 s a tick, with round numbers so that every
// figure can be checked manually.
FleetMetrics oneHour() {
  FleetMetrics m;
  m.ticks = 3600;
  for (std::uint32_t i = 1; i <= 2; ++i) {
    RobotMetrics r;
    r.id = RobotId{i};
    m.robots.push_back(r);
  }
  const auto give = [&m](std::size_t robot, Activity a, std::uint64_t ticks) {
    m.robots[robot].budget.ticks[static_cast<std::size_t>(a)] = ticks;
  };
  give(0, Activity::Moving, 2000);
  give(0, Activity::Waiting, 400);
  give(0, Activity::Handling, 400);
  give(0, Activity::Charging, 300);
  give(0, Activity::Idle, 500);
  give(1, Activity::Moving, 1800);
  give(1, Activity::Waiting, 200);
  give(1, Activity::Handling, 600);
  give(1, Activity::Idle, 1000);
  m.robots[0].cellsTravelled = 700;
  m.robots[1].cellsTravelled = 500;
  m.robots[0].deliveries = 20;
  m.robots[1].deliveries = 10;
  m.robots[0].conflictsPrevented = 5;
  m.robots[1].conflictsPrevented = 2;
  m.tasksSubmitted = 40;
  m.tasksCompleted = 30;
  m.tasksFailed = 1;
  m.tasksCancelled = 2;
  m.tasksOpen = 7;
  m.completionTicksTotal = 30 * 200;
  m.completionTicksMax = 650;
  m.conflictsPrevented = 7;
  m.conflictDetections = 90;
  m.reroutes = 4;
  m.repaths = 3;
  m.obstacleRepairs = 1;
  m.searches = 20;
  m.options.energy = {1, 15, 2};
  return m;
}

std::map<std::string, std::string> row(const FleetMetrics& metrics, const RunLabel& label) {
  const auto names = runColumns();
  const auto fields = splitCsvLine(runsCsvRow(label, metrics));
  std::map<std::string, std::string> result;
  EXPECT_EQ(names.size(), fields.size());
  for (std::size_t i = 0; i < names.size() && i < fields.size(); ++i) {
    result[names[i]] = fields[i];
  }
  return result;
}

const RunLabel kLabel{"compact", 7, 5};

// ---- CSV syntax ---------------------------------------------------------------------------------

TEST(CsvFieldTest, PlainTextIsLeftAloneAndSpecialCharactersAreQuoted) {
  EXPECT_EQ(csvField(""), "");
  EXPECT_EQ(csvField("compact"), "compact");
  EXPECT_EQ(csvField("two words"), "two words");
  EXPECT_EQ(csvField("a,b"), "\"a,b\"");
  EXPECT_EQ(csvField("say \"hi\""), "\"say \"\"hi\"\"\"");
  EXPECT_EQ(csvField("line\nbreak"), "\"line\nbreak\"");
  EXPECT_EQ(csvField("carriage\rreturn"), "\"carriage\rreturn\"");
  EXPECT_EQ(csvField("\""), "\"\"\"\"");
}

TEST(CsvFieldTest, SplittingUndoesQuotingForAnyText) {
  for (const std::string& text :
       {std::string(""), std::string("x"), std::string("a,b,c"), std::string("\"\""),
        std::string("\"quoted\", and more"), std::string(",,"),
        std::string(" leading and trailing "), std::string("naive \xC3\xA9")}) {
    const auto fields = splitCsvLine(csvField(text) + "," + csvField("tail"));
    ASSERT_EQ(fields.size(), 2U) << text;
    EXPECT_EQ(fields[0], text);
    EXPECT_EQ(fields[1], "tail");
  }
}

TEST(CsvFieldTest, EmptyFieldsAreFieldsAndABlankLineIsOneEmptyField) {
  EXPECT_EQ(splitCsvLine("a,,b"), (std::vector<std::string>{"a", "", "b"}));
  EXPECT_EQ(splitCsvLine(",x,"), (std::vector<std::string>{"", "x", ""}));
  EXPECT_EQ(splitCsvLine(""), (std::vector<std::string>{""}));
  EXPECT_EQ(splitCsvLine(","), (std::vector<std::string>{"", ""}));
}

TEST(CsvFieldTest, AQuoteLeftOpenIsAnErrorNotSilentlyAccepted) {
  EXPECT_THROW((void)splitCsvLine("\"never closed"), std::invalid_argument);
  EXPECT_THROW((void)splitCsvLine("a,\"b"), std::invalid_argument);
  EXPECT_THROW((void)splitCsvLine("\"a\"\""), std::invalid_argument);
}

// ---- the runs file ------------------------------------------------------------------------------

TEST(RunsCsvTest, TheColumnsAreAContractAndAppearInThisOrder) {
  EXPECT_EQ(runsCsvHeader(),
            "layout,robots,seed,spawn_interval,ticks,basis,sim_seconds_per_tick,tasks_submitted,"
            "tasks_completed,tasks_failed,tasks_cancelled,tasks_open_at_end,tasks_per_sim_hour,"
            "tasks_per_robot_hour,avg_completion_ticks,avg_completion_sim_s,max_completion_ticks,"
            "utilization,moving_fraction,waiting_fraction,handling_fraction,charging_fraction,"
            "idle_fraction,out_of_service_fraction,idle_robot_ticks,waiting_robot_ticks,"
            "cells_per_robot,cells_per_completed_task,conflicts_prevented,conflict_detections,"
            "collisions_occurred,replans,replans_per_sim_hour,reroutes,repaths,obstacle_repairs,"
            "searches,energy_units_est,energy_units_per_task_est,energy_idle_est,energy_moving_est,"
            "energy_handling_est");
  const auto names = runColumns();
  EXPECT_EQ(splitCsvLine(runsCsvHeader()), names);
  std::map<std::string, int> seen;
  for (const auto& name : names) {
    EXPECT_EQ(++seen[name], 1) << "duplicate column " << name;
  }
}

TEST(RunsCsvTest, EveryRowHasOneFieldForEveryColumn) {
  EXPECT_EQ(splitCsvLine(runsCsvRow(kLabel, oneHour())).size(), runColumns().size());
  EXPECT_EQ(splitCsvLine(runsCsvRow(kLabel, FleetMetrics{})).size(), runColumns().size());
  EXPECT_EQ(splitCsvLine(runsCsvRow({"a,\"b\"", 0, 0}, oneHour())).size(), runColumns().size());
}

TEST(RunsCsvTest, EveryFigureOfARunIsWhatTheMetricsSayCheckedByHand) {
  const auto f = row(oneHour(), kLabel);
  EXPECT_EQ(f.at("layout"), "compact");
  EXPECT_EQ(f.at("robots"), "2");
  EXPECT_EQ(f.at("seed"), "7");
  EXPECT_EQ(f.at("spawn_interval"), "5");
  EXPECT_EQ(f.at("ticks"), "3600");
  EXPECT_EQ(f.at("sim_seconds_per_tick"), "1.0000");
  EXPECT_EQ(f.at("tasks_submitted"), "40");
  EXPECT_EQ(f.at("tasks_completed"), "30");
  EXPECT_EQ(f.at("tasks_failed"), "1");
  EXPECT_EQ(f.at("tasks_cancelled"), "2");
  EXPECT_EQ(f.at("tasks_open_at_end"), "7");
  EXPECT_EQ(f.at("tasks_per_sim_hour"), "30.0000");  // 30 in one hour
  EXPECT_EQ(f.at("tasks_per_robot_hour"), "15.0000");
  EXPECT_EQ(f.at("avg_completion_ticks"), "200.0000");
  EXPECT_EQ(f.at("avg_completion_sim_s"), "200.0000");
  EXPECT_EQ(f.at("max_completion_ticks"), "650");
  // robot-ticks: 3600 x 2 = 7200 (robot 1: 3600 exactly; robot 2: 3600 exactly)
  EXPECT_EQ(f.at("moving_fraction"), "0.5278");    // 3800 / 7200
  EXPECT_EQ(f.at("waiting_fraction"), "0.0833");   // 600 / 7200
  EXPECT_EQ(f.at("handling_fraction"), "0.1389");  // 1000 / 7200
  EXPECT_EQ(f.at("charging_fraction"), "0.0417");  // 300 / 7200
  EXPECT_EQ(f.at("idle_fraction"), "0.2083");      // 1500 / 7200
  EXPECT_EQ(f.at("out_of_service_fraction"), "0.0000");
  EXPECT_EQ(f.at("utilization"), "0.7500");  // (3800 + 600 + 1000) / 7200
  EXPECT_EQ(f.at("idle_robot_ticks"), "1500");
  EXPECT_EQ(f.at("waiting_robot_ticks"), "600");
  EXPECT_EQ(f.at("cells_per_robot"), "600.0000");
  EXPECT_EQ(f.at("cells_per_completed_task"), "40.0000");
  EXPECT_EQ(f.at("conflicts_prevented"), "7");
  EXPECT_EQ(f.at("conflict_detections"), "90");
  EXPECT_EQ(f.at("collisions_occurred"), "0");
  EXPECT_EQ(f.at("replans"), "8");
  EXPECT_EQ(f.at("replans_per_sim_hour"), "8.0000");
  EXPECT_EQ(f.at("reroutes"), "4");
  EXPECT_EQ(f.at("repaths"), "3");
  EXPECT_EQ(f.at("obstacle_repairs"), "1");
  EXPECT_EQ(f.at("searches"), "20");
  EXPECT_EQ(f.at("energy_idle_est"), "1500");      // 1500 idle ticks x 1
  EXPECT_EQ(f.at("energy_moving_est"), "18000");   // 1200 cells x 15
  EXPECT_EQ(f.at("energy_handling_est"), "2000");  // 1000 handling ticks x 2
  EXPECT_EQ(f.at("energy_units_est"), "21500");
  EXPECT_EQ(f.at("energy_units_per_task_est"), "716.6667");
}

TEST(RunsCsvTest, EveryRowSaysItIsSimulated) {
  EXPECT_EQ(row(oneHour(), kLabel).at("basis"), "simulated");
  EXPECT_EQ(row(FleetMetrics{}, kLabel).at("basis"), "simulated");
  EXPECT_EQ(std::string(warehousesim::metrics::kBasis), "simulated");
}

TEST(RunsCsvTest, AFigureThatIsUndefinedIsAnEmptyFieldNeverAZeroOrNaN) {
  const auto f = row(FleetMetrics{}, kLabel);  // nothing observed, nobody in the fleet
  for (const char* undefined :
       {"tasks_per_sim_hour", "tasks_per_robot_hour", "avg_completion_ticks",
        "avg_completion_sim_s", "utilization", "moving_fraction", "waiting_fraction",
        "idle_fraction", "cells_per_robot", "cells_per_completed_task", "replans_per_sim_hour",
        "energy_units_per_task_est"}) {
    EXPECT_EQ(f.at(undefined), "") << undefined;
  }
  // The counts, by contrast, are zeros.
  EXPECT_EQ(f.at("tasks_completed"), "0");
  EXPECT_EQ(f.at("conflicts_prevented"), "0");
  EXPECT_EQ(f.at("energy_units_est"), "0");
  const std::string text = runsCsvRow(kLabel, FleetMetrics{});
  EXPECT_TRUE(text.find("nan") == std::string::npos && text.find("inf") == std::string::npos);
}

TEST(RunsCsvTest, ARunThatCompletedNothingHasNoMeanButAFullTimeBudget) {
  FleetMetrics m = oneHour();
  m.tasksCompleted = 0;
  m.completionTicksTotal = 0;
  const auto f = row(m, kLabel);
  EXPECT_EQ(f.at("avg_completion_ticks"), "");
  EXPECT_EQ(f.at("cells_per_completed_task"), "");
  EXPECT_EQ(f.at("energy_units_per_task_est"), "");
  EXPECT_EQ(f.at("tasks_per_sim_hour"), "0.0000");  // a real zero: time passed and nothing finished
  EXPECT_EQ(f.at("utilization"), "0.7500");
}

TEST(RunsCsvTest, ALabelWithACommaOrAQuoteStaysOneField) {
  const auto f = row(oneHour(), {"my \"best\", hall", 3, 4});
  EXPECT_EQ(f.at("layout"), "my \"best\", hall");
  EXPECT_EQ(f.at("seed"), "3");
}

TEST(RunsCsvTest, NumbersAreWrittenTheSameWhateverTheLocale) {
  const std::locale before = std::locale::global(std::locale::classic());
  const std::string classic = runsCsvRow(kLabel, oneHour());
  try {
    std::locale::global(std::locale("de_DE.UTF-8"));  // a comma for a decimal point, if installed
  } catch (const std::runtime_error&) {
    // not installed: the classic locale is all there is, and the test is then a repeat
  }
  const std::string other = runsCsvRow(kLabel, oneHour());
  std::locale::global(before);
  EXPECT_EQ(classic, other);
  EXPECT_EQ(splitCsvLine(classic).size(), runColumns().size());
}

TEST(RunsCsvTest, AWholeFileIsTheHeaderAndOneLinePerRunInOrderEndingInALineBreak) {
  FleetMetrics other = oneHour();
  other.tasksCompleted = 12;
  const std::string text = runsCsv({{kLabel, oneHour()}, {{"demo", 8, 5}, other}});
  ASSERT_TRUE(!text.empty() && text.back() == '\n');
  const auto all = lines(text);
  ASSERT_EQ(all.size(), 3U);
  EXPECT_EQ(all[0], runsCsvHeader());
  EXPECT_EQ(splitCsvLine(all[1])[0], "compact");
  EXPECT_EQ(splitCsvLine(all[2])[0], "demo");
  EXPECT_EQ(lines(runsCsv({})).size(), 1U);  // no runs: just the header
}

TEST(RunsCsvTest, TheSameRunAlwaysGivesTheSameText) {
  EXPECT_EQ(runsCsvRow(kLabel, oneHour()), runsCsvRow(kLabel, oneHour()));
}

// ---- writing the files --------------------------------------------------------------------------

TEST(WriteRunFilesTest, ThreeFilesAreWrittenWithExactlyTheTextOfTheirFunctions) {
  const ScratchDir dir("metrics_write");
  const FleetMetrics m = oneHour();
  const auto base = dir.file("run");
  EXPECT_FALSE(warehousesim::metrics::writeRunFiles(base, kLabel, m).has_value());
  EXPECT_EQ(readFile(dir.file("run.runs.csv")), runsCsv({{kLabel, m}}));
  EXPECT_EQ(readFile(dir.file("run.robots.csv")), robotsCsv(m));
  EXPECT_EQ(readFile(dir.file("run.series.csv")), seriesCsv(m));
}

TEST(WriteRunFilesTest, NoTemporaryFileIsLeftBehindAndAnOldFileIsReplacedWhole) {
  const ScratchDir dir("metrics_write_twice");
  const auto base = dir.file("again");
  FleetMetrics first = oneHour();
  FleetMetrics second = oneHour();
  second.tasksCompleted = 3;
  EXPECT_FALSE(warehousesim::metrics::writeRunFiles(base, kLabel, first).has_value());
  EXPECT_FALSE(warehousesim::metrics::writeRunFiles(base, kLabel, second).has_value());
  EXPECT_EQ(readFile(dir.file("again.runs.csv")), runsCsv({{kLabel, second}}));
  std::size_t files = 0;
  for (const auto& entry : std::filesystem::directory_iterator(dir.path())) {
    ++files;
    EXPECT_NE(entry.path().extension(), ".tmp") << entry.path();
  }
  EXPECT_EQ(files, 3U);
}

TEST(WriteRunFilesTest, AFolderThatDoesNotExistIsReportedAndNothingIsCreated) {
  const ScratchDir dir("metrics_write_missing");
  const auto problem =
      warehousesim::metrics::writeRunFiles(dir.file("no/such/folder/run"), kLabel, oneHour());
  ASSERT_TRUE(problem.has_value());
  EXPECT_NE(problem->find("run.runs.csv"), std::string::npos) << *problem;
  EXPECT_FALSE(std::filesystem::exists(dir.file("no")));
}

TEST(WriteRunFilesTest, AFileThatCannotBeReplacedIsReportedWithItsNameAndLeavesNoTemporary) {
  const ScratchDir dir("metrics_write_blocked");
  std::filesystem::create_directory(dir.file("blocked.robots.csv"));  // a folder where a file goes
  const auto problem = warehousesim::metrics::writeRunFiles(dir.file("blocked"), kLabel, oneHour());
  ASSERT_TRUE(problem.has_value());
  EXPECT_NE(problem->find("blocked.robots.csv"), std::string::npos) << *problem;
  EXPECT_FALSE(std::filesystem::exists(dir.file("blocked.robots.csv.tmp")));
  EXPECT_TRUE(std::filesystem::is_directory(dir.file("blocked.robots.csv")));  // not touched
}

TEST(WriteRunFilesTest, TheLabelOfARunWithAnAwkwardNameStaysOneFieldInTheFile) {
  const ScratchDir dir("metrics_write_label");
  EXPECT_FALSE(
      warehousesim::metrics::writeRunFiles(dir.file("x"), {"my \"map\", v2.json", 4, 5}, oneHour())
          .has_value());
  const auto fields = splitCsvLine(lines(readFile(dir.file("x.runs.csv")))[1]);
  EXPECT_EQ(fields.size(), runColumns().size());
  EXPECT_EQ(fields[0], "my \"map\", v2.json");
}

// ---- the other two files ------------------------------------------------------------------------

TEST(RobotsCsvTest, OneLinePerRobotWithItsOwnBudgetAndUtilizationPinnedExactly) {
  EXPECT_EQ(robotsCsv(oneHour()),
            "robot,basis,moving_ticks,waiting_ticks,handling_ticks,charging_ticks,idle_ticks,"
            "out_of_service_ticks,cells,deliveries,conflicts_prevented,utilization\n"
            "1,simulated,2000,400,400,300,500,0,700,20,5,0.7778\n"
            "2,simulated,1800,200,600,0,1000,0,500,10,2,0.7222\n");
}

TEST(RobotsCsvTest, ARobotThatWasNeverInServiceHasNoUtilization) {
  FleetMetrics m;
  RobotMetrics r;
  r.id = RobotId{4};
  r.budget.ticks[static_cast<std::size_t>(Activity::Unavailable)] = 50;
  m.robots.push_back(r);
  const auto all = lines(robotsCsv(m));
  ASSERT_EQ(all.size(), 2U);
  const auto fields = splitCsvLine(all[1]);
  EXPECT_EQ(fields.back(), "");
  EXPECT_EQ(fields[7], "50");
}

TEST(RobotsCsvTest, NoRobotsIsJustTheHeader) {
  EXPECT_EQ(lines(robotsCsv(FleetMetrics{})).size(), 1U);
}

TEST(SeriesCsvTest, OneLinePerSampleWithRatesInTheScaleOfTheRunPinnedExactly) {
  FleetMetrics m = oneHour();
  m.options.secondsPerTick = 36.0;  // a window of 100 ticks is one hour
  MetricsSample a;
  a.tick = 100;
  a.windowTicks = 100;
  a.tasksCompleted = 12;
  a.busyRobotTicks = 150;
  a.serviceRobotTicks = 200;
  a.waitingRobotTicks = 20;
  a.tasksOpen = 9;
  MetricsSample b;
  b.tick = 200;
  b.windowTicks = 100;  // nothing done and nobody in service
  m.samples = {a, b};
  EXPECT_EQ(
      seriesCsv(m),
      "tick,basis,window_ticks,tasks_completed,tasks_per_sim_hour,utilization,waiting_fraction,"
      "tasks_open\n"
      "100,simulated,100,12,12.0000,0.7500,0.1000,9\n"
      "200,simulated,100,0,0.0000,,0.0000,0\n");
}

TEST(SeriesCsvTest, NoSamplesIsJustTheHeader) { EXPECT_EQ(lines(seriesCsv(oneHour())).size(), 1U); }

TEST(SeriesCsvTest, ASampleOfNoTicksHasNoRate) {
  FleetMetrics m = oneHour();
  MetricsSample empty;
  m.samples = {empty};
  const auto fields = splitCsvLine(lines(seriesCsv(m))[1]);
  EXPECT_EQ(fields[4], "");
}

// ---- durations ----------------------------------------------------------------------------------

TEST(FormatDurationTest, ReadsAsHoursMinutesAndSecondsDroppingTheEmptyLargeUnits) {
  EXPECT_EQ(formatDuration(0.0), "0s");
  EXPECT_EQ(formatDuration(0.4), "0s");
  EXPECT_EQ(formatDuration(0.6), "1s");
  EXPECT_EQ(formatDuration(42.0), "42s");
  EXPECT_EQ(formatDuration(59.4), "59s");
  EXPECT_EQ(formatDuration(60.0), "1m 0s");
  EXPECT_EQ(formatDuration(300.0), "5m 0s");
  EXPECT_EQ(formatDuration(3599.0), "59m 59s");
  EXPECT_EQ(formatDuration(3600.0), "1h 0m 0s");
  EXPECT_EQ(formatDuration(5000.0), "1h 23m 20s");
  EXPECT_EQ(formatDuration(360000.0), "100h 0m 0s");
}

TEST(FormatDurationTest, ATimeThatMeansNothingIsNotAvailableAndNeverUndefinedBehaviour) {
  EXPECT_EQ(formatDuration(-1.0), "n/a");
  EXPECT_EQ(formatDuration(std::numeric_limits<double>::quiet_NaN()), "n/a");
  EXPECT_EQ(formatDuration(std::numeric_limits<double>::infinity()), "n/a");
  EXPECT_EQ(formatDuration(-std::numeric_limits<double>::infinity()), "n/a");
  EXPECT_EQ(formatDuration(1e30), "n/a");
  EXPECT_EQ(formatDuration(std::numeric_limits<double>::max()), "n/a");
  EXPECT_NE(formatDuration(1e15), "n/a");  // the largest it prints
}

// ---- the dashboard ------------------------------------------------------------------------------

std::string all(const std::vector<std::string>& text) {
  std::string joined;
  for (const std::string& line : text) {
    joined += line + "\n";
  }
  return joined;
}

TEST(DashboardTest, ItBeginsByCallingTheFiguresSimulatedAndEndsWithTheDisclaimer) {
  const auto text = dashboardLines(oneHour());
  ASSERT_GE(text.size(), 3U);
  EXPECT_NE(text.front().find("simulated"), std::string::npos) << text.front();
  EXPECT_EQ(text.back(), std::string(kDisclaimer));
  EXPECT_NE(std::string(kDisclaimer).find("Not a claim about real robots"), std::string::npos);
  const auto empty = dashboardLines(FleetMetrics{});
  EXPECT_NE(empty.front().find("simulated"), std::string::npos);
  EXPECT_EQ(empty.back(), std::string(kDisclaimer));
}

TEST(DashboardTest, NoLineEndsInASpace) {
  for (const FleetMetrics& m : {oneHour(), FleetMetrics{}}) {
    for (const std::string& line : dashboardLines(m)) {
      EXPECT_TRUE(line.empty() || line.back() != ' ') << "[" << line << "]";
    }
  }
}

TEST(DashboardTest, EveryLabelIsSeparatedFromItsTextByASpaceAndTheTitleReadsAsOne) {
  const auto text = dashboardLines(oneHour());
  EXPECT_EQ(text.front(), "FLEET METRICS (simulated)");
  for (const std::string& line : text) {
    const auto gap = line.find(' ');
    EXPECT_NE(gap, std::string::npos) << line;
  }
  EXPECT_EQ(text[1].substr(0, 12), "sim time    ");  // short labels fill the 12-column field
}

TEST(DashboardTest, ItShowsEachOfTheTenFiguresAsked) {
  const std::string text = all(dashboardLines(oneHour()));
  EXPECT_NE(text.find("done 30"), std::string::npos) << text;
  EXPECT_NE(text.find("30.0 tasks/sim-hour"), std::string::npos) << text;
  EXPECT_NE(text.find("mean 200.0 ticks"), std::string::npos) << text;
  EXPECT_NE(text.find("75.0% of in-service"), std::string::npos) << text;
  EXPECT_NE(text.find("600.0 cells/robot"), std::string::npos) << text;  // average travel
  EXPECT_NE(text.find("idle 20.8%"), std::string::npos) << text;         // idle time
  EXPECT_NE(text.find("waiting 8.3%"), std::string::npos) << text;       // waiting time
  EXPECT_NE(text.find("7 prevented"), std::string::npos) << text;        // conflicts
  EXPECT_NE(text.find("8 (8.0/sim-hour)"), std::string::npos) << text;   // replanning
  EXPECT_NE(text.find("21500 units"), std::string::npos) << text;        // energy
}

TEST(DashboardTest, WhatIsUndefinedReadsNotAvailableNotZeroOrNaN) {
  const std::string text = all(dashboardLines(FleetMetrics{}));
  EXPECT_NE(text.find("n/a"), std::string::npos);
  EXPECT_EQ(text.find("nan"), std::string::npos);
  EXPECT_EQ(text.find("inf"), std::string::npos);
  EXPECT_NE(text.find("tasks"), std::string::npos);
}

TEST(DashboardTest, EveryFigureThatIsUndefinedSaysSoWhereItWouldHaveBeenAZero) {
  const std::string text = all(dashboardLines(FleetMetrics{}));
  EXPECT_NE(text.find("n/a tasks/sim-hour  (n/a per robot-hour)"), std::string::npos) << text;
  EXPECT_NE(text.find("mean n/a ticks = n/a s"), std::string::npos) << text;
  EXPECT_NE(text.find("n/a of in-service robot time"), std::string::npos) << text;
  EXPECT_NE(text.find("moving n/a  waiting n/a  handling n/a"), std::string::npos) << text;
  EXPECT_NE(text.find("n/a cells/robot  n/a cells/task"), std::string::npos) << text;
  EXPECT_NE(text.find("(n/a/sim-hour)"), std::string::npos) << text;
  EXPECT_NE(text.find("(n/a/task)"), std::string::npos) << text;
  EXPECT_EQ(text.find("0.0"), std::string::npos)
      << "a figure that is not defined is not zero: " << text;
}

TEST(DashboardTest, ThereAreNoCollisionsToReportInTheNormalCase) {
  EXPECT_NE(all(dashboardLines(oneHour())).find("0 collisions occurred"), std::string::npos);
}

TEST(DashboardTest, ACollisionIsReportedPlainly) {
  FleetMetrics m = oneHour();
  m.collisionsOccurred = 3;
  EXPECT_NE(all(dashboardLines(m)).find("3 collisions occurred"), std::string::npos);
}

TEST(DashboardTest, NoLineIsLongerThanTheDashboardIsWideEvenWithHugeNumbers) {
  FleetMetrics m = oneHour();
  m.ticks = 1'000'000'000'000ULL;
  m.tasksCompleted = 1'000'000'000'000ULL;
  m.tasksFailed = 1'000'000'000'000ULL;
  m.tasksCancelled = 1'000'000'000'000ULL;
  m.tasksOpen = 1'000'000'000'000ULL;
  m.completionTicksTotal = 1'000'000'000'000'000'000ULL;
  m.completionTicksMax = 1'000'000'000'000ULL;
  m.reroutes = m.repaths = 1'000'000'000'000ULL;
  m.conflictsPrevented = m.collisionsOccurred = 1'000'000'000'000ULL;
  m.options.secondsPerTick = 1e9;
  for (const std::string& line : dashboardLines(m)) {
    EXPECT_LE(line.size(), kDashboardColumns) << line;
  }
  for (const std::string& line : dashboardLines(oneHour())) {
    EXPECT_LE(line.size(), kDashboardColumns) << line;
  }
  for (const std::string& line : dashboardLines(FleetMetrics{})) {
    EXPECT_LE(line.size(), kDashboardColumns) << line;
  }
}

TEST(DashboardTest, AVerySmallTimeScaleIsNotShownAsZero) {
  FleetMetrics m = oneHour();
  m.options.secondsPerTick = 0.000001;
  EXPECT_NE(all(dashboardLines(m)).find("at 0.000001 s/tick"), std::string::npos);
}

TEST(DashboardTest, ATimeScaleOfHalfASecondIsShownAsSuch) {
  FleetMetrics m = oneHour();
  m.options.secondsPerTick = 0.5;
  EXPECT_NE(all(dashboardLines(m)).find("at 0.5 s/tick"), std::string::npos);
  m.options.secondsPerTick = 1.0;
  EXPECT_NE(all(dashboardLines(m)).find("at 1 s/tick"), std::string::npos);
}

TEST(DashboardTest, TheEnergyModelIsStatedBesideTheFigure) {
  const std::string text = all(dashboardLines(oneHour()));
  EXPECT_NE(text.find("model: idle 1/tick, cell 15, handling 2/tick"), std::string::npos) << text;
}

TEST(DashboardTest, TheSameRunAlwaysGivesTheSameText) {
  EXPECT_EQ(dashboardLines(oneHour()), dashboardLines(oneHour()));
}

}  // namespace
