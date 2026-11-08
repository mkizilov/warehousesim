#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "scenario_support.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"

namespace {

using namespace warehousesim_test;
using std::chrono::milliseconds;
using warehousesim::metrics::MetricsOptions;
using warehousesim::simulation::MetricsCollector;
using warehousesim::simulation::ResetScenario;
using warehousesim::simulation::SimulationRunner;
using warehousesim::simulation::SubmitTask;

constexpr milliseconds kTimeout{15000};

warehousesim::simulation::ScenarioFactory demo() {
  return *warehousesim::simulation::layoutScenario("demo");
}

std::string rowOf(const warehousesim::metrics::FleetMetrics& m) {
  return warehousesim::metrics::runsCsvRow({"demo", 1, 5}, m);
}

TEST(RunnerMetricsTest, ARunnerMeasuresEveryStepItTakes) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(1), milliseconds(1), demo());
  runner.start(120);
  ASSERT_TRUE(runner.waitForTick(120, kTimeout));
  const auto m = runner.metrics();
  EXPECT_EQ(m.ticks, 120U);
  EXPECT_EQ(m.robotTicks(), 120U * m.robots.size());

  auto reference = warehousesim::simulation::makeDemoSimulation(1);
  MetricsCollector collector(reference);
  for (int i = 0; i < 120; ++i) {
    reference.step();
    collector.observe(reference);
  }
  EXPECT_EQ(rowOf(m), rowOf(collector.report()));  // the same figures as measuring directly
  runner.stop();
}

TEST(RunnerMetricsTest, NothingIsMeasuredBeforeTheFirstStepOrWhilePaused) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(1), milliseconds(1), demo());
  EXPECT_EQ(runner.metrics().ticks, 0U);
  runner.setPaused(true);
  runner.start();
  std::this_thread::sleep_for(milliseconds(30));
  EXPECT_EQ(runner.metrics().ticks, 0U);
  runner.setPaused(false);
  ASSERT_TRUE(runner.waitForTick(10, kTimeout));
  EXPECT_GE(runner.metrics().ticks, 10U);
  runner.stop();
}

TEST(RunnerMetricsTest, TheMetricsAreAlwaysInStepWithTheSnapshotItself) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(2), milliseconds(1), demo());
  runner.start(200);
  ASSERT_TRUE(runner.waitForTick(200, kTimeout));
  const auto m = runner.metrics();
  const auto snapshot = runner.snapshot();
  EXPECT_EQ(m.ticks, snapshot.tick);
  EXPECT_EQ(m.tasksCompleted, snapshot.completedTasks);
  EXPECT_EQ(m.tasksOpen, snapshot.activeTasks.size());
  EXPECT_EQ(m.collisionsOccurred, 0U);
  runner.stop();
}

TEST(RunnerMetricsTest, MeasuringDoesNotChangeTheRunWhateverHowOftenItIsAskedFor) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(3), milliseconds(1), demo());
  runner.start(150);
  while (runner.snapshot().tick < 150) {
    (void)runner.metrics();
    std::this_thread::yield();
  }
  ASSERT_TRUE(runner.waitForTick(150, kTimeout));
  auto reference = warehousesim::simulation::makeDemoSimulation(3);
  reference.run(150);
  EXPECT_EQ(warehousesim::simulation::toJson(runner.snapshot()).dump(),
            warehousesim::simulation::toJson(reference.snapshot()).dump());
  runner.stop();
}

TEST(RunnerMetricsTest, ARestartStartsTheMeasurementOverToo) {
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(1), milliseconds(1), demo());
  runner.start(60);
  ASSERT_TRUE(runner.waitForTick(60, kTimeout));
  ASSERT_EQ(runner.metrics().ticks, 60U);
  runner.setPaused(true);
  ASSERT_TRUE(runner.post(ResetScenario{}));
  for (int i = 0; i < 4000 && runner.frame().epoch == 0; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  ASSERT_EQ(runner.frame().epoch, 1U);
  const auto fresh = runner.metrics();
  EXPECT_EQ(fresh.ticks, 0U);
  EXPECT_EQ(fresh.tasksCompleted, 0U);
  EXPECT_EQ(fresh.tasksSubmitted, 0U);
  runner.setPaused(false);
  ASSERT_TRUE(runner.waitForTick(20, kTimeout));
  EXPECT_GE(runner.metrics().ticks, 20U);
  EXPECT_LE(runner.metrics().ticks, 60U);  // counted from the new run, not carried over
  runner.stop();
}

TEST(RunnerMetricsTest, TasksAddedByCommandAreCounted) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  SimulationRunner runner(Simulation(layout({"P...D", "....."}), {P(0, 1)}, config),
                          milliseconds(1));
  runner.setPaused(true);
  runner.start();
  ASSERT_TRUE(runner.post(SubmitTask{P(0, 0), P(4, 0)}));
  for (int i = 0; i < 4000 && runner.commandLog().empty(); ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  runner.setPaused(false);
  for (int i = 0; i < 8000 && runner.metrics().tasksCompleted == 0; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  const auto m = runner.metrics();
  EXPECT_EQ(m.tasksSubmitted, 1U);
  EXPECT_EQ(m.tasksCompleted, 1U);
  runner.stop();
}

TEST(RunnerMetricsTest, TheTimeScaleAndTheEnergyModelAreTheOnesGiven) {
  MetricsOptions options;
  options.secondsPerTick = 4.0;
  options.energy = {0, 1, 0};
  SimulationRunner runner(warehousesim::simulation::makeDemoSimulation(1), milliseconds(1), demo(),
                          options);
  runner.start(50);
  ASSERT_TRUE(runner.waitForTick(50, kTimeout));
  const auto m = runner.metrics();
  EXPECT_EQ(m.options.secondsPerTick, 4.0);
  EXPECT_NEAR(m.simulatedSeconds(), 200.0, 1e-9);
  EXPECT_EQ(m.energyEstimate().total(), m.cellsTravelled());  // one unit a cell, nothing else
  runner.stop();
}

TEST(RunnerMetricsTest, ABadTimeScaleIsRefusedWhenTheRunnerIsMade) {
  MetricsOptions options;
  options.secondsPerTick = -1.0;
  EXPECT_THROW(SimulationRunner(warehousesim::simulation::makeDemoSimulation(1), milliseconds(1),
                                demo(), options),
               std::invalid_argument);
}

}  // namespace
