#include "warehousesim/metrics/fleet_metrics.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "warehousesim/metrics/accumulator.hpp"

namespace {

using warehousesim::core::RobotId;
using warehousesim::metrics::Activity;
using warehousesim::metrics::EnergyModel;
using warehousesim::metrics::FleetMetrics;
using warehousesim::metrics::MetricsOptions;
using warehousesim::metrics::RobotMetrics;
using warehousesim::metrics::TimeBudget;

constexpr double kEps = 1e-12;

FleetMetrics fleetOf(std::size_t robots) {
  FleetMetrics m;
  for (std::size_t i = 0; i < robots; ++i) {
    RobotMetrics r;
    r.id = RobotId{static_cast<std::uint32_t>(i + 1)};
    m.robots.push_back(r);
  }
  return m;
}

void give(RobotMetrics& robot, Activity activity, std::uint64_t ticks) {
  robot.budget.ticks[static_cast<std::size_t>(activity)] += ticks;
}

TEST(FleetMetricsTest, ADefaultReportHasNothingToSayAndSaysNothing) {
  const FleetMetrics m;
  EXPECT_EQ(m.ticks, 0U);
  EXPECT_EQ(m.simulatedSeconds(), 0.0);
  EXPECT_EQ(m.simulatedHours(), 0.0);
  EXPECT_FALSE(m.tasksPerHour().has_value());
  EXPECT_FALSE(m.utilization().has_value());
  EXPECT_FALSE(m.cellsPerRobot().has_value());
  EXPECT_FALSE(m.replansPerHour().has_value());
  EXPECT_EQ(m.replanEvents(), 0U);
  EXPECT_EQ(m.energyEstimate().total(), 0U);
}

TEST(FleetMetricsTest, SimulatedTimeIsTicksTimesTheScaleAndHoursAreSecondsOver3600) {
  FleetMetrics m;
  m.ticks = 7200;
  m.options.secondsPerTick = 0.5;
  EXPECT_NEAR(m.simulatedSeconds(), 3600.0, kEps);
  EXPECT_NEAR(m.simulatedHours(), 1.0, kEps);
}

TEST(FleetMetricsTest, RatesNeedTimeAndTheirDenominatorsButAZeroNumeratorIsAGoodZero) {
  FleetMetrics m = fleetOf(2);
  m.ticks = 3600;
  ASSERT_TRUE(m.tasksPerHour().has_value());
  EXPECT_EQ(*m.tasksPerHour(), 0.0);
  m.tasksCompleted = 30;
  EXPECT_NEAR(*m.tasksPerHour(), 30.0, kEps);
  EXPECT_NEAR(*m.tasksPerRobotHour(), 15.0, kEps);
  m.reroutes = 6;
  m.repaths = 3;
  m.obstacleRepairs = 1;
  EXPECT_EQ(m.replanEvents(), 10U);
  EXPECT_NEAR(*m.replansPerHour(), 10.0, kEps);
}

TEST(FleetMetricsTest, ConflictsPreventedPerHourNeedsTimeButNotConflicts) {
  FleetMetrics m = fleetOf(2);
  EXPECT_FALSE(m.conflictsPreventedPerHour().has_value());  // no time observed
  m.ticks = 1800;
  ASSERT_TRUE(m.conflictsPreventedPerHour().has_value());
  EXPECT_EQ(*m.conflictsPreventedPerHour(), 0.0);
  m.conflictsPrevented = 5;
  EXPECT_NEAR(*m.conflictsPreventedPerHour(), 10.0, kEps);  // 5 in half an hour
}

TEST(FleetMetricsTest, UtilizationCountsMovingWaitingAndHandlingAgainstEverythingButOutOfService) {
  FleetMetrics m = fleetOf(2);
  give(m.robots[0], Activity::Moving, 30);
  give(m.robots[0], Activity::Waiting, 10);
  give(m.robots[0], Activity::Handling, 20);
  give(m.robots[0], Activity::Charging, 10);
  give(m.robots[0], Activity::Idle, 30);
  give(m.robots[1], Activity::Unavailable, 100);
  EXPECT_NEAR(*m.utilization(), 60.0 / 100.0, kEps);
  EXPECT_NEAR(*m.fraction(Activity::Unavailable), 0.5, kEps);  // of all 200 robot-ticks
  EXPECT_NEAR(*m.fraction(Activity::Idle), 30.0 / 200.0, kEps);
  EXPECT_EQ(m.robotTicks(), 200U);
}

TEST(FleetMetricsTest, TheTimeBudgetTotalIsTheSumOverAllActivities) {
  TimeBudget budget;
  EXPECT_EQ(budget.total(), 0U);
  for (std::size_t i = 0; i < budget.ticks.size(); ++i) {
    budget.ticks[i] = i + 1;
  }
  EXPECT_EQ(budget.total(), 21U);
  EXPECT_EQ(budget.in(Activity::Moving), 1U);
  EXPECT_EQ(budget.in(Activity::Unavailable), 6U);
}

TEST(FleetMetricsTest, EnergyUsesTheModelAndOnlyThoseThreeThings) {
  FleetMetrics m = fleetOf(1);
  m.options.energy = {3, 7, 2};
  give(m.robots[0], Activity::Idle, 10);
  give(m.robots[0], Activity::Handling, 5);
  give(m.robots[0], Activity::Moving, 99);    // moving time as such costs nothing: cells do
  give(m.robots[0], Activity::Waiting, 99);   // standing still while waiting costs nothing here
  give(m.robots[0], Activity::Charging, 99);  // charging draws no energy
  m.robots[0].cellsTravelled = 4;
  const auto energy = m.energyEstimate();
  EXPECT_EQ(energy.idle, 30U);
  EXPECT_EQ(energy.moving, 28U);
  EXPECT_EQ(energy.handling, 10U);
  EXPECT_EQ(energy.total(), 68U);
}

TEST(FleetMetricsTest, ADifferentModelGivesADifferentEstimateForTheSameRun) {
  FleetMetrics m = fleetOf(1);
  give(m.robots[0], Activity::Idle, 10);
  m.robots[0].cellsTravelled = 10;
  m.options.energy = {0, 0, 0};
  EXPECT_EQ(m.energyEstimate().total(), 0U);
  m.options.energy = EnergyModel::standard();
  EXPECT_EQ(m.energyEstimate().total(), 10U * 1U + 10U * 15U);
}

TEST(FleetMetricsTest, NoEnergyFigureOverflowsForRunsOfAnyRealisticSize) {
  FleetMetrics m = fleetOf(1);
  m.robots[0].cellsTravelled = 4'000'000'000ULL;  // more than a 32-bit count
  m.options.energy = {1, 4'000'000'000U, 1};
  EXPECT_EQ(m.energyEstimate().moving, 4'000'000'000ULL * 4'000'000'000ULL);
}

TEST(FleetMetricsTest, OptionsAreValidatedForTheTimeScaleOnly) {
  MetricsOptions o;
  EXPECT_NO_THROW(o.validate());
  o.secondsPerTick = 0.0;
  EXPECT_THROW(o.validate(), std::invalid_argument);
  o.secondsPerTick = std::numeric_limits<double>::denorm_min();
  EXPECT_NO_THROW(o.validate());
}

// ---- pooling runs -------------------------------------------------------------------------------

TEST(FleetMetricsMergeTest, TwoRunsOfTheSameFleetPoolTheirCountsAndTheirFigures) {
  FleetMetrics a = fleetOf(2);
  a.ticks = 1800;
  a.tasksCompleted = 10;
  a.completionTicksTotal = 100;
  a.completionTicksMax = 30;
  a.conflictsPrevented = 4;
  a.reroutes = 1;
  a.robots[0].cellsTravelled = 20;
  a.robots[1].deliveries = 3;
  give(a.robots[0], Activity::Moving, 600);
  a.samples.push_back({});

  FleetMetrics b = fleetOf(2);
  b.ticks = 1800;
  b.tasksCompleted = 30;
  b.completionTicksTotal = 700;
  b.completionTicksMax = 90;
  b.conflictsPrevented = 6;
  b.reroutes = 3;
  b.robots[0].cellsTravelled = 40;
  b.robots[1].deliveries = 5;
  give(b.robots[0], Activity::Moving, 400);

  a.merge(b);
  EXPECT_EQ(a.ticks, 3600U);
  EXPECT_EQ(a.tasksCompleted, 40U);
  EXPECT_NEAR(*a.tasksPerHour(), 40.0, kEps);
  EXPECT_NEAR(*a.meanCompletionTicks(), 800.0 / 40.0, kEps);  // pooled, not the mean of two means
  EXPECT_EQ(a.completionTicksMax, 90U);
  EXPECT_EQ(a.conflictsPrevented, 10U);
  EXPECT_EQ(a.reroutes, 4U);
  EXPECT_EQ(a.robots[0].cellsTravelled, 60U);
  EXPECT_EQ(a.robots[1].deliveries, 8U);
  EXPECT_EQ(a.robots[0].budget.in(Activity::Moving), 1000U);
  EXPECT_TRUE(a.samples.empty());  // windows of two runs do not line up
}

TEST(FleetMetricsMergeTest, TheMeanOfTheMergeIsNotTheMeanOfTheMeans) {
  FleetMetrics a = fleetOf(1);
  a.tasksCompleted = 1;
  a.completionTicksTotal = 10;  // mean 10
  FleetMetrics b = fleetOf(1);
  b.tasksCompleted = 9;
  b.completionTicksTotal = 90 * 20;  // mean 200
  FleetMetrics merged = a;
  merged.merge(b);
  EXPECT_NEAR(*merged.meanCompletionTicks(), (10.0 + 1800.0) / 10.0, kEps);
  EXPECT_GT(*merged.meanCompletionTicks(), (10.0 + 200.0) / 2.0);
}

TEST(FleetMetricsMergeTest, MergingWithAnEmptyRunChangesNothing) {
  FleetMetrics a = fleetOf(2);
  a.ticks = 100;
  a.tasksCompleted = 5;
  give(a.robots[0], Activity::Idle, 100);
  const FleetMetrics before = a;
  a.merge(fleetOf(2));
  EXPECT_EQ(a.ticks, before.ticks);
  EXPECT_EQ(a.tasksCompleted, before.tasksCompleted);
  EXPECT_TRUE(a.robots == before.robots);
}

TEST(FleetMetricsMergeTest, RunsOfOtherFleetsOrOtherOptionsCannotBePooled) {
  FleetMetrics a = fleetOf(2);
  a.ticks = 10;
  FleetMetrics three = fleetOf(3);
  EXPECT_THROW(a.merge(three), std::invalid_argument);
  FleetMetrics renamed = fleetOf(2);
  renamed.robots[1].id = RobotId{7};
  EXPECT_THROW(a.merge(renamed), std::invalid_argument);
  FleetMetrics slower = fleetOf(2);
  slower.options.secondsPerTick = 2.0;
  EXPECT_THROW(a.merge(slower), std::invalid_argument);
  FleetMetrics other = fleetOf(2);
  other.options.energy = {9, 9, 9};
  EXPECT_THROW(a.merge(other), std::invalid_argument);
  EXPECT_EQ(a.ticks, 10U);  // and a refused merge changes nothing
}

}  // namespace
