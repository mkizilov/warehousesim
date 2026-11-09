#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "warehousesim/metrics/accumulator.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::Tick;
using warehousesim::metrics::Activity;
using warehousesim::metrics::EnergyModel;
using warehousesim::metrics::FleetMetrics;
using warehousesim::metrics::MetricsAccumulator;
using warehousesim::metrics::MetricsOptions;
using warehousesim::metrics::RobotObservation;
using warehousesim::metrics::SystemCounters;
using warehousesim::metrics::TaskEnd;
using warehousesim::metrics::TaskEnding;
using warehousesim::metrics::TickObservation;
using warehousesim::metrics::WaitCause;

constexpr double kEps = 1e-12;

RobotObservation robot(std::uint32_t id, Activity activity, Point at = {0, 0},
                       WaitCause cause = WaitCause::None, std::uint64_t cells = 0,
                       std::uint64_t deliveries = 0) {
  return {RobotId{id}, at, activity, cause, cells, deliveries};
}

TickObservation observation(Tick tick, std::vector<RobotObservation> robots,
                            SystemCounters counters = {}, std::uint64_t open = 0,
                            std::vector<TaskEnding> ended = {}) {
  TickObservation result;
  result.tick = tick;
  result.robots = std::move(robots);
  result.counters = counters;
  result.tasksOpen = open;
  result.tasksEnded = std::move(ended);
  return result;
}

// Robots at distinct cells, all in the given activity.
std::vector<RobotObservation> fleet(std::size_t count, Activity activity) {
  std::vector<RobotObservation> robots;
  for (std::size_t i = 0; i < count; ++i) {
    robots.push_back(
        robot(static_cast<std::uint32_t>(i + 1), activity, {static_cast<std::int32_t>(i) * 2, 0}));
  }
  return robots;
}

MetricsOptions options(double secondsPerTick = 1.0, Tick sampleInterval = 0) {
  MetricsOptions o;
  o.secondsPerTick = secondsPerTick;
  o.sampleInterval = sampleInterval;
  return o;
}

// Feeds `count` ticks of an unchanging fleet.
void idleFor(MetricsAccumulator& acc, std::size_t robots, Tick count,
             Activity activity = Activity::Idle) {
  for (Tick i = 0; i < count; ++i) {
    acc.add(observation(acc.lastTick() + 1, fleet(robots, activity)));
  }
}

// ---- nothing observed ---------------------------------------------------------------------------

TEST(MetricsAccumulatorTest, BeforeAnyTickEveryDerivedFigureIsUndefinedNotZero) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  const FleetMetrics& m = acc.report();
  EXPECT_EQ(m.ticks, 0U);
  EXPECT_EQ(m.robots.size(), 2U);
  EXPECT_FALSE(m.tasksPerHour().has_value());
  EXPECT_FALSE(m.tasksPerRobotHour().has_value());
  EXPECT_FALSE(m.meanCompletionTicks().has_value());
  EXPECT_FALSE(m.meanCompletionSeconds().has_value());
  EXPECT_FALSE(m.utilization().has_value());
  EXPECT_FALSE(m.fraction(Activity::Idle).has_value());
  EXPECT_FALSE(m.cellsPerCompletedTask().has_value());
  EXPECT_FALSE(m.replansPerHour().has_value());
  EXPECT_FALSE(m.energyPerCompletedTask().has_value());
  EXPECT_EQ(m.simulatedHours(), 0.0);
  EXPECT_EQ(m.robotTicks(), 0U);
  EXPECT_EQ(m.tasksCompleted, 0U);
  EXPECT_EQ(m.energyEstimate().total(), 0U);
  ASSERT_TRUE(m.cellsPerRobot().has_value());
  EXPECT_EQ(*m.cellsPerRobot(), 0.0);  // two robots that have driven nowhere: a defined zero
}

TEST(MetricsAccumulatorTest, AFleetOfNoRobotsStillTellsTimeAndTasksButNothingPerRobot) {
  MetricsAccumulator acc(options(36.0), observation(0, {}));
  for (Tick t = 1; t <= 100; ++t) {
    std::vector<TaskEnding> ended;
    if (t % 10 == 0) {
      ended.push_back({TaskEnd::Completed, t - 5});
    }
    acc.add(observation(t, {}, {}, 0, ended));
  }
  const FleetMetrics& m = acc.report();
  EXPECT_EQ(m.ticks, 100U);
  EXPECT_NEAR(m.simulatedHours(), 1.0, kEps);  // 100 ticks of 36 s
  ASSERT_TRUE(m.tasksPerHour().has_value());
  EXPECT_NEAR(*m.tasksPerHour(), 10.0, kEps);
  EXPECT_FALSE(m.tasksPerRobotHour().has_value());
  EXPECT_FALSE(m.utilization().has_value());
  EXPECT_FALSE(m.cellsPerRobot().has_value());
  EXPECT_EQ(m.robotTicks(), 0U);
}

// ---- the time budget ----------------------------------------------------------------------------

TEST(MetricsAccumulatorTest, EveryRobotIsInExactlyOneActivityEveryTick) {
  MetricsAccumulator acc(options(), observation(0, fleet(3, Activity::Idle)));
  const std::vector<std::vector<Activity>> script{
      {Activity::Moving, Activity::Idle, Activity::Charging},
      {Activity::Moving, Activity::Waiting, Activity::Charging},
      {Activity::Handling, Activity::Waiting, Activity::Unavailable},
      {Activity::Idle, Activity::Moving, Activity::Unavailable}};
  for (std::size_t t = 0; t < script.size(); ++t) {
    std::vector<RobotObservation> robots = fleet(3, Activity::Idle);
    for (std::size_t r = 0; r < 3; ++r) {
      robots[r].activity = script[t][r];
      if (script[t][r] == Activity::Waiting) {
        robots[r].waitCause = WaitCause::Other;
      }
    }
    acc.add(observation(t + 1, robots));
  }
  const FleetMetrics& m = acc.report();
  EXPECT_EQ(m.robots[0].budget.in(Activity::Moving), 2U);
  EXPECT_EQ(m.robots[0].budget.in(Activity::Handling), 1U);
  EXPECT_EQ(m.robots[0].budget.in(Activity::Idle), 1U);
  EXPECT_EQ(m.robots[1].budget.in(Activity::Waiting), 2U);
  EXPECT_EQ(m.robots[1].budget.in(Activity::Idle), 1U);
  EXPECT_EQ(m.robots[1].budget.in(Activity::Moving), 1U);
  EXPECT_EQ(m.robots[2].budget.in(Activity::Charging), 2U);
  EXPECT_EQ(m.robots[2].budget.in(Activity::Unavailable), 2U);
  for (const auto& r : m.robots) {
    EXPECT_EQ(r.budget.total(), 4U);  // one activity per robot per tick, nothing more or less
  }
  EXPECT_EQ(m.robotTicks(), 12U);
  EXPECT_EQ(m.ticksIn(Activity::Moving), 3U);
  ASSERT_TRUE(m.fraction(Activity::Moving).has_value());
  EXPECT_NEAR(*m.fraction(Activity::Moving), 3.0 / 12.0, kEps);
  double sum = 0;
  for (const Activity a : warehousesim::metrics::kAllActivities) {
    sum += *m.fraction(a);
  }
  EXPECT_NEAR(sum, 1.0, kEps);  // the fractions of a whole
}

TEST(MetricsAccumulatorTest, UtilizationIsBusyTimeOverInServiceTimeAndChargingIsNotBusy) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  // Robot 1: moving, waiting, handling, idle, charging. Robot 2 out of service throughout.
  const Activity first[] = {Activity::Moving, Activity::Waiting, Activity::Handling, Activity::Idle,
                            Activity::Charging};
  for (Tick t = 0; t < 5; ++t) {
    std::vector<RobotObservation> robots = fleet(2, Activity::Unavailable);
    robots[0].activity = first[t];
    robots[0].waitCause = first[t] == Activity::Waiting ? WaitCause::Other : WaitCause::None;
    acc.add(observation(t + 1, robots));
  }
  ASSERT_TRUE(acc.report().utilization().has_value());
  EXPECT_NEAR(*acc.report().utilization(), 3.0 / 5.0, kEps);  // robot 2 is left out of the base
}

TEST(MetricsAccumulatorTest, UtilizationIsUndefinedWhenNothingWasInService) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Unavailable)));
  idleFor(acc, 2, 5, Activity::Unavailable);
  EXPECT_FALSE(acc.report().utilization().has_value());
  ASSERT_TRUE(acc.report().fraction(Activity::Unavailable).has_value());
  EXPECT_NEAR(*acc.report().fraction(Activity::Unavailable), 1.0, kEps);
}

TEST(MetricsAccumulatorTest, IdleTimeAndWaitingTimeAreTheRobotTicksInThoseActivities) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  for (Tick t = 1; t <= 6; ++t) {
    std::vector<RobotObservation> robots = fleet(2, Activity::Idle);
    if (t <= 4) {
      robots[1].activity = Activity::Waiting;
      robots[1].waitCause = WaitCause::Vertex;
    }
    acc.add(observation(t, robots));
  }
  EXPECT_EQ(acc.report().ticksIn(Activity::Idle), 6U + 2U);
  EXPECT_EQ(acc.report().ticksIn(Activity::Waiting), 4U);
  EXPECT_NEAR(*acc.report().fraction(Activity::Waiting), 4.0 / 12.0, kEps);
}

// ---- tasks --------------------------------------------------------------------------------------

TEST(MetricsAccumulatorTest, TasksPerHourFollowsTheTimeScaleOfATick) {
  for (const auto& [secondsPerTick, expected] :
       std::vector<std::pair<double, double>>{{36.0, 10.0}, {72.0, 5.0}, {18.0, 20.0}}) {
    MetricsAccumulator acc(options(secondsPerTick), observation(0, fleet(2, Activity::Idle)));
    for (Tick t = 1; t <= 100; ++t) {
      std::vector<TaskEnding> ended;
      if (t % 10 == 0) {
        ended.push_back({TaskEnd::Completed, t - 1});
      }
      acc.add(observation(t, fleet(2, Activity::Idle), {}, 0, ended));
    }
    EXPECT_EQ(acc.report().tasksCompleted, 10U);
    EXPECT_NEAR(*acc.report().tasksPerHour(), expected, 1e-9) << secondsPerTick;
    EXPECT_NEAR(*acc.report().tasksPerRobotHour(), expected / 2.0, 1e-9) << secondsPerTick;
  }
}

TEST(MetricsAccumulatorTest,
     CompletionTimeIsFromSubmissionToTheTickOfDeliveryAndOnlyForCompletedTasks) {
  MetricsAccumulator acc(options(2.0), observation(0, fleet(1, Activity::Idle)));
  idleFor(acc, 1, 9);
  acc.add(observation(10, fleet(1, Activity::Idle), {}, 0,
                      {{TaskEnd::Completed, 0},     // took 10
                       {TaskEnd::Completed, 4},     // took 6
                       {TaskEnd::Failed, 0},        // not a delivery
                       {TaskEnd::Cancelled, 2}}));  // not a delivery
  const FleetMetrics& m = acc.report();
  EXPECT_EQ(m.tasksCompleted, 2U);
  EXPECT_EQ(m.tasksFailed, 1U);
  EXPECT_EQ(m.tasksCancelled, 1U);
  EXPECT_EQ(m.completionTicksTotal, 16U);
  EXPECT_EQ(m.completionTicksMax, 10U);
  EXPECT_NEAR(*m.meanCompletionTicks(), 8.0, kEps);
  EXPECT_NEAR(*m.meanCompletionSeconds(), 16.0, kEps);  // 2 s per tick
}

TEST(MetricsAccumulatorTest, ATaskMadeAndDeliveredWithinOneStepTookOneTick) {
  MetricsAccumulator acc(options(), observation(5, fleet(1, Activity::Idle)));
  acc.add(observation(6, fleet(1, Activity::Idle), {}, 0, {{TaskEnd::Completed, 5}}));
  EXPECT_EQ(acc.report().completionTicksTotal, 1U);
  EXPECT_EQ(acc.report().completionTicksMax, 1U);
}

TEST(MetricsAccumulatorTest, ATaskThatEndsInTheTickItWasCreatedAtTookNoTime) {
  MetricsAccumulator acc(options(), observation(5, fleet(1, Activity::Idle)));
  acc.add(observation(6, fleet(1, Activity::Idle), {}, 0, {{TaskEnd::Completed, 6}}));
  EXPECT_EQ(acc.report().completionTicksTotal, 0U);
  ASSERT_TRUE(acc.report().meanCompletionTicks().has_value());
  EXPECT_EQ(*acc.report().meanCompletionTicks(), 0.0);  // a defined zero, not "unknown"
}

TEST(MetricsAccumulatorTest, WithoutACompletedTaskTheMeanIsUndefinedEvenIfOthersFailed) {
  MetricsAccumulator acc(options(), observation(0, fleet(1, Activity::Idle)));
  acc.add(observation(1, fleet(1, Activity::Idle), {}, 0, {{TaskEnd::Failed, 0}}));
  EXPECT_FALSE(acc.report().meanCompletionTicks().has_value());
  EXPECT_FALSE(acc.report().cellsPerCompletedTask().has_value());
  EXPECT_FALSE(acc.report().energyPerCompletedTask().has_value());
  EXPECT_EQ(acc.report().tasksFailed, 1U);
}

TEST(MetricsAccumulatorTest, SubmittedAndOpenTasksCountFromTheStartOfTheInterval) {
  SystemCounters start;
  start.tasksSubmitted = 7;
  MetricsAccumulator acc(options(), observation(0, fleet(1, Activity::Idle), start, 3));
  EXPECT_EQ(acc.report().tasksOpen, 3U);
  SystemCounters later;
  later.tasksSubmitted = 12;
  acc.add(observation(1, fleet(1, Activity::Idle), later, 8));
  EXPECT_EQ(acc.report().tasksSubmitted, 5U);
  EXPECT_EQ(acc.report().tasksOpen, 8U);
}

// ---- travel and energy --------------------------------------------------------------------------

TEST(MetricsAccumulatorTest, TravelIsWhatTheRobotsAddedSinceTheStart) {
  std::vector<RobotObservation> start = fleet(2, Activity::Idle);
  start[0].cellsTravelled = 5;
  start[1].cellsTravelled = 100;
  start[1].deliveries = 4;
  MetricsAccumulator acc(options(), observation(0, start));
  std::vector<RobotObservation> now = start;
  now[0].cellsTravelled = 12;   // +7
  now[1].cellsTravelled = 103;  // +3
  now[1].deliveries = 6;        // +2
  acc.add(observation(1, now, {}, 0, {{TaskEnd::Completed, 0}, {TaskEnd::Completed, 0}}));
  const FleetMetrics& m = acc.report();
  EXPECT_EQ(m.robots[0].cellsTravelled, 7U);
  EXPECT_EQ(m.robots[1].cellsTravelled, 3U);
  EXPECT_EQ(m.robots[1].deliveries, 2U);
  EXPECT_EQ(m.cellsTravelled(), 10U);
  EXPECT_NEAR(*m.cellsPerRobot(), 5.0, kEps);
  EXPECT_NEAR(*m.cellsPerCompletedTask(), 5.0, kEps);  // 10 cells, two tasks
}

TEST(MetricsAccumulatorTest, CountsNearTheLimitOfTheirTypeDoNotOverflowTheDifference) {
  const std::uint64_t big = std::numeric_limits<std::uint64_t>::max() - 5;
  std::vector<RobotObservation> start = fleet(1, Activity::Idle);
  start[0].cellsTravelled = big;
  MetricsAccumulator acc(options(), observation(0, start));
  std::vector<RobotObservation> now = start;
  now[0].cellsTravelled = std::numeric_limits<std::uint64_t>::max();
  acc.add(observation(1, now));
  EXPECT_EQ(acc.report().robots[0].cellsTravelled, 5U);
}

TEST(MetricsAccumulatorTest, EnergyIsEstimatedFromTheTimeBudgetAndTheDistanceWithTheStatedModel) {
  MetricsOptions o = options();
  o.energy = EnergyModel{2, 10, 3};
  MetricsAccumulator acc(o, observation(0, fleet(1, Activity::Idle)));
  const Activity script[] = {Activity::Idle,     Activity::Idle,     Activity::Idle,
                             Activity::Idle,     Activity::Handling, Activity::Handling,
                             Activity::Handling, Activity::Moving,   Activity::Charging};
  std::uint64_t cells = 0;
  for (Tick t = 0; t < 9; ++t) {
    cells += script[t] == Activity::Moving ? 5U : 0U;
    std::vector<RobotObservation> robots = fleet(1, script[t]);
    robots[0].cellsTravelled = cells;
    acc.add(observation(
        t + 1, robots, {}, 0,
        t == 8 ? std::vector<TaskEnding>{{TaskEnd::Completed, 0}, {TaskEnd::Completed, 0}}
               : std::vector<TaskEnding>{}));
  }
  const auto energy = acc.report().energyEstimate();
  EXPECT_EQ(energy.idle, 4U * 2U);      // four idle ticks
  EXPECT_EQ(energy.moving, 5U * 10U);   // five cells
  EXPECT_EQ(energy.handling, 3U * 3U);  // three handling ticks; charging draws nothing here
  EXPECT_EQ(energy.total(), 67U);
  EXPECT_NEAR(*acc.report().energyPerCompletedTask(), 33.5, kEps);
}

// ---- conflicts and replanning -------------------------------------------------------------------

TEST(MetricsAccumulatorTest, AConflictPreventedIsOneEpisodeOfBeingHeldBackByAnotherRobot) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  const auto at = [&](Tick t, Activity a, WaitCause c) {
    std::vector<RobotObservation> robots = fleet(2, Activity::Idle);
    robots[0].activity = a;
    robots[0].waitCause = c;
    acc.add(observation(t, robots));
  };
  at(1, Activity::Waiting, WaitCause::Vertex);  // episode 1 begins
  at(2, Activity::Waiting, WaitCause::Vertex);
  at(3, Activity::Waiting, WaitCause::Swap);  // same episode: still held back by a robot
  at(4, Activity::Moving, WaitCause::None);
  at(5, Activity::Waiting, WaitCause::Swap);  // episode 2
  at(6, Activity::Moving, WaitCause::None);
  at(7, Activity::Waiting, WaitCause::Other);  // not a robot: no episode
  at(8, Activity::Waiting, WaitCause::Other);
  EXPECT_EQ(acc.report().conflictsPrevented, 2U);
  EXPECT_EQ(acc.report().robots[0].conflictsPrevented, 2U);
  EXPECT_EQ(acc.report().robots[1].conflictsPrevented, 0U);
  // but the waiting time counts all of it: ticks 1, 2, 3, 5, 7 and 8
  EXPECT_EQ(acc.report().ticksIn(Activity::Waiting), 6U);
}

TEST(MetricsAccumulatorTest, AWaitAlreadyUnderWayAtTheStartIsNotANewConflict) {
  std::vector<RobotObservation> start = fleet(1, Activity::Waiting);
  start[0].waitCause = WaitCause::Vertex;
  MetricsAccumulator acc(options(), observation(0, start));
  acc.add(observation(1, start));
  acc.add(observation(2, start));
  EXPECT_EQ(acc.report().conflictsPrevented, 0U);
  EXPECT_EQ(acc.report().ticksIn(Activity::Waiting), 2U);
}

TEST(MetricsAccumulatorTest, ConflictsOfSeveralRobotsAreCountedPerRobotAndSummed) {
  MetricsAccumulator acc(options(), observation(0, fleet(3, Activity::Idle)));
  for (Tick t = 1; t <= 4; ++t) {
    std::vector<RobotObservation> robots = fleet(3, Activity::Moving);
    if (t % 2 == 1) {
      robots[0].activity = robots[2].activity = Activity::Waiting;
      robots[0].waitCause = robots[2].waitCause = WaitCause::Vertex;
    }
    acc.add(observation(t, robots));
  }
  EXPECT_EQ(acc.report().robots[0].conflictsPrevented, 2U);
  EXPECT_EQ(acc.report().robots[1].conflictsPrevented, 0U);
  EXPECT_EQ(acc.report().robots[2].conflictsPrevented, 2U);
  EXPECT_EQ(acc.report().conflictsPrevented, 4U);
}

TEST(MetricsAccumulatorTest, ReplansAndDetectionsAreTheCountersGrowthSinceTheStart) {
  SystemCounters start;
  start.conflictDetections = 40;
  start.reroutes = 3;
  start.repaths = 5;
  start.obstacleRepairs = 1;
  start.searches = 20;
  MetricsAccumulator acc(options(36.0), observation(0, fleet(1, Activity::Idle), start));
  SystemCounters now = start;
  now.conflictDetections += 11;
  now.reroutes += 4;
  now.repaths += 2;
  now.obstacleRepairs += 1;
  now.searches += 30;
  acc.add(observation(1, fleet(1, Activity::Idle), now));
  const FleetMetrics& m = acc.report();
  EXPECT_EQ(m.conflictDetections, 11U);
  EXPECT_EQ(m.reroutes, 4U);
  EXPECT_EQ(m.repaths, 2U);
  EXPECT_EQ(m.obstacleRepairs, 1U);
  EXPECT_EQ(m.searches, 30U);
  EXPECT_EQ(m.replanEvents(), 7U);
  // One tick of 36 s is a hundredth of an hour.
  EXPECT_NEAR(*m.replansPerHour(), 700.0, 1e-9);
}

TEST(MetricsAccumulatorTest, DetectionsAreNotConflictsPreventedAndNeitherIsACollision) {
  SystemCounters counters;
  counters.conflictDetections = 50;  // the table finds the same clash again at every step
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  acc.add(observation(1, fleet(2, Activity::Moving), counters));
  EXPECT_EQ(acc.report().conflictDetections, 50U);
  EXPECT_EQ(acc.report().conflictsPrevented, 0U);
  EXPECT_EQ(acc.report().collisionsOccurred, 0U);
}

// ---- the audit for collisions -------------------------------------------------------------------

TEST(MetricsAccumulatorTest, RobotsInDifferentCellsAreNoCollision) {
  MetricsAccumulator acc(options(), observation(0, fleet(4, Activity::Idle)));
  idleFor(acc, 4, 10, Activity::Moving);
  EXPECT_EQ(acc.report().collisionsOccurred, 0U);
}

TEST(MetricsAccumulatorTest, TwoRobotsInOneCellAreOneCollisionForEveryTickTheyShareIt) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  for (Tick t = 1; t <= 3; ++t) {
    std::vector<RobotObservation> robots = fleet(2, Activity::Moving);
    robots[1].position = robots[0].position;
    acc.add(observation(t, robots));
  }
  EXPECT_EQ(acc.report().collisionsOccurred, 3U);
}

TEST(MetricsAccumulatorTest, ThreeRobotsInOneCellAreThreePairs) {
  MetricsAccumulator acc(options(), observation(0, fleet(3, Activity::Idle)));
  std::vector<RobotObservation> robots = fleet(3, Activity::Moving);
  robots[1].position = robots[0].position;
  robots[2].position = robots[0].position;
  acc.add(observation(1, robots));
  EXPECT_EQ(acc.report().collisionsOccurred, 3U);
}

TEST(MetricsAccumulatorTest, TwoRobotsThatChangePlacesAreACollisionCountedOnce) {
  std::vector<RobotObservation> start = fleet(2, Activity::Idle);
  start[0].position = {3, 4};
  start[1].position = {4, 4};
  MetricsAccumulator acc(options(), observation(0, start));
  std::vector<RobotObservation> now = start;
  std::swap(now[0].position, now[1].position);
  acc.add(observation(1, now));
  EXPECT_EQ(acc.report().collisionsOccurred, 1U);
}

TEST(MetricsAccumulatorTest, RobotsFollowingEachOtherOrTurningTheCornerAreNoCollision) {
  std::vector<RobotObservation> start = fleet(2, Activity::Idle);
  start[0].position = {1, 0};
  start[1].position = {0, 0};
  MetricsAccumulator acc(options(), observation(0, start));
  std::vector<RobotObservation> now = start;
  now[0].position = {2, 0};  // 0 leads, 1 follows into the cell 0 has just left
  now[1].position = {1, 0};
  acc.add(observation(1, now));
  now[0].position = {2, 1};
  now[1].position = {2, 0};
  acc.add(observation(2, now));
  EXPECT_EQ(acc.report().collisionsOccurred, 0U);
}

// ---- samples ------------------------------------------------------------------------------------

TEST(MetricsAccumulatorTest, ASampleIsTakenForEveryFullWindowAndNotForThePartialOneAtTheEnd) {
  MetricsAccumulator acc(options(1.0, 2), observation(0, fleet(2, Activity::Idle)));
  for (Tick t = 1; t <= 5; ++t) {
    std::vector<RobotObservation> robots = fleet(2, Activity::Idle);
    robots[0].activity = t <= 2 ? Activity::Moving : Activity::Waiting;
    robots[0].waitCause = t <= 2 ? WaitCause::None : WaitCause::Other;
    std::vector<TaskEnding> ended;
    if (t == 2 || t == 3) {
      ended.push_back({TaskEnd::Completed, 0});
    }
    acc.add(observation(t, robots, {}, 10 - t, ended));
  }
  const auto& samples = acc.report().samples;
  ASSERT_EQ(samples.size(), 2U);  // ticks 2 and 4; tick 5 is a window of one
  EXPECT_EQ(samples[0].tick, 2U);
  EXPECT_EQ(samples[0].windowTicks, 2U);
  EXPECT_EQ(samples[0].tasksCompleted, 1U);
  EXPECT_EQ(samples[0].busyRobotTicks, 2U);     // robot 0 moving twice
  EXPECT_EQ(samples[0].serviceRobotTicks, 4U);  // 2 robots x 2 ticks
  EXPECT_EQ(samples[0].waitingRobotTicks, 0U);
  EXPECT_EQ(samples[0].tasksOpen, 8U);
  EXPECT_EQ(samples[1].tick, 4U);
  EXPECT_EQ(samples[1].tasksCompleted, 1U);  // the one at tick 3 only
  EXPECT_EQ(samples[1].busyRobotTicks, 2U);  // waiting counts as busy
  EXPECT_EQ(samples[1].waitingRobotTicks, 2U);
  EXPECT_EQ(samples[1].tasksOpen, 6U);
}

TEST(MetricsAccumulatorTest, ASampleCountsMovingWaitingAndHandlingAsBusyAndTheRestAsNot) {
  const std::vector<std::pair<Activity, std::uint64_t>> cases = {
      {Activity::Moving, 1}, {Activity::Waiting, 1},  {Activity::Handling, 1},
      {Activity::Idle, 0},   {Activity::Charging, 0}, {Activity::Unavailable, 0}};
  for (const auto& [activity, busy] : cases) {
    MetricsAccumulator acc(options(1.0, 1), observation(0, fleet(1, Activity::Idle)));
    std::vector<RobotObservation> robots = fleet(1, activity);
    robots[0].waitCause = activity == Activity::Waiting ? WaitCause::Other : WaitCause::None;
    acc.add(observation(1, robots));
    ASSERT_EQ(acc.report().samples.size(), 1U);
    EXPECT_EQ(acc.report().samples[0].busyRobotTicks, busy) << toString(activity);
    EXPECT_EQ(acc.report().samples[0].serviceRobotTicks,
              activity == Activity::Unavailable ? 0U : 1U)
        << toString(activity);
  }
}

TEST(MetricsAccumulatorTest, OutOfServiceRobotsAreNotInTheServiceTimeOfASample) {
  MetricsAccumulator acc(options(1.0, 1), observation(0, fleet(2, Activity::Idle)));
  std::vector<RobotObservation> robots = fleet(2, Activity::Unavailable);
  robots[0].activity = Activity::Moving;
  acc.add(observation(1, robots));
  ASSERT_EQ(acc.report().samples.size(), 1U);
  EXPECT_EQ(acc.report().samples[0].serviceRobotTicks, 1U);
  EXPECT_EQ(acc.report().samples[0].busyRobotTicks, 1U);
}

TEST(MetricsAccumulatorTest, OnlyTheLatestSamplesAreKeptSoMemoryDoesNotGrowWithTheRun) {
  MetricsOptions o = options(1.0, 1);
  o.maxSamples = 3;
  MetricsAccumulator acc(o, observation(0, fleet(1, Activity::Idle)));
  idleFor(acc, 1, 10);
  const auto& samples = acc.report().samples;
  ASSERT_EQ(samples.size(), 3U);
  EXPECT_EQ(samples[0].tick, 8U);
  EXPECT_EQ(samples[2].tick, 10U);
}

TEST(MetricsAccumulatorTest, NoSamplesWhenTheIntervalIsZeroOrTheCapIsZero) {
  MetricsAccumulator none(options(1.0, 0), observation(0, fleet(1, Activity::Idle)));
  idleFor(none, 1, 10);
  EXPECT_TRUE(none.report().samples.empty());

  MetricsOptions o = options(1.0, 1);
  o.maxSamples = 0;
  MetricsAccumulator capped(o, observation(0, fleet(1, Activity::Idle)));
  idleFor(capped, 1, 10);
  EXPECT_TRUE(capped.report().samples.empty());
}

// ---- refusing what cannot be measured -----------------------------------------------------------

TEST(MetricsAccumulatorTest, ASkippedTickIsRefusedAndLeavesTheReportAsItWas) {
  MetricsAccumulator acc(options(), observation(0, fleet(1, Activity::Idle)));
  idleFor(acc, 1, 3);
  const FleetMetrics before = acc.report();
  EXPECT_THROW(acc.add(observation(5, fleet(1, Activity::Moving))), std::invalid_argument);
  EXPECT_THROW(acc.add(observation(3, fleet(1, Activity::Moving))),
               std::invalid_argument);  // again
  EXPECT_THROW(acc.add(observation(0, fleet(1, Activity::Moving))), std::invalid_argument);  // back
  EXPECT_EQ(acc.ticksObserved(), 3U);
  EXPECT_EQ(acc.lastTick(), 3U);
  EXPECT_TRUE(acc.report().ticks == before.ticks && acc.report().robots == before.robots);
  acc.add(observation(4, fleet(1, Activity::Moving)));  // and the right one still works
  EXPECT_EQ(acc.ticksObserved(), 4U);
}

TEST(MetricsAccumulatorTest, AFleetThatChangesIsRefused) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  EXPECT_THROW(acc.add(observation(1, fleet(3, Activity::Idle))), std::invalid_argument);
  EXPECT_THROW(acc.add(observation(1, fleet(1, Activity::Idle))), std::invalid_argument);
  std::vector<RobotObservation> swapped = fleet(2, Activity::Idle);
  std::swap(swapped[0], swapped[1]);
  EXPECT_THROW(acc.add(observation(1, swapped)), std::invalid_argument);
  std::vector<RobotObservation> renamed = fleet(2, Activity::Idle);
  renamed[1].id = RobotId{9};
  EXPECT_THROW(acc.add(observation(1, renamed)), std::invalid_argument);
  EXPECT_EQ(acc.ticksObserved(), 0U);
}

TEST(MetricsAccumulatorTest, ACountThatGoesDownIsRefused) {
  std::vector<RobotObservation> start = fleet(1, Activity::Idle);
  start[0].cellsTravelled = 10;
  start[0].deliveries = 2;
  SystemCounters counters;
  counters.reroutes = 5;
  MetricsAccumulator acc(options(), observation(0, start, counters));
  std::vector<RobotObservation> back = start;
  back[0].cellsTravelled = 9;
  EXPECT_THROW(acc.add(observation(1, back, counters)), std::invalid_argument);
  back = start;
  back[0].deliveries = 1;
  EXPECT_THROW(acc.add(observation(1, back, counters)), std::invalid_argument);
  SystemCounters lower = counters;
  lower.reroutes = 4;
  EXPECT_THROW(acc.add(observation(1, start, lower)), std::invalid_argument);
  SystemCounters fewer = counters;
  fewer.tasksSubmitted = 0;
  SystemCounters more = counters;
  more.tasksSubmitted = 3;
  acc.add(observation(1, start, more));
  EXPECT_THROW(acc.add(observation(2, start, fewer)), std::invalid_argument);
  EXPECT_EQ(acc.ticksObserved(), 1U);
}

TEST(MetricsAccumulatorTest, ATaskThatEndedBeforeItWasCreatedIsRefused) {
  MetricsAccumulator acc(options(), observation(0, fleet(1, Activity::Idle)));
  EXPECT_THROW(acc.add(observation(1, fleet(1, Activity::Idle), {}, 0, {{TaskEnd::Completed, 2}})),
               std::invalid_argument);
  EXPECT_EQ(acc.report().tasksCompleted, 0U);
  EXPECT_EQ(acc.ticksObserved(), 0U);
}

TEST(MetricsAccumulatorTest, TwoRobotsWithOneIdAreRefusedAtTheStart) {
  std::vector<RobotObservation> twins = fleet(2, Activity::Idle);
  twins[1].id = twins[0].id;
  EXPECT_THROW(MetricsAccumulator(options(), observation(0, twins)), std::invalid_argument);
}

TEST(MetricsAccumulatorTest, ATickThatIsNotFiniteAndPositiveIsRefusedAsATimeScale) {
  for (const double bad :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(MetricsAccumulator(options(bad), observation(0, fleet(1, Activity::Idle))),
                 std::invalid_argument)
        << bad;
  }
  EXPECT_NO_THROW(MetricsAccumulator(options(1e-9), observation(0, fleet(1, Activity::Idle))));
  EXPECT_NO_THROW(MetricsAccumulator(options(1e9), observation(0, fleet(1, Activity::Idle))));
}

TEST(MetricsAccumulatorTest, AnIntervalNeedNotStartAtTickZero) {
  MetricsAccumulator acc(options(), observation(1000, fleet(1, Activity::Idle)));
  EXPECT_THROW(acc.add(observation(1, fleet(1, Activity::Idle))), std::invalid_argument);
  acc.add(observation(1001, fleet(1, Activity::Idle)));
  EXPECT_EQ(acc.ticksObserved(), 1U);
  EXPECT_EQ(acc.lastTick(), 1001U);
}

// ---- reading it twice ---------------------------------------------------------------------------

TEST(MetricsAccumulatorTest, ReadingTheReportDoesNotChangeIt) {
  MetricsAccumulator acc(options(), observation(0, fleet(2, Activity::Idle)));
  idleFor(acc, 2, 5, Activity::Moving);
  const FleetMetrics first = acc.report();
  (void)acc.report().utilization();
  (void)acc.report().energyEstimate();
  EXPECT_TRUE(acc.report().robots == first.robots);
  EXPECT_EQ(acc.report().ticks, first.ticks);
}

TEST(MetricsAccumulatorTest, TheSameObservationsAlwaysGiveTheSameReport) {
  const auto run = [] {
    MetricsAccumulator acc(options(1.0, 4), observation(0, fleet(3, Activity::Idle)));
    for (Tick t = 1; t <= 50; ++t) {
      std::vector<RobotObservation> robots = fleet(3, Activity::Idle);
      for (std::size_t r = 0; r < 3; ++r) {
        robots[r].activity = warehousesim::metrics::kAllActivities[(t + r) % 6];
        robots[r].waitCause =
            robots[r].activity == Activity::Waiting ? WaitCause::Vertex : WaitCause::None;
        robots[r].cellsTravelled = t * (r + 1);
      }
      acc.add(observation(t, robots, {}, t % 7,
                          t % 5 == 0 ? std::vector<TaskEnding>{{TaskEnd::Completed, t - 3}}
                                     : std::vector<TaskEnding>{}));
    }
    return acc.report();
  };
  const FleetMetrics a = run();
  const FleetMetrics b = run();
  EXPECT_TRUE(a.robots == b.robots && a.samples == b.samples);
  EXPECT_EQ(a.tasksCompleted, b.tasksCompleted);
  EXPECT_EQ(a.completionTicksTotal, b.completionTicksTotal);
}

TEST(MetricsAccumulatorPropertyTest,
     WhateverTheObservationsTheBudgetAddsUpAndFractionsStayInRange) {
  std::uint64_t state = 12345;
  const auto next = [&state] {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return state >> 33;
  };
  for (int round = 0; round < 40; ++round) {
    const std::size_t robots = 1 + next() % 5;
    MetricsAccumulator acc(options(1.0, 1 + next() % 7),
                           observation(0, fleet(robots, Activity::Idle)));
    std::vector<std::uint64_t> cells(robots, 0);
    for (Tick t = 1; t <= 60; ++t) {
      std::vector<RobotObservation> now = fleet(robots, Activity::Idle);
      for (std::size_t r = 0; r < robots; ++r) {
        now[r].activity = warehousesim::metrics::kAllActivities[next() % 6];
        now[r].waitCause = now[r].activity == Activity::Waiting
                               ? static_cast<WaitCause>(1 + next() % 3)
                               : WaitCause::None;
        cells[r] += next() % 2;
        now[r].cellsTravelled = cells[r];
      }
      acc.add(observation(t, now));
    }
    const FleetMetrics& m = acc.report();
    ASSERT_EQ(m.robotTicks(), robots * 60U);
    for (const auto& r : m.robots) {
      ASSERT_EQ(r.budget.total(), 60U);
    }
    for (const Activity a : warehousesim::metrics::kAllActivities) {
      ASSERT_TRUE(m.fraction(a).has_value());
      ASSERT_GE(*m.fraction(a), 0.0);
      ASSERT_LE(*m.fraction(a), 1.0);
    }
    if (m.utilization()) {
      ASSERT_GE(*m.utilization(), 0.0);
      ASSERT_LE(*m.utilization(), 1.0);
    }
    for (const auto& s : m.samples) {
      ASSERT_LE(s.busyRobotTicks, s.serviceRobotTicks);
      ASSERT_LE(s.waitingRobotTicks, s.busyRobotTicks);
    }
  }
}

}  // namespace
