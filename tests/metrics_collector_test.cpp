#include "warehousesim/simulation/metrics_collector.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using namespace warehousesim_test;
using warehousesim::metrics::Activity;
using warehousesim::metrics::EnergyModel;
using warehousesim::metrics::FleetMetrics;
using warehousesim::metrics::MetricsOptions;
using warehousesim::robots::RobotState;
using warehousesim::simulation::energyModelOf;
using warehousesim::simulation::MetricsCollector;
using warehousesim::simulation::ScenarioOptions;
using warehousesim::traffic::ConflictKind;

constexpr double kEps = 1e-9;

// Runs `ticks` steps of `sim`, measured from where it is now.
FleetMetrics measured(Simulation& sim, std::uint64_t ticks, MetricsOptions options = {}) {
  MetricsCollector collector(sim, options);
  for (std::uint64_t i = 0; i < ticks; ++i) {
    sim.step();
    collector.observe(sim);
  }
  return collector.report();
}

std::vector<std::string> bundledLayouts() {
  std::vector<std::string> names;
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    names.emplace_back(info.name);
  }
  return names;
}

// ---- measuring changes nothing ------------------------------------------------------------------

TEST(MetricsCollectorTest, AMeasuredRunIsTheSameRunStepForStepAsAnUnmeasuredOne) {
  for (const std::string& name : bundledLayouts()) {
    Simulation plain = *warehousesim::simulation::makeLayoutSimulation(name, 5);
    Simulation watched = *warehousesim::simulation::makeLayoutSimulation(name, 5);
    MetricsCollector collector(watched);
    for (int tick = 0; tick < 400; ++tick) {
      plain.step();
      watched.step();
      collector.observe(watched);
      ASSERT_TRUE(plain.snapshot() == watched.snapshot())
          << name << " differs at tick " << tick + 1;
    }
    EXPECT_EQ(collector.report().ticks, 400U);
  }
}

TEST(MetricsCollectorTest, ReadingTheReportAsOftenAsOneLikesChangesNothingEither) {
  Simulation a = warehousesim::simulation::makeDemoSimulation(2);
  Simulation b = warehousesim::simulation::makeDemoSimulation(2);
  MetricsCollector watched(a);
  for (int i = 0; i < 200; ++i) {
    a.step();
    b.step();
    watched.observe(a);
    (void)watched.report().utilization();
    (void)warehousesim::metrics::dashboardLines(watched.report());
  }
  EXPECT_TRUE(a.snapshot() == b.snapshot());
}

// ---- the books balance against what the simulation itself says ----------------------------------

TEST(MetricsCollectorTest, TheTaskAndTravelCountsAreTheSimulationsOwn) {
  for (const std::string& name : bundledLayouts()) {
    Simulation sim = *warehousesim::simulation::makeLayoutSimulation(name, 3);
    const FleetMetrics m = measured(sim, 600);
    const auto snapshot = sim.snapshot();
    EXPECT_EQ(m.tasksCompleted, snapshot.completedTasks) << name;
    EXPECT_EQ(m.tasksFailed, snapshot.failedTasks) << name;
    EXPECT_EQ(m.tasksCancelled, snapshot.cancelledTasks) << name;
    EXPECT_EQ(m.tasksOpen, snapshot.activeTasks.size()) << name;
    EXPECT_EQ(m.tasksSubmitted, sim.tasks().all().size()) << name;
    EXPECT_EQ(m.tasksSubmitted, m.tasksCompleted + m.tasksFailed + m.tasksCancelled + m.tasksOpen)
        << name << ": every task is somewhere";
    std::uint64_t cells = 0;
    std::uint64_t deliveries = 0;
    for (const auto& robot : sim.robots()) {
      cells += robot.cellsTravelled();
      deliveries += robot.deliveries();
    }
    EXPECT_EQ(m.cellsTravelled(), cells) << name;
    EXPECT_EQ(deliveries, m.tasksCompleted) << name << ": a delivery is a completed task";
    EXPECT_EQ(m.conflictDetections, sim.trafficStats().vertex + sim.trafficStats().swaps) << name;
    EXPECT_EQ(m.reroutes, sim.planningStats().reroutes) << name;
    EXPECT_EQ(m.repaths, sim.planningStats().repaths) << name;
    EXPECT_EQ(m.obstacleRepairs, sim.planningStats().obstacleRepairs) << name;
    EXPECT_EQ(m.searches, sim.planningStats().searches) << name;
  }
}

TEST(MetricsCollectorTest, EveryRobotIsAccountedForInEveryTick) {
  Simulation sim = *warehousesim::simulation::makeLayoutSimulation("fulfilment", 4);
  const FleetMetrics m = measured(sim, 500);
  ASSERT_EQ(m.robots.size(), sim.robots().size());
  for (const auto& robot : m.robots) {
    EXPECT_EQ(robot.budget.total(), 500U);
  }
  EXPECT_EQ(m.robotTicks(), 500U * sim.robots().size());
}

TEST(MetricsCollectorTest, NoBundledLayoutEverHasARobotCollideWithAnother) {
  for (const std::string& name : bundledLayouts()) {
    for (const std::uint64_t seed : {1U, 2U, 3U}) {
      Simulation sim = *warehousesim::simulation::makeLayoutSimulation(name, seed);
      const FleetMetrics m = measured(sim, 500);
      EXPECT_EQ(m.collisionsOccurred, 0U) << name << " seed " << seed;
    }
  }
}

TEST(MetricsCollectorTest, TheSameRunMeasuredTwiceGivesTheSameFigures) {
  Simulation a = *warehousesim::simulation::makeLayoutSimulation("compact", 8);
  Simulation b = *warehousesim::simulation::makeLayoutSimulation("compact", 8);
  const FleetMetrics ma = measured(a, 700);
  const FleetMetrics mb = measured(b, 700);
  const warehousesim::metrics::RunLabel label{"compact", 8, 5};
  EXPECT_EQ(warehousesim::metrics::runsCsvRow(label, ma),
            warehousesim::metrics::runsCsvRow(label, mb));
  EXPECT_EQ(warehousesim::metrics::robotsCsv(ma), warehousesim::metrics::robotsCsv(mb));
  EXPECT_EQ(warehousesim::metrics::seriesCsv(ma), warehousesim::metrics::seriesCsv(mb));
}

TEST(MetricsCollectorTest, AnotherSeedIsAnotherRunWithOtherFigures) {
  Simulation a = *warehousesim::simulation::makeLayoutSimulation("demo", 1);
  Simulation b = *warehousesim::simulation::makeLayoutSimulation("demo", 2);
  const warehousesim::metrics::RunLabel label{"demo", 0, 5};
  EXPECT_NE(warehousesim::metrics::runsCsvRow(label, measured(a, 600)),
            warehousesim::metrics::runsCsvRow(label, measured(b, 600)));
}

// ---- hand-checked scenarios ---------------------------------------------------------------------

TEST(MetricsCollectorTest, ALoneRobotDoingOneTaskIsNeverHeldBackAndItsTimeIsExactlyAccountedFor) {
  Scenario scenario(layout({"P...D"}), {Job{P(2, 0), P(0, 0), P(4, 0)}}, ideal());
  MetricsCollector collector(scenario.sim);
  std::uint64_t navigating = 0;
  std::uint64_t idle = 0;
  std::uint64_t handling = 0;
  const auto done = runToCompletion(scenario, 100, [&](const Simulation& sim) {
    collector.observe(sim);
    switch (sim.robots()[0].state()) {
      case RobotState::Navigating:
        ++navigating;
        break;
      case RobotState::Idle:
        ++idle;
        break;
      case RobotState::Loading:
      case RobotState::Unloading:
        ++handling;
        break;
      default:
        ADD_FAILURE() << "unexpected state";
    }
  });
  ASSERT_GT(done[0], 0U);
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(m.ticks, done[0]);
  EXPECT_EQ(m.tasksCompleted, 1U);
  EXPECT_EQ(m.completionTicksTotal, done[0]);  // made at tick 0, delivered at tick done[0]
  EXPECT_EQ(m.completionTicksMax, done[0]);
  EXPECT_EQ(m.ticksIn(Activity::Moving), navigating);
  EXPECT_EQ(m.ticksIn(Activity::Idle), idle);
  EXPECT_EQ(m.ticksIn(Activity::Handling), handling);
  EXPECT_EQ(m.ticksIn(Activity::Waiting), 0U);
  EXPECT_EQ(m.conflictsPrevented, 0U);
  EXPECT_EQ(m.collisionsOccurred, 0U);
  EXPECT_EQ(m.cellsTravelled(), 2U + 4U);  // two cells to the pickup, four to the dropoff
  EXPECT_EQ(m.robots[0].deliveries, 1U);
  EXPECT_EQ(m.replanEvents(), 0U);
}

TEST(MetricsCollectorTest, TheRobotThatGivesWayAtACrossingIsTheOneThatIsHeldBack) {
  const std::vector<Job> jobs{{P(0, 5), P(10, 5), P(10, 4)}, {P(5, 0), P(5, 10), P(4, 10)}};
  Scenario scenario(openFloor(11, 11), jobs, ideal());
  MetricsCollector collector(scenario.sim);
  std::uint64_t heldTicks[2] = {0, 0};
  const auto done = runToCompletion(scenario, 400, [&](const Simulation& sim) {
    collector.observe(sim);
    for (std::size_t i = 0; i < 2; ++i) {
      const auto held = sim.heldBack(sim.robots()[i].id());
      if (held) {
        ++heldTicks[i];
        // What "held back" means: it wanted to go, is still where it was, and is at rest.
        EXPECT_EQ(sim.robots()[i].state(), RobotState::Navigating);
        EXPECT_EQ(sim.robots()[i].speed(), 0.0);
      }
    }
  });
  ASSERT_GT(done[0], 0U);
  ASSERT_GT(done[1], 0U);
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(heldTicks[0], 0U);  // robot 1 has priority and drives as if alone
  ASSERT_GT(heldTicks[1], 0U);
  EXPECT_EQ(m.robots[0].budget.in(Activity::Waiting), 0U);
  EXPECT_EQ(m.robots[1].budget.in(Activity::Waiting), heldTicks[1]);
  EXPECT_EQ(m.robots[0].conflictsPrevented, 0U);
  EXPECT_GE(m.robots[1].conflictsPrevented, 1U);
  EXPECT_LE(m.robots[1].conflictsPrevented, heldTicks[1]);  // an episode is at least a tick
  EXPECT_EQ(m.conflictsPrevented, m.robots[1].conflictsPrevented);
  EXPECT_EQ(m.collisionsOccurred, 0U);
  EXPECT_EQ(m.completionTicksTotal, done[0] + done[1]);
  EXPECT_EQ(m.completionTicksMax, std::max(done[0], done[1]));
  EXPECT_GT(m.conflictDetections, 0U);
}

TEST(MetricsCollectorTest, ALongWaitIsOneConflictPreventedAndManyTicksOfWaitingTime) {
  // Robot 2 is held outside the corridor until robot 1 has come all the way through it.
  Scenario scenario(hall(),
                    std::vector<Job>{{P(0, 1), P(13, 0), P(14, 0)}, {P(14, 2), P(1, 0), P(0, 0)}},
                    ideal());
  MetricsCollector collector(scenario.sim);
  std::uint64_t refused = 0;
  std::uint64_t waitingInSnapshot = 0;
  (void)runToCompletion(scenario, 400, [&](const Simulation& sim) {
    collector.observe(sim);
    const auto snapshot = sim.snapshot();
    for (const auto& robot : sim.robots()) {
      refused += sim.heldBack(robot.id()) ? 1 : 0;
      waitingInSnapshot += snapshot.robots[robot.id().value - 1].waiting ? 1 : 0;
    }
  });
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(m.ticksIn(Activity::Waiting), refused);
  EXPECT_GE(refused, 5U);  // a real wait, not a blink
  EXPECT_EQ(m.robots[0].budget.in(Activity::Waiting), 0U);
  EXPECT_GE(m.robots[1].conflictsPrevented, 1U);
  EXPECT_LT(m.robots[1].conflictsPrevented, refused);  // the ticks of one wait are one conflict
  // The snapshot's `waiting` follows the robot's own counter of ticks blocked, which starts over
  // whenever it tries to resolve a block: measuring with it would have shown a long wait as a run
  // of shorter ones with gaps. Both see the wait; the collector's answer is one per step.
  EXPECT_GT(waitingInSnapshot, 0U);
}

TEST(MetricsCollectorTest, ARobotThatIsTurningOrAcceleratingIsMovingNotWaiting) {
  Scenario scenario(openFloor(12, 12), std::vector<Job>{{P(0, 0), P(6, 0), P(6, 8)}}, vehicles());
  MetricsCollector collector(scenario.sim);
  (void)runToCompletion(scenario, 400, [&](const Simulation& sim) { collector.observe(sim); });
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(m.ticksIn(Activity::Waiting), 0U);
  EXPECT_GT(m.ticksIn(Activity::Moving), m.cellsTravelled() / 2);  // it takes time to get going
  EXPECT_GT(m.ticksIn(Activity::Handling), 0U);                    // vehicles load and unload
}

// ---- waits and conflicts are what the engine says they are, step by step ------------------------

// Whether the engine held robot `index` back this step for another robot (a clash of either kind).
bool heldByARobot(const Simulation& sim, std::size_t index) {
  const auto held = sim.heldBack(sim.robots()[index].id());
  return held && held->kind != ConflictKind::Blocked;
}

// Steps `sim` and feeds the collector; returns, for each robot, how many waits for another robot
// began during those steps, counted here from the engine's own answer after each step. A wait that
// is under way when this starts is not one that begins inside it.
std::vector<std::uint64_t> stepCountingWaits(Simulation& sim, MetricsCollector& collector,
                                             int ticks) {
  std::vector<std::uint64_t> begun(sim.robots().size(), 0);
  std::vector<bool> before;
  for (std::size_t i = 0; i < sim.robots().size(); ++i) {
    before.push_back(heldByARobot(sim, i));
  }
  for (int tick = 0; tick < ticks; ++tick) {
    sim.step();
    collector.observe(sim);
    for (std::size_t i = 0; i < sim.robots().size(); ++i) {
      const bool now = heldByARobot(sim, i);
      begun[i] += now && !before[i] ? 1 : 0;
      before[i] = now;
    }
  }
  return begun;
}

std::vector<Job> oppositeRooms() {
  return {{P(0, 1), P(13, 0), P(14, 0)}, {P(14, 2), P(1, 0), P(0, 0)}};
}

TEST(MetricsCollectorTest, ConflictsPreventedAreTheWaitsForAnotherRobotAsTheEngineReportsThem) {
  for (const SimulationConfig& config : {ideal(), vehicles()}) {
    Scenario scenario(hall(), oppositeRooms(), config);
    MetricsCollector collector(scenario.sim);
    const auto begun = stepCountingWaits(scenario.sim, collector, 120);
    const FleetMetrics& m = collector.report();
    ASSERT_EQ(m.robots.size(), 2U);
    EXPECT_EQ(m.robots[0].conflictsPrevented, begun[0]);
    EXPECT_EQ(m.robots[1].conflictsPrevented, begun[1]);
    // The engine calls the ticks of this one wait a clash of cells and a swap of cells in turn; it
    // is still one wait, and whichever way a tick is labelled it is a wait for another robot.
    EXPECT_EQ(m.robots[1].conflictsPrevented, 1U);
    EXPECT_GE(m.robots[1].budget.in(Activity::Waiting), 5U);
  }
}

TEST(MetricsCollectorTest,
     AWaitAlreadyUnderWayWhenMeasuringBeginsIsNotAConflictThatBeginsInsideIt) {
  Scenario scenario(hall(), oppositeRooms(), ideal());
  for (int i = 0; i < 5; ++i) {
    scenario.sim.step();
  }
  ASSERT_TRUE(heldByARobot(scenario.sim, 1)) << "robot 2 should be in the middle of its wait";
  MetricsCollector collector(scenario.sim);
  const auto begun = stepCountingWaits(scenario.sim, collector, 100);
  EXPECT_EQ(begun[1], 0U);
  EXPECT_EQ(collector.report().robots[1].conflictsPrevented, 0U);
  EXPECT_GT(collector.report().robots[1].budget.in(Activity::Waiting),
            0U);  // the rest of it is time
}

TEST(MetricsCollectorTest, ARobotThatDroveInAStepIsNotWaitingInIt) {
  // Robot 2 stands parked in a road one cell wide. Robot 1 drives up to it and, whatever it is,
  // comes to rest part-way through a step: it moved in that step, and is refused only from the next
  // on.
  SimulationConfig slow = ideal();
  slow.robot.maxSpeed = 0.4;
  for (SimulationConfig config : {slow, vehicles()}) {
    config.maxBlockedTicks = 1000;  // nobody gets pushed aside
    Scenario scenario(openFloor(10, 1), {P(0, 0), P(6, 0)}, {Errand{0, P(8, 0), P(9, 0)}}, config);
    MetricsCollector collector(scenario.sim);
    bool arrived = false;
    std::uint64_t refused = 0;
    for (int tick = 0; tick < 80; ++tick) {
      const auto before = scenario.sim.robots()[0].pose();
      scenario.sim.step();
      collector.observe(scenario.sim);
      const Robot& robot = scenario.sim.robots()[0];
      const bool moved = robot.pose().x != before.x;
      if (scenario.sim.heldBack(robot.id())) {
        ++refused;
        EXPECT_FALSE(moved) << "held back at tick " << scenario.sim.tick() << " and yet it moved";
      } else if (moved && robot.speed() == 0.0 && robot.position().x == 5) {
        arrived = true;  // it drove up to the parked robot and stopped, in this very step
      }
    }
    EXPECT_TRUE(arrived);
    EXPECT_GT(refused, 40U);
    EXPECT_EQ(collector.report().robots[0].budget.in(Activity::Waiting), refused);
  }
}

TEST(MetricsCollectorTest, ARobotOnAChargerIsChargingNotIdleAndAnIdleOneIsNotCharging) {
  SimulationConfig config;
  config.robot.chargePerTick = 1000;
  config.robot.lowBatteryLevel = 5000;
  config.robot.initialCharge = 1000;
  Simulation sim(layout({"C.."}), {P(0, 0)}, config);
  MetricsCollector collector(sim);
  std::uint64_t charging = 0;
  std::uint64_t idle = 0;
  for (int i = 0; i < 15; ++i) {
    sim.step();
    collector.observe(sim);
    charging += sim.robots()[0].state() == RobotState::Charging ? 1 : 0;
    idle += sim.robots()[0].state() == RobotState::Idle ? 1 : 0;
  }
  ASSERT_GT(charging, 5U);  // charged up from a low battery,
  ASSERT_GT(idle, 0U);      // and then stood idle
  EXPECT_EQ(charging + idle, 15U);
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(m.ticksIn(Activity::Charging), charging);
  EXPECT_EQ(m.ticksIn(Activity::Idle), idle);
  ASSERT_TRUE(m.utilization().has_value());
  EXPECT_EQ(*m.utilization(), 0.0);  // charging is not busy
}

TEST(MetricsCollectorTest, ATasksCompletionTimeRunsFromWhenItWasMadeNotFromTheStartOfTheRun) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P...D"}), {P(2, 0)}, config);
  MetricsCollector collector(sim);
  for (int i = 0; i < 7; ++i) {
    sim.step();
    collector.observe(sim);
  }
  const TaskId id = sim.submitTask(P(0, 0), P(4, 0));
  const warehousesim::core::Tick madeAt = sim.tick();
  ASSERT_EQ(madeAt, 7U);
  warehousesim::core::Tick doneAt = 0;
  for (int i = 0; i < 60 && doneAt == 0; ++i) {
    sim.step();
    collector.observe(sim);
    doneAt = sim.tasks().find(id)->status == TaskStatus::Completed ? sim.tick() : 0;
  }
  ASSERT_GT(doneAt, madeAt);
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(m.tasksCompleted, 1U);
  EXPECT_EQ(m.completionTicksTotal, doneAt - madeAt);
  EXPECT_EQ(m.completionTicksMax, doneAt - madeAt);
}

TEST(MetricsCollectorTest, ReplansAreCountedWhereTheyHappenInCrowdedTraffic) {
  ScenarioOptions options;
  options.robots = 8;
  Simulation sim = (*warehousesim::simulation::layoutScenario("compact", options))(1);
  const FleetMetrics m = measured(sim, 800);
  // Eight robots in a small room are held up and go round each other; the counts are the engine's
  // own and must really be there, or equality with them would hold for zeros.
  EXPECT_GT(sim.planningStats().reroutes, 0U);
  EXPECT_GT(sim.planningStats().repaths, 0U);
  EXPECT_EQ(m.reroutes, sim.planningStats().reroutes);
  EXPECT_EQ(m.repaths, sim.planningStats().repaths);
  EXPECT_EQ(m.replanEvents(), m.reroutes + m.repaths + m.obstacleRepairs);
  EXPECT_GT(*m.replansPerHour(), 0.0);
}

TEST(MetricsCollectorTest, AStepRefusedForBeingTooFarAheadLeavesNoTraceOfWhatItShowed) {
  const auto make = [] { return *warehousesim::simulation::makeLayoutSimulation("compact", 4); };
  Simulation watched = make();
  Simulation reference = make();  // the same run, measured without any mishap
  Simulation ahead = make();      // the same run, seen 60 ticks further on
  ahead.run(60);
  MetricsCollector collector(watched);
  MetricsCollector undisturbed(reference);
  watched.step();
  reference.step();
  collector.observe(watched);
  undisturbed.observe(reference);
  EXPECT_THROW(collector.observe(ahead), std::invalid_argument);
  for (int i = 0; i < 400; ++i) {
    watched.step();
    reference.step();
    collector.observe(watched);
    undisturbed.observe(reference);
  }
  const FleetMetrics& a = collector.report();
  const FleetMetrics& b = undisturbed.report();
  EXPECT_EQ(a.ticks, b.ticks);
  EXPECT_EQ(a.tasksSubmitted, b.tasksSubmitted);
  EXPECT_EQ(a.tasksCompleted, b.tasksCompleted);
  EXPECT_EQ(a.tasksFailed, b.tasksFailed);
  EXPECT_EQ(a.tasksOpen, b.tasksOpen);
  EXPECT_EQ(a.completionTicksTotal, b.completionTicksTotal);
  EXPECT_EQ(a.completionTicksMax, b.completionTicksMax);
  EXPECT_GT(b.tasksCompleted, 0U);
}

TEST(MetricsCollectorTest, ARobotOutOfServiceIsLeftOutOfUtilizationAndRestoredRobotsComeBack) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P...D", "....."}), {P(0, 1), P(4, 1)}, config);
  MetricsCollector collector(sim);
  ASSERT_TRUE(sim.disableRobot(RobotId{2}));
  for (int i = 0; i < 10; ++i) {
    sim.step();
    collector.observe(sim);
  }
  ASSERT_TRUE(sim.restoreRobot(RobotId{2}));
  for (int i = 0; i < 10; ++i) {
    sim.step();
    collector.observe(sim);
  }
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(m.robots[1].budget.in(Activity::Unavailable), 10U);
  EXPECT_EQ(m.robots[1].budget.in(Activity::Idle), 10U);
  EXPECT_EQ(m.robots[0].budget.in(Activity::Unavailable), 0U);
  // Nothing to do for either robot, so none of the 30 in-service robot-ticks is busy.
  ASSERT_TRUE(m.utilization().has_value());
  EXPECT_EQ(*m.utilization(), 0.0);
}

TEST(MetricsCollectorTest, TasksAddedAndCancelledByHandAreCountedAsSuch) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P...D", "....."}), {P(0, 1)}, config);
  MetricsCollector collector(sim);
  const TaskId kept = sim.submitTask(P(0, 0), P(4, 0));
  const TaskId dropped = sim.submitTask(P(1, 0), P(3, 0));
  ASSERT_TRUE(sim.cancelTask(dropped));
  for (int i = 0; i < 40; ++i) {
    sim.step();
    collector.observe(sim);
  }
  const FleetMetrics& m = collector.report();
  EXPECT_EQ(m.tasksSubmitted, 2U);
  EXPECT_EQ(m.tasksCancelled, 1U);
  EXPECT_EQ(m.tasksCompleted, 1U);
  EXPECT_EQ(sim.tasks().find(kept)->status, TaskStatus::Completed);
  EXPECT_EQ(m.tasksOpen, 0U);
}

TEST(MetricsCollectorTest, ATaskTheDispatcherRejectsAtOnceIsAFailureNotACompletion) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P.#.D", "..#.."}), {P(0, 1)}, config);  // a wall splits the floor
  MetricsCollector collector(sim);
  (void)sim.submitTask(P(0, 0), P(4, 0));
  for (int i = 0; i < 5; ++i) {
    sim.step();
    collector.observe(sim);
  }
  EXPECT_EQ(collector.report().tasksFailed, 1U);
  EXPECT_EQ(collector.report().tasksCompleted, 0U);
  EXPECT_FALSE(collector.report().meanCompletionTicks().has_value());
}

TEST(MetricsCollectorTest, TheTrafficCountsKeepCountingWhenTheLayoutIsEdited) {
  Simulation sim = *warehousesim::simulation::makeLayoutSimulation("demo", 2);
  std::uint64_t before = 0;
  Point edited{0, 0};
  for (int i = 0; i < 400 && before == 0; ++i) {
    sim.step();
    before = sim.trafficStats().vertex + sim.trafficStats().swaps;
  }
  ASSERT_GT(before, 0U) << "the demo hall should see a clash within 400 ticks";
  bool placed = false;
  for (int x = 1; x < 19 && !placed; ++x) {
    edited = Point{x, 3};
    placed = sim.placeObstacle(edited);
  }
  ASSERT_TRUE(placed);
  EXPECT_GE(sim.trafficStats().vertex + sim.trafficStats().swaps, before);
  EXPECT_GE(sim.snapshot().traffic.vertex + sim.snapshot().traffic.swaps, before);
  sim.run(50);
  EXPECT_GE(sim.trafficStats().vertex + sim.trafficStats().swaps, before);
  EXPECT_TRUE(sim.clearObstacle(edited));
  EXPECT_GE(sim.trafficStats().vertex + sim.trafficStats().swaps, before);
}

TEST(MetricsCollectorTest, ObstaclesPlacedWhileRunningAreRepairsOfRoutes) {
  Simulation sim = *warehousesim::simulation::makeLayoutSimulation("demo", 1);
  MetricsCollector collector(sim);
  for (int i = 0; i < 300; ++i) {
    sim.step();
    collector.observe(sim);
    if (i % 25 == 0) {
      (void)sim.placeObstacle(Point{(i / 25 * 3) % 18 + 1, 3});  // refused if a robot is there
    }
  }
  EXPECT_EQ(collector.report().obstacleRepairs, sim.planningStats().obstacleRepairs);
  EXPECT_EQ(collector.report().collisionsOccurred, 0U);
  EXPECT_EQ(collector.report().tasksSubmitted, sim.tasks().all().size());
}

// ---- starting in the middle ---------------------------------------------------------------------

TEST(MetricsCollectorTest, MeasuringFromTheMiddleGivesTheDifferenceOfTwoMeasurementsFromTheStart) {
  for (const std::string& name : bundledLayouts()) {
    Simulation sim = *warehousesim::simulation::makeLayoutSimulation(name, 6);
    MetricsCollector fromStart(sim);
    for (int i = 0; i < 150; ++i) {
      sim.step();
      fromStart.observe(sim);
    }
    const FleetMetrics at150 = fromStart.report();
    MetricsCollector fromMiddle(sim);  // from tick 150 on
    for (int i = 0; i < 250; ++i) {
      sim.step();
      fromStart.observe(sim);
      fromMiddle.observe(sim);
    }
    const FleetMetrics& whole = fromStart.report();
    const FleetMetrics& part = fromMiddle.report();
    EXPECT_EQ(part.ticks, 250U) << name;
    EXPECT_EQ(part.tasksCompleted, whole.tasksCompleted - at150.tasksCompleted) << name;
    EXPECT_EQ(part.tasksFailed, whole.tasksFailed - at150.tasksFailed) << name;
    EXPECT_EQ(part.tasksSubmitted, whole.tasksSubmitted - at150.tasksSubmitted) << name;
    EXPECT_EQ(part.tasksOpen, whole.tasksOpen) << name;
    EXPECT_EQ(part.cellsTravelled(), whole.cellsTravelled() - at150.cellsTravelled()) << name;
    EXPECT_EQ(part.conflictsPrevented, whole.conflictsPrevented - at150.conflictsPrevented) << name;
    EXPECT_EQ(part.reroutes, whole.reroutes - at150.reroutes) << name;
    EXPECT_EQ(part.conflictDetections, whole.conflictDetections - at150.conflictDetections) << name;
    for (const Activity a : warehousesim::metrics::kAllActivities) {
      EXPECT_EQ(part.ticksIn(a), whole.ticksIn(a) - at150.ticksIn(a)) << name << " " << toString(a);
    }
  }
}

TEST(MetricsCollectorTest, ACollectorThatMissesAStepRefusesToGuess) {
  Simulation sim = warehousesim::simulation::makeDemoSimulation(1);
  MetricsCollector collector(sim);
  sim.step();
  collector.observe(sim);
  sim.step();  // not observed
  sim.step();
  EXPECT_THROW(collector.observe(sim), std::invalid_argument);
  EXPECT_EQ(collector.report().ticks, 1U);
  EXPECT_THROW(collector.observe(sim), std::invalid_argument);  // and it stays refused
}

TEST(MetricsCollectorTest, ARefusedStepLeavesTheCollectorAsItWasSoTheNextStepIsMeasuredInFull) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  Simulation sim(layout({"P...D", "....."}), {P(0, 1)}, config);
  MetricsCollector collector(sim);
  (void)sim.submitTask(P(0, 0), P(4, 0));
  sim.step();
  collector.observe(sim);
  EXPECT_THROW(collector.observe(sim), std::invalid_argument);  // the same step again
  EXPECT_THROW(collector.observe(sim), std::invalid_argument);
  const FleetMetrics before = collector.report();
  for (int i = 0; i < 40; ++i) {
    sim.step();
    collector.observe(sim);
  }
  // The task was delivered after the refusals; had they swallowed it, it would be missing here.
  EXPECT_EQ(before.tasksCompleted, 0U);
  EXPECT_EQ(collector.report().tasksCompleted, 1U);
  EXPECT_EQ(collector.report().tasksOpen, 0U);
  EXPECT_EQ(collector.report().ticks, 41U);
}

TEST(MetricsCollectorTest, ObservingTheSameStepTwiceIsRefused) {
  Simulation sim = warehousesim::simulation::makeDemoSimulation(1);
  MetricsCollector collector(sim);
  sim.step();
  collector.observe(sim);
  EXPECT_THROW(collector.observe(sim), std::invalid_argument);
  EXPECT_EQ(collector.report().ticks, 1U);
}

// ---- the stated scale and model -----------------------------------------------------------------

TEST(MetricsCollectorTest, TheTimeScaleOnlyRescalesTimeNeverTheCounts) {
  Simulation a = warehousesim::simulation::makeDemoSimulation(1);
  Simulation b = warehousesim::simulation::makeDemoSimulation(1);
  MetricsOptions slow;
  slow.secondsPerTick = 2.0;
  const FleetMetrics fast = measured(a, 400);
  const FleetMetrics scaled = measured(b, 400, slow);
  EXPECT_EQ(fast.tasksCompleted, scaled.tasksCompleted);
  EXPECT_TRUE(fast.robots == scaled.robots);
  EXPECT_NEAR(*scaled.tasksPerHour(), *fast.tasksPerHour() / 2.0, kEps);
  EXPECT_NEAR(*scaled.meanCompletionSeconds(), *fast.meanCompletionSeconds() * 2.0, kEps);
}

TEST(MetricsCollectorTest, TheStandardEnergyModelIsTheOneTheBundledRobotsWithBatteriesHave) {
  // metrics::EnergyModel::standard() is written out separately in the metrics module, which knows
  // nothing about scenarios; this keeps it from drifting from the robots it describes.
  EXPECT_TRUE(energyModelOf(*warehousesim::simulation::makeLayoutSimulation("compact", 1)) ==
              EnergyModel::standard());
  EXPECT_TRUE(energyModelOf(*warehousesim::simulation::makeLayoutSimulation("fulfilment", 1)) ==
              EnergyModel::standard());
  // The demo hall's robots have no battery model, so no energy of their own...
  EXPECT_TRUE(energyModelOf(warehousesim::simulation::makeDemoSimulation(1)) ==
              (EnergyModel{0, 0, 0}));
}

TEST(MetricsCollectorTest, TheEstimateUsesTheStatedModelSoTheDemoHallIsNotReportedAsUsingNothing) {
  Simulation sim = warehousesim::simulation::makeDemoSimulation(1);
  const FleetMetrics m = measured(sim, 500);
  EXPECT_GT(m.energyEstimate().total(), 0U);
  EXPECT_TRUE(m.options.energy == EnergyModel::standard());
  Simulation again = warehousesim::simulation::makeDemoSimulation(1);
  MetricsOptions own;
  own.energy = energyModelOf(again);
  EXPECT_EQ(measured(again, 500, own).energyEstimate().total(), 0U);
}

TEST(MetricsCollectorTest, WithTheSimulationsOwnModelTheEstimateIsWhatTheBatteriesActuallyLost) {
  // With a battery model and no recharging the estimate is exactly the drain: the collector's
  // figure and the robots' own battery levels are two ways of counting the same energy.
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 5;
  config.robot.batteryCapacity = 1'000'000;
  config.robot.idleDrainPerTick = 1;
  config.robot.moveCostPerCell = 15;
  config.robot.handlingDrainPerTick = 2;
  config.robot.loadTicks = 2;
  config.robot.unloadTicks = 2;
  Simulation sim(warehousesim::world::makeDemoWarehouse(), {P(0, 0), P(2, 0), P(4, 0)}, config);
  const FleetMetrics m = measured(sim, 400, [&] {
    MetricsOptions o;
    o.energy = energyModelOf(sim);
    return o;
  }());
  std::uint64_t drained = 0;
  for (const auto& robot : sim.robots()) {
    drained += robot.battery().capacity() - robot.battery().level();
  }
  EXPECT_EQ(m.energyEstimate().total(), drained);
}

}  // namespace
