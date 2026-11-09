#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/recording.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/world/cell_type.hpp"
#include "warehousesim/world/layouts.hpp"

namespace {

using namespace warehousesim_test;
using warehousesim::simulation::apply;
using warehousesim::simulation::CancelTask;
using warehousesim::simulation::ClearObstacle;
using warehousesim::simulation::PlaceObstacle;
using warehousesim::simulation::ScenarioSpec;
using warehousesim::simulation::SetRobotService;
using warehousesim::simulation::SetTaskPriority;
using warehousesim::simulation::SubmitTask;
using warehousesim::world::CellType;

// Two idle robots and no random tasks: only what a test does to it changes it.
Simulation quiet(std::uint64_t seed = 1) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  config.seed = seed;
  return Simulation(layout({"P.....D", ".......", "S.SSS.S", "C......"}), {P(0, 3), P(2, 3)},
                    config);
}

std::vector<std::string> bundledLayouts() {
  std::vector<std::string> names;
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    names.emplace_back(info.name);
  }
  return names;
}

// ---- equal runs, equal checksums ----------------------------------------------------------------

TEST(StateChecksumTest, TwoSimulationsBuiltTheSameWayAgreeAtEveryTick) {
  for (const std::string& name : bundledLayouts()) {
    Simulation a = *warehousesim::simulation::makeLayoutSimulation(name, 5);
    Simulation b = *warehousesim::simulation::makeLayoutSimulation(name, 5);
    for (int tick = 0; tick <= 300; ++tick) {
      ASSERT_EQ(a.stateChecksum(), b.stateChecksum()) << name << " at tick " << tick;
      a.step();
      b.step();
    }
  }
}

TEST(StateChecksumTest, ASimulationBuiltFromTheOriginOfAnotherIsTheSameSimulation) {
  Simulation first = *warehousesim::simulation::makeLayoutSimulation("compact", 9);
  first.run(150);  // the origin is not what it has become
  Simulation again(first.origin());
  Simulation fresh = *warehousesim::simulation::makeLayoutSimulation("compact", 9);
  EXPECT_EQ(again.stateChecksum(), fresh.stateChecksum());
  EXPECT_EQ(again.tick(), 0U);
  for (int tick = 0; tick < 150; ++tick) {
    again.step();
    fresh.step();
  }
  EXPECT_EQ(again.stateChecksum(), first.stateChecksum());
  EXPECT_TRUE(again.snapshot() == first.snapshot());
}

TEST(StateChecksumTest, TheOriginHoldsTheLayoutAsBuiltWhateverIsDoneToItLater) {
  Simulation sim = quiet();
  const std::vector<std::string> before = sim.warehouse().toRows();
  ASSERT_TRUE(sim.placeObstacle(P(3, 1)));
  EXPECT_NE(sim.warehouse().toRows(), before);
  EXPECT_EQ(sim.origin().warehouse.toRows(), before);
  EXPECT_EQ(sim.origin().robotStarts, (std::vector<Point>{P(0, 3), P(2, 3)}));
  EXPECT_EQ(sim.origin().config.seed, 1U);
  EXPECT_TRUE(sim.usesDefaultComponents());
}

TEST(StateChecksumTest, ASimulationWithItsOwnDispatcherSaysItIsNotRebuildable) {
  Scenario scenario(openFloor(5, 1), {Job{P(0, 0), P(1, 0), P(2, 0)}}, ideal());
  EXPECT_FALSE(scenario.sim.usesDefaultComponents());
}

// ---- the checksum notices what differs ----------------------------------------------------------

TEST(StateChecksumTest, ItMovesWithTheTickAndWithWhatTheRobotsDo) {
  Simulation sim = *warehousesim::simulation::makeLayoutSimulation("demo", 3);
  std::uint64_t previous = sim.stateChecksum();
  for (int tick = 0; tick < 100; ++tick) {
    sim.step();
    const std::uint64_t now = sim.stateChecksum();
    EXPECT_NE(now, previous) << "no change from tick " << tick << " to " << tick + 1;
    previous = now;
  }
}

TEST(StateChecksumTest, ADifferentSeedIsSeenBeforeAnyTaskIsMade) {
  // The random stream is part of the state: at tick 0 the snapshots are equal, the futures are not.
  Simulation a = *warehousesim::simulation::makeLayoutSimulation("demo", 1);
  Simulation b = *warehousesim::simulation::makeLayoutSimulation("demo", 2);
  EXPECT_TRUE(a.snapshot() == b.snapshot());
  EXPECT_NE(a.stateChecksum(), b.stateChecksum());
}

TEST(StateChecksumTest, EveryKindOfChangeFromOutsideShowsInIt) {
  const std::vector<std::pair<std::string, std::function<void(Simulation&)>>> changes = {
      {"a task", [](Simulation& s) { (void)apply(s, SubmitTask{P(0, 0), P(6, 0)}); }},
      {"a task elsewhere", [](Simulation& s) { (void)apply(s, SubmitTask{P(1, 0), P(6, 0)}); }},
      {"a task with a priority",
       [](Simulation& s) { (void)apply(s, SubmitTask{P(0, 0), P(6, 0), TaskPriority::Urgent}); }},
      {"a robot out of service",
       [](Simulation& s) { (void)apply(s, SetRobotService{RobotId{1}, false}); }},
      {"a wall", [](Simulation& s) { (void)apply(s, PlaceObstacle{P(3, 1), CellType::Wall}); }},
      {"a shelf", [](Simulation& s) { (void)apply(s, PlaceObstacle{P(3, 1), CellType::Shelf}); }},
      {"a shelf taken down", [](Simulation& s) { (void)apply(s, ClearObstacle{P(0, 2)}); }},
      {"a step", [](Simulation& s) { s.step(); }},
  };
  const std::uint64_t untouched = quiet().stateChecksum();
  std::vector<std::uint64_t> seen{untouched};
  for (const auto& [name, change] : changes) {
    Simulation a = quiet();
    Simulation b = quiet();
    change(a);
    change(b);
    EXPECT_NE(a.stateChecksum(), untouched) << name;
    EXPECT_EQ(a.stateChecksum(), b.stateChecksum()) << name << ": the same change, the same state";
    seen.push_back(a.stateChecksum());
  }
  std::sort(seen.begin(), seen.end());
  EXPECT_EQ(std::unique(seen.begin(), seen.end()), seen.end()) << "two different changes agreed";
}

TEST(StateChecksumTest, ThingsDoneToATaskAfterItWasMadeShowToo) {
  const auto with = [](const std::function<void(Simulation&)>& change) {
    Simulation sim = quiet();
    (void)apply(sim, SubmitTask{P(0, 0), P(6, 0)});
    change(sim);
    return sim.stateChecksum();
  };
  const std::uint64_t base = with([](Simulation&) {});
  const std::uint64_t reprioritised =
      with([](Simulation& s) { (void)apply(s, SetTaskPriority{TaskId{1}, TaskPriority::High}); });
  const std::uint64_t cancelled =
      with([](Simulation& s) { (void)apply(s, CancelTask{TaskId{1}}); });
  EXPECT_NE(base, reprioritised);
  EXPECT_NE(base, cancelled);
  EXPECT_NE(reprioritised, cancelled);
}

TEST(StateChecksumTest, ARefusedCommandLeavesItAsItWas) {
  Simulation sim = quiet();
  const std::uint64_t before = sim.stateChecksum();
  EXPECT_FALSE(apply(sim, CancelTask{TaskId{4}}).accepted);
  EXPECT_FALSE(apply(sim, SubmitTask{P(-1, 0), P(6, 0)}).accepted);
  EXPECT_FALSE(apply(sim, PlaceObstacle{P(2, 3), CellType::Wall}).accepted);
  EXPECT_FALSE(apply(sim, ClearObstacle{P(3, 1)}).accepted);
  EXPECT_FALSE(apply(sim, SetRobotService{RobotId{1}, true}).accepted);
  EXPECT_EQ(sim.stateChecksum(), before);
}

TEST(StateChecksumTest, TheWholeHistoryOfTasksCountsNotOnlyWhatASnapshotKeepsOfIt) {
  // A snapshot keeps the last few finished tasks and counters for the rest. The checksum keeps the
  // lot: two runs that differ only in how an old task started are not the same run.
  const auto run = [](Point firstPickup) {
    Simulation sim = quiet();
    (void)apply(sim, SubmitTask{firstPickup, P(6, 0)});
    for (int i = 0; i < 20; ++i) {
      (void)apply(sim, SubmitTask{P(0, 0), P(6, 0)});
    }
    for (std::uint32_t id = 1; id <= 21; ++id) {
      (void)apply(sim, CancelTask{TaskId{id}});
    }
    return sim;
  };
  Simulation a = run(P(0, 0));
  Simulation b = run(P(1, 0));
  EXPECT_TRUE(a.snapshot() == b.snapshot());
  EXPECT_NE(a.stateChecksum(), b.stateChecksum());
}

TEST(StateChecksumTest, ADifferentFleetOrLayoutIsADifferentState) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  const Simulation one(layout({"P..D"}), {P(0, 0)}, config);
  const Simulation two(layout({"P..D"}), {P(0, 0), P(1, 0)}, config);
  const Simulation moved(layout({"P..D"}), {P(1, 0)}, config);
  const Simulation other(layout({"P.SD"}), {P(0, 0)}, config);
  const Simulation empty(layout({"P..D"}), {}, config);
  std::vector<std::uint64_t> all = {one.stateChecksum(), two.stateChecksum(), moved.stateChecksum(),
                                    other.stateChecksum(), empty.stateChecksum()};
  std::sort(all.begin(), all.end());
  EXPECT_EQ(std::unique(all.begin(), all.end()), all.end());
}

// ---- asking changes nothing ---------------------------------------------------------------------

TEST(StateChecksumTest, LookingAtASimulationDoesNotChangeIt) {
  Simulation looked = *warehousesim::simulation::makeLayoutSimulation("fulfilment", 4);
  Simulation unseen = *warehousesim::simulation::makeLayoutSimulation("fulfilment", 4);
  warehousesim::simulation::MetricsCollector collector(looked);
  for (int tick = 0; tick < 200; ++tick) {
    looked.step();
    unseen.step();
    collector.observe(looked);
    (void)looked.snapshot();
    (void)looked.stateChecksum();
    (void)looked.stateChecksum();
    for (const auto& robot : looked.robots()) {
      (void)looked.heldBack(robot.id());
      (void)looked.trafficPlan(robot.id());
    }
  }
  EXPECT_EQ(looked.stateChecksum(), unseen.stateChecksum());
}

// ---- pinned -------------------------------------------------------------------------------------

// If this fails after a change that was meant to alter what a simulation does, update the numbers
// and raise warehousesim::simulation::kEngineRevision in the same change: recordings made before it
// would diverge from their replay. If the change was not meant to alter behaviour, it is a bug.
TEST(StateChecksumTest, ThePinnedScenarioHasThePinnedChecksums) {
  Simulation sim = *warehousesim::simulation::makeLayoutSimulation("compact", 7);
  EXPECT_EQ(sim.stateChecksum(), 0x991feb2149e1fe06ULL) << "tick 0";
  sim.run(100);
  EXPECT_EQ(sim.stateChecksum(), 0x8039285935d120dfULL) << "tick 100";
  sim.run(200);
  EXPECT_EQ(sim.stateChecksum(), 0xa1a801b56de760f1ULL) << "tick 300";
  EXPECT_EQ(warehousesim::simulation::kEngineRevision, 2);
}

}  // namespace
