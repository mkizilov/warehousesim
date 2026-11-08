#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/tasks/dispatcher.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/traffic/reservation_table.hpp"
#include "warehousesim/world/layouts.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotState;
using warehousesim::simulation::makeDemoSimulation;
using warehousesim::simulation::makeLayoutSimulation;
using warehousesim::simulation::RobotSnapshot;
using warehousesim::simulation::Simulation;
using warehousesim::simulation::SimulationConfig;
using warehousesim::tasks::TaskStatus;
using warehousesim::traffic::ConflictKind;
using warehousesim::world::Warehouse;
using warehousesim_test::hall;
using warehousesim_test::ideal;
using warehousesim_test::Job;
using warehousesim_test::layout;
using warehousesim_test::openFloor;
using warehousesim_test::P;
using warehousesim_test::R;
using warehousesim_test::runToCompletion;
using warehousesim_test::Scenario;
using warehousesim_test::soloTime;
using warehousesim_test::TrafficAudit;
using warehousesim_test::vehicles;

// ---- crossing paths -----------------------------------------------------------------------

//   R1 drives east along row 5 and R2 south down column 5 of an open floor; with the same speed
//   they reach (5, 5) on the very same tick.
std::vector<Job> crossing() {
  return {{P(0, 5), P(10, 5), P(10, 4)}, {P(5, 0), P(5, 10), P(4, 10)}};
}

TEST(CollisionAvoidanceTest, OfTwoRobotsCrossingTheOneWithPriorityIsNeverDelayedAndTheOtherWaits) {
  const std::vector<Job> jobs = crossing();
  const std::uint64_t first = soloTime(openFloor(11, 11), jobs, 0, ideal());
  const std::uint64_t second = soloTime(openFloor(11, 11), jobs, 1, ideal());
  ASSERT_GT(first, 0U);
  ASSERT_GT(second, 0U);

  Scenario scenario(openFloor(11, 11), jobs, ideal());
  std::optional<warehousesim::simulation::RobotWait> seenWaiting;
  bool firstEverWaited = false;
  const auto done = runToCompletion(scenario, 400, [&](const Simulation& sim) {
    const auto snapshot = sim.snapshot();
    firstEverWaited = firstEverWaited || snapshot.robots[0].waiting.has_value();
    if (!seenWaiting && snapshot.robots[1].waiting) {
      seenWaiting = snapshot.robots[1].waiting;
    }
  });

  EXPECT_EQ(done[0], first);   // robot 1 drives as if the floor were empty
  EXPECT_GT(done[1], second);  // robot 2 gives way
  EXPECT_FALSE(firstEverWaited);
  ASSERT_TRUE(seenWaiting.has_value());
  EXPECT_EQ(seenWaiting->kind, ConflictKind::Vertex);
  EXPECT_TRUE(seenWaiting->other == R(1));
  EXPECT_GT(scenario.sim.trafficStats().vertex, 0U);
}

TEST(CollisionAvoidanceTest, TheGivingWayRobotLosesOnlyAFewTicksNotAWholeDetour) {
  const std::vector<Job> jobs = crossing();
  const std::uint64_t second = soloTime(openFloor(11, 11), jobs, 1, ideal());
  Scenario scenario(openFloor(11, 11), jobs, ideal());
  const auto done = runToCompletion(scenario, 400);
  EXPECT_LE(done[1], second + 6);
}

TEST(CollisionAvoidanceTest, SwappingWhichRobotHasThePriorityChangesWhoWaits) {
  const std::vector<Job> jobs = {crossing()[1], crossing()[0]};  // the southbound robot is id 1
  const std::uint64_t southbound = soloTime(openFloor(11, 11), jobs, 0, ideal());
  Scenario scenario(openFloor(11, 11), jobs, ideal());
  const auto done = runToCompletion(scenario, 400);
  EXPECT_EQ(done[0], southbound);
  EXPECT_GT(done[1], soloTime(openFloor(11, 11), jobs, 1, ideal()));
}

TEST(CollisionAvoidanceTest, VehiclesThatAccelerateAndTurnCrossWithoutEverOverlapping) {
  Scenario scenario(openFloor(11, 11), crossing(), vehicles());
  const auto done = runToCompletion(scenario, 600);
  EXPECT_GT(done[0], 0U);
  EXPECT_GT(done[1], 0U);
}

// ---- intersections --------------------------------------------------------------------------

//   Two one-cell-wide roads that cross at (4, 4); neither robot has anywhere else to go.
Warehouse crossroads() {
  return layout({"####.####", "####.####", "####.####", "####.####", ".........", "####.####",
                 "####.####", "####.####", "####.####"});
}

TEST(CollisionAvoidanceTest, AtAnIntersectionOfNarrowRoadsOneRobotWaitsInItsRoadForTheOther) {
  const std::vector<Job> jobs = {{P(0, 4), P(8, 4), P(7, 4)}, {P(4, 0), P(4, 8), P(4, 7)}};
  const std::uint64_t first = soloTime(crossroads(), jobs, 0, ideal());
  Scenario scenario(crossroads(), jobs, ideal());
  bool waitedBeforeTheCrossing = false;
  const auto done = runToCompletion(scenario, 300, [&](const Simulation& sim) {
    const Robot& waiting = sim.robots()[1];
    // Robot 2 holds back in its own road, short of the crossing, and never sits on it.
    if (sim.snapshot().robots[1].waiting) {
      waitedBeforeTheCrossing = true;
      EXPECT_EQ(waiting.position().x, 4);
      EXPECT_LT(waiting.position().y, 4);
    }
  });
  EXPECT_EQ(done[0], first);
  EXPECT_GT(done[1], 0U);
  EXPECT_TRUE(waitedBeforeTheCrossing);
  EXPECT_EQ(scenario.sim.replans(), 0U);  // there is no other way to go
}

TEST(CollisionAvoidanceTest, ThreeRobotsThroughTheSameJunctionAreServedInIdOrder) {
  // From the west, the north and the south, all heading for the east arm, through (4, 4).
  const std::vector<Job> jobs = {
      {P(0, 4), P(8, 4), P(7, 4)}, {P(4, 0), P(8, 4), P(7, 4)}, {P(4, 8), P(8, 4), P(7, 4)}};
  Scenario scenario(crossroads(),
                    {jobs[0], {jobs[1].start, P(7, 4), P(6, 4)}, {jobs[2].start, P(6, 4), P(5, 4)}},
                    ideal());
  std::map<std::uint32_t, std::uint64_t> through;  // first tick each robot is east of the junction
  const auto done = runToCompletion(scenario, 400, [&](const Simulation& sim) {
    for (const Robot& robot : sim.robots()) {
      if (robot.position().x > 4 && robot.position().y == 4 &&
          !through.contains(robot.id().value)) {
        through[robot.id().value] = sim.tick();
      }
    }
  });
  for (const auto tick : done) {
    EXPECT_GT(tick, 0U);
  }
  ASSERT_EQ(through.size(), 3U);
  EXPECT_LT(through[1], through[2]);
  EXPECT_LT(through[2], through[3]);
}

// ---- simultaneous arrivals --------------------------------------------------------------------

TEST(CollisionAvoidanceTest, RobotsArrivingAtOneCellOnTheSameTickTakeTurnsInIdOrder) {
  // Three tasks with the same pickup cell; the robots all arrive in four ticks.
  const std::vector<Job> jobs = {
      {P(0, 4), P(4, 4), P(3, 4)}, {P(4, 0), P(4, 4), P(4, 3)}, {P(8, 4), P(4, 4), P(5, 4)}};
  SimulationConfig config = ideal();
  config.robot.loadTicks = 2;  // so that each one stays on the cell for a while
  Scenario scenario(openFloor(9, 9), jobs, config);
  std::map<std::uint32_t, std::uint64_t> loadingFrom;
  const auto done = runToCompletion(scenario, 600, [&](const Simulation& sim) {
    for (const Robot& robot : sim.robots()) {
      if (robot.state() == RobotState::Loading && !loadingFrom.contains(robot.id().value)) {
        loadingFrom[robot.id().value] = sim.tick();
        EXPECT_EQ(robot.position(), P(4, 4));
      }
    }
  });
  for (const auto tick : done) {
    EXPECT_GT(tick, 0U);
  }
  ASSERT_EQ(loadingFrom.size(), 3U);
  EXPECT_LT(loadingFrom[1], loadingFrom[2]);
  EXPECT_LT(loadingFrom[2], loadingFrom[3]);
}

TEST(CollisionAvoidanceTest, TheSameHoldsWhenEveryRobotArrivesFromTheSameSide) {
  const std::vector<Job> jobs = {
      {P(0, 2), P(5, 4), P(5, 5)}, {P(0, 4), P(5, 4), P(5, 6)}, {P(0, 6), P(5, 4), P(5, 7)}};
  Scenario scenario(openFloor(9, 9), jobs, ideal());
  const auto done = runToCompletion(scenario, 400);
  for (const auto tick : done) {
    EXPECT_GT(tick, 0U);
  }
}

// ---- opposing traffic in a corridor -------------------------------------------------------------

//   R1 sets off from the left room, R2 from the right room; each is going to the other room, and
//   each has room to wait where it is, off the other's way.
std::vector<Job> opposing() {
  return {{P(0, 1), P(13, 0), P(14, 0)}, {P(14, 2), P(1, 0), P(0, 0)}};
}

bool inCorridor(const Robot& robot) {
  return robot.position().y == 1 && robot.position().x >= 3 && robot.position().x <= 11;
}

TEST(CollisionAvoidanceTest, RobotsComingFromOppositeRoomsNeverMeetInsideTheCorridor) {
  const std::vector<Job> jobs = opposing();
  const std::uint64_t first = soloTime(hall(), jobs, 0, ideal());
  Scenario scenario(hall(), jobs, ideal());
  std::optional<warehousesim::simulation::RobotWait> firstWait;
  bool bothInside = false;
  const auto done = runToCompletion(scenario, 400, [&](const Simulation& sim) {
    bothInside = bothInside || (inCorridor(sim.robots()[0]) && inCorridor(sim.robots()[1]));
    if (!firstWait) {
      firstWait = sim.snapshot().robots[1].waiting;
    }
  });
  EXPECT_EQ(done[0], first);
  EXPECT_GT(done[1], 0U);
  ASSERT_TRUE(firstWait.has_value());
  // It is held outside for a robot that is still far away down the corridor, not for a robot on the
  // cell in front of it.
  EXPECT_TRUE(firstWait->other == R(1));
  EXPECT_TRUE(firstWait->kind == ConflictKind::Swap || firstWait->kind == ConflictKind::Vertex);
  EXPECT_FALSE(
      bothInside);  // the one with the lower priority stays out until the corridor is clear
  EXPECT_GT(scenario.sim.trafficStats().swaps + scenario.sim.trafficStats().vertex, 0U);
}

TEST(CollisionAvoidanceTest, ARobotWithNowhereToWaitCarriesOnTowardsItsGoalInsteadOfFreezing) {
  // Robot 2 is already inside the corridor, facing robot 1, and every cell it could stop on is on
  // robot 1's route. Waiting would not help it, so the table leaves it to the world's own claims.
  const std::vector<Job> jobs = {{P(0, 1), P(13, 0), P(14, 0)}, {P(10, 1), P(1, 0), P(0, 0)}};
  Scenario scenario(hall(), jobs, ideal());
  scenario.sim.step();
  scenario.sim.step();
  EXPECT_LT(scenario.sim.robots()[1].position().x, 10);
  EXPECT_GT(scenario.sim.trafficStats().inTheWay, 0U);
}

TEST(CollisionAvoidanceTest, TheSameWithVehiclesThatNeedRoomToStop) {
  Scenario scenario(hall(), opposing(), vehicles());
  bool bothInside = false;
  const auto done = runToCompletion(scenario, 600, [&](const Simulation& sim) {
    bothInside = bothInside || (inCorridor(sim.robots()[0]) && inCorridor(sim.robots()[1]));
  });
  EXPECT_GT(done[0], 0U);
  EXPECT_GT(done[1], 0U);
  EXPECT_FALSE(bothInside);
}

TEST(CollisionAvoidanceTest, RobotsThatMeetInsideTheCorridorSortItOutAtTheNiche) {
  // Both set off inside the corridor, facing each other, and meet at the niche above (7, 1).
  const std::vector<Job> jobs = {{P(4, 1), P(13, 1), P(13, 2)}, {P(10, 1), P(1, 1), P(1, 2)}};
  Scenario scenario(hall(), jobs, ideal());
  const auto done = runToCompletion(scenario, 600);
  EXPECT_GT(done[0], 0U);
  EXPECT_GT(done[1], 0U);
}

TEST(CollisionAvoidanceTest, WithReservationsOffRobotsStillNeverCollideTheyJustMeetHeadOn) {
  SimulationConfig config = ideal();
  config.reservations = false;
  Scenario scenario(hall(), opposing(), config);
  bool bothInside = false;
  const auto done = runToCompletion(scenario, 600, [&](const Simulation& sim) {
    bothInside = bothInside || (inCorridor(sim.robots()[0]) && inCorridor(sim.robots()[1]));
  });
  EXPECT_GT(done[0], 0U);
  EXPECT_GT(done[1], 0U);
  EXPECT_TRUE(bothInside);  // without the table they only find out when they are face to face
  EXPECT_EQ(scenario.sim.trafficStats().vertex + scenario.sim.trafficStats().swaps, 0U);
  EXPECT_EQ(scenario.sim.reservations().occupancyCount(), 0U);
}

// ---- waiting, and asking for another way --------------------------------------------------------

//   R1 crosses R2's road and stops on the crossing to load for a long time.
std::vector<Job> loadingOnTheRoad() {
  return {{P(0, 3), P(3, 3), P(6, 3)}, {P(3, 0), P(3, 6), P(2, 6)}};
}

SimulationConfig slowLoading() {
  SimulationConfig config = vehicles();
  config.robot.loadTicks = 12;
  config.maxBlockedTicks = 1;  // ask at once, while the cell is still free
  return config;
}

TEST(CollisionAvoidanceTest, ARobotHeldUpByOneLoadingOnItsRoadGoesRoundItAndNeverOverIt) {
  Scenario scenario(openFloor(7, 7), loadingOnTheRoad(), slowLoading());
  bool wentRound = false;
  const auto done = runToCompletion(scenario, 300, [&](const Simulation& sim) {
    const Robot& loader = sim.robots()[0];
    const Robot& other = sim.robots()[1];
    wentRound = wentRound || other.position().x != 3;
    if (loader.state() == RobotState::Loading) {
      EXPECT_NE(other.position(), loader.position());
    }
  });
  EXPECT_GT(done[0], 0U);
  EXPECT_GT(done[1], 0U);
  EXPECT_TRUE(wentRound);
}

// How many times, in all, robots asked for another way because a reservation held them up.
std::uint64_t demoReplans(std::size_t maxDetourCells) {
  std::uint64_t replans = 0;
  for (std::uint64_t seed = 1; seed <= 4; ++seed) {
    SimulationConfig config = vehicles();
    config.seed = seed;
    config.taskSpawnInterval = 5;
    config.varyTaskPriority = true;
    config.maxDetourCells = maxDetourCells;
    Simulation sim(warehousesim::world::makeDemoWarehouse(), {P(0, 0), P(2, 0), P(4, 0), P(6, 0)},
                   config);
    sim.run(800);
    replans += sim.replans();
  }
  return replans;
}

TEST(CollisionAvoidanceTest, RobotsHeldUpInRealTrafficDoAskForAnotherWay) {
  EXPECT_GT(demoReplans(4), 0U);
}

TEST(CollisionAvoidanceTest, ADetourLongerThanTheAllowanceIsNotTaken) {
  // In a one-cell-wide road there is no way round at all: the robot waits for however long it
  // takes.
  const Warehouse road = layout({"####.####", "####.####", "####.####", "####.####", ".........",
                                 "####.####", "####.####", "####.####", "####.####"});
  const std::vector<Job> jobs = {{P(0, 4), P(4, 4), P(8, 4)}, {P(4, 0), P(4, 8), P(4, 7)}};
  Scenario scenario(road, jobs, slowLoading());
  const auto done = runToCompletion(scenario, 400);
  EXPECT_GT(done[0], 0U);
  EXPECT_GT(done[1], 0U);
  EXPECT_EQ(scenario.sim.replans(), 0U);
  EXPECT_GT(done[1], done[0] - 12);  // it had to wait for the loading robot to finish and leave
}

// ---- what the picture is told -------------------------------------------------------------------

TEST(CollisionAvoidanceTest, VehiclesReportTheCellsTheyHoldAheadOfThemselves) {
  Scenario scenario(openFloor(11, 11), crossing(), vehicles());
  bool sawCellsAhead = false;
  runToCompletion(scenario, 400, [&](const Simulation& sim) {
    const auto snapshot = sim.snapshot();
    for (std::size_t i = 0; i < snapshot.robots.size(); ++i) {
      EXPECT_EQ(snapshot.robots[i].holds, sim.world().holdsOf(sim.robots()[i].id()));
      sawCellsAhead = sawCellsAhead || snapshot.robots[i].holds.size() > 1;
    }
  });
  EXPECT_TRUE(sawCellsAhead);
}

TEST(CollisionAvoidanceTest, TheSnapshotCarriesTheRouteTheCellsHeldAndWhatARobotWaitsFor) {
  Scenario scenario(openFloor(11, 11), crossing(), ideal());
  bool sawWaiting = false;
  runToCompletion(scenario, 400, [&](const Simulation& sim) {
    const auto snapshot = sim.snapshot();
    for (std::size_t i = 0; i < snapshot.robots.size(); ++i) {
      const RobotSnapshot& view = snapshot.robots[i];
      const Robot& robot = sim.robots()[i];
      EXPECT_EQ(view.path, std::vector<Point>(robot.path().begin(), robot.path().end()));
      ASSERT_FALSE(view.holds.empty());
      EXPECT_EQ(view.holds.front(), robot.position());
      EXPECT_EQ(view.holds, sim.world().holdsOf(robot.id()));
      if (view.waiting) {
        sawWaiting = true;
        EXPECT_EQ(view.state, RobotState::Navigating);
        EXPECT_FALSE(view.path.empty());
        EXPECT_EQ(view.waiting->cell, view.path.front());  // the cell it may not enter yet
        EXPECT_EQ(view.speed, 0.0);
      }
    }
  });
  EXPECT_TRUE(sawWaiting);
  EXPECT_FALSE(scenario.sim.snapshot().robots[0].waiting.has_value());
}

TEST(CollisionAvoidanceTest, TheJsonCarriesRoutesHeldCellsWhatARobotWaitsForAndTheTrafficCounts) {
  Scenario scenario(openFloor(11, 11), crossing(), ideal());
  nlohmann::json json;
  for (int tick = 0; tick < 40; ++tick) {
    scenario.sim.step();
    json = warehousesim::simulation::toJson(scenario.sim.snapshot());
    if (!json["robots"][1]["waiting"].is_null()) {
      break;
    }
  }
  const auto& first = json["robots"][0];
  const auto& second = json["robots"][1];
  ASSERT_FALSE(second["waiting"].is_null());
  EXPECT_TRUE(first["waiting"].is_null());
  EXPECT_EQ(second["waiting"]["other"], 1);
  EXPECT_TRUE(second["waiting"]["kind"].is_string());
  EXPECT_EQ(second["waiting"]["cell"], second["path"][0]);  // the next cell it may not enter
  EXPECT_EQ(second["holds"][0], second["position"]);
  EXPECT_EQ(second["path"].size(), scenario.sim.robots()[1].path().size());
  EXPECT_GT(json["traffic"]["vertexConflicts"].get<std::uint64_t>(), 0U);
  for (const char* key : {"blocked", "swapConflicts", "inTheWay", "replans"}) {
    EXPECT_TRUE(json["traffic"].contains(key)) << key;
  }
  EXPECT_EQ(json["traffic"]["replans"], scenario.sim.replans());
}

TEST(CollisionAvoidanceTest, TheReservationTableOfTheLastStepIsAvailableToLookAt) {
  Scenario scenario(openFloor(11, 11), crossing(), ideal());
  scenario.sim.step();
  scenario.sim.step();
  const auto& table = scenario.sim.reservations();
  EXPECT_GT(table.occupancyCount(), 2U);
  // Robot 1 holds where it is now (step 2) and has reserved the cells ahead for the steps to come.
  EXPECT_TRUE(table.holder(scenario.sim.robots()[0].position(), 2) == R(1));
  EXPECT_TRUE(table.holder(P(5, 5), 5) == R(1));
  EXPECT_FALSE(table.holder(P(5, 5), 5) == R(2));  // taken: robot 2 reserves it later or not at all
  EXPECT_TRUE(scenario.sim.trafficPlan(R(2)).waiting());
  EXPECT_THROW((void)scenario.sim.trafficPlan(R(3)), std::out_of_range);
  EXPECT_THROW((void)scenario.sim.trafficPlan(RobotId{}), std::out_of_range);
}

TEST(CollisionAvoidanceTest, AShortHorizonIsRefusedAtTheDoor) {
  SimulationConfig config = ideal();
  config.reservationHorizon = 1;
  EXPECT_THROW(Simulation(openFloor(3, 3), {P(0, 0)}, config), std::invalid_argument);
}

// ---- whatever the traffic -----------------------------------------------------------------------

TEST(CollisionAvoidancePropertyTest, NoBundledLayoutOrSeedEverHasRobotsOverlapSwapOrLeaveTheFloor) {
  for (const std::string name : {"demo", "compact", "fulfilment"}) {
    for (std::uint64_t seed = 1; seed <= 3; ++seed) {
      auto made = makeLayoutSimulation(name, seed);
      ASSERT_TRUE(made.has_value());
      Simulation& sim = *made;
      TrafficAudit audit(sim);
      for (int tick = 0; tick < 500; ++tick) {
        sim.step();
        audit.check(sim);
      }
      EXPECT_GT(sim.snapshot().completedTasks, 0U) << name << " seed " << seed;
      EXPECT_EQ(sim.snapshot().failedTasks, 0U) << name << " seed " << seed;
    }
  }
}

TEST(CollisionAvoidancePropertyTest, TheDemoHallIsSafeWithAndWithoutReservations) {
  for (const bool reservations : {true, false}) {
    for (std::uint64_t seed = 1; seed <= 4; ++seed) {
      SimulationConfig config = vehicles();
      config.seed = seed;
      config.taskSpawnInterval = 5;
      config.varyTaskPriority = true;
      config.reservations = reservations;
      Simulation sim(warehousesim::world::makeDemoWarehouse(), {P(0, 0), P(2, 0), P(4, 0), P(6, 0)},
                     config);
      TrafficAudit audit(sim);
      for (int tick = 0; tick < 600; ++tick) {
        sim.step();
        audit.check(sim);
      }
      EXPECT_GT(sim.snapshot().completedTasks, 5U);
    }
  }
}

TEST(CollisionAvoidancePropertyTest, NoSeedOfTheDemoHallEverComesToAStandstill) {
  // Regression: robots queuing for a corner dropoff used to lock each other in for good (one robot
  // leaving needs the cell of one arriving, and the third blocks the way round).
  for (std::uint64_t seed = 1; seed <= 30; ++seed) {
    Simulation sim = makeDemoSimulation(seed);
    sim.run(1500);
    const std::size_t before = sim.snapshot().completedTasks;
    sim.run(500);
    EXPECT_GE(sim.snapshot().completedTasks, before + 15U) << "seed " << seed;
  }
}

}  // namespace
