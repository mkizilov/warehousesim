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

using warehousesim::core::kPi;
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
using warehousesim::tasks::Assignment;
using warehousesim::tasks::Dispatcher;
using warehousesim::tasks::DispatchPlan;
using warehousesim::tasks::Task;
using warehousesim::tasks::TaskQueue;
using warehousesim::tasks::TaskStatus;
using warehousesim::traffic::ConflictKind;
using warehousesim::world::Warehouse;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

Warehouse openFloor(int width, int height) {
  return layout(std::vector<std::string>(static_cast<std::size_t>(height),
                                         std::string(static_cast<std::size_t>(width), '.')));
}

// Two 3x3 rooms joined by a one-cell-wide corridor (row 1, x = 3..11) with a niche above its
// middle.
//
//   . . . # # # # . # # # # . . .
//   . . . . . . . . . . . . . . .
//   . . . # # # # # # # # # . . .
Warehouse hall() { return layout({"...####.####...", "...............", "...#########..."}); }

SimulationConfig ideal() {
  SimulationConfig config;
  config.maxBlockedTicks = 3;
  return config;
}

SimulationConfig vehicles() {
  SimulationConfig config = ideal();
  config.robot.acceleration = 0.5;
  config.robot.deceleration = 1.0;
  config.robot.turnRate = kPi / 2.0;
  config.robot.loadTicks = 2;
  config.robot.unloadTicks = 2;
  return config;
}

// Hands every task to the robot named for it, as soon as that robot is free. The default dispatcher
// would give each task to the nearest robot, which is not always the one a scenario needs.
class ScriptedDispatcher final : public Dispatcher {
 public:
  explicit ScriptedDispatcher(std::vector<std::pair<TaskId, RobotId>> script)
      : script_(std::move(script)) {}

  [[nodiscard]] DispatchPlan plan(const Warehouse&, std::span<const Robot>,
                                  const TaskQueue& queue) override {
    DispatchPlan plan;
    for (const auto& [task, robot] : script_) {
      const Task* found = queue.find(task);
      if (found != nullptr && found->status == TaskStatus::Pending) {
        plan.assignments.push_back(Assignment{robot, task, 0});
      }
    }
    return plan;
  }

 private:
  std::vector<std::pair<TaskId, RobotId>> script_;
};

// A scenario: some robots, and for each a list of tasks it is to do in order.
struct Job {
  Point start;
  Point pickup;
  Point dropoff;
};

struct Scenario {
  Simulation sim;
  std::vector<TaskId> tasks;  // by robot, the first task of each

  Scenario(Warehouse warehouse, const std::vector<Job>& jobs, SimulationConfig config)
      : sim(makeSim(std::move(warehouse), jobs, config)) {
    for (std::size_t i = 0; i < jobs.size(); ++i) {
      tasks.push_back(sim.submitTask(jobs[i].pickup, jobs[i].dropoff));
    }
  }

 private:
  static Simulation makeSim(Warehouse warehouse, const std::vector<Job>& jobs,
                            SimulationConfig config) {
    std::vector<Point> starts;
    std::vector<std::pair<TaskId, RobotId>> script;
    for (std::size_t i = 0; i < jobs.size(); ++i) {
      starts.push_back(jobs[i].start);
      script.emplace_back(TaskId{static_cast<std::uint32_t>(i + 1)},
                          RobotId{static_cast<std::uint32_t>(i + 1)});
    }
    return Simulation(std::move(warehouse), starts, config, nullptr,
                      std::make_unique<ScriptedDispatcher>(std::move(script)));
  }
};

// What must hold on every tick, whatever the robots are up to.
class TrafficAudit {
 public:
  explicit TrafficAudit(const Simulation& sim) { remember(sim); }

  void check(const Simulation& sim) {
    const auto& robots = sim.robots();
    std::set<Point> held;
    for (const Robot& robot : robots) {
      for (const Point cell : sim.world().holdsOf(robot.id())) {
        EXPECT_TRUE(held.insert(cell).second)
            << "cell (" << cell.x << "," << cell.y << ") held twice, tick " << sim.tick();
        EXPECT_TRUE(sim.world().isWalkable(cell)) << "tick " << sim.tick();
      }
    }
    for (std::size_t i = 0; i < robots.size(); ++i) {
      const auto here = robots[i].pose();
      const auto before = poses_[i];
      EXPECT_LE(std::hypot(here.x - before.x, here.y - before.y), 1.0 + 1e-6)
          << "robot " << i + 1 << " jumped, tick " << sim.tick();
      for (std::size_t j = i + 1; j < robots.size(); ++j) {
        const auto other = robots[j].pose();
        EXPECT_GE(std::hypot(here.x - other.x, here.y - other.y), 1.0 - 1e-6)
            << "robots " << i + 1 << " and " << j + 1 << " overlap, tick " << sim.tick();
        const bool moved = cells_[i] != robots[i].position();
        EXPECT_FALSE(moved && cells_[i] == robots[j].position() &&
                     cells_[j] == robots[i].position())
            << "robots " << i + 1 << " and " << j + 1 << " swapped cells, tick " << sim.tick();
      }
    }
    remember(sim);
  }

 private:
  void remember(const Simulation& sim) {
    cells_.clear();
    poses_.clear();
    for (const Robot& robot : sim.robots()) {
      cells_.push_back(robot.position());
      poses_.push_back(robot.pose());
    }
  }

  std::vector<Point> cells_;
  std::vector<warehousesim::core::Vec2> poses_;
};

// Runs until every task of the scenario is completed (or `limit` ticks), auditing every tick, and
// returns the tick each one completed at (0 for a task that never did).
std::vector<std::uint64_t> runToCompletion(
    Scenario& scenario, int limit, const std::function<void(const Simulation&)>& watch = {}) {
  TrafficAudit audit(scenario.sim);
  std::vector<std::uint64_t> done(scenario.tasks.size(), 0);
  for (int i = 0; i < limit; ++i) {
    scenario.sim.step();
    audit.check(scenario.sim);
    if (watch) {
      watch(scenario.sim);
    }
    bool all = true;
    for (std::size_t t = 0; t < scenario.tasks.size(); ++t) {
      if (scenario.sim.tasks().find(scenario.tasks[t])->status == TaskStatus::Completed) {
        if (done[t] == 0) {
          done[t] = scenario.sim.tick();
        }
      } else {
        all = false;
      }
    }
    if (all) {
      break;
    }
  }
  return done;
}

// How long robot `which` of `jobs` takes if it is alone in the warehouse.
std::uint64_t soloTime(const Warehouse& warehouse, const std::vector<Job>& jobs, std::size_t which,
                       SimulationConfig config) {
  Scenario alone(warehouse, {jobs[which]}, config);
  return runToCompletion(alone, 400)[0];
}

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

// ---- waiting, and asking for another way
// ---------------------------------------------------------

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

// ---- what the picture is told
// ---------------------------------------------------------------------

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
