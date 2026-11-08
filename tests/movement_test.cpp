#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/layouts.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace {

using warehousesim::core::kPi;
using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::MotionPhase;
using warehousesim::robots::NavigationGoal;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotState;
using warehousesim::simulation::makeLayoutSimulation;
using warehousesim::simulation::Simulation;
using warehousesim::simulation::SimulationConfig;
using warehousesim::simulation::Snapshot;
using warehousesim::simulation::toJson;
using warehousesim::tasks::TaskStatus;
using warehousesim::world::Warehouse;

constexpr double kTolerance = 1e-9;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }

Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

// Robots that drive like vehicles: 1 cell/tick at most, 0.5 up and 1 down, a quarter turn per
// tick, stopping at every corner.
SimulationConfig vehicleConfig() {
  SimulationConfig config;
  config.robot.acceleration = 0.5;
  config.robot.deceleration = 1.0;
  config.robot.turnRate = kPi / 2.0;
  config.robot.batteryCapacity = 10000;
  config.robot.moveCostPerCell = 10;
  config.robot.loadTicks = 2;
  config.robot.unloadTicks = 1;
  return config;
}

// Checks, for every robot on every tick, everything the motion model promises, and gathers what
// the run got up to so that a test can insist it was an interesting one.
class MotionAudit {
 public:
  explicit MotionAudit(const Simulation& sim) : lastSpeed_(sim.robots().size(), 0.0) {}

  void check(const Simulation& sim) {
    const auto& robots = sim.robots();
    for (std::size_t i = 0; i < robots.size(); ++i) {
      const Robot& robot = robots[i];
      const RobotSpecView limits{robot.spec().maxSpeed, robot.spec().acceleration,
                                 robot.spec().deceleration};

      // The world holds exactly the robot's cell and the cells it was granted ahead.
      std::vector<Point> expected = {robot.position()};
      for (std::size_t k = 0; k < robot.authority(); ++k) {
        expected.push_back(robot.path()[k]);
      }
      EXPECT_EQ(sim.world().holdsOf(robot.id()), expected) << where(sim, robot);
      EXPECT_EQ(robot.inconsistency().value_or("consistent"), "consistent") << where(sim, robot);
      EXPECT_LE(robot.authority(), 3U);  // never a long reservation: just room to stop in
      EXPECT_EQ(robot.brakingViolations(), 0U) << where(sim, robot);

      const double speed = robot.speed();
      EXPECT_LE(speed, limits.maxSpeed + kTolerance) << where(sim, robot);
      if (limits.acceleration) {
        EXPECT_LE(speed - lastSpeed_[i], *limits.acceleration + kTolerance) << where(sim, robot);
      }
      if (limits.deceleration) {
        EXPECT_LE(lastSpeed_[i] - speed, *limits.deceleration + kTolerance) << where(sim, robot);
      }
      lastSpeed_[i] = speed;
      maxSpeed = std::max(maxSpeed, speed);

      // However the robot is placed between cells, it is only ever over cells it holds.
      const warehousesim::core::Vec2 pose = robot.pose();
      const Point under{static_cast<std::int32_t>(std::floor(pose.x)),
                        static_cast<std::int32_t>(std::floor(pose.y))};
      EXPECT_TRUE(std::find(expected.begin(), expected.end(), under) != expected.end())
          << "centre over a cell the robot does not hold; " << where(sim, robot);
      EXPECT_TRUE(sim.world().isWalkable(under)) << where(sim, robot);

      if (robot.segmentProgress() > 0.0) {
        ++ticksBetweenCells;
      }
      ++phases[static_cast<std::size_t>(robot.motion())];

      for (std::size_t j = i + 1; j < robots.size(); ++j) {
        const warehousesim::core::Vec2 other = robots[j].pose();
        const double gap = std::hypot(pose.x - other.x, pose.y - other.y);
        minGap = std::min(minGap, gap);
        EXPECT_GE(gap, 1.0 - kTolerance)
            << "robots " << i + 1 << " and " << j + 1 << " too close, " << where(sim, robot);
      }
    }
  }

  double maxSpeed = 0.0;
  double minGap = 1e9;
  long ticksBetweenCells = 0;
  long phases[5] = {0, 0, 0, 0, 0};  // indexed by MotionPhase

 private:
  struct RobotSpecView {
    double maxSpeed;
    std::optional<double> acceleration;
    std::optional<double> deceleration;
  };
  static std::string where(const Simulation& sim, const Robot& robot) {
    return "robot " + std::to_string(robot.id().value) + " tick " + std::to_string(sim.tick());
  }

  std::vector<double> lastSpeed_;
};

// Steps the simulation, auditing every tick: the audit compares each tick with the one before, so
// it must be shown all of them.
void runAudited(Simulation& sim, MotionAudit& audit, int ticks) {
  for (int i = 0; i < ticks; ++i) {
    sim.step();
    audit.check(sim);
  }
}

// ---- successful deliveries ---------------------------------------------------------------------

TEST(MovementTest, ARobotDrivesToThePickupLoadsCarriesTheCargoToTheDropoffAndUnloadsIt) {
  Simulation sim(layout({".............."}), {P(0, 0)}, vehicleConfig());
  const TaskId id = sim.submitTask(P(6, 0), P(12, 0));
  MotionAudit audit(sim);

  struct Phase {
    RobotState state;
    NavigationGoal goal;
    bool operator==(const Phase&) const = default;
  };
  std::vector<Phase> phases;
  int ticksToPickup = 0;
  for (int tick = 1; tick <= 400 && sim.tasks().find(id)->status != TaskStatus::Completed; ++tick) {
    sim.step();
    audit.check(sim);
    const Robot& robot = sim.robots()[0];
    const Phase phase{robot.state(), robot.goal()};
    if (phases.empty() || !(phases.back() == phase)) {
      phases.push_back(phase);
    }
    if (ticksToPickup == 0 && robot.position() == P(6, 0)) {
      ticksToPickup = tick;
    }
    // The cargo is aboard from the end of loading to the end of unloading, and only then.
    const bool aboard =
        (robot.state() == RobotState::Navigating && robot.goal() == NavigationGoal::Dropoff) ||
        robot.state() == RobotState::Unloading;
    EXPECT_EQ(robot.carriesCargo(), aboard) << "tick " << tick;
    EXPECT_EQ(sim.snapshot().robots[0].carrying, aboard) << "tick " << tick;
  }

  using S = RobotState;
  using G = NavigationGoal;
  EXPECT_EQ(phases, (std::vector<Phase>{{S::Navigating, G::Pickup},
                                        {S::Loading, G::None},
                                        {S::Navigating, G::Dropoff},
                                        {S::Unloading, G::None},
                                        {S::Idle, G::None}}));
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);

  // One tick turning to face east, then 6 cells from rest: 2 ticks / 1 cell speeding up, 4.5 at
  // full speed, 1 tick / half a cell braking = 7.5 ticks, so it is there during the 8th.
  EXPECT_EQ(ticksToPickup, 1 + 8);

  const Robot& robot = sim.robots()[0];
  EXPECT_EQ(robot.position(), P(12, 0));
  EXPECT_DOUBLE_EQ(robot.pose().x, 12.5);  // at rest in the middle of the dropoff cell
  EXPECT_DOUBLE_EQ(robot.pose().y, 0.5);
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_EQ(robot.deliveries(), 1U);
  EXPECT_FALSE(robot.carriesCargo());
  EXPECT_TRUE(robot.isAvailable());
  EXPECT_EQ(robot.cellsTravelled(), 12U);
  EXPECT_EQ(robot.battery().level(), 10000U - 12U * 10U);  // energy for each cell entered
  EXPECT_EQ(sim.snapshot().robots[0].deliveries, 1U);
  EXPECT_EQ(sim.snapshot().completedTasks, 1U);
  // At exactly one cell per tick the robot sits on a waypoint at the end of most ticks; it is
  // between cells while speeding up and braking, at the start and end of each of the two legs.
  EXPECT_GE(audit.ticksBetweenCells, 4);
  EXPECT_NEAR(audit.maxSpeed, 1.0, kTolerance);
}

TEST(MovementTest, ARobotTurnsAtEveryCornerOfItsRouteAndStillDelivers) {
  Simulation sim(layout({".....", ".###.", ".....", ".###.", "....."}), {P(0, 0)}, vehicleConfig());
  const TaskId id = sim.submitTask(P(4, 2), P(0, 4));  // a route with several corners
  MotionAudit audit(sim);
  int turningTicks = 0;
  int stoppedAtCorner = 0;
  for (int tick = 0; tick < 400 && sim.tasks().find(id)->status != TaskStatus::Completed; ++tick) {
    sim.step();
    audit.check(sim);
    const Robot& robot = sim.robots()[0];
    if (robot.motion() == MotionPhase::Turning) {
      ++turningTicks;
      EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
    }
    if (robot.state() == RobotState::Navigating && robot.speed() == 0.0 &&
        robot.segmentProgress() == 0.0 && robot.hasPath()) {
      ++stoppedAtCorner;  // at rest on a waypoint with the route still ahead
    }
  }
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  EXPECT_GE(turningTicks, 4);
  EXPECT_GE(stoppedAtCorner, 2);
  EXPECT_EQ(sim.robots()[0].position(), P(0, 4));
  EXPECT_EQ(sim.robots()[0].deliveries(), 1U);
}

TEST(MovementTest, TheSnapshotAndItsJsonCarryThePoseMotionAndCargo) {
  Simulation sim(layout({"........"}), {P(0, 0)}, vehicleConfig());
  (void)sim.submitTask(P(3, 0), P(7, 0));
  sim.run(3);  // turned, then a little way on
  const Snapshot snapshot = sim.snapshot();
  ASSERT_EQ(snapshot.robots.size(), 1U);
  const auto& robot = snapshot.robots[0];
  ASSERT_TRUE(robot.pose.has_value());
  EXPECT_EQ(robot.motion, sim.robots()[0].motion());
  EXPECT_DOUBLE_EQ(robot.pose->center.x, sim.robots()[0].pose().x);
  EXPECT_DOUBLE_EQ(robot.pose->heading, sim.robots()[0].headingAngle());
  EXPECT_GT(robot.center().x, robot.position.x + 0.5 - kTolerance);

  const nlohmann::json json = toJson(snapshot);
  const auto& r = json.at("robots").at(0);
  EXPECT_NEAR(r.at("pose").at("x").get<double>(), robot.center().x, kTolerance);
  EXPECT_NEAR(r.at("pose").at("y").get<double>(), 0.5, kTolerance);
  EXPECT_NEAR(r.at("headingAngle").get<double>(), 0.0, kTolerance);
  EXPECT_EQ(r.at("motion").get<std::string>(), warehousesim::robots::toString(robot.motion));
  EXPECT_FALSE(r.at("carrying").get<bool>());
  EXPECT_EQ(r.at("deliveries").get<int>(), 0);
  EXPECT_EQ(r.at("position").at("x").get<int>(), robot.position.x);  // the logical cell too
}

// ---- movement constraints -----------------------------------------------------------------------

TEST(MovementTest, EveryLayoutKeepsEverySpeedLimitHoldsAndGapOnEveryTick) {
  long ticksBetweenCells = 0;
  long phaseCount[5] = {0, 0, 0, 0, 0};
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    for (const std::uint64_t seed : {1U, 2U}) {
      auto sim = makeLayoutSimulation(info.name, seed);
      ASSERT_TRUE(sim.has_value());
      MotionAudit audit(*sim);
      for (int tick = 0; tick < 500; ++tick) {
        sim->step();
        audit.check(*sim);
        if (::testing::Test::HasFailure()) {
          FAIL() << info.name << " seed " << seed << " tick " << tick;
        }
      }
      EXPECT_GE(audit.minGap, 1.0 - kTolerance);
      EXPECT_EQ(sim->snapshot().failedTasks, 0U);
      ticksBetweenCells += audit.ticksBetweenCells;
      for (int p = 0; p < 5; ++p) {
        phaseCount[p] += audit.phases[p];
      }
    }
  }
  EXPECT_GT(ticksBetweenCells, 1000);
  for (int p = 0; p < 5; ++p) {
    EXPECT_GT(phaseCount[p], 100) << "phase " << p;  // every kind of motion happened
  }
}

TEST(MovementTest, TheSameRunTwiceGivesIdenticalPosesBitForBit) {
  const auto run = [] {
    auto sim = makeLayoutSimulation("fulfilment", 9);
    std::vector<Snapshot> states;
    for (int tick = 0; tick < 300; ++tick) {
      sim->step();
      states.push_back(sim->snapshot());
    }
    return states;
  };
  EXPECT_TRUE(run() == run());
}

TEST(MovementTest, RobotsSpecsAreValidatedWhenTheSimulationIsBuilt) {
  SimulationConfig weakBrakes = vehicleConfig();
  weakBrakes.robot.deceleration = 0.4;  // stopping from top speed would take 1.25 cells
  EXPECT_THROW(Simulation(layout({"..."}), {P(0, 0)}, weakBrakes), std::invalid_argument);
  SimulationConfig badTurn = vehicleConfig();
  badTurn.robot.turnRate = 0.0;
  EXPECT_THROW(Simulation(layout({"..."}), {P(0, 0)}, badTurn), std::invalid_argument);
  EXPECT_NO_THROW(Simulation(layout({"..."}), {P(0, 0)}, vehicleConfig()));
}

// ---- invalid movement is prevented ----------------------------------------------------

// A robot parked in the way, out of service: the driver must go round it and never into it.
// Returns whether the driver was held up (waited) and whether it was given another route.
std::pair<bool, bool> driveAroundAParkedRobot(SimulationConfig config) {
  Simulation sim(layout({"......", "......"}), {P(0, 0), P(3, 0)}, config);
  EXPECT_TRUE(sim.disableRobot(R(2)));
  const TaskId id = sim.submitTask(P(5, 0), P(5, 1));
  MotionAudit audit(sim);
  bool waited = false;
  for (int tick = 0; tick < 300 && sim.tasks().find(id)->status != TaskStatus::Completed; ++tick) {
    sim.step();
    audit.check(sim);
    const Robot& driver = sim.robots()[0];
    EXPECT_NE(driver.position(), P(3, 0));
    EXPECT_GE(std::hypot(driver.pose().x - 3.5, driver.pose().y - 0.5), 1.0 - kTolerance);
    waited = waited || driver.blockedTicks() > 0;
    EXPECT_EQ(sim.robots()[1].position(), P(3, 0));  // the parked robot never moved
    EXPECT_DOUBLE_EQ(sim.robots()[1].speed(), 0.0);
  }
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);  // it went around
  EXPECT_EQ(sim.robots()[0].position(), P(5, 1));
  return {waited, sim.planningStats().reroutes > 0};
}

TEST(MovementTest, ARobotNeverDrivesIntoACellAnotherRobotHoldsAndFindsWayAroundIt) {
  // Without cooperative planning the driver runs up to the parked robot, is held up there and only
  // then routes round it.
  SimulationConfig plain = vehicleConfig();
  plain.cooperative = false;
  const auto [waited, rerouted] = driveAroundAParkedRobot(plain);
  EXPECT_TRUE(waited);  // it really was held up
  EXPECT_FALSE(rerouted);
}

TEST(MovementTest, WithCooperativePlanningTheDriverIsRoutedRoundTheParkedRobotBeforeItIsHeldUp) {
  const auto outcome = driveAroundAParkedRobot(vehicleConfig());
  EXPECT_TRUE(outcome.second);  // the table held it back and the search found another way
}

TEST(MovementTest, AnIdleRobotInTheWayStepsAsideWhenTheOneBehindHasWaited) {
  Simulation sim(layout({"......", "......"}), {P(0, 0), P(3, 0)}, vehicleConfig());
  const TaskId id = sim.submitTask(P(5, 0), P(5, 0));  // robot 1 is nearer, robot 2 is in its way
  MotionAudit audit(sim);
  for (int tick = 0; tick < 300 && sim.tasks().find(id)->status != TaskStatus::Completed; ++tick) {
    sim.step();
    audit.check(sim);
  }
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
  // Whoever took the job, the other robot was never run into.
  EXPECT_GE(audit.minGap, 1.0 - kTolerance);
}

TEST(MovementTest, ACellInFrontOfAWaitingRobotIsNeverEnteredInTheMeantime) {
  // Robot 2 is Unavailable in a one-wide corridor: nobody can ever pass, so the blocked robot must
  // simply stop in front of it and wait, however long that is.
  Simulation sim(layout({"......"}), {P(0, 0), P(4, 0)}, vehicleConfig());
  ASSERT_TRUE(sim.disableRobot(R(2)));
  (void)sim.submitTask(P(5, 0), P(5, 0));
  MotionAudit audit(sim);
  bool waited = false;
  for (int tick = 0; tick < 300; ++tick) {
    sim.step();
    audit.check(sim);
    EXPECT_LE(sim.robots()[0].position().x, 3);
    EXPECT_LE(sim.robots()[0].pose().x, 3.5 + kTolerance);  // never in the parked robot's cell
    waited = waited || sim.robots()[0].blockedTicks() > 0;
  }
  EXPECT_TRUE(waited);
  EXPECT_EQ(sim.robots()[0].position(), P(3, 0));
  EXPECT_DOUBLE_EQ(sim.robots()[0].speed(), 0.0);
  EXPECT_EQ(sim.robots()[0].authority(), 0U);
}

// ---- withdrawn or lost in mid-move --------------------------------------------------------------

TEST(MovementTest, CancellingATaskAtEveryMomentOfADriveLeavesTheRobotStoppedAndReusable) {
  int coasted = 0;
  int cancelledInTime = 0;
  for (int moment = 0; moment <= 26; ++moment) {
    Simulation sim(layout({"................"}), {P(0, 0)}, vehicleConfig());
    const TaskId id = sim.submitTask(P(12, 0), P(14, 0));
    MotionAudit audit(sim);
    runAudited(sim, audit, moment);

    const double speedAtCancel = sim.robots()[0].speed();
    if (!sim.cancelTask(id)) {
      // Too late: the whole job (about 24 ticks) was already done.
      EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed) << "moment " << moment;
      continue;
    }
    ++cancelledInTime;
    const Robot& robot = sim.robots()[0];
    EXPECT_EQ(robot.state(), RobotState::Idle) << "moment " << moment;
    EXPECT_FALSE(robot.task().has_value());
    audit.check(sim);  // the world's holds shrank with the robot's
    if (robot.hasPath()) {
      ++coasted;
      EXPECT_FALSE(robot.isAvailable());
    }

    double fastest = 0.0;
    for (int tick = 0; tick < 25; ++tick) {
      sim.step();
      audit.check(sim);
      fastest = std::max(fastest, robot.speed());
    }
    EXPECT_LE(fastest, speedAtCancel + kTolerance) << "moment " << moment;  // only slowed down
    EXPECT_FALSE(robot.hasPath()) << "moment " << moment;
    EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
    EXPECT_DOUBLE_EQ(robot.segmentProgress(), 0.0);
    EXPECT_TRUE(robot.isAvailable());
    EXPECT_EQ(sim.world().holdsOf(robot.id()), (std::vector<Point>{robot.position()}));
    EXPECT_DOUBLE_EQ(robot.pose().x, robot.position().x + 0.5);  // stopped in a cell centre
    EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Cancelled);

    // And it can be given another job straight away.
    const TaskId next = sim.submitTask(P(2, 0), P(4, 0));
    for (int tick = 0; tick < 120 && sim.tasks().find(next)->status != TaskStatus::Completed;
         ++tick) {
      sim.step();
      audit.check(sim);
    }
    EXPECT_EQ(sim.tasks().find(next)->status, TaskStatus::Completed) << "moment " << moment;
    EXPECT_EQ(sim.snapshot().completedTasks, 1U);
    EXPECT_EQ(sim.snapshot().cancelledTasks, 1U);
  }
  EXPECT_GE(cancelledInTime, 20);
  EXPECT_GT(coasted, 12);  // many of those moments caught the robot between cells
}

TEST(MovementTest, ARobotTakenOutOfServiceWhileDrivingCoastsToAStopBeforeItIsUnavailable) {
  Simulation sim(layout({"................"}), {P(0, 0)}, vehicleConfig());
  const TaskId id = sim.submitTask(P(14, 0), P(15, 0));
  MotionAudit audit(sim);
  runAudited(sim, audit, 6);
  const Robot& robot = sim.robots()[0];
  ASSERT_GT(robot.speed(), 0.0);
  const double speedAtDisable = robot.speed();

  ASSERT_TRUE(sim.disableRobot(R(1)));
  EXPECT_EQ(robot.state(), RobotState::Unavailable);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Pending);  // handed back, uncollected
  EXPECT_FALSE(robot.isAvailable());
  audit.check(sim);
  double fastest = 0.0;
  for (int tick = 0; tick < 20; ++tick) {
    sim.step();
    audit.check(sim);
    fastest = std::max(fastest, robot.speed());
    EXPECT_EQ(robot.state(), RobotState::Unavailable);
  }
  EXPECT_LE(fastest, speedAtDisable + kTolerance);
  EXPECT_FALSE(robot.hasPath());
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_EQ(sim.world().holdsOf(robot.id()), (std::vector<Point>{robot.position()}));

  // Back in service, it picks the job up again from where it stopped.
  ASSERT_TRUE(sim.restoreRobot(R(1)));
  for (int tick = 0; tick < 200 && sim.tasks().find(id)->status != TaskStatus::Completed; ++tick) {
    sim.step();
    audit.check(sim);
  }
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Completed);
}

TEST(MovementTest, AFlatBatteryWhileDrivingTakesTheRobotOutOfServiceAtTheNextWaypoint) {
  SimulationConfig config = vehicleConfig();
  config.robot.batteryCapacity = 60;  // six cells
  Simulation sim(layout({"..............................."}), {P(0, 0)}, config);
  const TaskId id = sim.submitTask(P(25, 0), P(28, 0));
  MotionAudit audit(sim);
  for (int tick = 0; tick < 100 && sim.robots()[0].state() != RobotState::Unavailable; ++tick) {
    sim.step();
    audit.check(sim);
  }
  const Robot& robot = sim.robots()[0];
  ASSERT_EQ(robot.state(), RobotState::Unavailable);
  EXPECT_EQ(robot.battery().level(), 0U);
  EXPECT_EQ(sim.tasks().find(id)->status, TaskStatus::Pending);  // back in the queue for others

  // It was doing full speed when the last of the energy went; it did not stop dead.
  for (int tick = 0; tick < 20; ++tick) {
    sim.step();
    audit.check(sim);
  }
  EXPECT_FALSE(robot.hasPath());
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_GE(robot.position().x, 6);
  EXPECT_LE(robot.position().x, 8);  // within a braking distance of where the energy ran out
  EXPECT_EQ(robot.cellsTravelled(), static_cast<std::uint64_t>(robot.position().x));
}

}  // namespace
