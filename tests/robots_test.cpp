#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/robots/battery.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/robots/robot_spec.hpp"
#include "warehousesim/robots/robot_state.hpp"

namespace {

using warehousesim::core::Direction;
using warehousesim::core::Point;
using warehousesim::core::Rng;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Battery;
using warehousesim::robots::InvalidTransition;
using warehousesim::robots::isStationary;
using warehousesim::robots::isValidTransition;
using warehousesim::robots::kAllRobotStates;
using warehousesim::robots::NavigationGoal;
using warehousesim::robots::Robot;
using warehousesim::robots::RobotPath;
using warehousesim::robots::RobotSpec;
using warehousesim::robots::RobotState;
using warehousesim::robots::Velocity;

Point P(int x, int y) { return Point{x, y}; }
RobotId R(std::uint32_t id) { return RobotId{id}; }
TaskId T(std::uint32_t id) { return TaskId{id}; }

// A small battery with round numbers so expected levels can be computed manually.
RobotSpec energySpec() {
  RobotSpec spec;
  spec.batteryCapacity = 1000;
  spec.idleDrainPerTick = 1;
  spec.moveCostPerCell = 10;
  spec.handlingDrainPerTick = 3;
  spec.chargePerTick = 100;
  spec.lowBatteryLevel = 300;
  spec.loadTicks = 2;
  spec.unloadTicks = 1;
  return spec;
}

// The cells strictly after `from` along +x: (from.x+1, y) ... (from.x+count, y).
RobotPath eastward(Point from, int count) {
  RobotPath path;
  for (int i = 1; i <= count; ++i) {
    path.push_back(P(from.x + i, from.y));
  }
  return path;
}

void expectConsistent(const Robot& robot) {
  EXPECT_EQ(robot.inconsistency().value_or("consistent"), "consistent");
}

// One tick of motion: grants the robot the cells it wants (the part the simulation plays, since
// a robot never looks at the world) and drives it. Returns how many cells it passed.
std::uint32_t step(Robot& robot, double dt = 1.0) {
  while (robot.authority() < robot.wantedAuthority(dt)) {
    robot.extendAuthority();
  }
  return robot.drive(dt);
}

// Walks the robot along its whole path.
void driveToDestination(Robot& robot) {
  while (robot.hasPath()) {
    (void)step(robot);
  }
}

// Everything observable about a robot, to check that a rejected operation changed nothing.
struct Observed {
  RobotState state;
  NavigationGoal goal;
  Point position;
  Direction heading;
  Battery::Units battery;
  std::optional<TaskId> task;
  std::optional<Point> destination;
  std::size_t pathLength;
  double speed;
  std::uint32_t dwell;
  std::uint64_t cells;
  friend bool operator==(const Observed&, const Observed&) = default;
};

Observed observe(const Robot& r) {
  return {r.state(),           r.goal(),           r.position(),      r.heading(),
          r.battery().level(), r.task(),           r.destination(),   r.path().size(),
          r.speed(),           r.dwellRemaining(), r.cellsTravelled()};
}

// Builds a robot that is in `state` using only legitimate operations.
Robot robotIn(RobotState state, const RobotSpec& spec = energySpec()) {
  Robot robot(R(1), P(0, 0), spec);
  switch (state) {
    case RobotState::Idle:
      break;
    case RobotState::Navigating:
      robot.beginTask(T(1), P(2, 0), eastward(P(0, 0), 2));
      break;
    case RobotState::Loading:
      robot.beginTask(T(1), P(1, 0), eastward(P(0, 0), 1));
      driveToDestination(robot);
      robot.beginLoading();
      break;
    case RobotState::Unloading:
      robot.beginTask(T(1), P(1, 0), eastward(P(0, 0), 1));
      driveToDestination(robot);
      robot.beginLoading();
      while (robot.dwellRemaining() > 0) {
        robot.elapseTick();
      }
      robot.finishLoading(P(2, 0), eastward(P(1, 0), 1));
      driveToDestination(robot);
      robot.beginUnloading();
      break;
    case RobotState::Charging:
      robot.beginCharging();
      break;
    case RobotState::Unavailable:
      robot.makeUnavailable();
      break;
  }
  return robot;
}

// ---- creation -------------------------------------------------------------------------

TEST(RobotCreationTest, NewRobotIsIdleAtRestWithAFullBattery) {
  const Robot robot(R(3), P(4, 5));
  EXPECT_EQ(robot.id(), R(3));
  EXPECT_EQ(robot.position(), P(4, 5));
  EXPECT_EQ(robot.heading(), Direction::North);
  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_EQ(robot.goal(), NavigationGoal::None);
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_EQ(robot.velocity(), (Velocity{0.0, 0.0}));
  EXPECT_DOUBLE_EQ(robot.maxSpeed(), 1.0);
  EXPECT_TRUE(robot.battery().isFull());
  EXPECT_EQ(robot.battery().level(), 10000U);
  EXPECT_FALSE(robot.task().has_value());
  EXPECT_FALSE(robot.destination().has_value());
  EXPECT_FALSE(robot.hasPath());
  EXPECT_FALSE(robot.nextStep().has_value());
  EXPECT_EQ(robot.cellsTravelled(), 0U);
  EXPECT_EQ(robot.dwellRemaining(), 0U);
  EXPECT_FALSE(robot.needsCharge());
  EXPECT_TRUE(robot.isAvailable());
  EXPECT_FALSE(robot.isStationary());
  expectConsistent(robot);
}

TEST(RobotCreationTest, SpecAndInitialHeadingAreHonoured) {
  RobotSpec spec;
  spec.maxSpeed = 0.5;
  spec.batteryCapacity = 500;
  spec.initialCharge = 200;
  const Robot robot(R(1), P(0, 0), spec, Direction::West);
  EXPECT_DOUBLE_EQ(robot.maxSpeed(), 0.5);
  EXPECT_EQ(robot.battery().capacity(), 500U);
  EXPECT_EQ(robot.battery().level(), 200U);
  EXPECT_EQ(robot.heading(), Direction::West);
  EXPECT_EQ(robot.spec().batteryCapacity, 500U);
  expectConsistent(robot);
}

TEST(RobotCreationTest, InvalidIdIsRejected) {
  EXPECT_THROW(Robot(RobotId{}, P(0, 0)), std::invalid_argument);
}

TEST(RobotCreationTest, InvalidSpecsAreRejected) {
  const double nan = std::nan("");
  const auto rejects = [](const RobotSpec& spec) {
    try {
      const Robot robot(R(1), P(0, 0), spec);
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  RobotSpec spec;
  spec.maxSpeed = 0.0;
  EXPECT_TRUE(rejects(spec));
  spec.maxSpeed = -0.5;
  EXPECT_TRUE(rejects(spec));
  spec.maxSpeed = 1.5;
  EXPECT_TRUE(rejects(spec));  // a robot enters at most one cell per tick
  spec.maxSpeed = nan;
  EXPECT_TRUE(rejects(spec));
  spec = RobotSpec{};
  spec.batteryCapacity = 0;
  EXPECT_TRUE(rejects(spec));
  spec = RobotSpec{};
  spec.initialCharge = spec.batteryCapacity + 1;
  EXPECT_TRUE(rejects(spec));
  spec = RobotSpec{};
  spec.chargePerTick = 0;
  EXPECT_TRUE(rejects(spec));
  spec = RobotSpec{};
  spec.lowBatteryLevel = spec.batteryCapacity;
  EXPECT_TRUE(rejects(spec));
  EXPECT_FALSE(rejects(RobotSpec{}));
  spec = RobotSpec{};
  spec.maxSpeed = 1.0;  // the boundary values are valid
  spec.initialCharge = 0;
  EXPECT_FALSE(rejects(spec));
}

// ---- the transition table --------------------------------------------------------------

TEST(RobotStateTest, TransitionTableMatchesTheDocumentedDesign) {
  using S = RobotState;
  const std::set<std::pair<S, S>> allowed = {
      {S::Idle, S::Navigating},       {S::Idle, S::Charging},          {S::Idle, S::Unavailable},
      {S::Navigating, S::Loading},    {S::Navigating, S::Unloading},   {S::Navigating, S::Charging},
      {S::Navigating, S::Idle},       {S::Navigating, S::Unavailable}, {S::Loading, S::Navigating},
      {S::Loading, S::Idle},          {S::Loading, S::Unavailable},    {S::Unloading, S::Idle},
      {S::Unloading, S::Unavailable}, {S::Charging, S::Idle},          {S::Charging, S::Navigating},
      {S::Charging, S::Unavailable},  {S::Unavailable, S::Idle},
  };
  for (const S from : kAllRobotStates) {
    for (const S to : kAllRobotStates) {
      EXPECT_EQ(isValidTransition(from, to), allowed.contains({from, to}));
    }
  }
}

TEST(RobotStateTest, NoStateTransitionsToItself) {
  for (const RobotState state : kAllRobotStates) {
    EXPECT_FALSE(isValidTransition(state, state));
  }
}

TEST(RobotStateTest, UnavailableOnlyLeadsBackToIdle) {
  for (const RobotState to : kAllRobotStates) {
    EXPECT_EQ(isValidTransition(RobotState::Unavailable, to), to == RobotState::Idle);
  }
  for (const RobotState from : kAllRobotStates) {
    if (from != RobotState::Unavailable) {
      EXPECT_TRUE(isValidTransition(from, RobotState::Unavailable));
    }
  }
}

TEST(RobotStateTest, EveryStateCanBeReachedFromIdleAndLeftAgain) {
  std::set<RobotState> reached = {RobotState::Idle};
  std::vector<RobotState> frontier = {RobotState::Idle};
  while (!frontier.empty()) {
    const RobotState from = frontier.back();
    frontier.pop_back();
    for (const RobotState to : kAllRobotStates) {
      if (isValidTransition(from, to) && reached.insert(to).second) {
        frontier.push_back(to);
      }
    }
  }
  EXPECT_EQ(reached.size(), kAllRobotStates.size());
  for (const RobotState state : kAllRobotStates) {
    bool canLeave = false;
    for (const RobotState to : kAllRobotStates) {
      canLeave = canLeave || isValidTransition(state, to);
    }
    EXPECT_TRUE(canLeave);  // no dead ends
  }
}

TEST(RobotStateTest, StationaryStatesAreTheOnesBusyInPlaceOrOutOfService) {
  EXPECT_FALSE(isStationary(RobotState::Idle));
  EXPECT_FALSE(isStationary(RobotState::Navigating));
  EXPECT_TRUE(isStationary(RobotState::Loading));
  EXPECT_TRUE(isStationary(RobotState::Unloading));
  EXPECT_TRUE(isStationary(RobotState::Charging));
  EXPECT_TRUE(isStationary(RobotState::Unavailable));
}

TEST(RobotStateTest, NamesAreDistinct) {
  std::set<std::string> states;
  for (const RobotState state : kAllRobotStates) {
    states.insert(warehousesim::robots::toString(state));
  }
  EXPECT_EQ(states.size(), 6U);
  std::set<std::string> goals;
  for (const NavigationGoal goal : {NavigationGoal::None, NavigationGoal::Pickup,
                                    NavigationGoal::Dropoff, NavigationGoal::Charger}) {
    goals.insert(warehousesim::robots::toString(goal));
  }
  EXPECT_EQ(goals.size(), 4U);
}

TEST(RobotStateTest, RobotsReportTheirOwnTableThroughCanTransitionTo) {
  for (const RobotState from : kAllRobotStates) {
    const Robot robot = robotIn(from);
    ASSERT_EQ(robot.state(), from);
    for (const RobotState to : kAllRobotStates) {
      EXPECT_EQ(robot.canTransitionTo(to), isValidTransition(from, to));
    }
  }
}

// ---- valid lifecycles ------------------------------------------------------------------

TEST(RobotLifecycleTest, FullTaskCycleWalksTheStateMachine) {
  Robot robot(R(5), P(0, 0), energySpec());  // 1000 units, 10 per cell, 3 per handling tick
  robot.beginTask(T(9), P(2, 0), eastward(P(0, 0), 2));
  EXPECT_EQ(robot.state(), RobotState::Navigating);
  EXPECT_EQ(robot.goal(), NavigationGoal::Pickup);
  EXPECT_EQ(robot.task(), T(9));
  EXPECT_EQ(robot.destination(), P(2, 0));
  EXPECT_EQ(robot.path().size(), 2U);
  EXPECT_FALSE(robot.hasArrived());
  EXPECT_THROW(robot.beginLoading(), InvalidTransition);  // not there yet
  expectConsistent(robot);

  EXPECT_EQ(step(robot), 1U);
  EXPECT_EQ(robot.heading(), Direction::East);
  EXPECT_DOUBLE_EQ(robot.speed(), 1.0);  // still driving: one more cell to go
  EXPECT_EQ(robot.velocity(), (Velocity{1.0, 0.0}));
  EXPECT_EQ(robot.position(), P(1, 0));
  EXPECT_EQ(robot.battery().level(), 990U);
  EXPECT_EQ(step(robot), 1U);
  EXPECT_EQ(robot.position(), P(2, 0));
  EXPECT_EQ(robot.battery().level(), 980U);
  EXPECT_TRUE(robot.hasArrived());
  EXPECT_FALSE(robot.hasPath());
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);  // stopped on arrival
  expectConsistent(robot);

  robot.beginLoading();
  EXPECT_EQ(robot.state(), RobotState::Loading);
  EXPECT_EQ(robot.goal(), NavigationGoal::None);
  EXPECT_EQ(robot.task(), T(9));  // still holding the task
  EXPECT_FALSE(robot.destination().has_value());
  EXPECT_EQ(robot.dwellRemaining(), 2U);
  EXPECT_TRUE(robot.isStationary());
  EXPECT_THROW(robot.finishLoading(P(4, 0), eastward(P(2, 0), 2)), InvalidTransition);
  robot.elapseTick();
  EXPECT_EQ(robot.dwellRemaining(), 1U);
  robot.elapseTick();
  EXPECT_EQ(robot.dwellRemaining(), 0U);
  EXPECT_EQ(robot.battery().level(), 974U);  // 2 ticks x 3
  expectConsistent(robot);

  robot.finishLoading(P(4, 0), eastward(P(2, 0), 2));
  EXPECT_EQ(robot.state(), RobotState::Navigating);
  EXPECT_EQ(robot.goal(), NavigationGoal::Dropoff);
  EXPECT_EQ(robot.task(), T(9));
  driveToDestination(robot);
  EXPECT_EQ(robot.position(), P(4, 0));
  EXPECT_EQ(robot.battery().level(), 954U);

  robot.beginUnloading();
  EXPECT_EQ(robot.state(), RobotState::Unloading);
  EXPECT_EQ(robot.dwellRemaining(), 1U);
  EXPECT_THROW(robot.finishUnloading(), InvalidTransition);
  robot.elapseTick();
  robot.finishUnloading();
  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_FALSE(robot.task().has_value());
  EXPECT_EQ(robot.battery().level(), 951U);
  EXPECT_EQ(robot.id(), R(5));  // identity never changes
  EXPECT_EQ(robot.cellsTravelled(), 4U);
  expectConsistent(robot);
}

TEST(RobotLifecycleTest, ZeroLengthDwellCompletesImmediately) {
  const RobotSpec spec;  // loadTicks == unloadTicks == 0
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(0, 0), {});  // already at the pickup
  EXPECT_TRUE(robot.hasArrived());
  robot.beginLoading();
  EXPECT_EQ(robot.dwellRemaining(), 0U);
  robot.finishLoading(P(0, 0), {});
  robot.beginUnloading();
  robot.finishUnloading();
  EXPECT_EQ(robot.state(), RobotState::Idle);
  expectConsistent(robot);
}

TEST(RobotLifecycleTest, ChargingTripEndsWithAFullBattery) {
  RobotSpec spec = energySpec();
  spec.initialCharge = 250;  // below the 300 reserve
  Robot robot(R(2), P(0, 0), spec);
  EXPECT_TRUE(robot.needsCharge());
  EXPECT_FALSE(robot.isAvailable());

  robot.beginChargerTrip(P(2, 0), eastward(P(0, 0), 2));
  EXPECT_EQ(robot.state(), RobotState::Navigating);
  EXPECT_EQ(robot.goal(), NavigationGoal::Charger);
  EXPECT_FALSE(robot.task().has_value());                  // a charging trip carries no task
  EXPECT_THROW(robot.beginCharging(), InvalidTransition);  // has not arrived
  driveToDestination(robot);
  EXPECT_EQ(robot.battery().level(), 230U);

  robot.beginCharging();
  EXPECT_EQ(robot.state(), RobotState::Charging);
  EXPECT_FALSE(robot.destination().has_value());
  std::uint32_t ticks = 0;
  while (!robot.battery().isFull()) {
    robot.elapseTick();
    ++ticks;
    expectConsistent(robot);
  }
  EXPECT_EQ(ticks, 8U);  // (1000 - 230) / 100 rounded up
  EXPECT_FALSE(robot.needsCharge());
  robot.stopCharging();
  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_TRUE(robot.isAvailable());
}

TEST(RobotLifecycleTest, AnIdleRobotCanStartChargingWhereItStands) {
  Robot robot(R(1), P(3, 3), energySpec());
  robot.beginCharging();
  EXPECT_EQ(robot.state(), RobotState::Charging);
  robot.stopCharging();
  EXPECT_EQ(robot.state(), RobotState::Idle);
}

TEST(RobotLifecycleTest, AChargingRobotMayBeGivenATaskDirectly) {
  Robot robot = robotIn(RobotState::Charging);
  robot.beginTask(T(4), P(1, 0), eastward(P(0, 0), 1));
  EXPECT_EQ(robot.state(), RobotState::Navigating);
  EXPECT_EQ(robot.goal(), NavigationGoal::Pickup);
  expectConsistent(robot);
}

TEST(RobotLifecycleTest, AbortTaskReturnsToIdleAndDropsEverythingTaskRelated) {
  for (const RobotState state :
       {RobotState::Navigating, RobotState::Loading, RobotState::Unloading}) {
    Robot robot = robotIn(state);
    robot.abortTask();
    EXPECT_EQ(robot.state(), RobotState::Idle);
    EXPECT_EQ(robot.goal(), NavigationGoal::None);
    EXPECT_FALSE(robot.task().has_value());
    EXPECT_FALSE(robot.destination().has_value());
    EXPECT_FALSE(robot.hasPath());
    EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
    EXPECT_EQ(robot.dwellRemaining(), 0U);
    expectConsistent(robot);
  }
}

TEST(RobotLifecycleTest, AnyActiveRobotCanBeTakenOutOfServiceAndIsThenInert) {
  for (const RobotState state : {RobotState::Idle, RobotState::Navigating, RobotState::Loading,
                                 RobotState::Unloading, RobotState::Charging}) {
    Robot robot = robotIn(state);
    robot.makeUnavailable();
    EXPECT_EQ(robot.state(), RobotState::Unavailable);
    EXPECT_FALSE(robot.task().has_value());
    EXPECT_FALSE(robot.hasPath());
    EXPECT_FALSE(robot.destination().has_value());
    EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
    EXPECT_TRUE(robot.isStationary());
    EXPECT_FALSE(robot.isAvailable());
    const std::uint32_t before = robot.battery().level();
    robot.elapseTick();
    robot.elapseTick();
    EXPECT_EQ(robot.battery().level(), before);  // no drain, no charge
    expectConsistent(robot);
  }
}

TEST(RobotLifecycleTest, RestoreBringsAnUnavailableRobotBackToIdle) {
  Robot robot = robotIn(RobotState::Unavailable);
  robot.restore();
  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_TRUE(robot.isAvailable());
  expectConsistent(robot);
}

TEST(RobotLifecycleTest, ARobotWithAFlatBatteryNeedsItsBatterySwappedBeforeRestoring) {
  RobotSpec spec = energySpec();
  spec.initialCharge = 0;
  Robot robot(R(1), P(0, 0), spec);
  robot.makeUnavailable();
  EXPECT_THROW(robot.restore(), InvalidTransition);
  EXPECT_EQ(robot.state(), RobotState::Unavailable);
  robot.replaceBattery();
  EXPECT_TRUE(robot.battery().isFull());
  robot.restore();
  EXPECT_EQ(robot.state(), RobotState::Idle);
}

// ---- invalid transitions ---------------------------------------------------------------

struct Operation {
  const char* name;
  void (*apply)(Robot&);
  std::set<RobotState> allowedFrom;
};

// Every transition operation with arguments that are themselves valid, so any rejection is down
// to the robot's state. Paths are empty, destinations are the robot's own cell.
const std::vector<Operation>& operations() {
  using S = RobotState;
  static const std::vector<Operation> ops = {
      {"beginTask", [](Robot& r) { r.beginTask(T(7), r.position(), {}); }, {S::Idle, S::Charging}},
      {"beginChargerTrip", [](Robot& r) { r.beginChargerTrip(r.position(), {}); }, {S::Idle}},
      {"beginLoading", [](Robot& r) { r.beginLoading(); }, {S::Navigating}},
      {"finishLoading", [](Robot& r) { r.finishLoading(r.position(), {}); }, {S::Loading}},
      {"beginUnloading", [](Robot& r) { r.beginUnloading(); }, {S::Navigating}},
      {"finishUnloading", [](Robot& r) { r.finishUnloading(); }, {S::Unloading}},
      {"beginCharging", [](Robot& r) { r.beginCharging(); }, {S::Idle, S::Navigating}},
      {"stopCharging", [](Robot& r) { r.stopCharging(); }, {S::Charging}},
      {"abortTask", [](Robot& r) { r.abortTask(); }, {S::Navigating, S::Loading, S::Unloading}},
      {"makeUnavailable",
       [](Robot& r) { r.makeUnavailable(); },
       {S::Idle, S::Navigating, S::Loading, S::Unloading, S::Charging}},
      {"restore", [](Robot& r) { r.restore(); }, {S::Unavailable}},
      {"replaceBattery", [](Robot& r) { r.replaceBattery(); }, {S::Unavailable}},
      {"redirectChargerTrip",
       [](Robot& r) { r.redirectChargerTrip(r.position(), {}); },
       {S::Navigating}},
      {"setPath", [](Robot& r) { r.setPath({}); }, {S::Navigating}},
      {"sidestep", [](Robot& r) { r.sidestep(P(0, 1)); }, {S::Idle, S::Navigating}},
  };
  return ops;
}

TEST(RobotTransitionTest, EveryOperationIsRejectedFromEveryStateItDoesNotAllow) {
  for (const Operation& op : operations()) {
    for (const RobotState state : kAllRobotStates) {
      if (op.allowedFrom.contains(state)) {
        continue;
      }
      Robot robot = robotIn(state);
      const Observed before = observe(robot);
      bool threw = false;
      try {
        op.apply(robot);
      } catch (const InvalidTransition&) {
        threw = true;
      }
      if (!threw) {
        ADD_FAILURE();
        std::cerr << op.name << " was accepted in state " << toString(state) << "\n";
      }
      EXPECT_TRUE(observe(robot) == before);  // a rejected operation changes nothing
      expectConsistent(robot);
    }
  }
}

TEST(RobotTransitionTest, OperationsRequiringArrivalRejectARobotStillOnItsWay) {
  Robot toPickup = robotIn(RobotState::Navigating);
  EXPECT_THROW(toPickup.beginLoading(), InvalidTransition);
  EXPECT_THROW(toPickup.beginUnloading(), InvalidTransition);  // wrong goal
  EXPECT_THROW(toPickup.beginCharging(), InvalidTransition);   // wrong goal

  Robot toCharger(R(1), P(0, 0), energySpec());
  toCharger.beginChargerTrip(P(2, 0), eastward(P(0, 0), 2));
  EXPECT_THROW(toCharger.beginCharging(), InvalidTransition);  // not arrived
  driveToDestination(toCharger);
  EXPECT_THROW(toCharger.beginLoading(), InvalidTransition);  // wrong goal even though arrived
  EXPECT_THROW(toCharger.beginUnloading(), InvalidTransition);
  EXPECT_NO_THROW(toCharger.beginCharging());
}

TEST(RobotTransitionTest, ArrivingAtAPickupDoesNotAllowUnloadingThere) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginTask(T(1), P(1, 0), eastward(P(0, 0), 1));
  driveToDestination(robot);
  EXPECT_THROW(robot.beginUnloading(), InvalidTransition);
  EXPECT_NO_THROW(robot.beginLoading());
}

TEST(RobotTransitionTest, ANavigatingRobotCannotBeGivenAnotherTask) {
  Robot robot = robotIn(RobotState::Navigating);
  EXPECT_THROW(robot.beginTask(T(2), P(0, 0), {}), InvalidTransition);
  EXPECT_EQ(robot.task(), T(1));
}

TEST(RobotTransitionTest, TasksAndChargerTripsNeedAChargedBattery) {
  RobotSpec spec = energySpec();
  spec.initialCharge = 0;
  Robot robot(R(1), P(0, 0), spec);
  EXPECT_THROW(robot.beginTask(T(1), P(1, 0), eastward(P(0, 0), 1)), InvalidTransition);
  EXPECT_THROW(robot.beginChargerTrip(P(1, 0), eastward(P(0, 0), 1)), InvalidTransition);
  EXPECT_EQ(robot.state(), RobotState::Idle);
  EXPECT_NO_THROW(robot.beginCharging());  // a flat robot can still charge where it stands
}

TEST(RobotTransitionTest, MalformedArgumentsAreRejectedWithoutChangingTheRobot) {
  Robot robot(R(1), P(0, 0), energySpec());
  const Observed before = observe(robot);
  EXPECT_THROW(robot.beginTask(TaskId{}, P(1, 0), eastward(P(0, 0), 1)), std::invalid_argument);
  EXPECT_THROW(robot.beginTask(T(1), P(5, 0), {}), std::invalid_argument);  // empty, not there
  EXPECT_THROW(robot.beginTask(T(1), P(2, 0), eastward(P(0, 0), 1)),
               std::invalid_argument);  // path stops short
  EXPECT_THROW(robot.beginTask(T(1), P(2, 0), RobotPath{P(2, 0)}),
               std::invalid_argument);  // first step not adjacent
  EXPECT_THROW(robot.beginTask(T(1), P(1, 1), RobotPath{P(1, 1)}),
               std::invalid_argument);  // diagonal step
  EXPECT_TRUE(observe(robot) == before);
  expectConsistent(robot);
}

TEST(RobotTransitionTest, SetPathMustEndAtTheDestinationAndOnlyWorksWhileNavigating) {
  Robot robot = robotIn(RobotState::Navigating);  // heading to (2,0)
  EXPECT_THROW(robot.setPath(eastward(P(0, 0), 1)), std::invalid_argument);
  EXPECT_THROW(robot.setPath({}), std::invalid_argument);
  EXPECT_NO_THROW(robot.setPath(RobotPath{P(0, 1), P(1, 1), P(2, 1), P(2, 0)}));
  EXPECT_EQ(robot.path().size(), 4U);
  expectConsistent(robot);
}

TEST(RobotTransitionTest, RedirectChargerTripOnlyAppliesToChargerTrips) {
  Robot pickup = robotIn(RobotState::Navigating);
  EXPECT_THROW(pickup.redirectChargerTrip(P(0, 0), {}), InvalidTransition);
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginChargerTrip(P(2, 0), eastward(P(0, 0), 2));
  robot.redirectChargerTrip(P(0, 3), RobotPath{P(0, 1), P(0, 2), P(0, 3)});
  EXPECT_EQ(robot.destination(), P(0, 3));
  EXPECT_EQ(robot.goal(), NavigationGoal::Charger);
  expectConsistent(robot);
}

TEST(RobotTransitionTest, MovementPrimitivesRequireAPath) {
  Robot idle(R(1), P(0, 0), energySpec());
  EXPECT_THROW((void)idle.drive(), InvalidTransition);
  EXPECT_THROW(idle.extendAuthority(), InvalidTransition);
  Robot arrived(R(2), P(0, 0), energySpec());
  arrived.beginTask(T(1), P(0, 0), {});
  EXPECT_THROW((void)arrived.drive(), InvalidTransition);  // nowhere left to go
  EXPECT_THROW(arrived.extendAuthority(), InvalidTransition);
}

// ---- battery --------------------------------------------------------------------------

TEST(BatteryTest, StartsFullOrAtTheGivenLevel) {
  const Battery full(500);
  EXPECT_EQ(full.level(), 500U);
  EXPECT_TRUE(full.isFull());
  EXPECT_FALSE(full.isEmpty());
  EXPECT_DOUBLE_EQ(full.fraction(), 1.0);
  const Battery partial(500, 125);
  EXPECT_EQ(partial.level(), 125U);
  EXPECT_DOUBLE_EQ(partial.fraction(), 0.25);
  EXPECT_TRUE(Battery(500, 0).isEmpty());
}

TEST(BatteryTest, RejectsInvalidConstruction) {
  EXPECT_THROW(Battery(0), std::invalid_argument);
  EXPECT_THROW(Battery(0, 0), std::invalid_argument);
  EXPECT_THROW(Battery(100, 101), std::invalid_argument);
}

TEST(BatteryTest, ConsumeAndChargeReturnWhatWasActuallyMoved) {
  Battery battery(100, 60);
  EXPECT_EQ(battery.consume(25), 25U);
  EXPECT_EQ(battery.level(), 35U);
  EXPECT_EQ(battery.consume(1000), 35U);  // saturates at empty, never wraps around
  EXPECT_EQ(battery.level(), 0U);
  EXPECT_EQ(battery.consume(5), 0U);
  EXPECT_EQ(battery.charge(70), 70U);
  EXPECT_EQ(battery.charge(70), 30U);  // saturates at capacity
  EXPECT_TRUE(battery.isFull());
  EXPECT_EQ(battery.charge(1), 0U);
}

TEST(BatteryTest, ExtremeAmountsCannotOverflow) {
  constexpr Battery::Units kMax = std::numeric_limits<Battery::Units>::max();
  Battery battery(kMax, 10);
  EXPECT_EQ(battery.charge(kMax), kMax - 10);
  EXPECT_TRUE(battery.isFull());
  EXPECT_EQ(battery.consume(kMax), kMax);
  EXPECT_TRUE(battery.isEmpty());
}

TEST(BatteryTest, FillAndEquality) {
  Battery a(100, 10);
  const Battery b(100, 10);
  EXPECT_TRUE(a == b);
  a.fill();
  EXPECT_TRUE(a.isFull());
  EXPECT_FALSE(a == b);
}

TEST(RobotBatteryTest, MovingCostsEnergyPerCellEntered) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginTask(T(1), P(5, 0), eastward(P(0, 0), 5));
  driveToDestination(robot);
  EXPECT_EQ(robot.battery().level(), 1000U - 5U * 10U);
  EXPECT_EQ(robot.cellsTravelled(), 5U);
}

TEST(RobotBatteryTest, WaitingWhileNavigatingCostsNothing) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginTask(T(1), P(2, 0), eastward(P(0, 0), 2));
  for (int i = 0; i < 20; ++i) {
    robot.elapseTick();
    EXPECT_EQ(robot.drive(), 0U);  // never granted its first cell: blocked every tick
  }
  EXPECT_EQ(robot.battery().level(), 1000U);
}

TEST(RobotBatteryTest, IdlingLoadingAndUnloadingDrainPerTick) {
  Robot idle(R(1), P(0, 0), energySpec());
  for (int i = 0; i < 10; ++i) {
    idle.elapseTick();
  }
  EXPECT_EQ(idle.battery().level(), 990U);  // 1 per idle tick

  Robot loading = robotIn(RobotState::Loading);
  const std::uint32_t atArrival = loading.battery().level();
  loading.elapseTick();
  EXPECT_EQ(loading.battery().level(), atArrival - 3U);  // 3 per handling tick
}

TEST(RobotBatteryTest, ChargingAddsTheChargeRateAndStopsAtCapacity) {
  RobotSpec spec = energySpec();
  spec.initialCharge = 150;
  Robot robot(R(1), P(0, 0), spec);
  robot.beginCharging();
  robot.elapseTick();
  EXPECT_EQ(robot.battery().level(), 250U);
  for (int i = 0; i < 20; ++i) {
    robot.elapseTick();
  }
  EXPECT_EQ(robot.battery().level(), 1000U);
  EXPECT_TRUE(robot.battery().isFull());
}

TEST(RobotBatteryTest, RunningOutSaturatesAtZero) {
  RobotSpec spec = energySpec();
  spec.initialCharge = 15;
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));
  driveToDestination(robot);
  EXPECT_EQ(robot.battery().level(), 0U);  // 15 - 10 - 5 (clamped) - 0
  EXPECT_TRUE(robot.battery().isEmpty());
  expectConsistent(robot);
}

TEST(RobotBatteryTest, SidesteppingCostsAsMuchAsAnyOtherMove) {
  Robot robot(R(1), P(1, 1), energySpec());
  robot.sidestep(P(1, 2));
  EXPECT_EQ(robot.battery().level(), 990U);
  EXPECT_EQ(robot.cellsTravelled(), 1U);
}

TEST(RobotBatteryTest, NeedsChargeIsStrictlyBelowTheReserve) {
  RobotSpec spec = energySpec();  // reserve 300
  const std::vector<std::pair<Battery::Units, bool>> cases = {
      {1000, false}, {301, false}, {300, false}, {299, true}, {1, true}, {0, true}};
  for (const auto& [level, expected] : cases) {
    spec.initialCharge = level;
    const Robot robot(R(1), P(0, 0), spec);
    EXPECT_EQ(robot.needsCharge(), expected);
    EXPECT_EQ(robot.isAvailable(), !expected);
  }
  RobotSpec noReserve;  // lowBatteryLevel == 0: never "low", even when empty
  noReserve.initialCharge = 0;
  EXPECT_FALSE(Robot(R(1), P(0, 0), noReserve).needsCharge());
  EXPECT_FALSE(Robot(R(1), P(0, 0), noReserve).isAvailable());  // but nothing left: no task either
  noReserve.initialCharge = 1;
  EXPECT_TRUE(Robot(R(1), P(0, 0), noReserve).isAvailable());
}

TEST(RobotBatteryTest, TotalDrainEqualsTheSumOfWhatTheRobotDid) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.elapseTick();                                    // idle: 1
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));  // 3 cells: 30
  driveToDestination(robot);
  robot.beginLoading();
  robot.elapseTick();                                // 3
  robot.elapseTick();                                // 3
  robot.finishLoading(P(3, 1), RobotPath{P(3, 1)});  // 1 cell: 10
  driveToDestination(robot);
  robot.beginUnloading();
  robot.elapseTick();  // 3
  robot.finishUnloading();
  EXPECT_EQ(robot.battery().level(), 1000U - (1 + 30 + 3 + 3 + 10 + 3));
}

// ---- motion ----------------------------------------------------------------------------

TEST(RobotMotionTest, FullSpeedEntersACellEveryTick) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginTask(T(1), P(4, 0), eastward(P(0, 0), 4));
  for (int cell = 1; cell <= 4; ++cell) {
    EXPECT_EQ(step(robot), 1U);
    EXPECT_EQ(robot.position(), P(cell, 0));
  }
}

TEST(RobotMotionTest, HalfSpeedEntersACellEverySecondTick) {
  RobotSpec spec = energySpec();
  spec.maxSpeed = 0.5;
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));
  std::vector<bool> entered;
  for (int tick = 0; tick < 6; ++tick) {
    entered.push_back(step(robot) > 0);
    EXPECT_DOUBLE_EQ(robot.speed(), robot.hasPath() ? 0.5 : 0.0);  // stops on the last arrival
  }
  EXPECT_EQ(entered, (std::vector<bool>{false, true, false, true, false, true}));
  EXPECT_EQ(robot.position(), P(3, 0));
}

TEST(RobotMotionTest, FractionalSpeedsKeepTheirRemainderAcrossCells) {
  RobotSpec spec = energySpec();
  spec.maxSpeed = 0.75;
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(30, 0), eastward(P(0, 0), 30));
  std::uint32_t cells = 0;
  for (int tick = 0; tick < 40; ++tick) {
    cells += step(robot);
  }
  EXPECT_EQ(cells, 30U);  // 40 ticks x 0.75 = 30 cells exactly, with no drift
}

TEST(RobotMotionTest, ThirdSpeedSumsToAWholeCellDespiteRounding) {
  RobotSpec spec = energySpec();
  spec.maxSpeed = 1.0 / 3.0;
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(2, 0), eastward(P(0, 0), 2));
  int ticks = 0;
  while (robot.hasPath()) {
    ++ticks;
    (void)step(robot);
  }
  EXPECT_EQ(ticks, 6);
}

TEST(RobotMotionTest, VelocityPointsTheWayTheRobotIsFacingScaledBySpeed) {
  RobotSpec spec = energySpec();
  spec.maxSpeed = 0.5;
  struct Case {
    RobotPath path;
    Velocity expected;
    Direction heading;
  };
  const std::vector<Case> cases = {
      {{P(2, 1)}, {0.0, -0.5}, Direction::North},
      {{P(3, 2)}, {0.5, 0.0}, Direction::East},
      {{P(2, 3)}, {0.0, 0.5}, Direction::South},
      {{P(1, 2)}, {-0.5, 0.0}, Direction::West},
  };
  for (const Case& c : cases) {
    Robot robot(R(1), P(2, 2), spec);
    robot.beginTask(T(1), c.path.back(), c.path);
    EXPECT_EQ(robot.velocity(), (Velocity{0.0, 0.0}));  // not moving yet
    EXPECT_EQ(step(robot), 0U);                         // half a cell on the way
    EXPECT_EQ(robot.heading(), c.heading);
    EXPECT_EQ(robot.velocity(), c.expected);
  }
}

TEST(RobotMotionTest, VelocityDropsToZeroWhenBlockedAndOnArrival) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));
  EXPECT_EQ(step(robot), 1U);
  EXPECT_DOUBLE_EQ(robot.speed(), 1.0);
  EXPECT_EQ(robot.drive(), 0U);  // the next cell is not granted: the robot waits
  EXPECT_DOUBLE_EQ(robot.speed(), 0.0);
  EXPECT_EQ(robot.velocity(), (Velocity{0.0, 0.0}));
  EXPECT_EQ(step(robot), 1U);  // granted again next tick
  EXPECT_DOUBLE_EQ(robot.speed(), 1.0);
  EXPECT_EQ(step(robot), 1U);
  EXPECT_EQ(robot.velocity(), (Velocity{0.0, 0.0}));  // arrived
}

// A robot only drives into cells it has been granted, so waiting for a cell covers no distance
// and a long wait does not buy a head start once the cell is free.
TEST(RobotMotionTest, ABlockedRobotDoesNotBankTravelItCouldNotUse) {
  RobotSpec spec = energySpec();
  spec.maxSpeed = 0.5;
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(2, 0), eastward(P(0, 0), 2));
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(robot.drive(), 0U);
  }
  EXPECT_DOUBLE_EQ(robot.segmentProgress(), 0.0);
  EXPECT_EQ(robot.position(), P(0, 0));
  EXPECT_EQ(step(robot), 0U);  // half speed: the first cell takes two ticks from a standing start
  EXPECT_EQ(step(robot), 1U);
  EXPECT_EQ(robot.position(), P(1, 0));
}

TEST(RobotMotionTest, ACellIsOnlyEnteredOnceItHasBeenCoveredInFull) {
  RobotSpec spec = energySpec();
  spec.maxSpeed = 0.5;
  Robot robot(R(1), P(0, 0), spec);
  robot.beginTask(T(1), P(1, 0), eastward(P(0, 0), 1));
  EXPECT_EQ(step(robot), 0U);
  EXPECT_EQ(robot.position(), P(0, 0));  // half way there is still the old cell
  EXPECT_DOUBLE_EQ(robot.segmentProgress(), 0.5);
  EXPECT_EQ(robot.battery().level(), 1000U);  // energy is charged for cells entered
  EXPECT_EQ(step(robot), 1U);
  EXPECT_EQ(robot.position(), P(1, 0));
  EXPECT_EQ(robot.battery().level(), 990U);
}

TEST(RobotMotionTest, BlockedTickCounterResetsWhenTheRobotMovesOrReroutes) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginTask(T(1), P(3, 0), eastward(P(0, 0), 3));
  robot.noteBlocked();
  robot.noteBlocked();
  EXPECT_EQ(robot.blockedTicks(), 2U);
  EXPECT_EQ(step(robot), 1U);
  EXPECT_EQ(robot.blockedTicks(), 0U);
  robot.noteBlocked();
  robot.setPath(RobotPath{P(1, 1), P(2, 1), P(3, 1), P(3, 0)});
  EXPECT_EQ(robot.blockedTicks(), 0U);
  robot.noteBlocked();
  robot.clearBlocked();
  EXPECT_EQ(robot.blockedTicks(), 0U);
}

TEST(RobotMotionTest, SidestepMovesOneCellFacesThatWayAndKeepsTheJob) {
  Robot robot(R(1), P(0, 0), energySpec());
  robot.beginTask(T(4), P(3, 0), eastward(P(0, 0), 3));
  robot.noteBlocked();
  robot.sidestep(P(0, 1));
  EXPECT_EQ(robot.position(), P(0, 1));
  EXPECT_EQ(robot.heading(), Direction::South);
  EXPECT_FALSE(robot.hasPath());  // the old route no longer applies
  EXPECT_EQ(robot.blockedTicks(), 0U);
  EXPECT_EQ(robot.state(), RobotState::Navigating);
  EXPECT_EQ(robot.task(), T(4));
  EXPECT_EQ(robot.destination(), P(3, 0));
  expectConsistent(robot);
  robot.setPath(RobotPath{P(1, 1), P(2, 1), P(3, 1), P(3, 0)});  // and it can be re-routed
  expectConsistent(robot);
}

TEST(RobotMotionTest, SidestepRequiresAnAdjacentCell) {
  Robot robot(R(1), P(2, 2), energySpec());
  EXPECT_THROW(robot.sidestep(P(4, 2)), std::invalid_argument);
  EXPECT_THROW(robot.sidestep(P(3, 3)), std::invalid_argument);
  EXPECT_THROW(robot.sidestep(P(2, 2)), std::invalid_argument);
  EXPECT_EQ(robot.position(), P(2, 2));
}

// ---- consistency under arbitrary use ----------------------------------------------------

// Hammers robots with random operations, valid and invalid, and checks after every single one that
// the robot is internally consistent, kept its id, never skipped the transition table, and kept its
// state-machine fields when the operation was rejected.
TEST(RobotConsistencyTest, RandomOperationSequencesNeverBreakTheInvariants) {
  Rng rng(20240607);
  std::set<RobotState> visited;
  int accepted = 0;
  int rejected = 0;

  for (int round = 0; round < 12; ++round) {
    RobotSpec spec = energySpec();
    spec.maxSpeed = round % 3 == 0 ? 1.0 : (round % 3 == 1 ? 0.5 : 0.75);
    spec.loadTicks = static_cast<std::uint32_t>(round % 4);
    spec.unloadTicks = static_cast<std::uint32_t>((round + 1) % 3);
    if (round % 4 == 3) {
      spec.initialCharge = 40;
    }
    const auto id = static_cast<std::uint32_t>(round + 1);
    Robot robot(R(id), P(5, 5), spec);

    // A straight east-then-south route to a random nearby destination.
    const auto route = [&rng](Point from) {
      const Point to =
          P(from.x + static_cast<int>(rng.below(4)), from.y + static_cast<int>(rng.below(3)));
      RobotPath path;
      Point at = from;
      while (at.x < to.x) {
        at = P(at.x + 1, at.y);
        path.push_back(at);
      }
      while (at.y < to.y) {
        at = P(at.x, at.y + 1);
        path.push_back(at);
      }
      return std::pair<Point, RobotPath>(to, path);
    };

    // The operation that would naturally come next; indexes the switch below.
    const auto naturalOp = [&rng](const Robot& r) -> std::uint64_t {
      switch (r.state()) {
        case RobotState::Idle: {
          constexpr std::uint64_t kIdleChoices[] = {0, 0, 0, 1, 6, 9};
          return kIdleChoices[rng.below(6)];
        }
        case RobotState::Navigating:
          if (r.hasPath()) {
            return 16;
          }
          return r.goal() == NavigationGoal::Pickup    ? 2
                 : r.goal() == NavigationGoal::Dropoff ? 4
                                                       : 6;
        case RobotState::Loading:
          return r.dwellRemaining() > 0 ? 17 : 3;
        case RobotState::Unloading:
          return r.dwellRemaining() > 0 ? 17 : 5;
        case RobotState::Charging:
          return r.battery().isFull() ? 7 : 17;
        case RobotState::Unavailable:
          return 10 + rng.below(2);
      }
      return 17;
    };

    for (int step = 0; step < 600; ++step) {
      const RobotState stateBefore = robot.state();
      const Observed before = observe(robot);
      bool threw = false;
      try {
        auto [to, path] = route(robot.position());
        if (rng.below(8) == 0 && !path.empty()) {
          path.pop_back();  // occasionally corrupt the route
        }
        // Mostly the natural next operation, so the walk gets deep into the state machine;
        // otherwise anything at all, which is usually illegal.
        const std::uint64_t op = rng.below(10) < 6 ? naturalOp(robot) : rng.below(20);
        switch (op) {
          case 0:
            robot.beginTask(T(static_cast<std::uint32_t>(rng.below(5))), to, path);
            break;
          case 1:
            robot.beginChargerTrip(to, path);
            break;
          case 2:
            robot.beginLoading();
            break;
          case 3:
            robot.finishLoading(to, path);
            break;
          case 4:
            robot.beginUnloading();
            break;
          case 5:
            robot.finishUnloading();
            break;
          case 6:
            robot.beginCharging();
            break;
          case 7:
            robot.stopCharging();
            break;
          case 8:
            robot.abortTask();
            break;
          case 9:
            if (rng.below(6) == 0) {
              robot.makeUnavailable();
            }
            break;
          case 10:
            robot.restore();
            break;
          case 11:
            robot.replaceBattery();
            break;
          case 12:
            robot.redirectChargerTrip(to, path);
            break;
          case 13:
            robot.setPath(path);
            break;
          case 14:
            robot.sidestep(
                P(robot.position().x + static_cast<int>(rng.below(3)) - 1, robot.position().y));
            break;
          case 15:
            robot.extendAuthority();
            break;
          case 16:
            (void)robot.drive();
            break;
          default:
            robot.elapseTick();
            break;
        }
      } catch (const InvalidTransition&) {
        threw = true;
      } catch (const std::invalid_argument&) {
        threw = true;
      }

      ++(threw ? rejected : accepted);
      if (threw) {
        // A rejected operation must leave the state machine untouched.
        const Observed after = observe(robot);
        EXPECT_EQ(after.state, before.state);
        EXPECT_EQ(after.goal, before.goal);
        EXPECT_EQ(after.task, before.task);
        EXPECT_EQ(after.destination, before.destination);
        EXPECT_EQ(after.position, before.position);
        EXPECT_EQ(after.battery, before.battery);
      }
      EXPECT_EQ(robot.inconsistency().value_or("consistent"), "consistent");
      EXPECT_EQ(robot.id(), R(id));
      EXPECT_TRUE(robot.battery().level() <= robot.battery().capacity());
      EXPECT_TRUE(robot.state() == stateBefore || isValidTransition(stateBefore, robot.state()));
      visited.insert(robot.state());
    }
  }
  EXPECT_EQ(visited.size(), kAllRobotStates.size());  // the walk really reached every state
  EXPECT_GT(accepted, 500);
  EXPECT_GT(rejected, 500);
}

}  // namespace
