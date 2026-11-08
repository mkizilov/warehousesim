#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/vec2.hpp"
#include "warehousesim/planning/path_planner.hpp"
#include "warehousesim/robots/motion.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/robots/robot_spec.hpp"
#include "warehousesim/robots/robot_state.hpp"
#include "warehousesim/tasks/dispatcher.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/tasks/task_generator.hpp"
#include "warehousesim/tasks/task_queue.hpp"
#include "warehousesim/traffic/cooperative_router.hpp"
#include "warehousesim/traffic/progress_monitor.hpp"
#include "warehousesim/traffic/reservation_table.hpp"
#include "warehousesim/traffic/traffic_controller.hpp"
#include "warehousesim/world/world.hpp"

namespace warehousesim::simulation {

struct SimulationConfig {
  std::uint64_t seed = 1;
  // Spawn one random task every N ticks (starting at tick 0); 0 disables spawning.
  core::Tick taskSpawnInterval = 0;
  // Give spawned tasks random priorities. Off: every spawned task is Normal.
  bool varyTaskPriority = false;
  // A robot blocked this many consecutive ticks makes an idle or lower-priority (higher id)
  // blocker step aside, or routes around a higher-priority one. Robots that are busy in place
  // (loading, unloading, charging, unavailable) never step aside.
  std::uint32_t maxBlockedTicks = 3;
  // The model every robot is built from. The default is an idealised robot (see RobotSpec). If
  // the layout has no charging station, lowBatteryLevel is ignored (set to 0) because a robot
  // could never recharge.
  robots::RobotSpec robot;
  // Collision avoidance by space-time reservation (see TrafficController). Each step robots reserve
  // the cells they will hold over the next `reservationHorizon` steps, in ascending id order, and a
  // robot whose route clashes with a reservation of a lower id waits before the clash and, if the
  // wait drags on, asks for a different route. Off, robots only avoid each other by being unable
  // to claim a cell somebody holds (still collision-free, but they meet head-on instead of
  // seeing it coming). The deadlock-breaking rules (a blocker steps aside, face-to-face robots give
  // way) apply either way.
  bool reservations = true;
  std::size_t reservationHorizon = traffic::TrafficController::kDefaultHorizon;
  // A route found because of a reservation clash may be at most this many cells longer than the
  // one it replaces when that one could still be followed (0: only one of the same length);
  // otherwise the robot keeps waiting. Vehicles that stop at every corner lose more on a long
  // detour than on a short wait: over 40 seeds of the demo hall, 4 cells or fewer completed about
  // 2% more tasks than 12 to 20.
  std::size_t maxDetourCells = 4;

  // ---- cooperative planning (traffic::CooperativeRouter; needs `reservations`) ----
  // A robot whose route the table holds back searches in space and time, in priority order,
  // against the reservations of the robots before it, and takes the result if it gets there sooner
  // than waiting would. This also lets a robot that is in somebody's way dodge, which the table
  // alone cannot.
  bool cooperative = true;
  // How many cells longer a route may be when the old one could not be followed at all because the
  // robot is in somebody's way (a dodge into a side cell and back costs two or more).
  std::size_t maxDodgeCells = 12;
  // Search work (see planning::SpaceTimeOptions::maxWork) all robots may spend in one step, and
  // that one search may spend. When the step's share is used up the remaining searches wait for
  // the next step, where the robots that were put off go first.
  std::size_t planningWorkPerStep = 4000;
  std::size_t planningWorkPerSearch = 1500;
  // Ticks a robot waits before searching again after a search that did not change its route.
  core::Tick replanInterval = 4;
  // A robot that wants to move and does not for this many ticks is blocked for long: it is counted
  // and its searches may use twice the work.
  std::uint32_t prolongedBlockTicks = 20;
  // Robots waiting for each other in a ring, all stuck for this long, are deadlocked. The first
  // time round, the one with the lowest priority that has room is moved aside. If the ring is still
  // there after as many ticks again, and again, the lowest-priority robot in it that has not had
  // its turn is given the first place in the order of priority for priorityBoostTicks, so that the
  // others have to make way for it.
  std::uint32_t deadlockTicks = 6;
  core::Tick priorityBoostTicks = 40;
  // A robot stuck for this long gives up its task (back to the queue if not yet collected, failed
  // if it carries cargo) so that it does not hold up the others for ever.
  std::uint32_t abandonTicks = 150;
};

// What cooperative planning, blocking detection and recovery have done so far.
struct PlanningStats {
  std::uint64_t searches = 0;         // cooperative searches run
  std::uint64_t reroutes = 0;         // routes replaced by one of them
  std::uint64_t dodges = 0;           // ... of which the robot was in somebody's way
  std::uint64_t partialPlans = 0;     // ... of which only reached the end of the window
  std::uint64_t outOfBudget = 0;      // searches that ran out of their work
  std::uint64_t deferred = 0;         // searches put off because the step's work was used up
  std::uint64_t work = 0;             // cooperative search work in total
  std::uint64_t peakStepWork = 0;     // most planning work, of any kind, in one step
  std::uint64_t obstacleRepairs = 0;  // routes repaired after the layout changed
  std::uint64_t prolongedBlocks = 0;  // robots stuck for prolongedBlockTicks
  std::uint64_t deadlocks = 0;        // rings of robots waiting for each other, each counted once
  std::uint64_t deadlocksRecovered = 0;   // ... that a robot was moved aside for
  std::uint64_t deadlocksUnresolved = 0;  // ... that nobody had room to move out of
  std::uint64_t priorityBoosts = 0;       // robots put first in the order of priority to end a ring
  std::uint64_t abandoned = 0;            // tasks given up after abandonTicks

  bool operator==(const PlanningStats&) const = default;
};

// What a robot that has been held back is waiting for.
struct RobotWait {
  core::Point cell;  // the cell it may not enter yet
  traffic::ConflictKind kind = traffic::ConflictKind::Vertex;
  std::optional<core::RobotId> other;  // the robot in its way, if any

  bool operator==(const RobotWait&) const = default;
};

// Where a robot really is, as opposed to the cell it is logically in.
struct RobotPose {
  core::Vec2 center;     // world space: cell (x, y) has its centre at (x + 0.5, y + 0.5)
  double heading = 0.0;  // radians: east 0, south pi/2, west pi, north -pi/2

  bool operator==(const RobotPose&) const = default;
};

struct RobotSnapshot {
  core::RobotId id;
  // The logical cell: what occupancy, planning and tasks see.
  core::Point position;
  // The continuous pose, between cells while the robot drives. Absent means at rest in the
  // centre of `position`, facing `heading`.
  std::optional<RobotPose> pose;
  core::Direction heading = core::Direction::North;
  robots::RobotState state = robots::RobotState::Idle;
  robots::NavigationGoal goal = robots::NavigationGoal::None;
  double speed = 0.0;  // cells per tick
  robots::Battery::Units batteryLevel = 0;
  robots::Battery::Units batteryCapacity = 1;
  std::optional<core::TaskId> task;
  // The rest of the planned route, nearest cell first, and the cells the robot holds (its own cell
  // first). The picture draws them as the planned path and the occupied cells.
  std::vector<core::Point> path;
  std::vector<core::Point> holds;
  // Set while the robot is standing still because a reservation or a robot is in its way.
  std::optional<RobotWait> waiting;
  // Ticks in a row it has wanted to move and not moved.
  std::uint32_t stuckTicks = 0;
  robots::MotionPhase motion = robots::MotionPhase::Stopped;
  bool carrying = false;         // holds cargo between loading and unloading
  std::uint64_t deliveries = 0;  // cargo handed over at dropoffs so far

  // Where to draw the robot: its continuous pose, or the centre of its cell if it has none.
  [[nodiscard]] core::Vec2 center() const noexcept {
    return pose ? pose->center
                : core::Vec2{static_cast<double>(position.x) + 0.5,
                             static_cast<double>(position.y) + 0.5};
  }
  [[nodiscard]] double headingAngle() const noexcept {
    return pose ? pose->heading : core::angleOf(heading);
  }

  bool operator==(const RobotSnapshot&) const = default;
};

// Value copy of the simulation state, safe to hand to another thread. Terminal
// tasks are reduced to counters so the size stays bounded on long runs.
struct Snapshot {
  core::Tick tick = 0;
  std::vector<RobotSnapshot> robots;
  std::vector<tasks::Task> activeTasks;
  std::size_t completedTasks = 0;
  std::size_t failedTasks = 0;
  std::size_t cancelledTasks = 0;
  // Running totals of what collision avoidance has done: clashes found while planning, by kind,
  // and routes asked for because a robot was held back.
  traffic::TrafficStats traffic;
  std::uint64_t replans = 0;
  PlanningStats planning;

  bool operator==(const Snapshot&) const = default;
};

// Single-threaded, deterministic engine. The same warehouse, robots, config and
// submitted tasks always produce the same state after the same number of steps.
class Simulation {
 public:
  // Robots get ids 1..N in the order of `robotStarts`. Throws std::invalid_argument if a
  // start cell is not walkable or two robots share a cell. A null planner or dispatcher selects
  // AStarPlanner or GreedyDispatcher (with path-distance estimates).
  Simulation(world::Warehouse warehouse, const std::vector<core::Point>& robotStarts,
             SimulationConfig config = {}, std::unique_ptr<planning::PathPlanner> planner = nullptr,
             std::unique_ptr<tasks::Dispatcher> dispatcher = nullptr);

  // Adds a task to the queue. Throws std::invalid_argument unless both locations are walkable.
  // A task whose route cannot be driven is not refused here; the dispatcher rejects it (it ends
  // up Failed) the next time it is considered.
  core::TaskId submitTask(core::Point pickup, core::Point dropoff,
                          tasks::TaskPriority priority = tasks::TaskPriority::Normal);
  // Withdraws a task. A robot working on it stops and becomes idle (cargo it carried is dropped).
  // A robot that is between two cells at that moment does not stop dead: it brakes along the cells
  // it already holds and stops at the first waypoint it can reach, so it is Idle straight away
  // but not yet available. Returns false if the task had already completed, failed or been
  // cancelled; throws std::out_of_range for an unknown id.
  bool cancelTask(core::TaskId id);

  // Operator controls. Disabling takes a robot out of service (state Unavailable): a task it
  // had not collected yet goes back to Pending, a task it was carrying fails. Restoring returns
  // it to Idle, swapping in a fresh battery first if the old one is flat. Both return false when
  // the robot is already in the requested condition, and throw std::out_of_range for an
  // unknown id. A robot taken out of service between two cells coasts to a stop like above.
  bool disableRobot(core::RobotId id);
  bool restoreRobot(core::RobotId id);

  // Puts up a wall or a shelf on a free floor cell, or takes one down, while the simulation runs.
  // Refused (false) for a cell that is not a plain floor cell (or, for clearObstacle, not an
  // obstacle), and for a cell a robot stands on or holds. On success the table and distance
  // caches are rebuilt, open tasks that start or end on the cell are cancelled, and the route of
  // every robot that went through it is repaired: through the shortest way round, or, if there is
  // none, the robot's task goes back to the queue (not yet collected) or fails (cargo aboard).
  // Throws std::invalid_argument if `type` is not an obstacle.
  bool placeObstacle(core::Point cell, world::CellType type = world::CellType::Wall);
  bool clearObstacle(core::Point cell);

  void step();
  void run(core::Tick ticks);

  [[nodiscard]] core::Tick tick() const noexcept { return clock_.tick(); }
  [[nodiscard]] const world::World& world() const noexcept { return world_; }
  [[nodiscard]] const world::Warehouse& warehouse() const noexcept { return world_.layout(); }
  [[nodiscard]] const std::vector<robots::Robot>& robots() const noexcept { return robots_; }
  [[nodiscard]] const tasks::TaskQueue& tasks() const noexcept { return tasks_; }
  // The reservations of the step just run, how often planning found each kind of clash, and how
  // many times a robot asked for a new route because of one.
  [[nodiscard]] const traffic::ReservationTable& reservations() const noexcept {
    return traffic_.table();
  }
  [[nodiscard]] const traffic::TrafficStats& trafficStats() const noexcept {
    return traffic_.stats();
  }
  [[nodiscard]] std::uint64_t replans() const noexcept { return replans_; }
  [[nodiscard]] const PlanningStats& planningStats() const noexcept { return planningStats_; }
  // What the table decided for a robot in the step just run.
  [[nodiscard]] const traffic::TrafficPlan& trafficPlan(core::RobotId id) const;
  [[nodiscard]] Snapshot snapshot() const;

 private:
  void spawnTask();
  void assignTasks();
  void updateRobots();
  void updateRobot(robots::Robot& robot);
  void settle(robots::Robot& robot);
  void arrive(robots::Robot& robot);
  void finishLoading(robots::Robot& robot);
  bool dispatchToCharger(robots::Robot& robot);
  [[nodiscard]] std::optional<std::pair<core::Point, robots::RobotPath>> nearestFreeCharger(
      const robots::Robot& robot) const;
  void breakDownIfDepleted(robots::Robot& robot);
  void moveAlongPath(robots::Robot& robot, const traffic::TrafficPlan& plan);
  [[nodiscard]] std::optional<RobotWait> waitOf(const robots::Robot& robot) const;
  // Claims in the world, in path order, the cells the robot wants to be granted for driving
  // `timeToDrive` ticks and tells the robot about each; stops at the first cell somebody else
  // holds, or after `cleared` cells of the path (what the reservation table lets the robot have).
  void claimAhead(robots::Robot& robot, double timeToDrive, std::size_t cleared);
  // Gives back every claimed cell beyond what the robot has been granted.
  void syncHolds(robots::Robot& robot);
  bool resolveBlock(robots::Robot& robot, core::Point cell);
  // `a` has priority over `b`: a boosted robot over one that is not, otherwise the lower id.
  [[nodiscard]] bool outranks(core::RobotId a, core::RobotId b) const;
  // Moves the robot to a free neighbouring cell, one that `pusher`'s route does not need if there
  // is one. False if there is none, or the robot has been moved this step already.
  bool stepAside(robots::Robot& robot, const robots::Robot* pusher = nullptr);
  [[nodiscard]] planning::BlockedSet occupiedCells() const;
  bool tryRepath(robots::Robot& robot, const planning::BlockedSet& blocked);

  // ---- cooperative planning, supervision and recovery (cooperation.cpp) ----
  void beginPlanning();
  // The robot has just been planned by the table. If that held it back, searches for a better
  // route within this step's work and returns the plan to follow.
  traffic::TrafficPlan cooperate(robots::Robot& robot, const traffic::TrafficPlan& plan);
  [[nodiscard]] planning::BlockedSet stationaryCells(core::RobotId except) const;
  void recordProgress(const robots::Robot& robot, const traffic::TrafficPlan& plan);
  // After all robots have moved: stuck robots, deadlocks, giving up.
  void supervise();
  bool recoverDeadlock(const std::vector<core::RobotId>& ring);
  void boostOneOf(const std::vector<core::RobotId>& ring);
  void abandon(robots::Robot& robot);
  void repairRoutes();
  void rebuildTraffic();
  bool retargetCharger(robots::Robot& robot);
  void releaseTask(robots::Robot& robot);
  void failTask(robots::Robot& robot);
  void abortNavigation(robots::Robot& robot);
  [[nodiscard]] robots::Robot& robotById(core::RobotId id);

  // Kept in lock-step with the robots' own positions: every move goes through world_.
  world::World world_;
  std::vector<robots::Robot> robots_;
  std::vector<core::Point> chargers_;  // charging cells, row-major
  tasks::TaskQueue tasks_;
  traffic::TrafficController traffic_;
  std::vector<traffic::TrafficPlan> plans_;  // by robot, from the step just run
  std::vector<bool> displaced_;              // by robot: moved aside during the current step
  std::vector<bool> movedThisStep_;          // by robot: changed cell or was driving this step
  std::uint64_t replans_ = 0;
  traffic::CooperativeRouter router_;
  traffic::ProgressMonitor progress_{0};
  PlanningStats planningStats_;
  // By robot (index id - 1): when it may search again, how many steps in a row it was put off,
  // the factor on its search work after running out of it, whether it holds a reserved share of
  // this step's work, and whether its current deadlock ring has been counted.
  std::vector<core::Tick> nextSearch_;
  std::vector<core::Tick> boostUntil_;  // by robot: has first priority until this tick
  std::vector<std::uint32_t> deferred_;
  std::vector<std::uint32_t> workScale_;
  std::vector<bool> reservedShare_;
  std::vector<std::vector<core::RobotId>> ringsLastStep_;
  std::size_t workPool_ = 0;       // search work left for robots without a reserved share
  std::uint64_t stepWork_ = 0;     // planning work so far in the current step
  std::uint64_t plannerBase_ = 0;  // the planner's work total when the step began
  std::unique_ptr<planning::PathPlanner> planner_;
  std::unique_ptr<tasks::Dispatcher> dispatcher_;
  std::optional<tasks::TaskGenerator> generator_;
  SimulationConfig config_;
  core::SimClock clock_;
};

}  // namespace warehousesim::simulation
