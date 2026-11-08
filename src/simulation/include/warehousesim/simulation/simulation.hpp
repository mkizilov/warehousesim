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
  // A route asked for because of a reservation clash may be at most this many cells longer than
  // the one it replaces (0: only one of the same length); otherwise the robot keeps waiting.
  // Vehicles that stop at every corner lose more on a long detour than on a short wait: over 40
  // seeds of the demo hall, 4 cells or fewer completed about 2% more tasks than 12 to 20.
  std::size_t maxDetourCells = 4;
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
  // Nobody is in the way, but the reservation table says the next cell is taken for now and the
  // robot has waited long enough: ask the planner for a way round it. Returns true if it found one.
  bool requestReplan(robots::Robot& robot, const traffic::TrafficPlan& plan);
  // Gives back every claimed cell beyond what the robot has been granted.
  void syncHolds(robots::Robot& robot);
  bool resolveBlock(robots::Robot& robot, core::Point cell);
  bool stepAside(robots::Robot& robot);
  [[nodiscard]] planning::BlockedSet occupiedCells() const;
  bool tryRepath(robots::Robot& robot, const planning::BlockedSet& blocked,
                 std::optional<std::size_t> maxExtraCells = std::nullopt);
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
  std::uint64_t replans_ = 0;
  std::unique_ptr<planning::PathPlanner> planner_;
  std::unique_ptr<tasks::Dispatcher> dispatcher_;
  std::optional<tasks::TaskGenerator> generator_;
  SimulationConfig config_;
  core::SimClock clock_;
};

}  // namespace warehousesim::simulation
