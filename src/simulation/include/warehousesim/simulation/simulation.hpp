#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/path_planner.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/robots/robot_spec.hpp"
#include "warehousesim/robots/robot_state.hpp"
#include "warehousesim/tasks/dispatcher.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/tasks/task_generator.hpp"
#include "warehousesim/tasks/task_queue.hpp"
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
};

struct RobotSnapshot {
  core::RobotId id;
  core::Point position;
  core::Direction heading = core::Direction::North;
  robots::RobotState state = robots::RobotState::Idle;
  robots::NavigationGoal goal = robots::NavigationGoal::None;
  double speed = 0.0;  // cells per tick
  robots::Battery::Units batteryLevel = 0;
  robots::Battery::Units batteryCapacity = 1;
  std::optional<core::TaskId> task;

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
  // Returns false if the task had already completed, failed or been cancelled; throws
  // std::out_of_range for an unknown id.
  bool cancelTask(core::TaskId id);

  // Operator controls. Disabling takes a robot out of service (state Unavailable): a task it
  // had not collected yet goes back to Pending, a task it was carrying fails. Restoring returns
  // it to Idle, swapping in a fresh battery first if the old one is flat. Both return false when
  // the robot is already in the requested condition, and throw std::out_of_range for an
  // unknown id.
  bool disableRobot(core::RobotId id);
  bool restoreRobot(core::RobotId id);

  void step();
  void run(core::Tick ticks);

  [[nodiscard]] core::Tick tick() const noexcept { return clock_.tick(); }
  [[nodiscard]] const world::World& world() const noexcept { return world_; }
  [[nodiscard]] const world::Warehouse& warehouse() const noexcept { return world_.layout(); }
  [[nodiscard]] const std::vector<robots::Robot>& robots() const noexcept { return robots_; }
  [[nodiscard]] const tasks::TaskQueue& tasks() const noexcept { return tasks_; }
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
  void moveAlongPath(robots::Robot& robot);
  bool resolveBlock(robots::Robot& robot, core::Point cell);
  bool stepAside(robots::Robot& robot);
  [[nodiscard]] planning::BlockedSet occupiedCells() const;
  bool tryRepath(robots::Robot& robot, const planning::BlockedSet& blocked);
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
  std::unique_ptr<planning::PathPlanner> planner_;
  std::unique_ptr<tasks::Dispatcher> dispatcher_;
  std::optional<tasks::TaskGenerator> generator_;
  SimulationConfig config_;
  core::SimClock clock_;
};

}  // namespace warehousesim::simulation
