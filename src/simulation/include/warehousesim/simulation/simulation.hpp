#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/path_planner.hpp"
#include "warehousesim/robots/robot.hpp"
#include "warehousesim/tasks/assigner.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/tasks/task_generator.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::simulation {

struct SimulationConfig {
  std::uint64_t seed = 1;
  // Spawn one random task every N ticks (starting at tick 0); 0 disables spawning.
  core::Tick taskSpawnInterval = 0;
  // A robot blocked this many consecutive ticks makes an idle or lower-priority (higher id)
  // blocker step aside, or routes around a higher-priority one.
  std::uint32_t maxBlockedTicks = 3;
};

struct RobotSnapshot {
  core::RobotId id;
  core::Point position;
  robots::RobotStatus status = robots::RobotStatus::Idle;
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

  bool operator==(const Snapshot&) const = default;
};

// Single-threaded, deterministic engine. The same warehouse, robots, config and
// submitted tasks always produce the same state after the same number of steps.
class Simulation {
 public:
  // Robots get ids 1..N in the order of `robotStarts`. Throws std::invalid_argument if a
  // start cell is not walkable or two robots share a cell. Null planner/assigner select
  // AStarPlanner / NearestRobotAssigner.
  Simulation(world::Warehouse warehouse, const std::vector<core::Point>& robotStarts,
             SimulationConfig config = {}, std::unique_ptr<planning::PathPlanner> planner = nullptr,
             std::unique_ptr<tasks::TaskAssigner> assigner = nullptr);

  // Throws std::invalid_argument unless both locations are walkable.
  core::TaskId submitTask(core::Point pickup, core::Point dropoff);

  void step();
  void run(core::Tick ticks);

  [[nodiscard]] core::Tick tick() const noexcept { return clock_.tick(); }
  [[nodiscard]] const world::Warehouse& warehouse() const noexcept { return warehouse_; }
  [[nodiscard]] const std::vector<robots::Robot>& robots() const noexcept { return robots_; }
  [[nodiscard]] const tasks::TaskBoard& tasks() const noexcept { return tasks_; }
  [[nodiscard]] Snapshot snapshot() const;

 private:
  using Occupied = planning::BlockedSet;

  void spawnTask();
  void assignTasks();
  void moveRobots();
  void resolveArrival(robots::Robot& robot);
  bool resolveBlock(robots::Robot& robot, core::Point cell, Occupied& occupied);
  bool stepAside(robots::Robot& robot, Occupied& occupied);
  bool tryRepath(robots::Robot& robot, const Occupied& blocked);
  void failTask(robots::Robot& robot);

  world::Warehouse warehouse_;
  std::vector<robots::Robot> robots_;
  tasks::TaskBoard tasks_;
  std::unique_ptr<planning::PathPlanner> planner_;
  std::unique_ptr<tasks::TaskAssigner> assigner_;
  std::optional<tasks::TaskGenerator> generator_;
  SimulationConfig config_;
  core::SimClock clock_;
};

}  // namespace warehousesim::simulation
