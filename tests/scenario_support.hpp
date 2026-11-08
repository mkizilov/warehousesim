// Helpers shared by the simulation scenario tests: small layouts, a dispatcher that hands tasks to
// the robots a scenario names, a scenario runner, and an audit of what must hold on every tick.
#pragma once

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
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
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/tasks/dispatcher.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim_test {

using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::TaskId;
using warehousesim::robots::Robot;
using warehousesim::simulation::Simulation;
using warehousesim::simulation::SimulationConfig;
using warehousesim::tasks::TaskPriority;
using warehousesim::tasks::TaskStatus;
using warehousesim::world::Warehouse;

inline Point P(int x, int y) { return Point{x, y}; }
inline RobotId R(std::uint32_t id) { return RobotId{id}; }

inline Warehouse layout(const std::vector<std::string>& rows) {
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("bad test layout");
  }
  return *warehouse;
}

inline Warehouse openFloor(int width, int height) {
  return layout(std::vector<std::string>(static_cast<std::size_t>(height),
                                         std::string(static_cast<std::size_t>(width), '.')));
}

// Two 3x3 rooms joined by a one-cell-wide corridor (row 1, x = 3..11) with a niche above its
// middle.
//
//   . . . # # # # . # # # # . . .
//   . . . . . . . . . . . . . . .
//   . . . # # # # # # # # # . . .
inline Warehouse hall() {
  return layout({"...####.####...", "...............", "...#########..."});
}

// Idealised robots: one cell per tick, no acceleration, no turning time, no handling time.
inline SimulationConfig ideal() {
  SimulationConfig config;
  config.maxBlockedTicks = 3;
  return config;
}

// Vehicles: they accelerate, brake, stop to turn at corners, and take time to load and unload.
inline SimulationConfig vehicles() {
  SimulationConfig config = ideal();
  config.robot.acceleration = 0.5;
  config.robot.deceleration = 1.0;
  config.robot.turnRate = warehousesim::core::kPi / 2.0;
  config.robot.loadTicks = 2;
  config.robot.unloadTicks = 2;
  return config;
}

// Hands every task to the robot named for it, as soon as that robot is free. The default dispatcher
// would give each task to the nearest robot, which is not always the one a scenario needs.
class ScriptedDispatcher final : public warehousesim::tasks::Dispatcher {
 public:
  explicit ScriptedDispatcher(std::vector<std::pair<TaskId, RobotId>> script)
      : script_(std::move(script)) {}

  [[nodiscard]] warehousesim::tasks::DispatchPlan plan(
      const Warehouse&, std::span<const Robot>,
      const warehousesim::tasks::TaskQueue& queue) override {
    warehousesim::tasks::DispatchPlan plan;
    for (const auto& [task, robot] : script_) {
      const warehousesim::tasks::Task* found = queue.find(task);
      if (found != nullptr && found->status == TaskStatus::Pending) {
        plan.assignments.push_back(warehousesim::tasks::Assignment{robot, task, 0});
      }
    }
    return plan;
  }

 private:
  std::vector<std::pair<TaskId, RobotId>> script_;
};

// One robot with one task.
struct Job {
  Point start;
  Point pickup;
  Point dropoff;
};

// A task for the robot with the given index (0 is robot 1). A robot works through its errands in
// the order they are listed.
struct Errand {
  std::size_t robot;
  Point pickup;
  Point dropoff;
  TaskPriority priority = TaskPriority::Normal;
};

// Robots, the tasks they are to do, and the simulation that runs them.
struct Scenario {
  Simulation sim;
  std::vector<TaskId> tasks;  // in the order the errands were listed

  Scenario(Warehouse warehouse, const std::vector<Job>& jobs, SimulationConfig config)
      : sim(makeSim(std::move(warehouse), startsOf(jobs), errandsOf(jobs), config)) {
    submit(errandsOf(jobs));
  }

  Scenario(Warehouse warehouse, const std::vector<Point>& starts,
           const std::vector<Errand>& errands, SimulationConfig config)
      : sim(makeSim(std::move(warehouse), starts, errands, config)) {
    submit(errands);
  }

 private:
  static std::vector<Point> startsOf(const std::vector<Job>& jobs) {
    std::vector<Point> starts;
    for (const Job& job : jobs) {
      starts.push_back(job.start);
    }
    return starts;
  }
  static std::vector<Errand> errandsOf(const std::vector<Job>& jobs) {
    std::vector<Errand> errands;
    for (std::size_t i = 0; i < jobs.size(); ++i) {
      errands.push_back({i, jobs[i].pickup, jobs[i].dropoff});
    }
    return errands;
  }
  static Simulation makeSim(Warehouse warehouse, const std::vector<Point>& starts,
                            const std::vector<Errand>& errands, SimulationConfig config) {
    std::vector<std::pair<TaskId, RobotId>> script;
    for (std::size_t i = 0; i < errands.size(); ++i) {
      script.emplace_back(TaskId{static_cast<std::uint32_t>(i + 1)},
                          RobotId{static_cast<std::uint32_t>(errands[i].robot + 1)});
    }
    return Simulation(std::move(warehouse), starts, config, nullptr,
                      std::make_unique<ScriptedDispatcher>(std::move(script)));
  }
  void submit(const std::vector<Errand>& errands) {
    for (const Errand& errand : errands) {
      tasks.push_back(sim.submitTask(errand.pickup, errand.dropoff, errand.priority));
    }
  }
};

// What must hold on every tick, whatever the robots are up to: no cell held twice, no robot off the
// floor, no two robots overlapping or swapping cells, nobody moving faster than a cell per tick,
// and no task lost: a task a robot holds is that robot's, and a robot's task is held by that robot.
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
    for (const warehousesim::tasks::Task& task : sim.tasks().all()) {
      if (task.isAssigned()) {
        ASSERT_TRUE(task.assignee.has_value())
            << "task " << task.id.value << ", tick " << sim.tick();
        EXPECT_EQ(robots[task.assignee->value - 1].task(), std::optional<TaskId>(task.id))
            << "task " << task.id.value << " is held by nobody, tick " << sim.tick();
      }
    }
    for (const Robot& robot : robots) {
      if (robot.task()) {
        const warehousesim::tasks::Task* task = sim.tasks().find(*robot.task());
        EXPECT_TRUE(task != nullptr && task->isAssigned() && task->assignee == robot.id())
            << "robot " << robot.id().value << " works on a task it does not hold, tick "
            << sim.tick();
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
inline std::vector<std::uint64_t> runToCompletion(
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
inline std::uint64_t soloTime(const Warehouse& warehouse, const std::vector<Job>& jobs,
                              std::size_t which, SimulationConfig config) {
  Scenario alone(warehouse, {jobs[which]}, config);
  return runToCompletion(alone, 400)[0];
}

}  // namespace warehousesim_test
