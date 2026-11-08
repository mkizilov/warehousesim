// Helpers shared by the recording, replay and runner-recording tests: simulations the recorder can
// take (default planner and dispatcher), and a recorded run that uses every kind of command.
#pragma once

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/metrics/fleet_metrics.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/recording.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/world/cell_type.hpp"
#include "warehousesim/world/layouts.hpp"

namespace warehousesim_test {

using warehousesim::simulation::CancelTask;
using warehousesim::simulation::ClearObstacle;
using warehousesim::simulation::Command;
using warehousesim::simulation::PlaceObstacle;
using warehousesim::simulation::Recorder;
using warehousesim::simulation::Recording;
using warehousesim::simulation::SetRobotService;
using warehousesim::simulation::SetTaskPriority;
using warehousesim::simulation::SubmitTask;
using warehousesim::world::CellType;

// Two idle robots and no random tasks: only what a test does to it changes it. (Not the Scenario of
// scenario_support.hpp, which has a dispatcher of its own and so cannot be recorded.)
inline Simulation quietSimulation(std::uint64_t seed = 1) {
  SimulationConfig config = ideal();
  config.taskSpawnInterval = 0;
  config.seed = seed;
  return Simulation(layout({"P.....D", ".......", "S.SSS.S", "C......"}), {P(0, 3), P(2, 3)},
                    config);
}

inline std::vector<std::string> bundledLayouts() {
  std::vector<std::string> names;
  for (const auto& info : warehousesim::world::predefinedLayouts()) {
    names.emplace_back(info.name);
  }
  return names;
}

// How many commands of each kind (by variant index) were accepted.
inline std::vector<std::size_t> acceptedByKind(const Recording& recording) {
  std::vector<std::size_t> counts(std::variant_size_v<Command>, 0);
  for (const auto& entry : recording.commands) {
    counts[entry.command.index()] += entry.accepted ? 1 : 0;
  }
  return counts;
}

// What a recorded run looked like from outside while it ran: the recording, and the state and the
// performance figures it ended with, measured by a collector that watched every step.
struct LiveRun {
  Recording recording;
  warehousesim::simulation::Snapshot snapshot;
  std::uint64_t checksum = 0;
  warehousesim::metrics::FleetMetrics metrics;
};

// All the figures of a run as text, for comparing two runs: the runs row, the robots and the
// series.
inline std::string metricsText(const warehousesim::metrics::FleetMetrics& metrics) {
  return warehousesim::metrics::runsCsvRow({"run", 0, 0}, metrics) + "\n" +
         warehousesim::metrics::robotsCsv(metrics) + "\n" +
         warehousesim::metrics::seriesCsv(metrics);
}

// A run of a bundled layout, with its own random tasks, in which commands of every kind (and two
// that are refused) are given at different ticks. Needs `ticks` of at least 80.
inline LiveRun busyRun(const std::string& layoutName, std::uint64_t seed,
                       warehousesim::core::Tick ticks,
                       warehousesim::core::Tick checkpointInterval = 50) {
  Recorder recorder(*warehousesim::simulation::makeLayoutSimulation(layoutName, seed),
                    checkpointInterval);
  warehousesim::simulation::MetricsCollector collector(recorder.simulation());
  const std::vector<Point> floor = recorder.simulation().warehouse().cellsOfType(CellType::Free);
  const std::vector<Point> dropoffs =
      recorder.simulation().warehouse().cellsOfType(CellType::Dropoff);
  const auto at = [&](warehousesim::core::Tick tick) {
    while (recorder.simulation().tick() < tick) {
      recorder.step();
      collector.observe(recorder.simulation());
    }
  };

  at(10);
  const auto first =
      recorder.apply(SubmitTask{floor.front(), dropoffs.front(), TaskPriority::High});
  const auto second = recorder.apply(SubmitTask{floor.back(), dropoffs.back(), TaskPriority::Low});
  (void)recorder.apply(SubmitTask{Point{-1, 0}, dropoffs.front()});  // refused: off the map
  at(20);
  (void)recorder.apply(SetTaskPriority{*first.task, TaskPriority::Urgent});
  (void)recorder.apply(CancelTask{*second.task});
  (void)recorder.apply(CancelTask{TaskId{100000}});  // refused: there is no such task
  at(30);
  (void)recorder.apply(SetRobotService{RobotId{1}, false});
  at(45);
  (void)recorder.apply(SetRobotService{RobotId{1}, true});
  at(50);
  std::optional<Point> wall;
  for (const Point cell : floor) {
    if (recorder.apply(PlaceObstacle{cell, CellType::Wall}).accepted) {
      wall = cell;
      break;
    }
  }
  EXPECT_TRUE(wall.has_value()) << layoutName << ": nowhere to put a wall";
  at(70);
  if (wall) {
    (void)recorder.apply(ClearObstacle{*wall});
  }
  at(ticks);
  return {recorder.recording(), recorder.simulation().snapshot(),
          recorder.simulation().stateChecksum(), collector.report()};
}

inline Recording busyRecording(const std::string& layoutName, std::uint64_t seed,
                               warehousesim::core::Tick ticks,
                               warehousesim::core::Tick checkpointInterval = 50) {
  return busyRun(layoutName, seed, ticks, checkpointInterval).recording;
}

}  // namespace warehousesim_test
