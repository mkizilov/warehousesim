#include "warehousesim/simulation/scenarios.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "warehousesim/world/layouts.hpp"

namespace warehousesim::simulation {

namespace {

constexpr std::size_t kMaxRobots = 4;
constexpr core::Tick kTaskSpawnInterval = 5;

// Robots take two ticks to load and two to unload.
robots::RobotSpec handlingSpec() {
  robots::RobotSpec spec;
  spec.loadTicks = 2;
  spec.unloadTicks = 2;
  return spec;
}

// Adds an energy model on top of handlingSpec(). With a 10000-unit battery a robot can travel
// about 460 cells above its 30% reserve, then needs about 28 ticks on a charger. The reserve is
// larger than the energy of the longest task in the bundled layouts, so a robot that checks its
// battery between tasks always reaches a charger.
robots::RobotSpec energySpec() {
  robots::RobotSpec spec = handlingSpec();
  spec.idleDrainPerTick = 1;
  spec.moveCostPerCell = 15;
  spec.handlingDrainPerTick = 2;
  spec.chargePerTick = 250;
  spec.lowBatteryLevel = 3000;
  return spec;
}

SimulationConfig scenarioConfig(std::uint64_t seed, const robots::RobotSpec& robot) {
  SimulationConfig config;
  config.seed = seed;
  config.taskSpawnInterval = kTaskSpawnInterval;
  config.robot = robot;
  return config;
}

}  // namespace

Simulation makeDemoSimulation(std::uint64_t seed) {
  // The demo hall has no charging stations, so robots here have no battery model.
  return Simulation(world::makeDemoWarehouse(), {{0, 0}, {2, 0}, {4, 0}, {6, 0}},
                    scenarioConfig(seed, handlingSpec()));
}

std::optional<Simulation> makeLayoutSimulation(std::string_view layoutName, std::uint64_t seed) {
  if (layoutName == "demo") {
    return makeDemoSimulation(seed);
  }
  auto warehouse = world::makePredefinedWarehouse(layoutName);
  if (!warehouse) {
    return std::nullopt;
  }
  std::vector<core::Point> starts = warehouse->cellsOfType(world::CellType::Charging);
  starts.resize(std::min(starts.size(), kMaxRobots));
  return Simulation(*std::move(warehouse), starts, scenarioConfig(seed, energySpec()));
}

}  // namespace warehousesim::simulation
