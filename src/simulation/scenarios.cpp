#include "warehousesim/simulation/scenarios.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "warehousesim/world/layouts.hpp"

namespace warehousesim::simulation {

namespace {

constexpr std::size_t kMaxRobots = 4;
constexpr core::Tick kTaskSpawnInterval = 5;

SimulationConfig scenarioConfig(std::uint64_t seed) {
  SimulationConfig config;
  config.seed = seed;
  config.taskSpawnInterval = kTaskSpawnInterval;
  return config;
}

}  // namespace

Simulation makeDemoSimulation(std::uint64_t seed) {
  return Simulation(world::makeDemoWarehouse(), {{0, 0}, {2, 0}, {4, 0}, {6, 0}},
                    scenarioConfig(seed));
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
  return Simulation(*std::move(warehouse), starts, scenarioConfig(seed));
}

}  // namespace warehousesim::simulation
