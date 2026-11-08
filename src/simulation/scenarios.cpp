#include "warehousesim/simulation/scenarios.hpp"

#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::simulation {

Simulation makeDemoSimulation(std::uint64_t seed) {
  SimulationConfig config;
  config.seed = seed;
  config.taskSpawnInterval = 5;
  return Simulation(world::makeDemoWarehouse(), {{0, 0}, {2, 0}, {4, 0}, {6, 0}}, config);
}

}  // namespace warehousesim::simulation
