#pragma once

#include <cstdint>

#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::simulation {

// Demo warehouse with four robots and a new random task every 5 ticks.
[[nodiscard]] Simulation makeDemoSimulation(std::uint64_t seed);

}  // namespace warehousesim::simulation
