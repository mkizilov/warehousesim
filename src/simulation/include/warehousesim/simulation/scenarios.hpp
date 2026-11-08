#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::simulation {

// "demo" layout with four robots and a new random task every 5 ticks.
[[nodiscard]] Simulation makeDemoSimulation(std::uint64_t seed);

// Runs any predefined layout (see world::predefinedLayouts()) with the same task spawn rate.
// "demo" is makeDemoSimulation; other layouts start up to four robots on their charging
// stations. Returns nullopt for an unknown layout name.
[[nodiscard]] std::optional<Simulation> makeLayoutSimulation(std::string_view layoutName,
                                                             std::uint64_t seed);

}  // namespace warehousesim::simulation
