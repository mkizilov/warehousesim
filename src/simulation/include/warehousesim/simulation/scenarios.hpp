#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::simulation {

// Any layout with up to four robots and a new random task (of random priority) every 5 ticks.
// Robots spend two ticks loading and two unloading. A layout with charging stations starts its
// robots on them (up to four), with a battery model: they recharge when low. One without starts
// them on open floor, spaced out, with no battery model. Throws std::invalid_argument, naming the
// reason, if the layout cannot host a simulation (see world::checkLayout: no walkable cell, no
// dropoff, or nothing to collect); layouts that merely have unreachable parts are accepted.
[[nodiscard]] Simulation makeMapSimulation(world::Warehouse warehouse, std::uint64_t seed);

// "demo" layout: makeMapSimulation of the demo hall (robots on its top row, no battery model).
[[nodiscard]] Simulation makeDemoSimulation(std::uint64_t seed);

// Runs any predefined layout (see world::predefinedLayouts()) the same way. Returns nullopt for an
// unknown layout name.
[[nodiscard]] std::optional<Simulation> makeLayoutSimulation(std::string_view layoutName,
                                                             std::uint64_t seed);

}  // namespace warehousesim::simulation
