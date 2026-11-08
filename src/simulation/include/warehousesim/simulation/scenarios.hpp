#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::simulation {

// The most robots a scenario will start, however much room the layout has.
inline constexpr std::size_t kMaxScenarioRobots = 64;

// What can be changed about a scenario without changing its layout or its robot model.
struct ScenarioOptions {
  // The number of robots. Unset: the layout's usual fleet (up to four, see makeMapSimulation).
  // Set: exactly this many, on the charging stations first and then on open floor spaced at least
  // two cells apart, in reading order. Throws std::invalid_argument if it is 0 or more than
  // kMaxScenarioRobots, or if the layout has no room for so many.
  std::optional<std::size_t> robots;
  // A new random task every this many ticks. Unset: every 5 ticks. 0 turns random tasks off.
  std::optional<core::Tick> taskSpawnInterval;
};

// Any layout with up to four robots and a new random task (of random priority) every 5 ticks.
// Robots spend two ticks loading and two unloading. A layout with charging stations starts its
// robots on them (up to four), with a battery model: they recharge when low. One without starts
// them on open floor, spaced out, with no battery model. Throws std::invalid_argument, naming the
// reason, if the layout cannot host a simulation (see world::checkLayout: no walkable cell, no
// dropoff, or nothing to collect); layouts that merely have unreachable parts are accepted.
[[nodiscard]] Simulation makeMapSimulation(world::Warehouse warehouse, std::uint64_t seed);
// The same with the fleet size and the task rate chosen. ScenarioOptions{} gives exactly the above.
[[nodiscard]] Simulation makeMapSimulation(world::Warehouse warehouse, std::uint64_t seed,
                                           const ScenarioOptions& options);

// Builds the same scenario, from the start, for any seed: what "reset" needs. Everything about the
// scenario but the seed is fixed when the factory is made, so factory(seed) is always the same
// simulation for the same seed.
using ScenarioFactory = std::function<Simulation(std::uint64_t seed)>;

// The factory for makeMapSimulation of this layout. Throws std::invalid_argument, like
// makeMapSimulation does, if the layout cannot host a simulation.
[[nodiscard]] ScenarioFactory mapScenario(world::Warehouse warehouse);
[[nodiscard]] ScenarioFactory mapScenario(world::Warehouse warehouse,
                                          const ScenarioOptions& options);
// The factory for a predefined layout, or nullopt for an unknown name.
[[nodiscard]] std::optional<ScenarioFactory> layoutScenario(std::string_view layoutName);
[[nodiscard]] std::optional<ScenarioFactory> layoutScenario(std::string_view layoutName,
                                                            const ScenarioOptions& options);

// "demo" layout: makeMapSimulation of the demo hall (robots on its top row, no battery model).
[[nodiscard]] Simulation makeDemoSimulation(std::uint64_t seed);

// Runs any predefined layout (see world::predefinedLayouts()) the same way. Returns nullopt for an
// unknown layout name.
[[nodiscard]] std::optional<Simulation> makeLayoutSimulation(std::string_view layoutName,
                                                             std::uint64_t seed);

}  // namespace warehousesim::simulation
