#pragma once

#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/simulation/scenarios.hpp"

namespace warehousesim::apps {

// Where the simulated warehouse comes from: a predefined layout by name, or a map file made with
// the editor. At most one of the two may be given; neither means the "demo" layout. `robots` and
// `spawn` choose the size of the fleet and the ticks between random tasks (see ScenarioOptions);
// unset, the scenario is the usual one.
struct WarehouseSource {
  std::string layout;
  std::string mapFile;
  std::optional<std::uint64_t> robots;
  std::optional<std::uint64_t> spawn;

  [[nodiscard]] bool conflicting() const noexcept { return !layout.empty() && !mapFile.empty(); }
  [[nodiscard]] simulation::ScenarioOptions options() const {
    simulation::ScenarioOptions result;
    result.robots = robots;
    result.taskSpawnInterval = spawn;
    return result;
  }
};

struct ScenarioResult {
  std::optional<simulation::ScenarioFactory> scenario;
  bool unknownLayout = false;  // the caller shows its usage text and exits with 2
};

// The scenario to run: a function from a seed to a fresh simulation of the chosen warehouse. A map
// file that is missing, invalid or unable to host a simulation is reported on `err` (with every
// problem found in it) and gives an empty result.
[[nodiscard]] inline ScenarioResult makeScenario(const WarehouseSource& source, std::ostream& err) {
  try {
    if (source.mapFile.empty()) {
      auto scenario = simulation::layoutScenario(
          source.layout.empty() ? std::string_view("demo") : std::string_view(source.layout),
          source.options());
      const bool unknown = !scenario.has_value();
      return {std::move(scenario), unknown};
    }
    const mapio::MapLoad load = mapio::loadMapFile(source.mapFile);
    if (!load.ok()) {
      err << "error: " << source.mapFile << " is not a valid map:\n" << load.describe() << '\n';
      return {};
    }
    return {simulation::mapScenario(load.document->layout, source.options()), false};
  } catch (const std::invalid_argument& error) {
    err << "error: " << (source.mapFile.empty() ? source.layout : source.mapFile) << ": "
        << error.what() << '\n';
    return {};
  }
}

struct SourceResult {
  std::optional<simulation::Simulation> simulation;
  bool unknownLayout = false;
};

// Builds the simulation of makeScenario's scenario for one seed.
[[nodiscard]] inline SourceResult makeSimulation(const WarehouseSource& source, std::uint64_t seed,
                                                 std::ostream& err) {
  ScenarioResult made = makeScenario(source, err);
  if (!made.scenario) {
    return {std::nullopt, made.unknownLayout};
  }
  return {(*made.scenario)(seed), false};
}

}  // namespace warehousesim::apps
