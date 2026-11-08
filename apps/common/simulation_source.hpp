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
// the editor. At most one of the two may be given; neither means the "demo" layout.
struct WarehouseSource {
  std::string layout;
  std::string mapFile;

  [[nodiscard]] bool conflicting() const noexcept { return !layout.empty() && !mapFile.empty(); }
};

struct SourceResult {
  std::optional<simulation::Simulation> simulation;
  bool unknownLayout = false;  // the caller shows its usage text and exits with 2
};

// Builds the simulation. A map file that is missing, invalid or unable to host a simulation is
// reported on `err` (with every problem found in it) and gives an empty result.
[[nodiscard]] inline SourceResult makeSimulation(const WarehouseSource& source, std::uint64_t seed,
                                                 std::ostream& err) {
  if (source.mapFile.empty()) {
    auto simulation = simulation::makeLayoutSimulation(
        source.layout.empty() ? std::string_view("demo") : std::string_view(source.layout), seed);
    const bool unknown = !simulation.has_value();
    return {std::move(simulation), unknown};
  }
  const mapio::MapLoad load = mapio::loadMapFile(source.mapFile);
  if (!load.ok()) {
    err << "error: " << source.mapFile << " is not a valid map:\n" << load.describe() << '\n';
    return {};
  }
  try {
    return {simulation::makeMapSimulation(load.document->layout, seed), false};
  } catch (const std::invalid_argument& error) {
    err << "error: " << source.mapFile << ": " << error.what() << '\n';
    return {};
  }
}

}  // namespace warehousesim::apps
