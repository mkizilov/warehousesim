// Headless runner: simulates a predefined layout or a map file and prints the final snapshot as
// JSON.
#include <cstdint>
#include <exception>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "arguments.hpp"
#include "simulation_source.hpp"
#include "warehousesim/simulation/serialization.hpp"

namespace {

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_cli [--ticks N] [--seed N] [--layout NAME | --map FILE]\n"
         "  layouts: "
      << warehousesim::apps::layoutNames()
      << " (default: demo)\n"
         "  --map FILE   run a map made with warehousesim_editor instead of a layout\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::uint64_t ticks = 200;
  std::uint64_t seed = 1;
  warehousesim::apps::WarehouseSource source;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--ticks", ticks, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--seed", seed, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--layout", source.layout, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--map", source.mapFile, failed)) {
      continue;
    }
    failed = true;
  }
  if (failed || source.conflicting()) {
    printUsage(std::cerr);
    return 2;
  }

  try {
    auto made = warehousesim::apps::makeSimulation(source, seed, std::cerr);
    if (!made.simulation) {
      if (made.unknownLayout) {
        printUsage(std::cerr);
        return 2;
      }
      return 1;
    }
    made.simulation->run(ticks);
    std::cout << warehousesim::simulation::toJson(made.simulation->snapshot()).dump(2) << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
