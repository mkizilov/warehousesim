// Headless runner: simulates the demo scenario and prints the final snapshot as JSON.
#include <cstdint>
#include <exception>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string_view>

#include "arguments.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"

namespace {

void printUsage(std::ostream& out) { out << "usage: warehousesim_cli [--ticks N] [--seed N]\n"; }

}  // namespace

int main(int argc, char** argv) {
  std::uint64_t ticks = 200;
  std::uint64_t seed = 1;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--ticks", ticks, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--seed", seed, failed)) {
      continue;
    }
    failed = true;
  }
  if (failed) {
    printUsage(std::cerr);
    return 2;
  }

  try {
    auto simulation = warehousesim::simulation::makeDemoSimulation(seed);
    simulation.run(ticks);
    std::cout << warehousesim::simulation::toJson(simulation.snapshot()).dump(2) << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
