// Headless runner: simulates a predefined layout or a map file and prints the final snapshot as
// JSON. With --metrics it also prints the performance dashboard (to stderr, so that the JSON on
// stdout is the same either way), and with --metrics-csv BASE it writes the figures to CSV files.
// All performance figures are simulated.
#include <cstdint>
#include <exception>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

#include "arguments.hpp"
#include "metrics_output.hpp"
#include "simulation_source.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/serialization.hpp"

namespace {

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_cli [--ticks N] [--seed N] [--layout NAME | --map FILE]\n"
         "                        [--metrics] [--metrics-csv BASE]\n"
         "  layouts: "
      << warehousesim::apps::layoutNames()
      << " (default: demo)\n"
         "  --map FILE   run a map made with warehousesim_editor instead of a layout\n"
         "  --metrics    also print the (simulated) performance dashboard to stderr\n"
         "  --metrics-csv BASE  write BASE.runs.csv, BASE.robots.csv and BASE.series.csv\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::uint64_t ticks = 200;
  std::uint64_t seed = 1;
  warehousesim::apps::WarehouseSource source;
  std::string metricsBase;
  bool dashboard = false;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (std::string_view(argv[i]) == "--metrics") {
      dashboard = true;
      continue;
    }
    if (warehousesim::apps::readStringOption(argc, argv, i, "--metrics-csv", metricsBase, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--ticks", ticks, failed) ||
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
  const bool csv = !metricsBase.empty();
  if (csv && !warehousesim::apps::metricsBaseUsable(metricsBase, std::cerr)) {
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
    if (dashboard || csv) {
      warehousesim::simulation::MetricsCollector collector(*made.simulation);
      for (std::uint64_t tick = 0; tick < ticks; ++tick) {
        made.simulation->step();
        collector.observe(*made.simulation);
      }
      std::cout << warehousesim::simulation::toJson(made.simulation->snapshot()).dump(2) << '\n';
      if (dashboard) {
        for (const std::string& line : warehousesim::metrics::dashboardLines(collector.report())) {
          std::cerr << line << '\n';
        }
      }
      if (csv) {
        const warehousesim::metrics::RunLabel label{warehousesim::apps::warehouseLabel(source),
                                                    seed,
                                                    made.simulation->config().taskSpawnInterval};
        if (const auto problem =
                warehousesim::metrics::writeRunFiles(metricsBase, label, collector.report())) {
          std::cerr << "error: " << *problem << '\n';
          return 1;
        }
      }
      return 0;
    }
    made.simulation->run(ticks);
    std::cout << warehousesim::simulation::toJson(made.simulation->snapshot()).dump(2) << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
