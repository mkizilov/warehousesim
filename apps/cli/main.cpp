// Headless runner: simulates a predefined layout or a map file and prints the final snapshot as
// JSON. With --metrics it also prints the performance dashboard (to stderr, so that the JSON on
// stdout is the same either way), and with --metrics-csv BASE it writes the figures to CSV files.
// All performance figures are simulated.
//
// With --record FILE the run is also written down so that it can be played again: --replay FILE
// builds the same simulation from the file, plays it step for step and compares it with the
// checksums in the file, printing the same final snapshot if nothing differs.
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>

#include "arguments.hpp"
#include "metrics_output.hpp"
#include "recording_output.hpp"
#include "simulation_source.hpp"
#include "warehousesim/metrics/export.hpp"
#include "warehousesim/simulation/metrics_collector.hpp"
#include "warehousesim/simulation/recording.hpp"
#include "warehousesim/simulation/recording_file.hpp"
#include "warehousesim/simulation/replay.hpp"
#include "warehousesim/simulation/serialization.hpp"

namespace {

namespace sim = warehousesim::simulation;

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_cli [--ticks N] [--seed N] [--layout NAME | --map FILE]\n"
         "                        [--robots N] [--spawn N] [--threads N]\n"
         "                        [--metrics] [--metrics-csv BASE]\n"
         "                        [--record FILE [--checkpoint-interval N]]\n"
         "       warehousesim_cli --replay FILE [--ticks N] [--no-verify] [--threads N]\n"
         "                        [--metrics] [--metrics-csv BASE]\n"
         "  layouts: "
      << warehousesim::apps::layoutNames()
      << " (default: demo)\n"
         "  --map FILE   run a map made with warehousesim_editor instead of a layout\n"
         "  --robots N   start N robots instead of the layout's usual fleet (1 to "
      << sim::kMaxScenarioRobots
      << ")\n"
         "  --spawn N    a new random task every N ticks (default 5; 0: none)\n"
         "  --threads N  plan routes on N worker threads besides the main one (default 0). The run "
         "is\n"
         "               the same for every N; only the time it takes changes\n"
         "  --metrics    also print the (simulated) performance dashboard to stderr\n"
         "  --metrics-csv BASE  write BASE.runs.csv, BASE.robots.csv and BASE.series.csv\n"
         "  --record FILE  also write a recording of the run: what it was built from, a checksum "
         "of\n"
         "               the state every N ticks, and the tasks it made\n"
         "  --checkpoint-interval N  ticks between those checksums (default "
      << sim::kDefaultCheckpointInterval
      << "; 0: only at the start and the end)\n"
         "  --replay FILE  play a recording back from its initial state and check it against the\n"
         "               checksums in it. The layout, seed and settings come from the file, so "
         "none\n"
         "               of --layout, --map, --seed, --record may be given; --ticks N replays "
         "only\n"
         "               the first N ticks. Exits with 1 and says where if the replay differs.\n"
         "  --no-verify  with --replay: play it without comparing\n";
}

struct Metrics {
  bool dashboard = false;
  std::string csvBase;
  [[nodiscard]] bool wanted() const { return dashboard || !csvBase.empty(); }
};

// Prints the dashboard and writes the CSV files, as asked. False if a file could not be written.
bool report(const Metrics& metrics, const warehousesim::metrics::FleetMetrics& figures,
            const warehousesim::metrics::RunLabel& label) {
  if (metrics.dashboard) {
    for (const std::string& line : warehousesim::metrics::dashboardLines(figures)) {
      std::cerr << line << '\n';
    }
  }
  if (!metrics.csvBase.empty()) {
    if (const auto problem =
            warehousesim::metrics::writeRunFiles(metrics.csvBase, label, figures)) {
      std::cerr << "error: " << *problem << '\n';
      return false;
    }
  }
  return true;
}

int replayRecording(const std::string& file, std::optional<std::uint64_t> until, bool verify,
                    std::uint64_t threads, const Metrics& metrics) {
  const sim::RecordingLoad load = sim::loadRecordingFile(file);
  if (!load.ok()) {
    std::cerr << "error: " << file << " is not a usable recording:\n  " << load.problem->text()
              << '\n';
    return 1;
  }
  sim::ReplayOptions options;
  options.verify = verify;
  options.until = until;
  options.planningThreads = static_cast<std::size_t>(threads);
  const sim::ReplayResult result = sim::replay(*load.recording, options);
  if (!result.ok()) {
    std::cerr << "error: the replay of " << file << " differs from the recording at tick "
              << result.divergence->tick << ": " << result.divergence->what << '\n';
    return 1;
  }
  std::cout << sim::toJson(result.snapshot).dump(2) << '\n';
  if (!verify) {
    std::cerr << "replayed " << result.ticks << " ticks (not compared with the recording)\n";
  } else if (result.checkpointsVerified == 0 && !result.endVerified) {
    std::cerr << "replayed " << result.ticks
              << " ticks; the recording has nothing to compare the replay with\n";
  } else {
    std::cerr << "replayed " << result.ticks << " ticks: " << result.checkpointsVerified
              << " checkpoints" << (result.endVerified ? " and the end state" : "")
              << " match the recording\n";
  }
  if (!metrics.wanted()) {
    return 0;
  }
  const auto& scenario = load.recording->scenario;
  const warehousesim::metrics::RunLabel label{std::filesystem::path(file).filename().string(),
                                              scenario.config.seed,
                                              scenario.config.taskSpawnInterval};
  return report(metrics, result.metrics, label) ? 0 : 1;
}

// Runs `simulation` for `ticks` steps, measuring and recording as asked, and prints the final
// state.
int runLive(sim::Simulation simulation, std::uint64_t ticks, const Metrics& metrics,
            const std::string& recordFile, std::uint64_t checkpointInterval,
            const warehousesim::apps::WarehouseSource& source, std::uint64_t seed) {
  // With a recorder the simulation lives in it, and everything is stepped through it so that it
  // misses nothing.
  std::optional<sim::Recorder> recorder;
  if (!recordFile.empty()) {
    recorder.emplace(std::move(simulation), checkpointInterval);
  }
  const sim::Simulation& state = recorder ? recorder->simulation() : simulation;

  std::optional<sim::MetricsCollector> collector;
  if (metrics.wanted()) {
    collector.emplace(state);
  }
  for (std::uint64_t tick = 0; tick < ticks; ++tick) {
    if (recorder) {
      recorder->step();
    } else {
      simulation.step();
    }
    if (collector) {
      collector->observe(state);
    }
  }
  std::cout << sim::toJson(state.snapshot()).dump(2) << '\n';

  int status = 0;
  if (collector) {
    const warehousesim::metrics::RunLabel label{warehousesim::apps::warehouseLabel(source), seed,
                                                state.config().taskSpawnInterval};
    if (!report(metrics, collector->report(), label)) {
      status = 1;
    }
  }
  if (recorder) {
    const sim::Recording recording = recorder->recording();
    if (const auto problem = sim::saveRecordingFile(recordFile, recording)) {
      std::cerr << "error: " << problem->text() << '\n';
      return 1;
    }
    std::cerr << "recorded " << recording.ticks << " ticks to " << recordFile << " ("
              << recording.commands.size() << " commands, " << recording.taskEvents.size()
              << " task events, " << recording.checkpoints.size() << " checkpoints)\n";
  }
  return status;
}

}  // namespace

int main(int argc, char** argv) {
  std::uint64_t ticks = 200;
  std::uint64_t seed = 1;
  std::uint64_t checkpointInterval = sim::kDefaultCheckpointInterval;
  bool ticksGiven = false;
  bool seedGiven = false;
  bool intervalGiven = false;
  bool noVerify = false;
  std::uint64_t threads = 0;
  warehousesim::apps::WarehouseSource source;
  Metrics metrics;
  std::string recordFile;
  std::string replayFile;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    const std::string_view argument = argv[i];
    if (argument == "--metrics") {
      metrics.dashboard = true;
      continue;
    }
    if (argument == "--no-verify") {
      noVerify = true;
      continue;
    }
    if (argument == "--ticks") {
      ticksGiven = true;
    } else if (argument == "--seed") {
      seedGiven = true;
    } else if (argument == "--checkpoint-interval") {
      intervalGiven = true;
    }
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--threads", threads, failed) ||
        warehousesim::apps::readOptionalUnsignedOption(argc, argv, i, "--robots", source.robots,
                                                       failed) ||
        warehousesim::apps::readOptionalUnsignedOption(argc, argv, i, "--spawn", source.spawn,
                                                       failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--metrics-csv", metrics.csvBase,
                                             failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--record", recordFile, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--replay", replayFile, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--checkpoint-interval",
                                               checkpointInterval, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--ticks", ticks, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--seed", seed, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--layout", source.layout, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--map", source.mapFile, failed)) {
      continue;
    }
    failed = true;
  }
  const bool replaying = !replayFile.empty();
  const bool recording = !recordFile.empty();
  if (failed || source.conflicting() ||
      (replaying && (recording || seedGiven || intervalGiven || !source.layout.empty() ||
                     !source.mapFile.empty() || source.robots || source.spawn)) ||
      (!replaying && noVerify) || (intervalGiven && !recording)) {
    printUsage(std::cerr);
    return 2;
  }
  if (!metrics.csvBase.empty() &&
      !warehousesim::apps::metricsBaseUsable(metrics.csvBase, std::cerr)) {
    return 2;
  }
  if (recording && !warehousesim::apps::recordingFileUsable(recordFile, std::cerr)) {
    return 2;
  }

  try {
    if (replaying) {
      return replayRecording(replayFile,
                             ticksGiven ? std::optional<std::uint64_t>(ticks) : std::nullopt,
                             !noVerify, threads, metrics);
    }
    auto made = warehousesim::apps::makeSimulation(source, seed, std::cerr);
    if (!made.simulation) {
      if (made.unknownLayout) {
        printUsage(std::cerr);
        return 2;
      }
      return 1;
    }
    made.simulation->usePlanningThreads(static_cast<std::size_t>(threads));
    return runLive(std::move(*made.simulation), ticks, metrics, recordFile, checkpointInterval,
                   source, seed);
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
