// Graphical runner: the simulation steps on a worker thread, this thread renders with SDL3.
// Space pauses/resumes, Escape quits.
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <thread>

#include "arguments.hpp"
#include "warehousesim/rendering/sdl_window.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/scenarios.hpp"

namespace {

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_gui [--seed N] [--tick-ms N] [--frames N]\n"
         "  --frames N  exit after N rendered frames (0 = run until closed)\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::uint64_t seed = 1;
  std::uint64_t tickMs = 100;
  std::uint64_t maxFrames = 0;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--seed", seed, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--tick-ms", tickMs, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--frames", maxFrames, failed)) {
      continue;
    }
    failed = true;
  }
  if (failed) {
    printUsage(std::cerr);
    return 2;
  }

  try {
    warehousesim::simulation::SimulationRunner runner(
        warehousesim::simulation::makeDemoSimulation(seed),
        std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(tickMs)));
    warehousesim::rendering::SdlWindow window("WarehouseSim", 960, 600);
    runner.start();

    bool paused = false;
    std::uint64_t frames = 0;
    while (maxFrames == 0 || frames < maxFrames) {
      const auto input = window.pollEvents();
      if (input.quit) {
        break;
      }
      if (input.togglePause) {
        paused = !paused;
        runner.setPaused(paused);
      }
      window.draw(runner.warehouse(), runner.snapshot(), paused);
      ++frames;
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    runner.stop();
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
