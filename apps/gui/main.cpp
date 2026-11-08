// Graphical fleet interface. The simulation steps on a worker thread with a fixed timestep; this
// thread reads input, draws frames and paces itself to --fps. The interface never touches the
// simulation: everything it wants done to it (new tasks, priority changes, a restart) goes through
// SimulationRunner::post() as a command that the worker applies between two steps and records.
//
//   click / Tab / T  select a robot or a task    N, click, click  add a delivery task
//   1-4  priority of the new or the selected task   Space  pause   [ ] slower / faster   R  restart
//   P / O  routes / held cells   wheel +/- zoom   middle or right drag, arrows, WASD pan   0  fit
//   Esc  cancel, clear the selection, quit
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#include "arguments.hpp"
#include "simulation_source.hpp"
#include "warehousesim/core/fixed_timestep.hpp"
#include "warehousesim/rendering/sdl_window.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/view/fleet_controller.hpp"
#include "warehousesim/view/fleet_scene.hpp"
#include "warehousesim/view/fleet_session.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint64_t kMaxTickMs = 60000;
constexpr std::uint64_t kMaxFps = 1000;

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_gui [--seed N] [--layout NAME | --map FILE] [--tick-ms N]\n"
         "                        [--speed X] [--fps N] [--frames N]\n"
         "  --map FILE   run a map made with warehousesim_editor instead of a layout\n"
         "  --tick-ms N  fixed simulation step, in real milliseconds at speed 1 (default 100)\n"
         "  --speed X    initial playback speed multiplier, 0.0625 to 64 (default 1)\n"
         "  --fps N      frame rate cap, 1 to 1000 (default 60)\n"
         "  --frames N   exit after N rendered frames (0 = run until closed)\n"
         "  layouts: "
      << warehousesim::apps::layoutNames() << " (default: demo)\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::uint64_t seed = 1;
  std::uint64_t tickMs = 100;
  std::uint64_t fps = 60;
  std::uint64_t maxFrames = 0;
  double speed = 1.0;
  warehousesim::apps::WarehouseSource source;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--seed", seed, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--tick-ms", tickMs, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--fps", fps, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--frames", maxFrames, failed) ||
        warehousesim::apps::readDoubleOption(argc, argv, i, "--speed", speed, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--layout", source.layout, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--map", source.mapFile, failed)) {
      continue;
    }
    failed = true;
  }
  if (failed || source.conflicting() || tickMs == 0 || tickMs > kMaxTickMs || fps == 0 ||
      fps > kMaxFps ||
      !(speed >= warehousesim::core::FixedTimestep::kMinSpeed &&
        speed <= warehousesim::core::FixedTimestep::kMaxSpeed)) {
    printUsage(std::cerr);
    return 2;
  }

  try {
    auto made = warehousesim::apps::makeScenario(source, std::cerr);
    if (!made.scenario) {
      if (made.unknownLayout) {
        printUsage(std::cerr);
        return 2;
      }
      return 1;
    }

    // Declaration order matters for shutdown: the window (and SDL) is torn down before the
    // runner, and the runner's destructor joins its thread even if we leave through an exception.
    warehousesim::simulation::SimulationRunner runner(
        (*made.scenario)(seed),
        std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(tickMs)),
        *made.scenario);
    warehousesim::rendering::SdlWindow window("WarehouseSim fleet", 1280, 720);
    warehousesim::view::FleetController controller(window.renderSize(), runner.warehouse(), speed);
    runner.setSpeed(controller.speed());
    runner.start();

    const auto frameBudget = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(fps)));
    warehousesim::view::FleetSession session(runner, controller);
    auto previousFrame = Clock::now();
    std::uint64_t frames = 0;
    while (maxFrames == 0 || frames < maxFrames) {
      const auto frameStart = Clock::now();
      const double frameSeconds = std::chrono::duration<double>(frameStart - previousFrame).count();
      previousFrame = frameStart;

      const auto input = window.pollEvents(warehousesim::rendering::PollMode::Fleet);
      controller.resize(window.renderSize());
      // The simulation moves in whole steps; what is drawn is the point between the last two that
      // real time has reached, so motion is smooth at any frame rate.
      const warehousesim::simulation::Snapshot drawn = session.step(input, frameSeconds);
      if (controller.wantsQuit()) {
        break;
      }

      window.draw(warehousesim::view::buildFleetScene(runner.warehouse(), drawn, controller));
      ++frames;
      std::this_thread::sleep_until(frameStart + frameBudget);
    }
    runner.stop();
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
