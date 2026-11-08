// Graphical runner. The simulation steps on a worker thread with a fixed timestep; this thread
// reads input, draws frames and paces itself to --fps. Frame rate never affects the simulation,
// only how finely the motion between two steps is drawn.
//
//   Space pause/resume   [ ] slower/faster   wheel or +/- zoom   drag or arrows/WASD pan
//   0 fit the whole warehouse   Esc quit
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#include "arguments.hpp"
#include "warehousesim/core/fixed_timestep.hpp"
#include "warehousesim/rendering/sdl_window.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/view/interpolation.hpp"
#include "warehousesim/view/scene.hpp"
#include "warehousesim/view/view_controller.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint64_t kMaxTickMs = 60000;
constexpr std::uint64_t kMaxFps = 1000;

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_gui [--seed N] [--layout NAME] [--tick-ms N] [--speed X]\n"
         "                        [--fps N] [--frames N]\n"
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
  std::string layout = "demo";
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--seed", seed, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--tick-ms", tickMs, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--fps", fps, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--frames", maxFrames, failed) ||
        warehousesim::apps::readDoubleOption(argc, argv, i, "--speed", speed, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--layout", layout, failed)) {
      continue;
    }
    failed = true;
  }
  if (failed || tickMs == 0 || tickMs > kMaxTickMs || fps == 0 || fps > kMaxFps ||
      !(speed >= warehousesim::core::FixedTimestep::kMinSpeed &&
        speed <= warehousesim::core::FixedTimestep::kMaxSpeed)) {
    printUsage(std::cerr);
    return 2;
  }

  try {
    auto simulation = warehousesim::simulation::makeLayoutSimulation(layout, seed);
    if (!simulation) {
      printUsage(std::cerr);
      return 2;
    }

    // Declaration order matters for shutdown: the window (and SDL) is torn down before the
    // runner, and the runner's destructor joins its thread even if we leave through an exception.
    warehousesim::simulation::SimulationRunner runner(
        std::move(*simulation),
        std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(tickMs)));
    warehousesim::rendering::SdlWindow window("WarehouseSim", 960, 600);
    warehousesim::view::ViewController controller(window.renderSize(), runner.warehouse().size(),
                                                  speed);
    runner.setSpeed(controller.speed());
    runner.start();

    const auto frameBudget = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(fps)));
    auto previousFrame = Clock::now();
    std::uint64_t frames = 0;
    while (maxFrames == 0 || frames < maxFrames) {
      const auto frameStart = Clock::now();
      const double frameSeconds = std::chrono::duration<double>(frameStart - previousFrame).count();
      previousFrame = frameStart;

      const auto input = window.pollEvents();
      if (input.quit) {
        break;
      }
      controller.resize(window.renderSize());
      controller.apply(input, frameSeconds);
      if (input.togglePause) {
        runner.setPaused(controller.paused());
      }
      if (input.speedSteps != 0) {
        runner.setSpeed(controller.speed());
      }

      const warehousesim::view::ViewStatus status{controller.paused(), controller.speed(),
                                                  controller.pointer(), controller.showPaths(),
                                                  controller.showOccupied()};
      // The simulation moves in whole steps; what is drawn is the point between the last two that
      // real time has reached, so motion is smooth at any frame rate.
      const warehousesim::simulation::FrameState frame = runner.frame();
      window.draw(warehousesim::view::buildScene(
          runner.warehouse(), warehousesim::view::blend(frame.previous, frame.latest, frame.alpha),
          controller.camera(), status));
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
