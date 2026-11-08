// Interactive warehouse map editor. Paint walls, shelves, pickups, dropoffs and chargers with the
// mouse, resize the map, start from a predefined layout, and save and load maps in the versioned
// JSON format (see mapio/map_file.hpp). Run a saved map with `warehousesim_gui --map FILE`.
//
//   Left paint   Right erase   Middle drag/WASD pan   Wheel zoom   1-6 tool   0 fit view
//   Shift+arrows resize   Ctrl+Z/Ctrl+Y undo/redo   Ctrl+S save   Ctrl+O load   L layouts
//   Del clear   Esc quit
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#include "arguments.hpp"
#include "warehousesim/editor/editor_files.hpp"
#include "warehousesim/editor/map_editor.hpp"
#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/rendering/sdl_window.hpp"
#include "warehousesim/view/editor_controller.hpp"
#include "warehousesim/view/editor_scene.hpp"
#include "warehousesim/world/grid_size.hpp"

namespace {

using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;

constexpr std::uint64_t kMaxFps = 1000;
constexpr const char* kDefaultFile = "warehouse-map.json";

void printUsage(std::ostream& out) {
  out << "usage: warehousesim_editor [--file PATH] [--layout NAME] [--width N] [--height N]\n"
         "                           [--fps N] [--frames N]\n"
         "  --file PATH    the map file to edit, saved to and loaded from (default "
      << kDefaultFile
      << ")\n"
         "                 if it exists it is opened, otherwise a new map is started\n"
         "  --layout NAME  start the new map from a predefined layout: "
      << warehousesim::apps::layoutNames()
      << "\n"
         "  --width N, --height N  size of a new blank map, 1 to "
      << warehousesim::world::kMaxMapSide
      << " (default 20 x 12)\n"
         "  --fps N        frame rate cap, 1 to 1000 (default 60)\n"
         "  --frames N     exit after N rendered frames (0 = run until closed)\n"
         "  The options other than --file, --fps and --frames only apply to a new map.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string file = kDefaultFile;
  std::string layout;
  std::uint64_t width = 20;
  std::uint64_t height = 12;
  std::uint64_t fps = 60;
  std::uint64_t maxFrames = 0;
  bool sizeGiven = false;
  bool failed = false;
  for (int i = 1; i < argc && !failed; ++i) {
    if (warehousesim::apps::readStringOption(argc, argv, i, "--file", file, failed) ||
        warehousesim::apps::readStringOption(argc, argv, i, "--layout", layout, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--fps", fps, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--frames", maxFrames, failed)) {
      continue;
    }
    if (warehousesim::apps::readUnsignedOption(argc, argv, i, "--width", width, failed) ||
        warehousesim::apps::readUnsignedOption(argc, argv, i, "--height", height, failed)) {
      sizeGiven = true;
      continue;
    }
    failed = true;
  }
  const auto validSide = [](std::uint64_t side) {
    return side >= 1 && side <= static_cast<std::uint64_t>(warehousesim::world::kMaxMapSide);
  };
  const bool sizeOk = validSide(width) && validSide(height);
  if (failed || file.empty() || fps == 0 || fps > kMaxFps || !sizeOk) {
    printUsage(std::cerr);
    return 2;
  }
  if (!layout.empty() && sizeGiven) {
    std::cerr << "error: --layout and --width/--height both say how the new map starts\n";
    printUsage(std::cerr);
    return 2;
  }

  try {
    const fs::path path(file);
    const std::string stem = path.stem().string().substr(0, warehousesim::mapio::kMaxNameBytes);
    warehousesim::editor::MapEditor editor = warehousesim::editor::MapEditor::blank(
        static_cast<std::int32_t>(width), static_cast<std::int32_t>(height), stem);
    if (fs::exists(path)) {
      if (!layout.empty() || sizeGiven) {
        std::cerr << "error: " << file << " already exists, so it is opened as it is; "
                  << "--layout, --width and --height only apply to a new map\n";
        return 2;
      }
      if (!warehousesim::editor::loadMap(editor, path).ok) {
        // Never start on a blank map in place of a file that could not be read: a later save
        // would overwrite it.
        std::cerr << "error: " << file << " is not a valid map:\n"
                  << warehousesim::mapio::loadMapFile(path).describe() << '\n';
        return 1;
      }
    } else if (!layout.empty()) {
      if (!warehousesim::editor::loadPredefined(editor, layout).ok) {
        printUsage(std::cerr);
        return 2;
      }
    }

    warehousesim::rendering::SdlWindow window("WarehouseSim map editor", 1100, 720);
    warehousesim::view::EditorController controller(window.renderSize(), std::move(editor), path);

    const auto frameBudget = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(fps)));
    auto previousFrame = Clock::now();
    std::uint64_t frames = 0;
    while (maxFrames == 0 || frames < maxFrames) {
      const auto frameStart = Clock::now();
      const double frameSeconds = std::chrono::duration<double>(frameStart - previousFrame).count();
      previousFrame = frameStart;

      const auto input = window.pollEvents(warehousesim::rendering::PollMode::Editor);
      controller.resize(window.renderSize());
      controller.apply(input, frameSeconds);
      if (controller.wantsQuit()) {
        break;
      }
      window.draw(warehousesim::view::buildEditorScene(controller));
      ++frames;
      std::this_thread::sleep_until(frameStart + frameBudget);
    }
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
