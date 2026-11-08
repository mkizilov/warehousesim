#pragma once

#include <memory>
#include <string>

#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::rendering {

struct InputEvents {
  bool quit = false;
  bool togglePause = false;
};

// RAII wrapper for an SDL3 window and renderer. SDL types stay out of this header, and
// everything is released in reverse order of creation, including when the constructor
// throws part-way through.
class SdlWindow {
 public:
  // Throws std::runtime_error with SDL's error text if initialisation fails.
  SdlWindow(const std::string& title, int width, int height);
  ~SdlWindow();

  SdlWindow(const SdlWindow&) = delete;
  SdlWindow& operator=(const SdlWindow&) = delete;
  SdlWindow(SdlWindow&&) = delete;
  SdlWindow& operator=(SdlWindow&&) = delete;

  // Escape or closing the window requests quit; Space requests a pause toggle.
  [[nodiscard]] InputEvents pollEvents();

  // Renders one frame. Throws std::runtime_error if SDL reports a drawing error.
  void draw(const world::Warehouse& warehouse, const simulation::Snapshot& snapshot, bool paused);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace warehousesim::rendering
