#pragma once

#include <cstdint>

namespace warehousesim::view {

// What the user asked of the fleet interface with the keyboard, without any SDL types (see
// InputEvents). The window decides which key means what; the controller only sees the meaning.
// Clicks arrive as PointerActions (editor_input.hpp); pause, speed, zoom and pan are the same
// InputEvents fields the plain view uses.
enum class FleetKey : std::uint8_t {
  NextRobot,  // select the next robot (the first, if none is selected), wrapping round
  PreviousRobot,
  NextTask,  // select the next task in the lists, wrapping round
  PreviousTask,
  NewTask,    // start making a task: click a pickup cell, then a dropoff cell
  Priority1,  // low: the priority of the task being made or, if none is, of the selected task
  Priority2,  // normal
  Priority3,  // high
  Priority4,  // urgent
  Reset,      // start the scenario over
  Cancel      // leave task entry; else clear the selection; else quit
};

}  // namespace warehousesim::view
