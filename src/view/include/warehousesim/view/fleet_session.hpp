#pragma once

#include <cstddef>
#include <cstdint>

#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/view/fleet_controller.hpp"
#include "warehousesim/view/view_controller.hpp"

namespace warehousesim::view {

// The glue between the fleet interface and the simulation, one frame at a time: it reads what is to
// be drawn from the runner, tells the controller what became of the commands it posted, runs the
// controller on this frame's input and hands what it asked for to the runner. It is the only place
// where the two meet, and it does so only through the runner's public interface: commands go in
// through post(), playback state through setPaused() and setSpeed(). Both are borrowed and must
// outlive the session.
class FleetSession {
 public:
  FleetSession(simulation::SimulationRunner& runner, FleetController& controller);

  // Runs one frame and returns the snapshot to draw (the runner's last two states blended by how
  // far real time has got between them). Commands that the runner's queue has no room for are
  // reported on the controller's message line, never dropped silently.
  simulation::Snapshot step(const InputEvents& input, double frameSeconds);

 private:
  simulation::SimulationRunner& runner_;
  FleetController& controller_;
  std::uint64_t epoch_ = 0;
  std::size_t reported_ = 0;  // how much of the runner's command log the controller has seen
};

}  // namespace warehousesim::view
