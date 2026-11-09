#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/robots/robot.hpp"

namespace warehousesim::robots {

// What one fixed step of unobstructed motion did.
struct UnobstructedStep {
  std::size_t passed =
      0;  // path cells passed (they became the robot's position, one after another)
  // How many cells of the path as it was at the start of the step the robot held at some point in
  // the step: the ones it already had, the ones it asked for, and the ones it passed.
  std::size_t heldPrefix = 0;
};

// One step of `dt` ticks of the motion Simulation gives a robot when nothing is in its way:
// turn towards the next cell, be granted every cell it asks for, drive, and use any time a stop
// left over. A robot that is not navigating only coasts along the cells it already holds.
//
// This is the same sequence of Robot calls Simulation::moveAlongPath makes when every claim
// succeeds, so a robot stepped with it follows exactly the trajectory the simulation would give
// it on an empty floor (tests compare the two tick by tick).
UnobstructedStep stepUnobstructed(Robot& robot, double dt = 1.0);

// The cells `robot` would hold during each of the next `steps` ticks if nothing got in its way
// (element s is for the tick that is s ticks from now). Each element lists the cell the robot
// starts the tick on, followed by the path cells it holds at some point during the tick, in path
// order. A robot with no path holds just its own cell, tick after tick.
//
// `robot` is taken by value: forecasting never touches the real robot.
[[nodiscard]] std::vector<std::vector<core::Point>> forecastHolds(Robot robot, std::size_t steps);

// A forecast in flat storage, for callers that ask for one per robot per step: once the buffer has
// grown to the size of a forecast, asking again allocates nothing.
class ForecastBuffer {
 public:
  void clear() noexcept {
    cells_.clear();
    ends_.clear();
  }
  [[nodiscard]] std::size_t steps() const noexcept { return ends_.size(); }
  // The cells held during step `step` (see forecastHolds): the robot's own cell first.
  [[nodiscard]] std::span<const core::Point> holdsAt(std::size_t step) const noexcept {
    const std::size_t begin = step == 0 ? 0 : ends_[step - 1];
    return std::span<const core::Point>(cells_).subspan(begin, ends_[step] - begin);
  }
  [[nodiscard]] std::vector<std::vector<core::Point>> toVectors() const;

 private:
  friend void forecastHolds(Robot&, std::size_t, ForecastBuffer&);
  std::vector<core::Point> cells_;
  std::vector<std::uint32_t> ends_;  // by step: where its cells end in cells_
};

// The same forecast as above, written to `out` (which is cleared first). `walker` is a working copy
// of the robot, stepped through the forecast and left wherever it ends, so it must not be a real
// robot. Overwriting one working copy with the next robot (`walker = robot`, or
// Robot::assignRouteCutOf) reuses its storage, which a fresh copy would have to allocate.
void forecastHolds(Robot& walker, std::size_t steps, ForecastBuffer& out);

}  // namespace warehousesim::robots
