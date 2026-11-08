#pragma once

#include "warehousesim/simulation/simulation.hpp"

namespace warehousesim::view {

// A robot whose pose changed by more than this between two snapshots (in cells) did not drive
// there; it was moved aside or put somewhere. It is drawn at its new pose rather than swept
// through whatever lies between.
inline constexpr double kMaxInterpolatedDistance = 1.5;

// The picture to draw `alpha` of the way from `previous` to `latest`, both fixed-timestep states
// of the same simulation (one step apart, normally). alpha is clamped to [0, 1].
//
// Robot poses (centre and heading, along the shortest rotation) are interpolated; everything
// else (logical cells, states, batteries, tasks, counters, the tick) is taken from `latest`. Every
// robot in the result has a pose; at alpha = 1 it is exactly the one in `latest`.
//
// This is what keeps motion independent of the frame rate. The simulation only ever advances
// in whole steps; a renderer that draws between steps samples this function at the time it
// wants to show, and the result depends on that time alone: not on how many frames were drawn
// before it, nor how long they took. The cost is that the picture runs up to one step behind
// the simulation, which keeps it from ever showing a robot past where it really is.
[[nodiscard]] simulation::Snapshot blend(const simulation::Snapshot& previous,
                                         const simulation::Snapshot& latest, double alpha);

}  // namespace warehousesim::view
