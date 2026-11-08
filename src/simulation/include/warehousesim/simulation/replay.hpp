#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/metrics/fleet_metrics.hpp"
#include "warehousesim/simulation/recording.hpp"
#include "warehousesim/simulation/simulation.hpp"

// Playing a recording back from its initial state, and saying where it goes wrong if it does.
namespace warehousesim::simulation {

struct ReplayOptions {
  // Compare the replay with the recording as it goes: the checksum at every checkpoint, whether
  // each command is accepted as it was, and the tasks made and cancelled. Off, the replay only
  // plays and leaves the comparing to the caller (a recording with nothing to compare is played
  // the same way).
  bool verify = true;
  // Stop after this many steps (and the commands applied at that tick), if the recording is
  // longer. The end checkpoint is only checked when the replay reaches the end.
  std::optional<core::Tick> until;
  // How to measure the replay (see MetricsCollector).
  metrics::MetricsOptions metrics;
};

// The first place where the replay stopped agreeing with the recording, and in words how.
struct Divergence {
  core::Tick tick = 0;  // the tick the disagreement was found at
  std::string what;
};

struct ReplayResult {
  core::Tick ticks = 0;  // steps taken: the recording's, or fewer if it was cut short or diverged
  Snapshot snapshot;     // the state the replay ended in
  std::uint64_t checksum = 0;            // Simulation::stateChecksum() of it
  metrics::FleetMetrics metrics;         // the figures of the whole replay
  std::vector<TaskEvent> taskEvents;     // what the replay saw happen to tasks
  std::vector<Checkpoint> checkpoints;   // the replay's checksum at each tick the recording has one
                                         // for (when verifying)
  std::size_t checkpointsVerified = 0;   // how many of them matched
  bool endVerified = false;              // the end checkpoint matched
  std::optional<Divergence> divergence;  // set if anything did not match; the replay stopped there

  [[nodiscard]] bool ok() const noexcept { return !divergence.has_value(); }
};

// Builds the simulation the recording describes, applies its commands at their ticks and steps it,
// all in the order the run did, from tick 0. Never throws for any recording: one that cannot be
// replayed (it is not consistent, its scenario cannot be built, the engine throws) comes back as a
// divergence, with the state the replay got to. Replaying the same recording twice gives the same
// result, bit for bit, because nothing in it depends on the time, the thread or the machine's
// state (see the README on determinism).
[[nodiscard]] ReplayResult replay(const Recording& recording, const ReplayOptions& options = {});

}  // namespace warehousesim::simulation
