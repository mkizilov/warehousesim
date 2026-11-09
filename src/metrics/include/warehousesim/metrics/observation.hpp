#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"

// What the metrics are told about a running simulation, as plain data. The metrics module knows
// nothing about the simulation itself: an adapter (simulation::MetricsCollector) turns the
// simulation into these observations after every step, and MetricsAccumulator only ever sees them.
// That keeps the aggregation testable with directly constructed observations, including ones a real
// simulation never produces (two robots in one cell).
//
// Everything measured is *simulated*: ticks of a model, cells of a grid and units of an abstract
// energy model. Nothing here is a measurement of a real robot or warehouse.
namespace warehousesim::metrics {

// What a robot was doing at the end of a tick: exactly one per robot per tick.
enum class Activity : std::uint8_t {
  Moving,       // driving to a goal (a pickup, a dropoff or a charger), not held back
  Waiting,      // driving to a goal but held back by traffic: a reservation or another robot
  Handling,     // loading or unloading
  Charging,     // on a charger, charging
  Idle,         // in service with nothing to do
  Unavailable,  // out of service
};

inline constexpr std::size_t kActivityCount = 6;
inline constexpr std::array<Activity, kActivityCount> kAllActivities{
    Activity::Moving,   Activity::Waiting, Activity::Handling,
    Activity::Charging, Activity::Idle,    Activity::Unavailable};

[[nodiscard]] const char* toString(Activity activity) noexcept;

// Why a Waiting robot is held back. A Vertex or Swap wait is a collision that would have happened
// had the robot not waited: it is the one metric that counts as a "conflict prevented".
enum class WaitCause : std::uint8_t {
  None,    // not waiting
  Vertex,  // another robot holds, or is about to hold, the cell
  Swap,    // another robot is coming the other way along the same edge
  Other,   // anything else (a cell that is no longer floor)
};

struct RobotObservation {
  core::RobotId id;
  core::Point position;
  Activity activity = Activity::Idle;
  WaitCause waitCause = WaitCause::None;  // only meaningful while Waiting
  std::uint64_t cellsTravelled = 0;       // cumulative, from the robot
  std::uint64_t deliveries = 0;           // cumulative, from the robot
};

enum class TaskEnd : std::uint8_t { Completed, Failed, Cancelled };

// A task that reached a final state during the tick observed.
struct TaskEnding {
  TaskEnd how = TaskEnd::Completed;
  core::Tick createdAt = 0;
};

// Counters that the observed system keeps itself, cumulative since it was built. The accumulator
// reports the difference to what it was told at the start.
struct SystemCounters {
  std::uint64_t tasksSubmitted = 0;
  std::uint64_t conflictDetections = 0;  // clashes the reservation table found while planning
  std::uint64_t reroutes = 0;            // routes replaced after a clash
  std::uint64_t repaths = 0;             // routes recomputed to get round a blocking robot
  std::uint64_t obstacleRepairs = 0;     // routes repaired after the layout changed
  std::uint64_t searches = 0;            // cooperative route searches run (replans attempted)
};

// The state of the simulation after one step, or at the start.
struct TickObservation {
  core::Tick tick = 0;                   // the tick reached: the number of steps taken so far
  std::vector<RobotObservation> robots;  // the same robots in the same order every time
  SystemCounters counters;
  std::uint64_t tasksOpen = 0;         // submitted and not yet completed, failed or cancelled
  std::vector<TaskEnding> tasksEnded;  // those that ended during this tick (not read at the start)
};

}  // namespace warehousesim::metrics
