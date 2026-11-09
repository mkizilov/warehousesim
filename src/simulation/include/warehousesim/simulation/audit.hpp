#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/vec2.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::simulation {

// One thing that was found to be wrong, and when.
struct AuditViolation {
  core::Tick tick = 0;
  std::string what;
};

// Checks, state after state, the invariants that must hold of any run whatever the layout, the
// fleet or the task load. It only reads: an audited run is the same run as an unaudited one.
//
// Call observe() on the state before the first step and after every step. Per state it checks
//
//  * collision safety: no cell is held by two robots, every robot holds the cell it is on, no
//    robot stands on a wall or off the map, no two robots overlap (their centres are at least a
//    cell apart), no two robots exchange cells, and nobody moves further than a cell in a tick;
//  * ownership: a task a robot works on is that robot's, a robot's task is that task's, no task is
//    held twice, and every task is accounted for (completed + failed + cancelled + open = made);
//  * event ordering: the tick advances by exactly one, tasks are made in id order and never from
//    the future, and no counter (tasks finished, cargo delivered) ever goes down;
//  * numerical sanity: positions, headings and speeds are finite, speed is between zero and the
//    fastest a robot may go, and a pose is never further than a cell from the cell it is in.
//
// The Snapshot overload needs nothing but plain data, so a test can hand it a state that breaks a
// rule and see that the rule is noticed. The Simulation overload adds what only the engine can say
// (each robot's own consistency check, the exact number of tasks) and follows layout edits by
// itself.
class InvariantAuditor {
 public:
  // At most this many violations are kept (in the order found); all are counted.
  static constexpr std::size_t kMaxKept = 100;

  // `warehouse` is the layout the states are of.
  explicit InvariantAuditor(world::Warehouse warehouse);

  // The layout changed (Simulation::placeObstacle and clearObstacle): audit against the new one.
  void setLayout(world::Warehouse warehouse);

  void observe(const Snapshot& snapshot);
  void observe(const Simulation& simulation);

  [[nodiscard]] bool ok() const noexcept { return found_ == 0; }
  [[nodiscard]] std::size_t violationCount() const noexcept { return found_; }
  [[nodiscard]] const std::vector<AuditViolation>& violations() const noexcept { return kept_; }
  [[nodiscard]] std::size_t statesObserved() const noexcept { return observed_; }
  // "audited 3000 states: no violations", or the count and the first few violations.
  [[nodiscard]] std::string summary() const;

 private:
  struct RobotMemory {
    core::Point cell;
    core::Vec2 center;
    std::uint64_t deliveries = 0;
  };

  void report(std::string what);
  void checkRobots(const Snapshot& snapshot);
  void checkOverlaps(const Snapshot& snapshot);
  void checkTasks(const Snapshot& snapshot);
  void checkAgainstPrevious(const Snapshot& snapshot);
  void remember(const Snapshot& snapshot);

  world::Warehouse warehouse_;
  std::uint64_t layoutVersion_;
  core::Tick tick_ = 0;
  std::size_t observed_ = 0;
  std::size_t found_ = 0;
  std::vector<AuditViolation> kept_;

  // What the previous state looked like (empty before the first).
  bool haveBefore_ = false;
  core::Tick beforeTick_ = 0;
  std::vector<RobotMemory> before_;
  std::size_t beforeCompleted_ = 0;
  std::size_t beforeFailed_ = 0;
  std::size_t beforeCancelled_ = 0;

  // Scratch, back to -1 after every use: the robot holding each cell, and the robots in each cell
  // chained through nextInCell_.
  std::vector<std::int32_t> holderOf_;
  std::vector<std::int32_t> firstInCell_;
  std::vector<std::int32_t> nextInCell_;
};

}  // namespace warehousesim::simulation
