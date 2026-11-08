#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <utility>
#include <variant>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/world/grid_size.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::traffic {

// Simulation time as the table sees it: step s covers the time from tick s up to tick s + 1.
using Step = core::Tick;

// A half-open range of steps, [from, to).
struct StepRange {
  Step from = 0;
  Step to = 0;

  [[nodiscard]] constexpr bool empty() const noexcept { return to <= from; }
  [[nodiscard]] constexpr bool contains(Step step) const noexcept {
    return from <= step && step < to;
  }
  [[nodiscard]] constexpr bool overlaps(const StepRange& other) const noexcept {
    return from < other.to && other.from < to;
  }
  friend constexpr bool operator==(const StepRange&, const StepRange&) = default;
};

enum class ConflictKind : std::uint8_t {
  Blocked,  // the cell is a wall or a shelf, or is off the grid
  Vertex,   // another robot holds the cell during an overlapping step
  Swap,     // another robot goes the opposite way along the same edge in the same step
};

[[nodiscard]] const char* toString(ConflictKind kind) noexcept;

// Why a request was refused. `cell` is the cell the requester wanted (for a Swap, the one it was
// moving into), `step` the first step at which the clash happens, and `other` the robot it clashes
// with (empty for Blocked).
struct Conflict {
  ConflictKind kind = ConflictKind::Blocked;
  core::Point cell;
  Step step = 0;
  std::optional<core::RobotId> other;

  friend bool operator==(const Conflict&, const Conflict&) = default;
};

// What a robot can ask the table for.
//
// Occupy: the robot is on `cell` during `steps`. Two robots may not occupy a cell in overlapping
// steps. That one rule covers two robots entering a cell at once, a robot driving into one that is
// standing there, and two paths crossing at an intersection.
struct Occupy {
  core::Point cell;
  StepRange steps;
};

// Traverse: the robot goes from `from` to the adjacent cell `to` during `step`. Two robots may not
// traverse the same edge in opposite directions in the same step (a head-on swap).
struct Traverse {
  core::Point from;
  core::Point to;
  Step step = 0;
};

using Request = std::variant<Occupy, Traverse>;

struct Reservation {
  core::RobotId robot;
  core::Point cell;
  StepRange steps;

  friend bool operator==(const Reservation&, const Reservation&) = default;
};

// Who is where, when: a space-time reservation table over a warehouse layout.
//
// A robot reserves cells for ranges of steps. A request is refused if it clashes with a
// reservation of another robot, and a refused request changes nothing (reserveAll is atomic). A
// robot never clashes with itself, and its own reservations that overlap or touch merge.
//
// The table is first come, first served, and it is deterministic: whoever reserves first wins, so
// the order in which robots reserve is the priority between them (TrafficController reserves in
// ascending robot id order). When several reservations clash with one request, the one reported is
// the earliest in time, then the lowest robot id.
class ReservationTable {
 public:
  explicit ReservationTable(const world::Warehouse& warehouse);

  [[nodiscard]] world::GridSize size() const noexcept { return size_; }

  // The first reason `robot` could not make this request, if any. Throws std::invalid_argument for
  // an empty range, an invalid robot id, or a traversal between cells that are not adjacent.
  [[nodiscard]] std::optional<Conflict> check(core::RobotId robot, const Request& request) const;
  // The first conflict among the requests, in the order given.
  [[nodiscard]] std::optional<Conflict> checkAll(core::RobotId robot,
                                                 std::span<const Request> requests) const;
  // The conflict among the requests that happens soonest, whatever order they are in: the earliest
  // step, and at the same step Blocked before Swap before Vertex (two robots exchanging cells
  // clash on both the edge and the cells; the swap is the better description), then the first in
  // the order given. A request that spans many steps is judged by the step it first clashes.
  [[nodiscard]] std::optional<Conflict> earliestConflict(core::RobotId robot,
                                                         std::span<const Request> requests) const;

  // check() and, if there is no conflict, reserve. Returns the conflict, or nothing on success.
  std::optional<Conflict> reserve(core::RobotId robot, const Request& request);
  // All of the requests, or none of them.
  std::optional<Conflict> reserveAll(core::RobotId robot, std::span<const Request> requests);

  // Gives back everything the robot reserved.
  void release(core::RobotId robot);
  // Forgets every reservation; the layout stays.
  void clear();

  // The robot occupying `cell` during `step`, if any.
  [[nodiscard]] std::optional<core::RobotId> holder(core::Point cell, Step step) const;
  // Every occupied cell at `step` with its robot, in row-major cell order.
  [[nodiscard]] std::vector<std::pair<core::Point, core::RobotId>> occupiedAt(Step step) const;
  // The robot's occupancy reservations, in row-major cell order and then by first step.
  [[nodiscard]] std::vector<Reservation> reservationsOf(core::RobotId robot) const;
  [[nodiscard]] std::size_t occupancyCount() const;
  [[nodiscard]] std::size_t traversalCount() const;

 private:
  struct Entry {
    core::RobotId robot;
    StepRange steps;
  };
  struct EdgeEntry {
    core::RobotId robot;
    Step step;
  };
  using Edge = std::pair<core::Point, core::Point>;

  [[nodiscard]] std::optional<Conflict> checkOccupy(core::RobotId robot,
                                                    const Occupy& request) const;
  [[nodiscard]] std::optional<Conflict> checkTraverse(core::RobotId robot,
                                                      const Traverse& request) const;
  void insert(core::RobotId robot, const Request& request);
  [[nodiscard]] bool isWalkable(core::Point cell) const noexcept;
  // The indexes of the cells that hold entries, sorted and without repeats.
  [[nodiscard]] std::vector<std::size_t> touchedCells() const;

  world::GridSize size_;
  std::vector<bool> walkable_;
  std::vector<std::vector<Entry>> cells_;  // by cell index
  std::vector<std::size_t> touched_;       // cell indexes that may hold entries
  std::map<Edge, std::vector<EdgeEntry>> edges_;
};

}  // namespace warehousesim::traffic
