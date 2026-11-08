#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::planning {

// What other robots leave free, as the search asks for it. The planner knows nothing about robots:
// whoever implements this decides what "taken" means (the traffic module answers from its
// reservation table).
//
// A robot "holds" a cell during a step. Moving from a to b during step t means holding b during
// t and t + 1 (it is on its way in, then there), so `cellFree(b, t)`, `cellFree(b, t + 1)` and
// `edgeFree(a, b, t)` must all hold. Waiting on a from t to t + 1 needs `cellFree(a, t + 1)`.
class SpaceTimeConstraints {
 public:
  SpaceTimeConstraints() = default;
  virtual ~SpaceTimeConstraints() = default;

  // May the robot hold `cell` during `step`?
  [[nodiscard]] virtual bool cellFree(core::Point cell, core::Tick step) const = 0;
  // May the robot go from `from` to the adjacent cell `to` during `step` (that is, is nobody going
  // the other way along the same edge)?
  [[nodiscard]] virtual bool edgeFree(core::Point from, core::Point to, core::Tick step) const = 0;

 protected:
  // Only the implementations copy themselves; nobody slices them through the base.
  SpaceTimeConstraints(const SpaceTimeConstraints&) = default;
  SpaceTimeConstraints& operator=(const SpaceTimeConstraints&) = default;
};

struct SpaceTimeOptions {
  // How many steps ahead the search looks (1 to 256). The robot's last cell is also kept free until
  // the end of the window, so that a plan does not end by parking in somebody's way.
  std::size_t window = 12;
  // Give up after this much work (0 = never): the cells of a distance field that had to be filled
  // plus the states expanded. The same unit is used to share a budget between several searches.
  std::size_t maxWork = 0;
  // Reaching the goal ends the route. A state on the goal that cannot be kept to the end of the
  // window is a dead end instead of a place to pass through, so the goal appears in the route only
  // as its last cell. For a robot, which stops when it first gets to its destination.
  bool goalIsTerminal = false;
};

enum class SpaceTimeStatus : std::uint8_t {
  Found,           // the goal is reached inside the window and can be kept to its end
  Partial,         // it cannot be, so this is the best place to be when the window ends
  NoPlan,          // the goal is unreachable on the map, or every way on is taken
  BudgetExceeded,  // options.maxWork was used up first
  InvalidStart,    // the start is off the grid or not walkable
  InvalidGoal,     // the goal is off the grid or not walkable
};

[[nodiscard]] const char* toString(SpaceTimeStatus status) noexcept;

struct SpaceTimePlan {
  SpaceTimeStatus status = SpaceTimeStatus::NoPlan;
  // The cell held at each step from the start step on: timeline[i] is held during startStep + i,
  // timeline[0] is the start, and consecutive entries are the same cell (a wait) or neighbours (a
  // move). Empty unless the status is Found or Partial.
  std::vector<core::Point> timeline;
  // Found: 0. Partial: the distance on the empty map from the last cell to the goal.
  std::int32_t remaining = 0;
  std::size_t moves = 0;
  std::size_t waits = 0;
  std::size_t work = 0;

  // Steps until the goal is reached (Found) or the window ends (Partial).
  [[nodiscard]] std::size_t duration() const noexcept {
    return timeline.empty() ? 0 : timeline.size() - 1;
  }
};

// Cooperative A* in space and time: finds a route for one robot that avoids what the constraints
// say is taken, and may wait.
//
// * State: (cell, step). Actions: wait, or move one cell north, east, south or west. Every action
//   takes one step, so the cost of a route is its duration and waiting is as dear as moving.
// * Heuristic: the distance to the goal on the empty map, found by a breadth-first fill and
//   cached per goal (and per layout, so a changed layout cannot return stale distances). It never
//   over-estimates and never drops by more than one per step, so the first goal state taken off the
//   open list is a fastest one.
// * Window: the search never looks further than `window` steps. If the goal is not reached inside
//   it (and held to the window's end), the result is the state at the window's end that is nearest
//   the goal: a plan to follow for a while, to be extended later (windowed cooperative A*). That
//   makes the method incomplete and not optimal beyond the window.
// * Determinism: the open list is ordered by (f, h, state id), a unique key, neighbours are tried
//   north, east, south, west and then waiting, and among equally fast ways of reaching a state the
//   one with fewer moves is kept, so the same inputs give the same plan on every platform.
//
// Memory is (window + 1) states per cell, reused between searches. Not thread-safe.
class SpaceTimeAStar {
 public:
  // Throws std::invalid_argument for a window outside 1 to 256 or a grid with too many states.
  // `start` is held at `startStep` whatever the constraints say; they apply from the next step.
  [[nodiscard]] SpaceTimePlan search(const world::Warehouse& warehouse, core::Point start,
                                     core::Point goal, core::Tick startStep,
                                     const SpaceTimeConstraints& constraints,
                                     const SpaceTimeOptions& options = {});

  // Makes sure the distances to `goal` on the empty map are cached, which a search needs and which
  // costs a fill of the whole map (the cells it visits are its work). Returns the work done, 0 if
  // it was cached already, or nothing at all, having done nothing, if the fill would cost more than
  // `maxWork` (0: no limit). Throws std::invalid_argument for a goal that is not walkable.
  [[nodiscard]] std::optional<std::size_t> prepare(const world::Warehouse& warehouse,
                                                   core::Point goal, std::size_t maxWork = 0);

  // The length of the shortest route from `from` to `goal` on the empty map, or -1 if there is none
  // (or either cell is not walkable). Uses, and if need be fills, the cache of prepare().
  [[nodiscard]] std::int32_t distance(const world::Warehouse& warehouse, core::Point from,
                                      core::Point goal);

 private:
  struct Node {
    std::uint32_t stamp = 0;   // generation in which the rest is valid
    std::uint32_t closed = 0;  // generation in which the node was expanded
    std::uint32_t parent = 0;
    std::uint32_t moves = 0;
  };
  struct OpenEntry {
    std::int32_t f;
    std::int32_t h;
    std::uint32_t id;
  };
  struct Field {
    std::uint64_t layout;
    std::size_t goal;
    std::vector<std::int32_t> distance;  // -1: cannot reach the goal
  };

  // The cached distances to `goal`, filling the cache first if need be and if `maxWork` (0: no
  // limit) allows it; null if it does not. `work` is raised by the cells visited.
  const std::vector<std::int32_t>* distanceField(const world::Warehouse& warehouse,
                                                 core::Point goal, std::size_t& work,
                                                 std::size_t maxWork);
  void trace(std::uint32_t terminal, std::size_t cells, const world::GridSize& size,
             SpaceTimePlan& plan) const;

  std::vector<Node> nodes_;
  std::vector<OpenEntry> open_;
  std::vector<Field> fields_;
  std::uint32_t generation_ = 0;
};

}  // namespace warehousesim::planning
