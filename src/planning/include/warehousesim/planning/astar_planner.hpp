#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "warehousesim/core/direction_set.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/path_planner.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::planning {

struct AStarOptions {
  // The moves a robot may make. Each step goes one cell in one of these directions at cost 1.
  // Restricting the set (for example to East and West only) gives paths for robots confined to
  // rows; the heuristic stays admissible because it never over-estimates for any subset.
  core::DirectionSet directions = core::DirectionSet::all();
  // Give up after expanding this many cells (0 = never). A bounded search reports
  // SearchStatus::BudgetExceeded instead of running to exhaustion on huge unreachable regions.
  std::size_t maxExpansions = 0;
};

enum class SearchStatus : std::uint8_t {
  Found,           // `path` holds the route (empty if start == goal)
  InvalidStart,    // start is outside the grid or not walkable
  InvalidGoal,     // goal is outside the grid or not walkable
  GoalBlocked,     // goal is in the blocked set
  Unreachable,     // no route exists
  BudgetExceeded,  // options.maxExpansions was reached first; reachability is unknown
};

[[nodiscard]] const char* toString(SearchStatus status) noexcept;

struct SearchResult {
  SearchStatus status = SearchStatus::Unreachable;
  std::int32_t cost = 0;       // number of steps, valid when status == Found
  std::size_t expansions = 0;  // cells expanded; 0 when rejected before searching
};

// A* on the four-connected warehouse grid with unit step costs.
//
// * Heuristic: Manhattan distance to the goal. With unit costs and orthogonal moves it never
//   over-estimates (admissible) and never drops by more than the step cost (consistent), so every
//   cell is expanded at most once and the first route found is a shortest one.
// * Tie-breaking: the open list is ordered by (f, h, cell index) where the index is row-major, and
//   neighbours are tried in the fixed order north, east, south, west. A cell's parent changes only
//   on a strictly cheaper route. The keys are unique, so the result does not depend on the
//   standard library's heap implementation: the same inputs give the same path on every platform.
// * Memory: cost/parent arrays, the blocked-cell marks and the open list live in the planner and
//   are reused. They are invalidated by bumping a generation counter instead of being cleared, so
//   after the first search on a grid of a given size, planning allocates nothing (search() with a
//   reused `path` buffer) or exactly the returned path (findPath()).
//
// A planner is not thread-safe; use one per thread.
class AStarPlanner final : public PathPlanner {
 public:
  // Throws std::invalid_argument if options.directions is empty.
  explicit AStarPlanner(AStarOptions options = {});

  [[nodiscard]] const AStarOptions& options() const noexcept { return options_; }

  [[nodiscard]] std::optional<Path> findPath(const world::Warehouse& warehouse, core::Point start,
                                             core::Point goal, const BlockedSet& blocked) override;

  // Like findPath but reports why a search failed and how much work it did, and writes the route
  // into `path` (cleared first; its capacity is reused). `path` is empty unless the status is
  // Found.
  SearchResult search(const world::Warehouse& warehouse, core::Point start, core::Point goal,
                      const BlockedSet& blocked, Path& path);

 private:
  struct Node {
    std::uint32_t stamp = 0;  // generation in which `cost` and `parent` are valid
    std::int32_t cost = 0;
    std::uint32_t parent = 0;
  };
  struct OpenEntry {
    std::int32_t f;
    std::int32_t h;
    std::uint32_t index;
  };

  void prepare(std::size_t cellCount);
  [[nodiscard]] bool isBlocked(std::size_t index) const noexcept {
    return blockedStamp_[index] == generation_;
  }

  AStarOptions options_;
  std::array<core::Point, 4> offsets_{};
  std::size_t offsetCount_ = 0;

  std::vector<Node> nodes_;
  std::vector<std::uint32_t> blockedStamp_;
  std::vector<OpenEntry> open_;
  std::uint32_t generation_ = 0;
};

}  // namespace warehousesim::planning
