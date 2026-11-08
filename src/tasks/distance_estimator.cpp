#include "warehousesim/tasks/distance_estimator.hpp"

namespace warehousesim::tasks {

std::optional<std::int32_t> ManhattanDistance::distance(const world::Warehouse& /*warehouse*/,
                                                        core::Point from, core::Point to) {
  return core::manhattanDistance(from, to);
}

PathDistance::PathDistance(std::size_t maxCacheEntries) : maxCacheEntries_(maxCacheEntries) {}

std::optional<std::int32_t> PathDistance::distance(const world::Warehouse& warehouse,
                                                   core::Point from, core::Point to) {
  const Key key{from, to};
  if (const auto it = cache_.find(key); it != cache_.end()) {
    return it->second == kUnreachable ? std::nullopt : std::optional<std::int32_t>(it->second);
  }
  const planning::SearchResult result = planner_.search(warehouse, from, to, {}, scratch_);
  const bool found = result.status == planning::SearchStatus::Found;
  if (cache_.size() >= maxCacheEntries_) {
    cache_.clear();
  }
  cache_.emplace(key, found ? result.cost : kUnreachable);
  return found ? std::optional<std::int32_t>(result.cost) : std::nullopt;
}

}  // namespace warehousesim::tasks
