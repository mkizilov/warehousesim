#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>

#include "warehousesim/core/point.hpp"
#include "warehousesim/planning/astar_planner.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::tasks {

// How far apart two cells are for the purpose of choosing a robot. nullopt means "cannot get
// there at all". Estimates ignore other robots (they move), so they depend only on the static
// layout.
class DistanceEstimator {
 public:
  DistanceEstimator() = default;
  DistanceEstimator(const DistanceEstimator&) = delete;
  DistanceEstimator& operator=(const DistanceEstimator&) = delete;
  virtual ~DistanceEstimator() = default;

  // Not const: implementations may cache. Not thread-safe.
  [[nodiscard]] virtual std::optional<std::int32_t> distance(const world::Warehouse& warehouse,
                                                             core::Point from, core::Point to) = 0;
};

// Straight-line (Manhattan) distance. Free, but blind to walls, so it never reports "unreachable".
class ManhattanDistance final : public DistanceEstimator {
 public:
  [[nodiscard]] std::optional<std::int32_t> distance(const world::Warehouse& warehouse,
                                                     core::Point from, core::Point to) override;
};

// The length of the shortest route, found with A* and remembered per (from, to) pair, including
// "unreachable" answers. The layout is assumed not to change; call reset() if it does.
class PathDistance final : public DistanceEstimator {
 public:
  // At most `maxCacheEntries` answers are kept; the cache is emptied when it fills up.
  explicit PathDistance(std::size_t maxCacheEntries = 1U << 16);

  [[nodiscard]] std::optional<std::int32_t> distance(const world::Warehouse& warehouse,
                                                     core::Point from, core::Point to) override;

  [[nodiscard]] std::size_t cacheSize() const noexcept { return cache_.size(); }
  void reset() { cache_.clear(); }

 private:
  struct Key {
    core::Point from;
    core::Point to;
    friend bool operator==(const Key&, const Key&) = default;
  };
  struct KeyHash {
    [[nodiscard]] std::size_t operator()(const Key& key) const noexcept {
      const core::PointHash hash;
      return hash(key.from) * 0x9E3779B97F4A7C15ULL ^ hash(key.to);
    }
  };

  static constexpr std::int32_t kUnreachable = -1;

  std::size_t maxCacheEntries_;
  planning::AStarPlanner planner_;
  planning::Path scratch_;
  std::unordered_map<Key, std::int32_t, KeyHash> cache_;
};

}  // namespace warehousesim::tasks
