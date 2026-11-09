#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "warehousesim/core/point.hpp"
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

// The length of the shortest route, remembered per (from, to) pair, including "unreachable"
// answers. The layout is assumed not to change; call reset() if it does (an answer that has been
// given stays the answer until then).
//
// An answer is the length A* would find on the empty map. It is read off the distances from the
// goal to every cell, which one sweep of the map gives for all the starts a task is offered to at
// once, instead of one A* search for each of them: the sweep for a goal is kept for the next few
// questions about it.
class PathDistance final : public DistanceEstimator {
 public:
  // At most `maxCacheEntries` answers are kept; the cache is emptied when it fills up.
  explicit PathDistance(std::size_t maxCacheEntries = 1U << 16);

  [[nodiscard]] std::optional<std::int32_t> distance(const world::Warehouse& warehouse,
                                                     core::Point from, core::Point to) override;

  [[nodiscard]] std::size_t cacheSize() const noexcept { return answers_; }
  void reset() noexcept;

 private:
  static constexpr std::int32_t kUnreachable = -1;
  static constexpr std::size_t kMaxFields = 32;

  struct Slot {
    core::Point from;
    core::Point to;
    std::int32_t distance = kUnreachable;
    bool used = false;
  };
  // The distance from every cell to `goal` on the layout as it was when the field was made.
  struct Field {
    std::uint64_t layoutVersion = 0;
    core::Point goal;
    std::vector<std::int32_t> distance;  // -1: cannot reach the goal
  };

  [[nodiscard]] Slot* find(core::Point from, core::Point to) noexcept;
  void remember(core::Point from, core::Point to, std::int32_t distance);
  void grow();
  [[nodiscard]] const Field& fieldFor(const world::Warehouse& warehouse, core::Point goal);

  std::size_t maxCacheEntries_;
  std::vector<Slot> slots_;  // open addressing; a power of two long, at most half full
  std::size_t answers_ = 0;
  std::vector<Field> fields_;
  std::size_t nextField_ = 0;  // the one to replace when all kMaxFields are in use
  std::vector<std::uint32_t> queue_;
};

}  // namespace warehousesim::tasks
