#include "warehousesim/tasks/distance_estimator.hpp"

#include <algorithm>

namespace warehousesim::tasks {

namespace {

// SplitMix64's finaliser over the four coordinates: the table is a power of two long, so the low
// bits have to depend on all of them.
std::size_t hashOf(core::Point from, core::Point to) noexcept {
  const auto pack = [](core::Point p) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.x)) << 32U) |
           static_cast<std::uint32_t>(p.y);
  };
  std::uint64_t z = pack(from) * 0x9E3779B97F4A7C15ULL + pack(to);
  z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
  return static_cast<std::size_t>(z ^ (z >> 31U));
}

}  // namespace

std::optional<std::int32_t> ManhattanDistance::distance(const world::Warehouse& /*warehouse*/,
                                                        core::Point from, core::Point to) {
  return core::manhattanDistance(from, to);
}

PathDistance::PathDistance(std::size_t maxCacheEntries) : maxCacheEntries_(maxCacheEntries) {}

void PathDistance::reset() noexcept {
  std::fill(slots_.begin(), slots_.end(), Slot{});
  answers_ = 0;
}

PathDistance::Slot* PathDistance::find(core::Point from, core::Point to) noexcept {
  if (slots_.empty()) {
    return nullptr;
  }
  const std::size_t mask = slots_.size() - 1;
  for (std::size_t at = hashOf(from, to) & mask;; at = (at + 1) & mask) {
    Slot& slot = slots_[at];
    if (!slot.used) {
      return &slot;  // where it would go
    }
    if (slot.from == from && slot.to == to) {
      return &slot;
    }
  }
}

void PathDistance::grow() {
  std::vector<Slot> old = std::move(slots_);
  slots_.assign(old.empty() ? 64 : old.size() * 2, Slot{});
  for (const Slot& slot : old) {
    if (slot.used) {
      *find(slot.from, slot.to) = slot;
    }
  }
}

void PathDistance::remember(core::Point from, core::Point to, std::int32_t distance) {
  if (answers_ >= maxCacheEntries_) {
    reset();
  }
  if (slots_.empty() || (answers_ + 1) * 2 > slots_.size()) {
    grow();
  }
  *find(from, to) = Slot{from, to, distance, true};
  ++answers_;
}

// The distances from every cell to `goal`: one sweep out from the goal over the walkable cells.
// Four-connected unit steps, so this is the length of the route A* finds.
const PathDistance::Field& PathDistance::fieldFor(const world::Warehouse& warehouse,
                                                  core::Point goal) {
  for (const Field& field : fields_) {
    if (field.layoutVersion == warehouse.version() && field.goal == goal) {
      return field;
    }
  }
  Field* target = nullptr;
  if (fields_.size() < kMaxFields) {
    target = &fields_.emplace_back();
  } else {
    target = &fields_[nextField_];
    nextField_ = (nextField_ + 1) % kMaxFields;
  }
  const world::GridSize size = warehouse.size();
  target->layoutVersion = warehouse.version();
  target->goal = goal;
  target->distance.assign(size.cellCount(), kUnreachable);
  queue_.clear();
  const std::size_t goalIndex = size.indexOf(goal);
  target->distance[goalIndex] = 0;
  queue_.push_back(static_cast<std::uint32_t>(goalIndex));
  for (std::size_t head = 0; head < queue_.size(); ++head) {
    const std::uint32_t index = queue_[head];
    const core::Point at = size.pointAt(index);
    for (const core::Point offset : core::kCardinalDirections) {
      const core::Point next = at + offset;
      if (!warehouse.isWalkable(next)) {
        continue;
      }
      const std::size_t nextIndex = size.indexOf(next);
      if (target->distance[nextIndex] == kUnreachable) {
        target->distance[nextIndex] = target->distance[index] + 1;
        queue_.push_back(static_cast<std::uint32_t>(nextIndex));
      }
    }
  }
  return *target;
}

std::optional<std::int32_t> PathDistance::distance(const world::Warehouse& warehouse,
                                                   core::Point from, core::Point to) {
  if (const Slot* known = find(from, to); known != nullptr && known->used) {
    return known->distance == kUnreachable ? std::nullopt
                                           : std::optional<std::int32_t>(known->distance);
  }
  std::int32_t answer = kUnreachable;
  if (warehouse.isWalkable(from) && warehouse.isWalkable(to)) {
    answer = fieldFor(warehouse, to).distance[warehouse.size().indexOf(from)];
  }
  remember(from, to, answer);
  return answer == kUnreachable ? std::nullopt : std::optional<std::int32_t>(answer);
}

}  // namespace warehousesim::tasks
