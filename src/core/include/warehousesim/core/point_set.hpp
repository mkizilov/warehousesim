#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <vector>

#include "warehousesim/core/point.hpp"

namespace warehousesim::core {

// A set of grid points that allocates twice however big it gets (one array for the points, one for
// the index), where std::unordered_set<Point> allocates once per point. Building one per question
// ("which cells have robots on them?") is what the planner is asked a lot, so the difference is the
// bulk of the heap traffic of a big fleet.
//
// Iterating gives the points in the order they were first inserted, which is the same on every
// platform (unlike a hash set's). No erase: nothing needs it.
class PointSet {
 public:
  using value_type = Point;
  using const_iterator = std::vector<Point>::const_iterator;
  using iterator = const_iterator;

  PointSet() = default;
  PointSet(std::initializer_list<Point> points) {
    reserve(points.size());
    for (const Point point : points) {
      insert(point);
    }
  }
  template <std::input_iterator It, std::sentinel_for<It> End>
  PointSet(It first, End last) {
    if constexpr (std::forward_iterator<It>) {
      reserve(static_cast<std::size_t>(std::distance(first, last)));
    }
    for (; first != last; ++first) {
      insert(*first);
    }
  }

  // True if the point was not in the set yet.
  bool insert(Point point) {
    if (slots_.empty() || (items_.size() + 1) * 2 > slots_.size()) {
      grow(items_.size() + 1);
    }
    return place(point);
  }

  [[nodiscard]] bool contains(Point point) const noexcept {
    if (items_.empty()) {
      return false;
    }
    const std::size_t mask = slots_.size() - 1;
    for (std::size_t at = hashOf(point) & mask;; at = (at + 1) & mask) {
      const std::uint32_t slot = slots_[at];
      if (slot == 0) {
        return false;
      }
      if (items_[slot - 1] == point) {
        return true;
      }
    }
  }
  [[nodiscard]] std::size_t count(Point point) const noexcept { return contains(point) ? 1U : 0U; }

  [[nodiscard]] bool empty() const noexcept { return items_.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return items_.size(); }
  void clear() noexcept {
    items_.clear();
    std::fill(slots_.begin(), slots_.end(), 0U);
  }
  void reserve(std::size_t count) {
    items_.reserve(count);
    if (count > 0) {
      grow(count);
    }
  }

  [[nodiscard]] const_iterator begin() const noexcept { return items_.begin(); }
  [[nodiscard]] const_iterator end() const noexcept { return items_.end(); }

 private:
  // A well-mixed hash of the two coordinates (SplitMix64's finaliser). std::hash on an integer is
  // the identity in some standard libraries, which a power-of-two table cannot live with.
  [[nodiscard]] static std::size_t hashOf(Point point) noexcept {
    std::uint64_t z = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(point.x)) << 32U) |
                      static_cast<std::uint32_t>(point.y);
    z += 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return static_cast<std::size_t>(z ^ (z >> 31U));
  }

  // Makes room for `count` points at a load of at most one half, and indexes the ones held.
  void grow(std::size_t count) {
    std::size_t capacity = slots_.empty() ? 16 : slots_.size();
    while (capacity < count * 2) {
      capacity *= 2;
    }
    if (capacity == slots_.size()) {
      return;
    }
    slots_.assign(capacity, 0U);
    for (std::size_t i = 0; i < items_.size(); ++i) {
      index(items_[i], static_cast<std::uint32_t>(i + 1));
    }
  }
  void index(Point point, std::uint32_t slotValue) noexcept {
    const std::size_t mask = slots_.size() - 1;
    std::size_t at = hashOf(point) & mask;
    while (slots_[at] != 0) {
      at = (at + 1) & mask;
    }
    slots_[at] = slotValue;
  }
  bool place(Point point) {
    if (contains(point)) {
      return false;
    }
    items_.push_back(point);
    index(point, static_cast<std::uint32_t>(items_.size()));
    return true;
  }

  std::vector<Point> items_;
  std::vector<std::uint32_t> slots_;  // open addressing: index into items_ plus one, 0 for empty
};

}  // namespace warehousesim::core
