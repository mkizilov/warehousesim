#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "warehousesim/core/point.hpp"

namespace warehousesim::core {

// A set of the four compass directions, stored in one byte.
class DirectionSet {
 public:
  constexpr DirectionSet() noexcept = default;
  constexpr DirectionSet(std::initializer_list<Direction> directions) noexcept {
    for (const Direction direction : directions) {
      bits_ = static_cast<std::uint8_t>(bits_ | bit(direction));
    }
  }

  [[nodiscard]] static constexpr DirectionSet all() noexcept {
    return {Direction::North, Direction::East, Direction::South, Direction::West};
  }

  [[nodiscard]] constexpr DirectionSet with(Direction direction) const noexcept {
    DirectionSet result = *this;
    result.bits_ = static_cast<std::uint8_t>(bits_ | bit(direction));
    return result;
  }
  [[nodiscard]] constexpr DirectionSet without(Direction direction) const noexcept {
    DirectionSet result = *this;
    result.bits_ = static_cast<std::uint8_t>(bits_ & ~bit(direction));
    return result;
  }

  [[nodiscard]] constexpr bool contains(Direction direction) const noexcept {
    return (bits_ & bit(direction)) != 0;
  }
  [[nodiscard]] constexpr bool empty() const noexcept { return bits_ == 0; }
  [[nodiscard]] constexpr std::size_t size() const noexcept {
    std::size_t count = 0;
    for (const Direction direction : kAllDirections) {
      count += contains(direction) ? 1U : 0U;
    }
    return count;
  }

  friend constexpr bool operator==(const DirectionSet&, const DirectionSet&) = default;

 private:
  [[nodiscard]] static constexpr std::uint8_t bit(Direction direction) noexcept {
    return static_cast<std::uint8_t>(1U << static_cast<unsigned>(direction));
  }

  std::uint8_t bits_ = 0;
};

}  // namespace warehousesim::core
