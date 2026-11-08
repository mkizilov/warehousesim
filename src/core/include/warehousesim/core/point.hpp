#pragma once

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>

namespace warehousesim::core {

struct Point {
  std::int32_t x = 0;
  std::int32_t y = 0;

  friend constexpr auto operator<=>(const Point&, const Point&) = default;
};

[[nodiscard]] constexpr Point operator+(Point a, Point b) noexcept {
  return {a.x + b.x, a.y + b.y};
}

[[nodiscard]] constexpr std::int32_t manhattanDistance(Point a, Point b) noexcept {
  const std::int32_t dx = a.x > b.x ? a.x - b.x : b.x - a.x;
  const std::int32_t dy = a.y > b.y ? a.y - b.y : b.y - a.y;
  return dx + dy;
}

// Fixed order (up, right, down, left) keeps neighbour expansion deterministic.
inline constexpr std::array<Point, 4> kCardinalDirections{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

// Grid orientation: x grows east (right), y grows south (down). The enumerators are in the
// same order as kCardinalDirections.
enum class Direction : std::uint8_t { North, East, South, West };

inline constexpr std::array<Direction, 4> kAllDirections{Direction::North, Direction::East,
                                                         Direction::South, Direction::West};

[[nodiscard]] constexpr Point offsetOf(Direction direction) noexcept {
  return kCardinalDirections[static_cast<std::size_t>(direction)];
}

// The direction of travel from `from` to an orthogonally adjacent `to`, or nullopt when the
// two points are equal, diagonal, or further apart.
[[nodiscard]] constexpr std::optional<Direction> directionBetween(Point from, Point to) noexcept {
  if (manhattanDistance(from, to) != 1) {
    return std::nullopt;
  }
  for (const Direction direction : kAllDirections) {
    if (from + offsetOf(direction) == to) {
      return direction;
    }
  }
  return std::nullopt;
}

struct PointHash {
  [[nodiscard]] std::size_t operator()(Point p) const noexcept {
    const std::uint64_t packed =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.x)) << 32U) |
        static_cast<std::uint32_t>(p.y);
    return std::hash<std::uint64_t>{}(packed);
  }
};

}  // namespace warehousesim::core
