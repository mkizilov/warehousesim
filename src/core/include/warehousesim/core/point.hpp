#pragma once

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>

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

struct PointHash {
  [[nodiscard]] std::size_t operator()(Point p) const noexcept {
    const std::uint64_t packed =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.x)) << 32U) |
        static_cast<std::uint32_t>(p.y);
    return std::hash<std::uint64_t>{}(packed);
  }
};

}  // namespace warehousesim::core
