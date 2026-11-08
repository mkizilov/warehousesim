#pragma once

namespace warehousesim::core {

// A continuous position or displacement. Grid code uses integer Points; Vec2 is for things that
// move between cells, such as a robot's pose. In world space the centre of cell (x, y) is
// (x + 0.5, y + 0.5), x grows east and y grows south.
struct Vec2 {
  double x = 0.0;
  double y = 0.0;

  friend constexpr bool operator==(const Vec2&, const Vec2&) = default;
};

[[nodiscard]] constexpr Vec2 operator+(Vec2 a, Vec2 b) noexcept { return {a.x + b.x, a.y + b.y}; }
[[nodiscard]] constexpr Vec2 operator-(Vec2 a, Vec2 b) noexcept { return {a.x - b.x, a.y - b.y}; }
[[nodiscard]] constexpr Vec2 operator*(Vec2 a, double s) noexcept { return {a.x * s, a.y * s}; }

// a at t = 0 and exactly b at t = 1 (no rounding error at the ends), linear in between.
[[nodiscard]] constexpr Vec2 lerp(Vec2 a, Vec2 b, double t) noexcept {
  return {a.x * (1.0 - t) + b.x * t, a.y * (1.0 - t) + b.y * t};
}

}  // namespace warehousesim::core
