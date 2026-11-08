#pragma once

#include <cmath>
#include <numbers>

#include "warehousesim/core/point.hpp"

namespace warehousesim::core {

inline constexpr double kPi = std::numbers::pi;
// Two angles closer than this are the same heading.
inline constexpr double kAngleEpsilon = 1e-9;

// Angles are in radians, measured from east (+x) towards south (+y), i.e. clockwise on screen:
// East 0, South pi/2, West pi, North -pi/2.
[[nodiscard]] constexpr double angleOf(Direction direction) noexcept {
  switch (direction) {
    case Direction::North:
      return -kPi / 2.0;
    case Direction::East:
      return 0.0;
    case Direction::South:
      return kPi / 2.0;
    case Direction::West:
      return kPi;
  }
  return 0.0;
}

// Wraps any finite angle into (-pi, pi].
[[nodiscard]] inline double wrapAngle(double angle) noexcept {
  double wrapped = std::fmod(angle, 2.0 * kPi);
  if (wrapped > kPi) {
    wrapped -= 2.0 * kPi;
  } else if (wrapped <= -kPi) {
    wrapped += 2.0 * kPi;
  }
  return wrapped;
}

// The signed rotation of smallest magnitude that takes `from` to `to`, in (-pi, pi]. Exact
// opposites give +pi, so a U-turn always goes clockwise and never depends on rounding.
[[nodiscard]] inline double shortestRotation(double from, double to) noexcept {
  return wrapAngle(to - from);
}

// Interpolates along the shortest rotation; exactly `from` at t = 0 and exactly `to` at t = 1.
[[nodiscard]] inline double lerpAngle(double from, double to, double t) noexcept {
  if (t <= 0.0) {
    return from;
  }
  if (t >= 1.0) {
    return to;
  }
  return wrapAngle(from + shortestRotation(from, to) * t);
}

// The cardinal direction closest to `angle`; ties go to the first of north, east, south, west.
[[nodiscard]] inline Direction nearestDirection(double angle) noexcept {
  Direction best = Direction::North;
  double bestDistance = std::fabs(shortestRotation(angle, angleOf(Direction::North)));
  for (const Direction candidate : {Direction::East, Direction::South, Direction::West}) {
    const double distance = std::fabs(shortestRotation(angle, angleOf(candidate)));
    if (distance < bestDistance) {
      best = candidate;
      bestDistance = distance;
    }
  }
  return best;
}

}  // namespace warehousesim::core
