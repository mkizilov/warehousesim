#pragma once

#include <cstdint>
#include <optional>

namespace warehousesim::robots {

// What a robot's last step of motion amounted to.
enum class MotionPhase : std::uint8_t {
  Stopped,       // at rest
  Turning,       // rotating on the spot, not yet moving
  Accelerating,  // speeding up
  Cruising,      // holding a constant speed
  Braking,       // slowing down, including the step that ends in a stop
};

[[nodiscard]] const char* toString(MotionPhase phase) noexcept;

// Longitudinal limits of a robot driving along a straight line. Distance is in cells and time in
// ticks. An unset acceleration or deceleration means the speed can change instantly, which is the
// idealised robot that moves at maxSpeed the moment it starts and stops the moment it arrives.
struct MotionLimits {
  double maxSpeed = 1.0;
  std::optional<double> acceleration;  // cells per tick squared
  std::optional<double> deceleration;  // cells per tick squared

  // Distance needed to stop from `speed` at the nominal deceleration (0 if braking is instant).
  [[nodiscard]] double brakingDistance(double speed) const noexcept;
};

// What one fixed step of motion did.
struct MotionStep {
  double distance = 0.0;  // covered along the line; never more than the distance to the stop point
  double speed = 0.0;     // speed at the end of the step
  bool arrived = false;   // the stop point was reached during the step (speed is then 0)
  // Time spent moving: dt, or less if the robot arrived early and then stood still.
  double timeUsed = 0.0;
  // Speed just before stopping on arrival. With instant braking this is the speed it was
  // driving at, which lets a caller tell "ran out of authority" from "was already crawling".
  double speedOnArrival = 0.0;
  // The stop point was closer than the nominal deceleration allows. The step still stops
  // exactly there by braking harder, and says so. A caller that keeps a stopping distance in
  // hand never sees this.
  bool exceededDeceleration = false;
};

// Advances a robot that must be at rest at a stop point `distanceToStop` ahead, for `dt` ticks.
//
// It follows the fastest profile the limits allow: accelerate, cruise at maxSpeed, brake so as
// to come to rest exactly at the stop point. The profile is closed-form, so the result is the
// same however the time is split (one step of 1.0 or ten of 0.1 give the same distance and
// speed, up to rounding), which is what makes the motion independent of the step size.
//
// Throws std::invalid_argument for a non-finite or negative speed, distance or dt, or a
// non-positive maxSpeed, acceleration or deceleration.
[[nodiscard]] MotionStep advanceAlongRun(const MotionLimits& limits, double speed,
                                         double distanceToStop, double dt);

}  // namespace warehousesim::robots
