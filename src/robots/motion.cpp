#include "warehousesim/robots/motion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace warehousesim::robots {

namespace {

// Distances and times below this are rounding noise, not motion.
constexpr double kEpsilon = 1e-9;

void requireFinite(double value, const char* what) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(what) + " must be finite and non-negative");
  }
}

void requirePositive(const std::optional<double>& value, const char* what) {
  if (value && !(std::isfinite(*value) && *value > 0.0)) {
    throw std::invalid_argument(std::string(what) + " must be finite and positive");
  }
}

}  // namespace

const char* toString(MotionPhase phase) noexcept {
  switch (phase) {
    case MotionPhase::Stopped:
      return "stopped";
    case MotionPhase::Turning:
      return "turning";
    case MotionPhase::Accelerating:
      return "accelerating";
    case MotionPhase::Cruising:
      return "cruising";
    case MotionPhase::Braking:
      return "braking";
  }
  return "unknown";
}

double MotionLimits::brakingDistance(double speed) const noexcept {
  return deceleration ? speed * speed / (2.0 * *deceleration) : 0.0;
}

MotionStep advanceAlongRun(const MotionLimits& limits, double speed, double distanceToStop,
                           double dt) {
  requireFinite(speed, "speed");
  requireFinite(distanceToStop, "distance to stop");
  requireFinite(dt, "time step");
  if (!std::isfinite(limits.maxSpeed) || limits.maxSpeed <= 0.0) {
    throw std::invalid_argument("maxSpeed must be finite and positive");
  }
  requirePositive(limits.acceleration, "acceleration");
  requirePositive(limits.deceleration, "deceleration");

  MotionStep step;
  step.speed = speed;
  if (dt <= 0.0) {
    return step;
  }

  const bool brakes = limits.deceleration.has_value();
  if (distanceToStop <= kEpsilon) {
    step.arrived = true;
    step.speed = 0.0;
    step.speedOnArrival = speed;
    step.exceededDeceleration = brakes && speed > kEpsilon;
    return step;
  }

  // Too fast to stop in time: stop exactly at the line anyway, with the smallest deceleration
  // that does it, and report that the nominal one was not enough.
  if (brakes && limits.brakingDistance(speed) > distanceToStop + kEpsilon) {
    step.exceededDeceleration = true;
    const double stopTime = 2.0 * distanceToStop / speed;
    const double hardDeceleration = speed / stopTime;
    if (dt >= stopTime) {
      step.distance = distanceToStop;
      step.speed = 0.0;
      step.arrived = true;
      step.timeUsed = stopTime;
      return step;
    }
    step.distance = speed * dt - 0.5 * hardDeceleration * dt * dt;
    step.speed = speed - hardDeceleration * dt;
    step.timeUsed = dt;
    return step;
  }

  // The highest speed reached on this run: maxSpeed unless the braking curve cuts it short
  // (accelerating from `speed` and braking to rest has to fit in the distance).
  double peak = limits.maxSpeed;
  if (brakes) {
    const double d = *limits.deceleration;
    const double curve =
        limits.acceleration
            ? std::sqrt((2.0 * *limits.acceleration * d * distanceToStop + d * speed * speed) /
                        (*limits.acceleration + d))
            : std::sqrt(2.0 * d * distanceToStop);
    peak = std::min(peak, curve);
  }
  peak = std::max(peak, speed);

  // Three phases: speed up to the peak, cruise, brake to rest.
  double accelTime = 0.0;
  double accelDistance = 0.0;
  if (limits.acceleration && peak > speed) {
    accelTime = (peak - speed) / *limits.acceleration;
    accelDistance = (peak * peak - speed * speed) / (2.0 * *limits.acceleration);
  }
  double brakeTime = 0.0;
  double brakeDistance = 0.0;
  if (brakes) {
    brakeTime = peak / *limits.deceleration;
    brakeDistance = limits.brakingDistance(peak);
  }
  const double cruiseDistance = std::max(0.0, distanceToStop - accelDistance - brakeDistance);
  const double cruiseTime = cruiseDistance / peak;
  const double total = accelTime + cruiseTime + brakeTime;

  if (dt >= total - kEpsilon) {
    step.distance = distanceToStop;
    step.speed = 0.0;
    step.arrived = true;
    step.timeUsed = std::min(total, dt);
    step.speedOnArrival = brakes ? 0.0 : peak;
    return step;
  }

  step.timeUsed = dt;
  if (dt <= accelTime) {
    const double a = *limits.acceleration;
    step.speed = speed + a * dt;
    step.distance = speed * dt + 0.5 * a * dt * dt;
  } else if (dt <= accelTime + cruiseTime) {
    step.speed = peak;
    step.distance = accelDistance + peak * (dt - accelTime);
  } else {
    const double d = *limits.deceleration;
    const double tau = dt - accelTime - cruiseTime;
    step.speed = peak - d * tau;
    step.distance = accelDistance + cruiseDistance + peak * tau - 0.5 * d * tau * tau;
  }
  step.distance = std::clamp(step.distance, 0.0, distanceToStop);
  step.speed = std::clamp(step.speed, 0.0, peak);
  return step;
}

}  // namespace warehousesim::robots
