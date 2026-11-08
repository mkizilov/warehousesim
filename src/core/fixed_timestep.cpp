#include "warehousesim/core/fixed_timestep.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace warehousesim::core {

namespace {

// Absorbs floating-point error so that e.g. 0.3 s of 0.1 s steps is three steps, not two.
constexpr double kStepTolerance = 1e-9;

}  // namespace

FixedTimestep::FixedTimestep(double stepSeconds, std::uint32_t maxStepsPerAdvance)
    : stepSeconds_(stepSeconds), maxStepsPerAdvance_(maxStepsPerAdvance) {
  if (!std::isfinite(stepSeconds) || stepSeconds <= 0.0) {
    throw std::invalid_argument("step duration must be finite and positive");
  }
  if (maxStepsPerAdvance == 0) {
    throw std::invalid_argument("maxStepsPerAdvance must be at least 1");
  }
}

double FixedTimestep::clampSpeed(double speed) {
  if (!std::isfinite(speed) || speed <= 0.0) {
    throw std::invalid_argument("speed must be finite and positive");
  }
  return std::clamp(speed, kMinSpeed, kMaxSpeed);
}

void FixedTimestep::setSpeed(double speed) { speed_ = clampSpeed(speed); }

std::uint32_t FixedTimestep::advance(double elapsedRealSeconds) {
  if (!std::isfinite(elapsedRealSeconds) || elapsedRealSeconds < 0.0) {
    throw std::invalid_argument("elapsed time must be finite and non-negative");
  }
  accumulatedSeconds_ += elapsedRealSeconds * speed_;
  const double due = std::floor(accumulatedSeconds_ / stepSeconds_ + kStepTolerance);
  if (due > static_cast<double>(maxStepsPerAdvance_)) {
    accumulatedSeconds_ = 0.0;
    return maxStepsPerAdvance_;
  }
  const auto steps = static_cast<std::uint32_t>(due);
  accumulatedSeconds_ = std::max(0.0, accumulatedSeconds_ - due * stepSeconds_);
  return steps;
}

double FixedTimestep::secondsUntilNextStep() const noexcept {
  const double remaining = stepSeconds_ - accumulatedSeconds_;
  return remaining <= kStepTolerance * stepSeconds_ ? 0.0 : remaining / speed_;
}

}  // namespace warehousesim::core
