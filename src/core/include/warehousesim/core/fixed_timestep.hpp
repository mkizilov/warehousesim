#pragma once

#include <cstdint>

namespace warehousesim::core {

// Converts elapsed real (wall-clock) time into a whole number of fixed-size simulation
// steps, so the simulation advances by identical steps however irregular the caller's frames
// are. It never reads a clock itself: callers pass in the measured elapsed time, which keeps
// it deterministic and testable.
//
//   steps = advance(elapsedRealSeconds)   // 0, 1 or more steps owed
//
// Leftover time below one step is carried over to the next call. Playback speed scales real
// time (2.0 = twice as many steps per real second). If a single call would owe more than
// `maxStepsPerAdvance` steps (e.g. after the process was suspended) the surplus is dropped
// instead of being replayed, so the caller can never fall into a catch-up spiral.
class FixedTimestep {
 public:
  static constexpr double kMinSpeed = 1.0 / 16.0;
  static constexpr double kMaxSpeed = 64.0;
  static constexpr std::uint32_t kDefaultMaxSteps = 200;

  // Throws std::invalid_argument unless stepSeconds is finite and > 0 and
  // maxStepsPerAdvance >= 1.
  explicit FixedTimestep(double stepSeconds, std::uint32_t maxStepsPerAdvance = kDefaultMaxSteps);

  // Clamps a speed multiplier into [kMinSpeed, kMaxSpeed]. Throws std::invalid_argument for
  // NaN, infinite, zero or negative values (pause is a separate concept).
  [[nodiscard]] static double clampSpeed(double speed);

  [[nodiscard]] double stepSeconds() const noexcept { return stepSeconds_; }
  [[nodiscard]] double speed() const noexcept { return speed_; }
  // Same validation and clamping as clampSpeed().
  void setSpeed(double speed);

  // Adds `elapsedRealSeconds` (finite, >= 0, else std::invalid_argument) and returns how
  // many whole steps are now due.
  [[nodiscard]] std::uint32_t advance(double elapsedRealSeconds);

  // Real seconds until advance() would next return at least one step (0 if one is due).
  [[nodiscard]] double secondsUntilNextStep() const noexcept;

  // How far into the next step the carried-over time reaches, in [0, 1). A renderer that draws
  // between steps uses this to interpolate; it never changes what the steps compute.
  [[nodiscard]] double alpha() const noexcept { return accumulatedSeconds_ / stepSeconds_; }
  // The same fraction after `realSeconds` more real time have passed, without consuming them,
  // clamped to [0, 1]. Throws std::invalid_argument unless realSeconds is finite and >= 0.
  [[nodiscard]] double alphaAfter(double realSeconds) const;

  // Forgets carried-over time, e.g. when resuming after a pause, so that the time spent
  // paused is not replayed. `alpha` (clamped to [0, 1], 0 if not finite) is how far into a step
  // the clock should be taken to be, so that a display that froze part-way through a step
  // carries on from there instead of jumping back to its start.
  void reset(double alpha = 0.0) noexcept;

 private:
  double stepSeconds_;
  std::uint32_t maxStepsPerAdvance_;
  double speed_ = 1.0;
  double accumulatedSeconds_ = 0.0;  // simulated seconds not yet consumed by a step
};

}  // namespace warehousesim::core
