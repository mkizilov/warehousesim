#pragma once

#include <cstdint>

namespace warehousesim::core {

using Tick = std::uint64_t;

// Simulated time advances only through advance(); it never reads wall-clock time.
class SimClock {
 public:
  explicit SimClock(double secondsPerTick = 0.1);

  [[nodiscard]] Tick tick() const noexcept { return tick_; }
  [[nodiscard]] double secondsPerTick() const noexcept { return secondsPerTick_; }
  [[nodiscard]] double seconds() const noexcept {
    return static_cast<double>(tick_) * secondsPerTick_;
  }

  void advance() noexcept { ++tick_; }

 private:
  double secondsPerTick_;
  Tick tick_ = 0;
};

}  // namespace warehousesim::core
