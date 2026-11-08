#pragma once

#include <cstdint>

namespace warehousesim::robots {

// A battery measured in whole integer units, so charge and drain are exact and identical on
// every platform. The level always stays within [0, capacity]; draining below empty or
// charging above full saturates instead of wrapping.
class Battery {
 public:
  using Units = std::uint32_t;

  // Starts full. Throws std::invalid_argument if capacity is 0.
  explicit Battery(Units capacity);
  // Throws std::invalid_argument if capacity is 0 or level > capacity.
  Battery(Units capacity, Units level);

  [[nodiscard]] Units capacity() const noexcept { return capacity_; }
  [[nodiscard]] Units level() const noexcept { return level_; }
  [[nodiscard]] bool isEmpty() const noexcept { return level_ == 0; }
  [[nodiscard]] bool isFull() const noexcept { return level_ == capacity_; }
  // level / capacity in [0, 1], for display.
  [[nodiscard]] double fraction() const noexcept {
    return static_cast<double>(level_) / static_cast<double>(capacity_);
  }

  // Both return the amount actually moved, which is less than requested at the limits.
  Units consume(Units amount) noexcept;
  Units charge(Units amount) noexcept;
  void fill() noexcept { level_ = capacity_; }

  friend bool operator==(const Battery&, const Battery&) = default;

 private:
  Units capacity_;
  Units level_;
};

}  // namespace warehousesim::robots
