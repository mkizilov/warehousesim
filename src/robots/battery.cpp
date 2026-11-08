#include "warehousesim/robots/battery.hpp"

#include <algorithm>
#include <stdexcept>

namespace warehousesim::robots {

Battery::Battery(Units capacity) : Battery(capacity, capacity) {}

Battery::Battery(Units capacity, Units level) : capacity_(capacity), level_(level) {
  if (capacity == 0) {
    throw std::invalid_argument("battery capacity must be positive");
  }
  if (level > capacity) {
    throw std::invalid_argument("battery level exceeds capacity");
  }
}

Battery::Units Battery::consume(Units amount) noexcept {
  const Units drawn = std::min(amount, level_);
  level_ -= drawn;
  return drawn;
}

Battery::Units Battery::charge(Units amount) noexcept {
  const Units added = std::min(amount, capacity_ - level_);
  level_ += added;
  return added;
}

}  // namespace warehousesim::robots
