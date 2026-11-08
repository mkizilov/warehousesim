#include "warehousesim/core/clock.hpp"

#include <stdexcept>

namespace warehousesim::core {

SimClock::SimClock(double secondsPerTick) : secondsPerTick_(secondsPerTick) {
  if (!(secondsPerTick > 0.0)) {
    throw std::invalid_argument("secondsPerTick must be positive");
  }
}

}  // namespace warehousesim::core
