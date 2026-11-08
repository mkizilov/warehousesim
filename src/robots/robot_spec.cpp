#include "warehousesim/robots/robot_spec.hpp"

#include <cmath>
#include <stdexcept>

namespace warehousesim::robots {

void RobotSpec::validate() const {
  if (!std::isfinite(maxSpeed) || maxSpeed <= 0.0 || maxSpeed > 1.0) {
    throw std::invalid_argument("maxSpeed must be in (0, 1] cells per tick");
  }
  if (batteryCapacity == 0) {
    throw std::invalid_argument("batteryCapacity must be positive");
  }
  if (initialCharge && *initialCharge > batteryCapacity) {
    throw std::invalid_argument("initialCharge exceeds batteryCapacity");
  }
  if (chargePerTick == 0) {
    throw std::invalid_argument("chargePerTick must be positive");
  }
  if (lowBatteryLevel >= batteryCapacity) {
    throw std::invalid_argument("lowBatteryLevel must be below batteryCapacity");
  }
}

}  // namespace warehousesim::robots
