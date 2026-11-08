#include "warehousesim/robots/robot_spec.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace warehousesim::robots {

namespace {

// So that maxSpeed 1 with deceleration 0.5 (exactly one cell to stop) is accepted.
constexpr double kBrakingTolerance = 1e-9;

}  // namespace

void RobotSpec::validate() const {
  if (!std::isfinite(maxSpeed) || maxSpeed <= 0.0 || maxSpeed > 1.0) {
    throw std::invalid_argument("maxSpeed must be in (0, 1] cells per tick");
  }
  for (const auto& [value, name] :
       {std::pair{acceleration, "acceleration"}, std::pair{deceleration, "deceleration"},
        std::pair{turnRate, "turnRate"}}) {
    if (value && !(std::isfinite(*value) && *value > 0.0)) {
      throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
  }
  if (deceleration && motionLimits().brakingDistance(maxSpeed) > 1.0 + kBrakingTolerance) {
    throw std::invalid_argument(
        "deceleration is too weak: stopping from maxSpeed would take more than one cell");
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
