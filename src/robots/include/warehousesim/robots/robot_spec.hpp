#pragma once

#include <cstdint>
#include <optional>

#include "warehousesim/robots/battery.hpp"

namespace warehousesim::robots {

// The fixed physical characteristics of a robot model. Time is measured in simulation ticks
// and distance in grid cells. The defaults describe an idealised robot: it moves one cell per
// tick, loads and unloads instantly and uses no energy, which reproduces the behaviour the
// simulation had before robots had batteries and loading phases.
struct RobotSpec {
  // Cells per tick, in (0, 1]: a robot enters at most one cell per tick. A speed of 0.5 enters
  // a cell every second tick.
  double maxSpeed = 1.0;

  Battery::Units batteryCapacity = 10000;
  // Starting charge; the battery starts full when unset.
  std::optional<Battery::Units> initialCharge;

  // Energy drawn per tick while Idle, per cell entered, and per tick while Loading/Unloading.
  Battery::Units idleDrainPerTick = 0;
  Battery::Units moveCostPerCell = 0;
  Battery::Units handlingDrainPerTick = 0;
  // Energy gained per tick while Charging. Must be positive.
  Battery::Units chargePerTick = 500;
  // A robot whose charge is strictly below this wants to recharge (0 = never).
  Battery::Units lowBatteryLevel = 0;

  // Ticks spent in Loading / Unloading. Zero means the phase completes in the tick it starts.
  std::uint32_t loadTicks = 0;
  std::uint32_t unloadTicks = 0;

  // Throws std::invalid_argument describing the first invalid field.
  void validate() const;
};

}  // namespace warehousesim::robots
