#include "warehousesim/simulation/scenarios.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/world/layout_check.hpp"
#include "warehousesim/world/layouts.hpp"

namespace warehousesim::simulation {

namespace {

constexpr std::size_t kMaxRobots = 4;
constexpr core::Tick kTaskSpawnInterval = 5;

// Robots take two ticks to load and two to unload, and drive like vehicles rather than like
// sprites: they speed up at half a cell per tick squared, brake at one (so they stop within half
// a cell from top speed), and stop at every corner to turn a quarter turn per tick.
robots::RobotSpec handlingSpec() {
  robots::RobotSpec spec;
  spec.acceleration = 0.5;
  spec.deceleration = 1.0;
  spec.turnRate = core::kPi / 2.0;
  spec.loadTicks = 2;
  spec.unloadTicks = 2;
  return spec;
}

// Adds an energy model on top of handlingSpec(). With a 10000-unit battery a robot can travel
// about 460 cells above its 30% reserve, then needs about 28 ticks on a charger. The reserve is
// larger than the energy of the longest task in the bundled layouts, so a robot that checks its
// battery between tasks always reaches a charger.
robots::RobotSpec energySpec() {
  robots::RobotSpec spec = handlingSpec();
  spec.idleDrainPerTick = 1;
  spec.moveCostPerCell = 15;
  spec.handlingDrainPerTick = 2;
  spec.chargePerTick = 250;
  spec.lowBatteryLevel = 3000;
  return spec;
}

SimulationConfig scenarioConfig(std::uint64_t seed, const robots::RobotSpec& robot) {
  SimulationConfig config;
  config.seed = seed;
  config.taskSpawnInterval = kTaskSpawnInterval;
  config.varyTaskPriority = true;
  config.robot = robot;
  return config;
}

// Where the robots of a layout start. On charging stations if it has any (up to four of them).
// Otherwise on open floor, in reading order, each at least two cells from the one before: on the
// top row of the demo hall that gives x = 0, 2, 4 and 6. A layout with no open floor falls back on
// its pickups and dropoffs.
std::vector<core::Point> startCells(const world::Warehouse& warehouse) {
  std::vector<core::Point> starts = warehouse.cellsOfType(world::CellType::Charging);
  if (!starts.empty()) {
    starts.resize(std::min(starts.size(), kMaxRobots));
    return starts;
  }
  for (const world::CellType type :
       {world::CellType::Free, world::CellType::Pickup, world::CellType::Dropoff}) {
    for (const core::Point cell : warehouse.cellsOfType(type)) {
      const bool touching = std::any_of(starts.begin(), starts.end(), [cell](core::Point other) {
        return core::manhattanDistance(other, cell) <= 1;
      });
      if (!touching && starts.size() < kMaxRobots) {
        starts.push_back(cell);
      }
    }
    if (!starts.empty()) {
      break;
    }
  }
  return starts;
}

}  // namespace

Simulation makeMapSimulation(world::Warehouse warehouse, std::uint64_t seed) {
  const std::vector<world::LayoutIssue> issues = world::checkLayout(warehouse);
  const auto blocking =
      std::find_if(issues.begin(), issues.end(),
                   [](const world::LayoutIssue& issue) { return issue.blocksSimulation; });
  if (blocking != issues.end()) {
    throw std::invalid_argument("the layout cannot host a simulation: " + blocking->message);
  }
  // Without charging stations robots have no battery model (they could never recharge).
  const bool chargers = !warehouse.cellsOfType(world::CellType::Charging).empty();
  const std::vector<core::Point> starts = startCells(warehouse);
  return Simulation(std::move(warehouse), starts,
                    scenarioConfig(seed, chargers ? energySpec() : handlingSpec()));
}

Simulation makeDemoSimulation(std::uint64_t seed) {
  return makeMapSimulation(world::makeDemoWarehouse(), seed);
}

std::optional<Simulation> makeLayoutSimulation(std::string_view layoutName, std::uint64_t seed) {
  auto warehouse = world::makePredefinedWarehouse(layoutName);
  if (!warehouse) {
    return std::nullopt;
  }
  return makeMapSimulation(*std::move(warehouse), seed);
}

}  // namespace warehousesim::simulation
