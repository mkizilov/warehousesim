#pragma once

#include <optional>
#include <span>
#include <string_view>

#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::world {

// Predefined layouts for development and tests. Every one is a single connected walkable
// region with at least one dropoff and one source of goods (a Pickup cell, or failing that a
// Free cell next to a shelf).
struct LayoutInfo {
  std::string_view name;
  std::string_view description;
  Warehouse (*make)();
};

// "demo": 20x12 open hall with shelf blocks and dropoffs along the bottom edge. No walls,
//         pickups or chargers; goods are collected from the aisles beside the shelves.
[[nodiscard]] Warehouse makeDemoWarehouse();
// "compact": 12x8 walled room with two shelf rows, pickups, dropoffs and chargers.
[[nodiscard]] Warehouse makeCompactWarehouse();
// "fulfilment": 28x16 walled hall with six shelf rows, a pickup column, a dropoff row and
//               a row of chargers.
[[nodiscard]] Warehouse makeFulfilmentWarehouse();

[[nodiscard]] std::span<const LayoutInfo> predefinedLayouts();
// nullopt when no layout has that name.
[[nodiscard]] std::optional<Warehouse> makePredefinedWarehouse(std::string_view name);

}  // namespace warehousesim::world
