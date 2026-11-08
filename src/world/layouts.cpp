#include "warehousesim/world/layouts.hpp"

#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace warehousesim::world {

namespace {

template <std::size_t N>
Warehouse parseLayout(const std::array<std::string_view, N>& rows) {
  std::vector<std::string> lines;
  lines.reserve(N);
  for (const std::string_view row : rows) {
    lines.emplace_back(row);
  }
  auto warehouse = Warehouse::fromRows(lines);
  if (!warehouse) {
    throw std::logic_error("predefined warehouse layout is invalid");
  }
  return *std::move(warehouse);
}

constexpr std::array<LayoutInfo, 3> kLayouts{{
    {"demo", "20x12 open hall, shelf blocks, dropoffs along the bottom", &makeDemoWarehouse},
    {"compact", "12x8 walled room with pickups, dropoffs and chargers", &makeCompactWarehouse},
    {"fulfilment", "28x16 walled hall with six shelf rows", &makeFulfilmentWarehouse},
}};

}  // namespace

Warehouse makeDemoWarehouse() {
  return parseLayout(std::array<std::string_view, 12>{
      "....................",
      ".SSSS..SSSS..SSSS...",
      ".SSSS..SSSS..SSSS...",
      "....................",
      ".SSSS..SSSS..SSSS...",
      ".SSSS..SSSS..SSSS...",
      "....................",
      ".SSSS..SSSS..SSSS...",
      ".SSSS..SSSS..SSSS...",
      "....................",
      "....................",
      "D..D..D..D..........",
  });
}

Warehouse makeCompactWarehouse() {
  return parseLayout(std::array<std::string_view, 8>{
      "############",
      "#C.......DD#",
      "#.SSSSSS...#",
      "#..........#",
      "#.SSSSSS...#",
      "#..........#",
      "#PP.......C#",
      "############",
  });
}

Warehouse makeFulfilmentWarehouse() {
  return parseLayout(std::array<std::string_view, 16>{
      "############################",
      "#C..C..C..C..C..C..C..C..C.#",
      "#..........................#",
      "#P.SSSS.SSSS.SSSS.SSSS.SSSS#",
      "#P.SSSS.SSSS.SSSS.SSSS.SSSS#",
      "#..........................#",
      "#P.SSSS.SSSS.SSSS.SSSS.SSSS#",
      "#P.SSSS.SSSS.SSSS.SSSS.SSSS#",
      "#..........................#",
      "#P.SSSS.SSSS.SSSS.SSSS.SSSS#",
      "#P.SSSS.SSSS.SSSS.SSSS.SSSS#",
      "#..........................#",
      "#..........................#",
      "#DD..DD..DD..DD..DD..DD..DD#",
      "#..........................#",
      "############################",
  });
}

std::span<const LayoutInfo> predefinedLayouts() { return kLayouts; }

std::optional<Warehouse> makePredefinedWarehouse(std::string_view name) {
  for (const LayoutInfo& layout : kLayouts) {
    if (layout.name == name) {
      return layout.make();
    }
  }
  return std::nullopt;
}

}  // namespace warehousesim::world
