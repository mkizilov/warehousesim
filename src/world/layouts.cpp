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

constexpr std::array<LayoutInfo, 4> kLayouts{{
    {"demo", "20x12 open hall, shelf blocks, dropoffs along the bottom", &makeDemoWarehouse},
    {"compact", "12x8 walled room with pickups, dropoffs and chargers", &makeCompactWarehouse},
    {"fulfilment", "28x16 walled hall with six shelf rows", &makeFulfilmentWarehouse},
    {"large", "100x64 walled hall with wide aisles, for fleets of a hundred robots and more",
     &makeLargeWarehouse},
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

// Three rows of chargers along the top (147 of them, every other cell), then twelve bands of
// shelves two rows deep in blocks of six, with two-cell aisles between the blocks and between the
// bands, and 25 dropoffs along a row near the bottom. Aisles are two cells wide so that a large
// fleet is not stuck in the one-cell corridors that the other layouts have.
Warehouse makeLargeWarehouse() {
  constexpr int kWidth = 100;
  constexpr int kHeight = 64;
  std::vector<std::string> rows(kHeight, std::string(kWidth, '.'));
  for (int y = 0; y < kHeight; ++y) {
    rows[static_cast<std::size_t>(y)].front() = '#';
    rows[static_cast<std::size_t>(y)].back() = '#';
  }
  rows.front().assign(kWidth, '#');
  rows.back().assign(kWidth, '#');
  for (int y = 1; y <= 3; ++y) {
    for (int x = 1; x < kWidth - 1; x += 2) {
      rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] = 'C';
    }
  }
  for (int y = 6; y < 54; ++y) {
    if ((y - 6) % 4 >= 2) {
      continue;  // an aisle between two bands
    }
    for (int block = 0; block < 12; ++block) {
      for (int x = 3 + 8 * block; x < 9 + 8 * block; ++x) {
        rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] = 'S';
      }
    }
  }
  for (int x = 2; x < kWidth - 1; x += 4) {
    rows[57][static_cast<std::size_t>(x)] = 'D';
  }
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("predefined warehouse layout is invalid");
  }
  return *std::move(warehouse);
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
