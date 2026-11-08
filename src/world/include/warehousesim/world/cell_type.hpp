#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace warehousesim::world {

// What a grid cell is made of. This is static layout only; robots are tracked separately
// by Occupancy.
enum class CellType : std::uint8_t {
  Free,      // '.' open floor
  Wall,      // '#' static obstacle
  Shelf,     // 'S' static obstacle; robots work from the adjacent cells
  Pickup,    // 'P' walkable location where goods are collected
  Dropoff,   // 'D' walkable location where goods are delivered
  Charging,  // 'C' walkable charging station
};

inline constexpr std::array<CellType, 6> kAllCellTypes{CellType::Free,    CellType::Wall,
                                                       CellType::Shelf,   CellType::Pickup,
                                                       CellType::Dropoff, CellType::Charging};

// Wall and Shelf can never be entered.
[[nodiscard]] constexpr bool isObstacle(CellType type) noexcept {
  return type == CellType::Wall || type == CellType::Shelf;
}

[[nodiscard]] constexpr bool isWalkable(CellType type) noexcept { return !isObstacle(type); }

[[nodiscard]] char toChar(CellType type) noexcept;
[[nodiscard]] std::optional<CellType> cellTypeFromChar(char c) noexcept;
[[nodiscard]] const char* toString(CellType type) noexcept;

}  // namespace warehousesim::world
