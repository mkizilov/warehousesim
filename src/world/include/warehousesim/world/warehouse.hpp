#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/point.hpp"

namespace warehousesim::world {

enum class CellType : std::uint8_t {
  Floor,    // '.' walkable aisle
  Shelf,    // 'S' storage, not walkable; robots pick up from adjacent floor
  Station,  // 'D' drop-off station, walkable
  Wall,     // '#' not walkable
};

[[nodiscard]] char toChar(CellType type) noexcept;
[[nodiscard]] std::optional<CellType> cellTypeFromChar(char c) noexcept;

// Rectangular grid with the origin at the top-left; x grows right, y grows down.
class Warehouse {
 public:
  // Creates an all-floor warehouse. Throws std::invalid_argument for non-positive sizes.
  Warehouse(std::int32_t width, std::int32_t height);

  // Parses one string per row using the characters of CellType. Returns nullopt for an
  // empty, ragged, or unrecognised layout.
  [[nodiscard]] static std::optional<Warehouse> fromRows(const std::vector<std::string>& rows);

  [[nodiscard]] std::int32_t width() const noexcept { return width_; }
  [[nodiscard]] std::int32_t height() const noexcept { return height_; }

  [[nodiscard]] bool inBounds(core::Point p) const noexcept;
  [[nodiscard]] bool isWalkable(core::Point p) const noexcept;

  // Throw std::out_of_range when p is outside the grid.
  [[nodiscard]] CellType at(core::Point p) const;
  void set(core::Point p, CellType type);

  // Cells of the given type in row-major order.
  [[nodiscard]] std::vector<core::Point> cellsOfType(CellType type) const;

  // Floor cells orthogonally adjacent to a shelf, in row-major order.
  [[nodiscard]] std::vector<core::Point> shelfAccessPoints() const;

  [[nodiscard]] std::vector<std::string> toRows() const;

 private:
  [[nodiscard]] std::size_t index(core::Point p) const noexcept;

  std::int32_t width_;
  std::int32_t height_;
  std::vector<CellType> cells_;
};

[[nodiscard]] Warehouse makeDemoWarehouse();

}  // namespace warehousesim::world
