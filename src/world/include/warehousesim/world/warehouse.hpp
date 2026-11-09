#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/world/cell_type.hpp"
#include "warehousesim/world/grid_size.hpp"

namespace warehousesim::world {

// The static warehouse layout: a rectangular grid of CellTypes with the origin at the
// top-left (x grows east, y grows south). It knows nothing about robots; dynamic occupancy
// lives in Occupancy and is combined with the layout by World.
class Warehouse {
 public:
  // Creates an all-Free warehouse.
  explicit Warehouse(GridSize size);
  // Throws std::invalid_argument for non-positive sizes.
  Warehouse(std::int32_t width, std::int32_t height);

  // Parses one string per row using the characters of CellType (see toChar). Returns
  // nullopt for an empty, ragged, or unrecognised layout.
  [[nodiscard]] static std::optional<Warehouse> fromRows(const std::vector<std::string>& rows);

  [[nodiscard]] GridSize size() const noexcept { return size_; }
  [[nodiscard]] std::int32_t width() const noexcept { return size_.width(); }
  [[nodiscard]] std::int32_t height() const noexcept { return size_.height(); }

  [[nodiscard]] bool inBounds(core::Point p) const noexcept { return size_.contains(p); }
  // In bounds and not a wall or shelf.
  [[nodiscard]] bool isWalkable(core::Point p) const noexcept;
  // In bounds and a wall or shelf. Out-of-bounds points are neither walkable nor obstacles.
  [[nodiscard]] bool isObstacle(core::Point p) const noexcept;

  // Throw std::out_of_range when p is outside the grid.
  [[nodiscard]] CellType at(core::Point p) const;
  void set(core::Point p, CellType type);

  // In-bounds orthogonal neighbours in north, east, south, west order; empty when p itself
  // is out of bounds.
  [[nodiscard]] std::vector<core::Point> neighbors(core::Point p) const;
  // The subset of neighbors() that is walkable.
  [[nodiscard]] std::vector<core::Point> walkableNeighbors(core::Point p) const;

  // Cells of the given type in row-major order.
  [[nodiscard]] std::vector<core::Point> cellsOfType(CellType type) const;

  // Free cells orthogonally adjacent to a shelf, in row-major order.
  [[nodiscard]] std::vector<core::Point> shelfAccessPoints() const;

  [[nodiscard]] std::vector<std::string> toRows() const;

  // Names the layout as it is now: equal for a layout and its copies, and different, process-wide,
  // after every set(). Two layouts with the same version have the same cells; the converse need not
  // hold. For caches that would otherwise have to look at every cell to know that nothing changed.
  [[nodiscard]] std::uint64_t version() const noexcept { return version_; }

 private:
  GridSize size_;
  std::vector<CellType> cells_;
  std::uint64_t version_;
};

}  // namespace warehousesim::world
