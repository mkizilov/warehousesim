#include "warehousesim/world/warehouse.hpp"

#include <limits>
#include <stdexcept>

namespace warehousesim::world {

using core::Point;

Warehouse::Warehouse(GridSize size) : size_(size), cells_(size.cellCount(), CellType::Free) {}

Warehouse::Warehouse(std::int32_t width, std::int32_t height)
    : Warehouse(GridSize(width, height)) {}

std::optional<Warehouse> Warehouse::fromRows(const std::vector<std::string>& rows) {
  if (rows.empty() || rows.front().empty()) {
    return std::nullopt;
  }
  const auto maxDimension = static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max());
  if (rows.size() > maxDimension || rows.front().size() > maxDimension) {
    return std::nullopt;
  }
  Warehouse warehouse(static_cast<std::int32_t>(rows.front().size()),
                      static_cast<std::int32_t>(rows.size()));
  for (std::size_t y = 0; y < rows.size(); ++y) {
    if (rows[y].size() != rows.front().size()) {
      return std::nullopt;
    }
    for (std::size_t x = 0; x < rows[y].size(); ++x) {
      const auto type = cellTypeFromChar(rows[y][x]);
      if (!type) {
        return std::nullopt;
      }
      warehouse.cells_[y * rows[y].size() + x] = *type;
    }
  }
  return warehouse;
}

bool Warehouse::isWalkable(Point p) const noexcept {
  return inBounds(p) && world::isWalkable(cells_[size_.indexOf(p)]);
}

bool Warehouse::isObstacle(Point p) const noexcept {
  return inBounds(p) && world::isObstacle(cells_[size_.indexOf(p)]);
}

CellType Warehouse::at(Point p) const {
  if (!inBounds(p)) {
    throw std::out_of_range("point outside warehouse");
  }
  return cells_[size_.indexOf(p)];
}

void Warehouse::set(Point p, CellType type) {
  if (!inBounds(p)) {
    throw std::out_of_range("point outside warehouse");
  }
  cells_[size_.indexOf(p)] = type;
}

std::vector<Point> Warehouse::neighbors(Point p) const {
  std::vector<Point> result;
  if (!inBounds(p)) {
    return result;
  }
  for (const core::Direction direction : core::kAllDirections) {
    const Point neighbor = p + core::offsetOf(direction);
    if (inBounds(neighbor)) {
      result.push_back(neighbor);
    }
  }
  return result;
}

std::vector<Point> Warehouse::walkableNeighbors(Point p) const {
  std::vector<Point> result = neighbors(p);
  std::erase_if(result, [this](Point neighbor) { return !isWalkable(neighbor); });
  return result;
}

std::vector<Point> Warehouse::cellsOfType(CellType type) const {
  std::vector<Point> result;
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    if (cells_[i] == type) {
      result.push_back(size_.pointAt(i));
    }
  }
  return result;
}

std::vector<Point> Warehouse::shelfAccessPoints() const {
  std::vector<Point> result;
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    if (cells_[i] != CellType::Free) {
      continue;
    }
    const Point p = size_.pointAt(i);
    for (const Point neighbor : neighbors(p)) {
      if (cells_[size_.indexOf(neighbor)] == CellType::Shelf) {
        result.push_back(p);
        break;
      }
    }
  }
  return result;
}

std::vector<std::string> Warehouse::toRows() const {
  std::vector<std::string> rows;
  rows.reserve(static_cast<std::size_t>(height()));
  for (std::int32_t y = 0; y < height(); ++y) {
    std::string row;
    row.reserve(static_cast<std::size_t>(width()));
    for (std::int32_t x = 0; x < width(); ++x) {
      row.push_back(toChar(cells_[size_.indexOf({x, y})]));
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

}  // namespace warehousesim::world
