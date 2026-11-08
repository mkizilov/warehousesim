#include "warehousesim/world/warehouse.hpp"

#include <limits>
#include <stdexcept>

namespace warehousesim::world {

using core::Point;

char toChar(CellType type) noexcept {
  switch (type) {
    case CellType::Floor:
      return '.';
    case CellType::Shelf:
      return 'S';
    case CellType::Station:
      return 'D';
    case CellType::Wall:
      return '#';
  }
  return '?';
}

std::optional<CellType> cellTypeFromChar(char c) noexcept {
  switch (c) {
    case '.':
      return CellType::Floor;
    case 'S':
      return CellType::Shelf;
    case 'D':
      return CellType::Station;
    case '#':
      return CellType::Wall;
    default:
      return std::nullopt;
  }
}

Warehouse::Warehouse(std::int32_t width, std::int32_t height) : width_(width), height_(height) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("warehouse dimensions must be positive");
  }
  cells_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                CellType::Floor);
}

std::optional<Warehouse> Warehouse::fromRows(const std::vector<std::string>& rows) {
  if (rows.empty() || rows.front().empty()) {
    return std::nullopt;
  }
  const std::size_t maxDimension =
      static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max());
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

bool Warehouse::inBounds(Point p) const noexcept {
  return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_;
}

bool Warehouse::isWalkable(Point p) const noexcept {
  if (!inBounds(p)) {
    return false;
  }
  const CellType type = cells_[index(p)];
  return type == CellType::Floor || type == CellType::Station;
}

CellType Warehouse::at(Point p) const {
  if (!inBounds(p)) {
    throw std::out_of_range("point outside warehouse");
  }
  return cells_[index(p)];
}

void Warehouse::set(Point p, CellType type) {
  if (!inBounds(p)) {
    throw std::out_of_range("point outside warehouse");
  }
  cells_[index(p)] = type;
}

std::vector<Point> Warehouse::cellsOfType(CellType type) const {
  std::vector<Point> result;
  for (std::int32_t y = 0; y < height_; ++y) {
    for (std::int32_t x = 0; x < width_; ++x) {
      if (cells_[index({x, y})] == type) {
        result.push_back({x, y});
      }
    }
  }
  return result;
}

std::vector<Point> Warehouse::shelfAccessPoints() const {
  std::vector<Point> result;
  for (std::int32_t y = 0; y < height_; ++y) {
    for (std::int32_t x = 0; x < width_; ++x) {
      const Point p{x, y};
      if (cells_[index(p)] != CellType::Floor) {
        continue;
      }
      for (const Point dir : core::kCardinalDirections) {
        const Point neighbour = p + dir;
        if (inBounds(neighbour) && cells_[index(neighbour)] == CellType::Shelf) {
          result.push_back(p);
          break;
        }
      }
    }
  }
  return result;
}

std::vector<std::string> Warehouse::toRows() const {
  std::vector<std::string> rows;
  rows.reserve(static_cast<std::size_t>(height_));
  for (std::int32_t y = 0; y < height_; ++y) {
    std::string row;
    row.reserve(static_cast<std::size_t>(width_));
    for (std::int32_t x = 0; x < width_; ++x) {
      row.push_back(toChar(cells_[index({x, y})]));
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

std::size_t Warehouse::index(Point p) const noexcept {
  return static_cast<std::size_t>(p.y) * static_cast<std::size_t>(width_) +
         static_cast<std::size_t>(p.x);
}

Warehouse makeDemoWarehouse() {
  const std::vector<std::string> rows = {
      "....................", ".SSSS..SSSS..SSSS...", ".SSSS..SSSS..SSSS...",
      "....................", ".SSSS..SSSS..SSSS...", ".SSSS..SSSS..SSSS...",
      "....................", ".SSSS..SSSS..SSSS...", ".SSSS..SSSS..SSSS...",
      "....................", "....................", "D..D..D..D..........",
  };
  auto warehouse = Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::logic_error("demo warehouse layout is invalid");
  }
  return *std::move(warehouse);
}

}  // namespace warehousesim::world
