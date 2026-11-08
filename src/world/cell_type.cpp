#include "warehousesim/world/cell_type.hpp"

namespace warehousesim::world {

char toChar(CellType type) noexcept {
  switch (type) {
    case CellType::Free:
      return '.';
    case CellType::Wall:
      return '#';
    case CellType::Shelf:
      return 'S';
    case CellType::Pickup:
      return 'P';
    case CellType::Dropoff:
      return 'D';
    case CellType::Charging:
      return 'C';
  }
  return '?';
}

std::optional<CellType> cellTypeFromChar(char c) noexcept {
  switch (c) {
    case '.':
      return CellType::Free;
    case '#':
      return CellType::Wall;
    case 'S':
      return CellType::Shelf;
    case 'P':
      return CellType::Pickup;
    case 'D':
      return CellType::Dropoff;
    case 'C':
      return CellType::Charging;
    default:
      return std::nullopt;
  }
}

const char* toString(CellType type) noexcept {
  switch (type) {
    case CellType::Free:
      return "free";
    case CellType::Wall:
      return "wall";
    case CellType::Shelf:
      return "shelf";
    case CellType::Pickup:
      return "pickup";
    case CellType::Dropoff:
      return "dropoff";
    case CellType::Charging:
      return "charging";
  }
  return "unknown";
}

}  // namespace warehousesim::world
