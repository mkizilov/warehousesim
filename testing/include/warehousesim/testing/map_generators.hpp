#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/world/warehouse.hpp"

// Deterministic map builders for tests and benchmarks. Every random choice comes from the Rng
// passed in, so a seed names a map.
namespace warehousesim::testing {

[[nodiscard]] inline world::Warehouse makeOpenGrid(std::int32_t width, std::int32_t height) {
  return world::Warehouse(width, height);
}

// Each cell becomes an obstacle (60% wall, 40% shelf) with probability densityPerMille / 1000.
// A few of the remaining cells are Pickup/Dropoff/Charging so the other walkable types are used.
[[nodiscard]] inline world::Warehouse makeRandomObstacleMap(std::int32_t width, std::int32_t height,
                                                            std::uint32_t densityPerMille,
                                                            core::Rng& rng) {
  world::Warehouse warehouse(width, height);
  for (std::int32_t y = 0; y < height; ++y) {
    for (std::int32_t x = 0; x < width; ++x) {
      const core::Point p{x, y};
      if (rng.below(1000) < densityPerMille) {
        warehouse.set(p, rng.below(10) < 6 ? world::CellType::Wall : world::CellType::Shelf);
      } else if (rng.below(50) == 0) {
        constexpr world::CellType kSpecial[] = {world::CellType::Pickup, world::CellType::Dropoff,
                                                world::CellType::Charging};
        warehouse.set(p, kSpecial[rng.below(3)]);
      }
    }
  }
  return warehouse;
}

// A perfect maze (every two open cells joined by exactly one route) on a grid of
// (2 * cellsWide + 1) x (2 * cellsHigh + 1) cells, built by randomised depth-first search. The
// open "rooms" are the cells at odd coordinates; (1, 1) is the top-left room.
[[nodiscard]] inline world::Warehouse makeMaze(std::int32_t cellsWide, std::int32_t cellsHigh,
                                               core::Rng& rng) {
  world::Warehouse maze(2 * cellsWide + 1, 2 * cellsHigh + 1);
  for (std::int32_t y = 0; y < maze.height(); ++y) {
    for (std::int32_t x = 0; x < maze.width(); ++x) {
      maze.set({x, y}, world::CellType::Wall);
    }
  }
  std::vector<std::uint8_t> visited(static_cast<std::size_t>(cellsWide * cellsHigh), 0);
  const auto roomIndex = [cellsWide](std::int32_t cx, std::int32_t cy) {
    return static_cast<std::size_t>(cy * cellsWide + cx);
  };
  std::vector<std::pair<std::int32_t, std::int32_t>> stack{{0, 0}};
  visited[roomIndex(0, 0)] = 1;
  maze.set({1, 1}, world::CellType::Free);
  while (!stack.empty()) {
    const auto [cx, cy] = stack.back();
    std::vector<std::pair<std::int32_t, std::int32_t>> options;
    for (const core::Point step : core::kCardinalDirections) {
      const std::int32_t nx = cx + step.x;
      const std::int32_t ny = cy + step.y;
      if (nx >= 0 && ny >= 0 && nx < cellsWide && ny < cellsHigh &&
          visited[roomIndex(nx, ny)] == 0) {
        options.emplace_back(nx, ny);
      }
    }
    if (options.empty()) {
      stack.pop_back();
      continue;
    }
    const auto [nx, ny] = options[rng.below(options.size())];
    maze.set({2 * nx + 1, 2 * ny + 1}, world::CellType::Free);
    maze.set({cx + nx + 1, cy + ny + 1}, world::CellType::Free);  // the wall between the rooms
    visited[roomIndex(nx, ny)] = 1;
    stack.emplace_back(nx, ny);
  }
  return maze;
}

// One winding one-cell-wide corridor: `corridors` horizontal runs of `width` cells joined
// alternately at the right and left ends. Size: width x (2 * corridors - 1). From (0, 0) the far
// end is corridors * (width - 1) + 2 * (corridors - 1) steps away.
[[nodiscard]] inline world::Warehouse makeSerpentine(std::int32_t width, std::int32_t corridors) {
  world::Warehouse map(width, 2 * corridors - 1);
  for (std::int32_t y = 0; y < map.height(); ++y) {
    for (std::int32_t x = 0; x < width; ++x) {
      const bool corridorRow = y % 2 == 0;
      const bool connector = !corridorRow && x == ((y / 2) % 2 == 0 ? width - 1 : 0);
      map.set({x, y}, corridorRow || connector ? world::CellType::Free : world::CellType::Wall);
    }
  }
  return map;
}

// An open grid cut in two by a solid wall in column width / 2: nothing on one side can reach the
// other.
[[nodiscard]] inline world::Warehouse makeSplitMap(std::int32_t width, std::int32_t height) {
  world::Warehouse map(width, height);
  for (std::int32_t y = 0; y < height; ++y) {
    map.set({width / 2, y}, world::CellType::Wall);
  }
  return map;
}

}  // namespace warehousesim::testing
