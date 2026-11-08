#include "warehousesim/world/layout_check.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace warehousesim::world {

namespace {

using core::Point;

struct Regions {
  std::size_t count = 0;
  std::optional<Point> secondStart;  // the first cell (row-major) of the second region
};

// Flood-fills the walkable cells region by region, in row-major order of their first cell.
Regions countRegions(const Warehouse& warehouse) {
  const GridSize size = warehouse.size();
  std::vector<bool> seen(size.cellCount(), false);
  std::vector<Point> open;
  Regions regions;
  for (std::size_t i = 0; i < size.cellCount(); ++i) {
    const Point start = size.pointAt(i);
    if (seen[i] || !warehouse.isWalkable(start)) {
      continue;
    }
    ++regions.count;
    if (regions.count == 2) {
      regions.secondStart = start;
    }
    seen[i] = true;
    open.assign(1, start);
    while (!open.empty()) {
      const Point at = open.back();
      open.pop_back();
      for (const Point next : warehouse.walkableNeighbors(at)) {
        const std::size_t index = size.indexOf(next);
        if (!seen[index]) {
          seen[index] = true;
          open.push_back(next);
        }
      }
    }
  }
  return regions;
}

}  // namespace

std::vector<LayoutIssue> checkLayout(const Warehouse& warehouse) {
  std::vector<LayoutIssue> issues;
  const Regions regions = countRegions(warehouse);
  if (regions.count == 0) {
    issues.push_back({LayoutIssueKind::NoWalkableCells, true,
                      "the layout has no walkable cell: every cell is a wall or a shelf",
                      std::nullopt});
    return issues;  // everything else follows from this
  }
  if (warehouse.cellsOfType(CellType::Dropoff).empty()) {
    issues.push_back({LayoutIssueKind::NoDropoff, true,
                      "there is no dropoff, so goods have nowhere to be delivered", std::nullopt});
  }
  if (warehouse.cellsOfType(CellType::Pickup).empty() && warehouse.shelfAccessPoints().empty()) {
    issues.push_back({LayoutIssueKind::NoGoodsSource, true,
                      "there is no pickup and no free cell beside a shelf, so no goods can be "
                      "collected",
                      std::nullopt});
  }
  if (regions.count > 1) {
    issues.push_back({LayoutIssueKind::DisconnectedRegions, false,
                      "the walkable cells form " + std::to_string(regions.count) +
                          " separate regions, so tasks between them can never be done",
                      regions.secondStart});
  }
  return issues;
}

bool blocksSimulation(const std::vector<LayoutIssue>& issues) noexcept {
  return std::any_of(issues.begin(), issues.end(),
                     [](const LayoutIssue& issue) { return issue.blocksSimulation; });
}

}  // namespace warehousesim::world
