#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/point.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::world {

enum class LayoutIssueKind : std::uint8_t {
  NoWalkableCells,     // nothing a robot could stand on
  NoDropoff,           // goods have nowhere to be delivered
  NoGoodsSource,       // no Pickup cell and no free cell beside a shelf
  DisconnectedRegions  // the walkable cells fall into more than one region
};

struct LayoutIssue {
  LayoutIssueKind kind;
  // True if a simulation cannot do anything useful on the layout (no tasks can ever be created);
  // false if it runs but part of it is out of reach.
  bool blocksSimulation;
  std::string message;
  // A cell to look at, where there is one.
  std::optional<core::Point> where;
};

// Whether a layout can host a simulation. This is a property of the layout alone, which is why it
// lives here: whoever builds a layout (a predefined one, a map file, the editor) can ask. An empty
// result means the layout is ready; the predefined layouts all are. The issues come in the fixed
// order of LayoutIssueKind.
[[nodiscard]] std::vector<LayoutIssue> checkLayout(const Warehouse& warehouse);

// True if any issue blocks a simulation.
[[nodiscard]] bool blocksSimulation(const std::vector<LayoutIssue>& issues) noexcept;

}  // namespace warehousesim::world
