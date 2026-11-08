#pragma once

#include <nlohmann/json_fwd.hpp>

#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::simulation {

// JSON lives here, in the outermost headless layer, so the inner modules stay free of
// serialization dependencies. Object keys are emitted in sorted order, so dump() output
// is byte-stable for equal inputs.

[[nodiscard]] nlohmann::json toJson(const world::Warehouse& warehouse);
[[nodiscard]] nlohmann::json toJson(const Snapshot& snapshot);

// Throws std::invalid_argument when the document is not a valid warehouse description.
[[nodiscard]] world::Warehouse warehouseFromJson(const nlohmann::json& json);

}  // namespace warehousesim::simulation
