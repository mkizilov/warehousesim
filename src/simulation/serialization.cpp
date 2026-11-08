#include "warehousesim/simulation/serialization.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace warehousesim::simulation {

namespace {

using nlohmann::json;

json pointToJson(core::Point p) { return json{{"x", p.x}, {"y", p.y}}; }

json optionalIdToJson(const auto& id) { return id ? json(id->value) : json(nullptr); }

}  // namespace

json toJson(const world::Warehouse& warehouse) {
  return json{
      {"width", warehouse.width()}, {"height", warehouse.height()}, {"rows", warehouse.toRows()}};
}

json toJson(const Snapshot& snapshot) {
  json robotsJson = json::array();
  for (const RobotSnapshot& robot : snapshot.robots) {
    robotsJson.push_back({{"id", robot.id.value},
                          {"position", pointToJson(robot.position)},
                          {"status", robots::toString(robot.status)},
                          {"task", optionalIdToJson(robot.task)}});
  }
  json tasksJson = json::array();
  for (const tasks::Task& task : snapshot.activeTasks) {
    tasksJson.push_back({{"id", task.id.value},
                         {"pickup", pointToJson(task.pickup)},
                         {"dropoff", pointToJson(task.dropoff)},
                         {"status", tasks::toString(task.status)},
                         {"assignee", optionalIdToJson(task.assignee)},
                         {"createdAt", task.createdAt}});
  }
  return json{{"tick", snapshot.tick},
              {"robots", std::move(robotsJson)},
              {"activeTasks", std::move(tasksJson)},
              {"completedTasks", snapshot.completedTasks},
              {"failedTasks", snapshot.failedTasks}};
}

world::Warehouse warehouseFromJson(const json& document) {
  std::vector<std::string> rows;
  try {
    rows = document.at("rows").get<std::vector<std::string>>();
  } catch (const json::exception& error) {
    throw std::invalid_argument(std::string("invalid warehouse json: ") + error.what());
  }
  auto warehouse = world::Warehouse::fromRows(rows);
  if (!warehouse) {
    throw std::invalid_argument("invalid warehouse json: rows are empty, ragged or unknown");
  }
  for (const char* key : {"width", "height"}) {
    if (!document.contains(key)) {
      continue;
    }
    const auto expected = key[0] == 'w' ? warehouse->width() : warehouse->height();
    if (!document[key].is_number_integer() || document[key].get<std::int64_t>() != expected) {
      throw std::invalid_argument(std::string("invalid warehouse json: ") + key +
                                  " does not match rows");
    }
  }
  return *std::move(warehouse);
}

}  // namespace warehousesim::simulation
