#include "warehousesim/simulation/serialization.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace warehousesim::simulation {

namespace {

using nlohmann::json;

json pointToJson(core::Point p) { return json{{"x", p.x}, {"y", p.y}}; }

json optionalIdToJson(const auto& id) { return id ? json(id->value) : json(nullptr); }

json pointsToJson(const std::vector<core::Point>& points) {
  json result = json::array();
  for (const core::Point p : points) {
    result.push_back(pointToJson(p));
  }
  return result;
}

json waitToJson(const std::optional<RobotWait>& wait) {
  if (!wait) {
    return nullptr;
  }
  return json{{"cell", pointToJson(wait->cell)},
              {"kind", traffic::toString(wait->kind)},
              {"other", optionalIdToJson(wait->other)}};
}

}  // namespace

json toJson(const world::Warehouse& warehouse) {
  return json{
      {"width", warehouse.width()}, {"height", warehouse.height()}, {"rows", warehouse.toRows()}};
}

json toJson(const Snapshot& snapshot) {
  json robotsJson = json::array();
  for (const RobotSnapshot& robot : snapshot.robots) {
    robotsJson.push_back(
        {{"id", robot.id.value},
         {"position", pointToJson(robot.position)},
         {"pose", {{"x", robot.center().x}, {"y", robot.center().y}}},
         {"heading", core::toString(robot.heading)},
         {"headingAngle", robot.headingAngle()},
         {"state", robots::toString(robot.state)},
         {"goal", robot.goal == robots::NavigationGoal::None ? json(nullptr)
                                                             : json(robots::toString(robot.goal))},
         {"speed", robot.speed},
         {"motion", robots::toString(robot.motion)},
         {"carrying", robot.carrying},
         {"deliveries", robot.deliveries},
         {"battery", {{"level", robot.batteryLevel}, {"capacity", robot.batteryCapacity}}},
         {"task", optionalIdToJson(robot.task)},
         {"path", pointsToJson(robot.path)},
         {"holds", pointsToJson(robot.holds)},
         {"waiting", waitToJson(robot.waiting)},
         {"stuckTicks", robot.stuckTicks}});
  }
  json tasksJson = json::array();
  for (const tasks::Task& task : snapshot.activeTasks) {
    tasksJson.push_back({{"id", task.id.value},
                         {"pickup", pointToJson(task.pickup)},
                         {"dropoff", pointToJson(task.dropoff)},
                         {"priority", tasks::toString(task.priority)},
                         {"status", tasks::toString(task.status)},
                         {"assignee", optionalIdToJson(task.assignee)},
                         {"createdAt", task.createdAt}});
  }
  return json{{"tick", snapshot.tick},
              {"robots", std::move(robotsJson)},
              {"activeTasks", std::move(tasksJson)},
              {"completedTasks", snapshot.completedTasks},
              {"failedTasks", snapshot.failedTasks},
              {"cancelledTasks", snapshot.cancelledTasks},
              {"traffic",
               {{"blocked", snapshot.traffic.blocked},
                {"vertexConflicts", snapshot.traffic.vertex},
                {"swapConflicts", snapshot.traffic.swaps},
                {"inTheWay", snapshot.traffic.inTheWay},
                {"replans", snapshot.replans}}},
              {"planning",
               {{"searches", snapshot.planning.searches},
                {"reroutes", snapshot.planning.reroutes},
                {"dodges", snapshot.planning.dodges},
                {"partialPlans", snapshot.planning.partialPlans},
                {"outOfBudget", snapshot.planning.outOfBudget},
                {"deferred", snapshot.planning.deferred},
                {"work", snapshot.planning.work},
                {"peakStepWork", snapshot.planning.peakStepWork},
                {"obstacleRepairs", snapshot.planning.obstacleRepairs},
                {"prolongedBlocks", snapshot.planning.prolongedBlocks},
                {"deadlocks", snapshot.planning.deadlocks},
                {"deadlocksRecovered", snapshot.planning.deadlocksRecovered},
                {"deadlocksUnresolved", snapshot.planning.deadlocksUnresolved},
                {"priorityBoosts", snapshot.planning.priorityBoosts},
                {"abandoned", snapshot.planning.abandoned}}}};
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
