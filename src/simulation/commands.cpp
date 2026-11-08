#include "warehousesim/simulation/commands.hpp"

#include <iterator>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/world/cell_type.hpp"

namespace warehousesim::simulation {

namespace {

using core::Point;

CommandOutcome refused(std::string message) { return {false, std::move(message), std::nullopt}; }

std::string cellText(Point cell) {
  return "(" + std::to_string(cell.x) + ", " + std::to_string(cell.y) + ")";
}

// Why a cell cannot be the start or end of a task, or an empty string if it can.
std::string whyNotATaskCell(const Simulation& simulation, Point cell, const char* role) {
  const world::Warehouse& layout = simulation.warehouse();
  if (!layout.inBounds(cell)) {
    return std::string(role) + " " + cellText(cell) + " is outside the map";
  }
  if (!layout.isWalkable(cell)) {
    return std::string(role) + " " + cellText(cell) + " is a " + world::toString(layout.at(cell)) +
           ", not floor";
  }
  return {};
}

}  // namespace

CommandOutcome apply(Simulation& simulation, const SubmitTask& command) {
  for (const auto& [cell, role] :
       {std::pair{command.pickup, "pickup"}, std::pair{command.dropoff, "dropoff"}}) {
    if (std::string reason = whyNotATaskCell(simulation, cell, role); !reason.empty()) {
      return refused(std::move(reason));
    }
  }
  if (command.pickup == command.dropoff) {
    return refused("pickup and dropoff are the same cell " + cellText(command.pickup));
  }
  const core::TaskId id = simulation.submitTask(command.pickup, command.dropoff, command.priority);
  return {true,
          "task #" + std::to_string(id.value) + " added: " + cellText(command.pickup) + " to " +
              cellText(command.dropoff) + ", " + tasks::toString(command.priority),
          id};
}

CommandOutcome apply(Simulation& simulation, const SetTaskPriority& command) {
  const std::string name = "task #" + std::to_string(command.task.value);
  const tasks::Task* task = simulation.tasks().find(command.task);
  if (task == nullptr) {
    return refused("there is no " + name);
  }
  if (!simulation.setTaskPriority(command.task, command.priority)) {
    return refused(name + " is " + tasks::toString(task->status) +
                   "; only a task that is still waiting can change priority");
  }
  return {true, name + " is now " + tasks::toString(command.priority), command.task};
}

CommandOutcome apply(Simulation& simulation, const CancelTask& command) {
  const std::string name = "task #" + std::to_string(command.task.value);
  const tasks::Task* task = simulation.tasks().find(command.task);
  if (task == nullptr) {
    return refused("there is no " + name);
  }
  if (!simulation.cancelTask(command.task)) {
    return refused(name + " is " + tasks::toString(task->status) +
                   "; only an open task can be cancelled");
  }
  return {true, name + " cancelled", command.task};
}

CommandOutcome apply(Simulation& simulation, const SetRobotService& command) {
  const std::string name = "robot #" + std::to_string(command.robot.value);
  if (!command.robot.valid() || command.robot.value > simulation.robots().size()) {
    return refused("there is no " + name);
  }
  const bool changed = command.inService ? simulation.restoreRobot(command.robot)
                                         : simulation.disableRobot(command.robot);
  if (!changed) {
    return refused(name +
                   (command.inService ? " is already in service" : " is already out of service"));
  }
  return {true, name + (command.inService ? " back in service" : " taken out of service"),
          std::nullopt};
}

CommandOutcome apply(Simulation& simulation, const PlaceObstacle& command) {
  const world::Warehouse& layout = simulation.warehouse();
  if (!world::isObstacle(command.type)) {
    return refused(std::string(world::toString(command.type)) +
                   " is not something that can be put up");
  }
  if (!layout.inBounds(command.cell)) {
    return refused(cellText(command.cell) + " is outside the map");
  }
  if (layout.at(command.cell) != world::CellType::Free) {
    return refused(cellText(command.cell) + " is a " + world::toString(layout.at(command.cell)) +
                   ", not free floor");
  }
  if (!simulation.placeObstacle(command.cell, command.type)) {
    return refused("a robot stands on or holds " + cellText(command.cell));
  }
  return {true, std::string(world::toString(command.type)) + " put up at " + cellText(command.cell),
          std::nullopt};
}

CommandOutcome apply(Simulation& simulation, const ClearObstacle& command) {
  const world::Warehouse& layout = simulation.warehouse();
  if (!layout.inBounds(command.cell)) {
    return refused(cellText(command.cell) + " is outside the map");
  }
  if (!layout.isObstacle(command.cell)) {
    return refused(cellText(command.cell) +
                   " is not a wall or a shelf; there is nothing to take down");
  }
  if (!simulation.clearObstacle(command.cell)) {
    return refused(cellText(command.cell) + " cannot be cleared");
  }
  return {true, "obstacle taken down at " + cellText(command.cell), std::nullopt};
}

const char* commandName(const Command& command) noexcept {
  static constexpr const char* kNames[] = {"submitTask",      "setTaskPriority", "cancelTask",
                                           "setRobotService", "placeObstacle",   "clearObstacle",
                                           "resetScenario"};
  static_assert(std::size(kNames) == std::variant_size_v<Command>, "every command needs a name");
  return kNames[command.index()];
}

CommandOutcome apply(Simulation& simulation, const Command& command) {
  return std::visit(
      [&simulation](const auto& typed) -> CommandOutcome {
        if constexpr (std::is_same_v<std::decay_t<decltype(typed)>, ResetScenario>) {
          return refused("only a runner can start a scenario over");
        } else {
          return apply(simulation, typed);
        }
      },
      command);
}

}  // namespace warehousesim::simulation
