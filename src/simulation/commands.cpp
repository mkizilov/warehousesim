#include "warehousesim/simulation/commands.hpp"

#include <string>
#include <utility>

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

}  // namespace warehousesim::simulation
