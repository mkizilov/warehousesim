#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/tasks/task.hpp"

namespace warehousesim::simulation {

class Simulation;

// What an operator interface may ask of a running simulation, as plain values. An interface never
// touches a Simulation: it posts commands to a SimulationRunner, which applies them one at a time,
// between two steps, in the order they were posted, and records each one with the tick it was
// applied at. Which tick that is can depend on timing; what the simulation does afterwards cannot:
// the state at any tick is a function of the scenario and that record alone (see replay in the
// tests), so no amount of clicking can make a run irreproducible.
//
// Pausing, resuming and changing the speed are not commands: they decide when steps happen, never
// what a step computes, and so they are not part of the record.
struct SubmitTask {
  core::Point pickup;
  core::Point dropoff;
  tasks::TaskPriority priority = tasks::TaskPriority::Normal;

  bool operator==(const SubmitTask&) const = default;
};

// Changes the priority of a task that is still waiting for a robot.
struct SetTaskPriority {
  core::TaskId task;
  tasks::TaskPriority priority = tasks::TaskPriority::Normal;

  bool operator==(const SetTaskPriority&) const = default;
};

// Starts the scenario over: a new simulation at tick 0, from `seed` or, if none, the seed of the
// one being replaced. Handled by the runner, which owns the way to build a scenario.
struct ResetScenario {
  std::optional<std::uint64_t> seed;

  bool operator==(const ResetScenario&) const = default;
};

using Command = std::variant<SubmitTask, SetTaskPriority, ResetScenario>;

// What came of a command. A refused command changes nothing.
struct CommandOutcome {
  bool accepted = false;
  std::string message;               // one line for a status bar, e.g. "task #7 added" or why not
  std::optional<core::TaskId> task;  // the task a SubmitTask made

  bool operator==(const CommandOutcome&) const = default;
};

// A command as it was applied: after `tick` steps of the current run, before the next one.
struct CommandRecord {
  core::Tick tick = 0;
  Command command;
  CommandOutcome outcome;

  bool operator==(const CommandRecord&) const = default;
};

// Apply a command to a simulation. Invalid ones (a cell that is not floor, a task that is not
// waiting) are refused with a reason and leave the simulation exactly as it was; nothing throws
// for any value of the command.
CommandOutcome apply(Simulation& simulation, const SubmitTask& command);
CommandOutcome apply(Simulation& simulation, const SetTaskPriority& command);

}  // namespace warehousesim::simulation
