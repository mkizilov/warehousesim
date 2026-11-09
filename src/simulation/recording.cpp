#include "warehousesim/simulation/recording.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "warehousesim/world/grid_size.hpp"

namespace warehousesim::simulation {

namespace {

// The most robots and the most steps a recording may hold: far beyond anything the program runs,
// and what a damaged or hostile file is stopped at before anything is built from it.
constexpr std::size_t kMaxRobots = 4096;
constexpr core::Tick kMaxTicks = 1'000'000'000;

RecordingProblem problem(std::string location, std::string message) {
  return {std::move(location), std::move(message)};
}

std::string at(std::string_view list, std::size_t index) {
  return "/" + std::string(list) + "/" + std::to_string(index);
}

// Commands first, then the step, at one tick.
bool precedes(const TaskEvent& a, const TaskEvent& b) {
  return a.tick != b.tick ? a.tick < b.tick : a.source < b.source;
}

std::optional<RecordingProblem> validateCommands(const Recording& recording) {
  core::Tick previous = 0;
  for (std::size_t i = 0; i < recording.commands.size(); ++i) {
    const RecordedCommand& entry = recording.commands[i];
    if (std::holds_alternative<ResetScenario>(entry.command)) {
      return problem(at("commands", i) + "/command",
                     "a reset starts another run and cannot be part of this one");
    }
    if (entry.tick > recording.ticks) {
      return problem(at("commands", i) + "/tick", "tick " + std::to_string(entry.tick) +
                                                      " is after the end of the run (tick " +
                                                      std::to_string(recording.ticks) + ")");
    }
    if (entry.tick < previous) {
      return problem(at("commands", i) + "/tick",
                     "tick " + std::to_string(entry.tick) +
                         " is before the command before it (tick " + std::to_string(previous) +
                         "); commands are in the order they were applied");
    }
    previous = entry.tick;
  }
  return std::nullopt;
}

std::optional<RecordingProblem> validateEvents(const Recording& recording) {
  if (!recording.taskEventsRecorded && !recording.taskEvents.empty()) {
    return problem("/taskEvents", "the recording says it does not keep the tasks, and lists some");
  }
  std::uint32_t created = 0;
  std::set<std::uint32_t> cancelled;
  for (std::size_t i = 0; i < recording.taskEvents.size(); ++i) {
    const TaskEvent& event = recording.taskEvents[i];
    const std::string where = at("taskEvents", i);
    if (event.tick > recording.ticks) {
      return problem(where + "/tick", "tick " + std::to_string(event.tick) +
                                          " is after the end of the run (tick " +
                                          std::to_string(recording.ticks) + ")");
    }
    if (i > 0 && precedes(event, recording.taskEvents[i - 1])) {
      return problem(where,
                     "out of order: events are listed by tick, and at one tick those of "
                     "commands before those of the step");
    }
    if (event.kind == TaskEventKind::Created) {
      if (event.task.value != created + 1) {
        return problem(where + "/task",
                       "tasks are numbered 1, 2, 3 ... in the order they are made; "
                       "expected " +
                           std::to_string(created + 1) + ", not " +
                           std::to_string(event.task.value));
      }
      created = event.task.value;
    } else {
      if (event.task.value == 0 || event.task.value > created) {
        return problem(where + "/task", "task " + std::to_string(event.task.value) +
                                            " has not been made at this point");
      }
      if (!cancelled.insert(event.task.value).second) {
        return problem(where + "/task",
                       "task " + std::to_string(event.task.value) + " was cancelled before");
      }
    }
  }
  return std::nullopt;
}

std::optional<RecordingProblem> validateCheckpoints(const Recording& recording) {
  for (std::size_t i = 0; i < recording.checkpoints.size(); ++i) {
    const Checkpoint& checkpoint = recording.checkpoints[i];
    if (checkpoint.tick > recording.ticks) {
      return problem(at("checkpoints", i) + "/tick", "tick " + std::to_string(checkpoint.tick) +
                                                         " is after the end of the run (tick " +
                                                         std::to_string(recording.ticks) + ")");
    }
    if (i > 0 && checkpoint.tick <= recording.checkpoints[i - 1].tick) {
      return problem(at("checkpoints", i) + "/tick", "checkpoints are listed by increasing tick");
    }
  }
  if (recording.end && recording.end->tick != recording.ticks) {
    return problem("/end/tick", "the end checkpoint is at tick " +
                                    std::to_string(recording.end->tick) +
                                    ", not at the end of the run (tick " +
                                    std::to_string(recording.ticks) + ")");
  }
  return std::nullopt;
}

}  // namespace

const char* toString(EventSource source) noexcept {
  switch (source) {
    case EventSource::FromCommand:
      return "command";
    case EventSource::Engine:
      return "engine";
  }
  return "?";
}

const char* toString(TaskEventKind kind) noexcept {
  switch (kind) {
    case TaskEventKind::Created:
      return "created";
    case TaskEventKind::Cancelled:
      return "cancelled";
  }
  return "?";
}

std::string checksumText(std::uint64_t checksum) {
  constexpr std::string_view kDigits = "0123456789abcdef";
  std::string out(16, '0');
  for (std::size_t i = 0; i < 16; ++i) {
    out[15 - i] = kDigits[(checksum >> (4 * i)) & 0xFU];
  }
  return out;
}

std::optional<RecordingProblem> validate(const Recording& recording) {
  const world::Warehouse& layout = recording.scenario.warehouse;
  if (!world::isValidMapSide(layout.width()) || !world::isValidMapSide(layout.height())) {
    return problem("/scenario/layout", "the layout must be 1 to " +
                                           std::to_string(world::kMaxMapSide) + " cells on a side");
  }
  if (recording.scenario.robotStarts.size() > kMaxRobots) {
    return problem("/scenario/robots", "at most " + std::to_string(kMaxRobots) + " robots");
  }
  if (recording.ticks > kMaxTicks) {
    return problem("/ticks", "at most " + std::to_string(kMaxTicks) + " steps");
  }
  if (auto found = validateCommands(recording)) {
    return found;
  }
  if (auto found = validateEvents(recording)) {
    return found;
  }
  return validateCheckpoints(recording);
}

// ---- RunJournal ---------------------------------------------------------------------------------

RunJournal::RunJournal(const Simulation& simulation, core::Tick checkpointInterval)
    : scenario_(simulation.origin()), interval_(checkpointInterval) {
  if (simulation.tick() != 0) {
    throw std::invalid_argument("a recording starts at tick 0, not at tick " +
                                std::to_string(simulation.tick()));
  }
  if (!simulation.usesDefaultComponents()) {
    throw std::invalid_argument(
        "a simulation built with its own planner or dispatcher cannot be recorded: a replay builds "
        "the default ones");
  }
  if (!simulation.tasks().all().empty()) {
    throw std::invalid_argument("a recording starts before the first task is made");
  }
  // Anything done to it before now (a wall put up, a robot taken out of service) would be missing
  // from the recording, so the simulation must still be exactly what its origin builds.
  if (Simulation(scenario_).stateChecksum() != simulation.stateChecksum()) {
    throw std::invalid_argument("the simulation was changed before the recording began");
  }
  checkpoints_.push_back({0, simulation.stateChecksum()});
}

void RunJournal::commandApplied(const Simulation& simulation, const Command& command,
                                bool accepted) {
  if (std::holds_alternative<ResetScenario>(command)) {
    throw std::invalid_argument("a reset starts another run; it is not recorded in this one");
  }
  if (simulation.tick() != lastTick_) {
    throw std::logic_error("the journal missed a step: it is at tick " + std::to_string(lastTick_) +
                           " and the simulation at tick " + std::to_string(simulation.tick()));
  }
  commands_.push_back({simulation.tick(), command, accepted});
  noteTasks(simulation, EventSource::FromCommand, simulation.tick());
}

void RunJournal::stepTaken(const Simulation& simulation) {
  if (simulation.tick() != lastTick_ + 1) {
    throw std::logic_error("the journal missed a step: it is at tick " + std::to_string(lastTick_) +
                           " and the simulation at tick " + std::to_string(simulation.tick()));
  }
  lastTick_ = simulation.tick();
  noteTasks(simulation, EventSource::Engine, lastTick_ - 1);
  if (interval_ > 0 && lastTick_ % interval_ == 0) {
    checkpoints_.push_back({lastTick_, simulation.stateChecksum()});
  }
}

// New tasks, then tasks that were open and are cancelled now: each in order of id. Only tasks that
// are still open are looked at, so that a long run does not slow down as it makes tasks.
void RunJournal::noteTasks(const Simulation& simulation, EventSource source,
                           core::Tick cancelledAt) {
  const std::vector<tasks::Task>& all = simulation.tasks().all();
  for (std::size_t i = known_; i < all.size(); ++i) {
    const tasks::Task& task = all[i];
    TaskEvent event;
    event.tick = task.createdAt;
    event.kind = TaskEventKind::Created;
    event.task = task.id;
    event.source = source;
    event.pickup = task.pickup;
    event.dropoff = task.dropoff;
    event.priority = task.priority;
    events_.push_back(event);
    openTasks_.push_back(task.id);
  }
  known_ = all.size();

  std::vector<core::TaskId> stillOpen;
  stillOpen.reserve(openTasks_.size());
  for (const core::TaskId id : openTasks_) {
    const tasks::Task* task = simulation.tasks().find(id);
    if (task->status == tasks::TaskStatus::Cancelled) {
      TaskEvent event;
      event.tick = cancelledAt;
      event.kind = TaskEventKind::Cancelled;
      event.task = id;
      event.source = source;
      events_.push_back(event);
    } else if (task->isOpen()) {
      stillOpen.push_back(id);
    }
  }
  openTasks_ = std::move(stillOpen);
}

Recording RunJournal::recording(const Simulation& simulation) const {
  if (simulation.tick() != lastTick_) {
    throw std::logic_error("the journal is at tick " + std::to_string(lastTick_) +
                           " but the simulation is at tick " + std::to_string(simulation.tick()));
  }
  Recording result;
  result.scenario = scenario_;
  result.ticks = lastTick_;
  result.checkpointInterval = interval_;
  result.commands = commands_;
  result.taskEvents = events_;
  result.checkpoints = checkpoints_;
  result.end = Checkpoint{lastTick_, simulation.stateChecksum()};
  return result;
}

// ---- Recorder -----------------------------------------------------------------------------------

Recorder::Recorder(Simulation simulation, core::Tick checkpointInterval)
    : simulation_(std::move(simulation)), journal_(simulation_, checkpointInterval) {}

CommandOutcome Recorder::apply(const Command& command) {
  CommandOutcome outcome = simulation::apply(simulation_, command);
  if (!std::holds_alternative<ResetScenario>(command)) {
    journal_.commandApplied(simulation_, command, outcome.accepted);
  }
  return outcome;
}

void Recorder::step() {
  simulation_.step();
  journal_.stepTaken(simulation_);
}

void Recorder::run(core::Tick ticks) {
  for (core::Tick i = 0; i < ticks; ++i) {
    step();
  }
}

}  // namespace warehousesim::simulation
