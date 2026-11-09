#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/clock.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/tasks/task.hpp"

// Recording a run so that it can be played again, step for step, from its initial state.
//
// A simulation is a pure function of what it was built from (ScenarioSpec: the layout, where the
// robots start, the configuration with its random seed) and of what was done to it from outside,
// each thing after a whole number of steps (a Command: new tasks, a cancellation, a robot taken out
// of service, a wall put up). A Recording is exactly that, plus what the run did that can be
// checked later: when tasks were made and cancelled, and a checksum of the whole state every so
// often. Playing it back (see replay.hpp) builds the same simulation, applies the same commands at
// the same ticks, steps the same number of times, and compares: a replay that differs anywhere says
// where.
//
// Time in a recording is simulated time: ticks. Nothing in it comes from a clock.
namespace warehousesim::simulation {

// What the engine computes, as a number. It is raised on purpose, in the same change, whenever a
// change to the simulation alters what a scenario does (the pinned checksum in the replay tests
// then changes too). A recording made by another revision may well diverge from its replay, and
// says so when it does.
//   1: the first release candidate.
//   2: a robot that had taken its step is no longer pushed aside in the same step (it moved two
//      cells in one tick); found by the audit, it changes only runs in which that happened.
inline constexpr int kEngineRevision = 2;

// Ticks between the checksums a recording keeps, unless asked otherwise.
inline constexpr core::Tick kDefaultCheckpointInterval = 100;

// A command that was applied: after `tick` steps, before the next one. Commands come in the order
// they were applied, so those at one tick keep their order.
struct RecordedCommand {
  core::Tick tick = 0;
  Command command;        // never a ResetScenario: that starts another run, and another recording
  bool accepted = false;  // whether the simulation took it (a refused command changes nothing)

  bool operator==(const RecordedCommand&) const = default;
};

enum class EventSource : std::uint8_t {
  Command,  // a consequence of a command, seen right after it was applied
  Engine,   // of a step: the task generator made one
};

[[nodiscard]] const char* toString(EventSource source) noexcept;

enum class TaskEventKind : std::uint8_t { Created, Cancelled };

[[nodiscard]] const char* toString(TaskEventKind kind) noexcept;

// A task coming into being or being withdrawn. `tick` is the tick the step began at, or the tick
// the command was applied at, so a task has the tick it records as createdAt. Within a tick the
// events a command caused come first, then those of the step; within one of those, creations come
// before cancellations and tasks in order of id. A cancellation is reported whoever caused it,
// including the one that follows from putting a wall on a cell a task starts or ends on.
struct TaskEvent {
  core::Tick tick = 0;
  TaskEventKind kind = TaskEventKind::Created;
  core::TaskId task;
  EventSource source = EventSource::Engine;
  // What a created task is (unused for a cancellation).
  core::Point pickup;
  core::Point dropoff;
  tasks::TaskPriority priority = tasks::TaskPriority::Normal;

  bool operator==(const TaskEvent&) const = default;
};

// The state checksum (Simulation::stateChecksum) after `tick` steps. For a checkpoint at a tick
// where commands were also applied, it is taken before them; the end checkpoint after them.
struct Checkpoint {
  core::Tick tick = 0;
  std::uint64_t checksum = 0;

  bool operator==(const Checkpoint&) const = default;
};

struct Recording {
  int engineRevision = kEngineRevision;
  ScenarioSpec scenario{world::Warehouse(1, 1), {}, {}};  // a placeholder until it is filled in
  core::Tick ticks = 0;                                   // steps taken
  core::Tick checkpointInterval = 0;  // ticks between checkpoints; 0: only the first and the end
  std::vector<RecordedCommand> commands;
  // What the run did to tasks. False for a recording that does not say (a file made by hand, that
  // leaves them out): then taskEvents is empty, and the tasks a replay makes are not compared with
  // anything. True with an empty list is a run in which no task was made or cancelled.
  bool taskEventsRecorded = true;
  std::vector<TaskEvent> taskEvents;
  std::vector<Checkpoint> checkpoints;  // ticks ascending; the first is tick 0 in a recorded run
  std::optional<Checkpoint> end;        // after `ticks` steps and the commands of that tick
};

// A checksum as it is written in a file and in messages: 16 lowercase hexadecimal digits.
[[nodiscard]] std::string checksumText(std::uint64_t checksum);

// One thing wrong with a recording: `location` is a JSON Pointer into its file form
// ("/commands/3/tick"), or empty for the recording as a whole.
struct RecordingProblem {
  std::string location;
  std::string message;

  [[nodiscard]] std::string text() const {
    return location.empty() ? message : location + ": " + message;
  }
};

// Whether the recording is consistent with itself: commands, events and checkpoints in order and
// within `ticks`, no ResetScenario among the commands, created tasks numbered from 1, a
// cancellation only of a task created before it. Says nothing about whether it can be built or
// replayed to the same end; that is what replaying finds out. Returns the first problem found.
[[nodiscard]] std::optional<RecordingProblem> validate(const Recording& recording);

// Watches one run from its first tick and keeps what a recording needs. The caller reports, in the
// order they happen, every command it applies to the simulation and every step it takes; the
// journal reads what they did from the simulation. Not thread-safe (the runner calls it under its
// mutex).
class RunJournal {
 public:
  // The simulation must be at tick 0 with no task submitted yet, and have been built with the
  // default planner and dispatcher: those are what a replay rebuilds. Throws std::invalid_argument
  // otherwise. `checkpointInterval` of 0 keeps only the checkpoint at tick 0 and the end.
  RunJournal(const Simulation& simulation, core::Tick checkpointInterval);

  // Call right after applying `command` to `simulation`, whether or not it was accepted. Throws
  // std::invalid_argument for a ResetScenario, and std::logic_error if the simulation has taken a
  // step the journal has not been told about.
  void commandApplied(const Simulation& simulation, const Command& command, bool accepted);
  // Call right after every step. Throws std::logic_error if a step was missed.
  void stepTaken(const Simulation& simulation);

  // The recording of the run up to now. `simulation` must be the one being watched, at the tick
  // the journal last saw.
  [[nodiscard]] Recording recording(const Simulation& simulation) const;

  [[nodiscard]] const std::vector<TaskEvent>& taskEvents() const noexcept { return events_; }
  [[nodiscard]] const std::vector<Checkpoint>& checkpoints() const noexcept { return checkpoints_; }

 private:
  void noteTasks(const Simulation& simulation, EventSource source, core::Tick cancelledAt);

  ScenarioSpec scenario_;
  core::Tick interval_;
  core::Tick lastTick_ = 0;
  std::vector<RecordedCommand> commands_;
  std::vector<TaskEvent> events_;
  std::vector<Checkpoint> checkpoints_;
  std::size_t known_ = 0;                // tasks seen so far (ids 1..known_)
  std::vector<core::TaskId> openTasks_;  // of those, the ones not finished at the last look
};

// A simulation that is recorded while it runs: the way to run one headless. All changes to it go
// through the recorder (it only hands out a const reference), so the recording cannot miss one.
class Recorder {
 public:
  // Same conditions as RunJournal.
  explicit Recorder(Simulation simulation,
                    core::Tick checkpointInterval = kDefaultCheckpointInterval);

  // Applies the command (see simulation::apply) and records it with the tick and the outcome. A
  // ResetScenario is refused and not recorded.
  CommandOutcome apply(const Command& command);
  void step();
  void run(core::Tick ticks);

  [[nodiscard]] const Simulation& simulation() const noexcept { return simulation_; }
  [[nodiscard]] Recording recording() const { return journal_.recording(simulation_); }

 private:
  Simulation simulation_;
  RunJournal journal_;
};

}  // namespace warehousesim::simulation
