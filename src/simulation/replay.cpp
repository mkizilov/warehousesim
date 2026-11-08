#include "warehousesim/simulation/replay.hpp"

#include <algorithm>
#include <exception>
#include <string>
#include <utility>

#include "warehousesim/simulation/metrics_collector.hpp"

namespace warehousesim::simulation {

namespace {

std::string cellText(core::Point cell) {
  return "(" + std::to_string(cell.x) + ", " + std::to_string(cell.y) + ")";
}

std::string describe(const TaskEvent& event) {
  std::string text = "task #" + std::to_string(event.task.value);
  if (event.kind == TaskEventKind::Created) {
    text += " created at tick " + std::to_string(event.tick) + " (pickup " +
            cellText(event.pickup) + ", dropoff " + cellText(event.dropoff) + ", " +
            tasks::toString(event.priority) + ")";
  } else {
    text += " cancelled at tick " + std::to_string(event.tick);
  }
  return text + ", by the " + toString(event.source);
}

// The replay of one recording. Everything is checked as it goes so that the first disagreement is
// the one reported, with the tick it happened at.
class Replay {
 public:
  Replay(const Recording& recording, const ReplayOptions& options)
      : recording_(recording), options_(options) {}

  ReplayResult run() {
    if (const auto problem = validate(recording_)) {
      diverge(0, "the recording is not valid: " + problem->text());
      return std::move(result_);
    }
    std::optional<Simulation> built;
    try {
      built.emplace(recording_.scenario);
    } catch (const std::exception& error) {
      diverge(0, std::string("the scenario cannot be built: ") + error.what());
      return std::move(result_);
    }
    simulation_ = &*built;
    try {
      play();
    } catch (const std::exception& error) {
      diverge(simulation_->tick(), std::string("the engine threw: ") + error.what());
    }
    result_.ticks = simulation_->tick();
    result_.snapshot = simulation_->snapshot();
    result_.checksum = simulation_->stateChecksum();
    if (collector_) {
      result_.metrics = collector_->report();
    }
    if (journal_) {
      result_.taskEvents = journal_->taskEvents();
    }
    return std::move(result_);
  }

 private:
  void play() {
    const core::Tick target =
        options_.until ? std::min(*options_.until, recording_.ticks) : recording_.ticks;
    // No periodic checkpoints of its own: the ones to check are the recording's.
    journal_.emplace(*simulation_, 0);
    collector_.emplace(*simulation_, options_.metrics);
    if (!checkCheckpoint()) {
      return;
    }
    while (true) {
      if (!applyCommands() || !checkEvents(/*commandsOfThisTickAreDone=*/true)) {
        return;
      }
      if (simulation_->tick() >= target) {
        break;
      }
      simulation_->step();
      journal_->stepTaken(*simulation_);
      collector_->observe(*simulation_);
      if (!checkCheckpoint() || !checkEvents(/*commandsOfThisTickAreDone=*/false)) {
        return;
      }
    }
    if (options_.verify && target == recording_.ticks && recording_.end) {
      const std::uint64_t replayed = simulation_->stateChecksum();
      if (replayed != recording_.end->checksum) {
        diverge(simulation_->tick(), "the state at the end of the run (tick " +
                                         std::to_string(simulation_->tick()) + ") has checksum " +
                                         checksumText(replayed) + "; the recording says " +
                                         checksumText(recording_.end->checksum));
        return;
      }
      result_.endVerified = true;
    }
  }

  bool applyCommands() {
    const std::vector<RecordedCommand>& commands = recording_.commands;
    while (nextCommand_ < commands.size() && commands[nextCommand_].tick == simulation_->tick()) {
      const RecordedCommand& entry = commands[nextCommand_];
      // Qualified: a variant argument would otherwise find std::apply as well.
      const CommandOutcome outcome = simulation::apply(*simulation_, entry.command);
      journal_->commandApplied(*simulation_, entry.command, outcome.accepted);
      if (options_.verify && outcome.accepted != entry.accepted) {
        diverge(entry.tick, std::string("command ") + std::to_string(nextCommand_) + " (" +
                                commandName(entry.command) + ") was " +
                                (entry.accepted ? "accepted" : "refused") +
                                " in the recording, but the replay " +
                                (outcome.accepted ? "accepted" : "refused") +
                                " it: " + outcome.message);
        return false;
      }
      ++nextCommand_;
    }
    return true;
  }

  // The recording's checksum for this tick, if it has one, against the replay's.
  bool checkCheckpoint() {
    if (!options_.verify) {
      return true;
    }
    const std::vector<Checkpoint>& checkpoints = recording_.checkpoints;
    if (nextCheckpoint_ >= checkpoints.size() ||
        checkpoints[nextCheckpoint_].tick != simulation_->tick()) {
      return true;
    }
    const Checkpoint& expected = checkpoints[nextCheckpoint_++];
    const std::uint64_t replayed = simulation_->stateChecksum();
    result_.checkpoints.push_back({expected.tick, replayed});
    if (replayed != expected.checksum) {
      diverge(expected.tick, "the state at tick " + std::to_string(expected.tick) +
                                 " has checksum " + checksumText(replayed) +
                                 "; the recording says " + checksumText(expected.checksum));
      return false;
    }
    ++result_.checkpointsVerified;
    return true;
  }

  // What the replay made of tasks so far against what the recording says, event by event, and
  // whether it has made everything the recording has said so far.
  bool checkEvents(bool commandsOfThisTickAreDone) {
    if (!options_.verify || !recording_.taskEventsRecorded) {
      return true;
    }
    const std::vector<TaskEvent>& made = journal_->taskEvents();
    const std::vector<TaskEvent>& recorded = recording_.taskEvents;
    for (; checkedEvents_ < made.size(); ++checkedEvents_) {
      const TaskEvent& event = made[checkedEvents_];
      if (checkedEvents_ >= recorded.size()) {
        diverge(event.tick,
                "the replay made " + describe(event) + ", which the recording does not have");
        return false;
      }
      if (event != recorded[checkedEvents_]) {
        diverge(event.tick, "the recording has " + describe(recorded[checkedEvents_]) +
                                ", but the replay made " + describe(event));
        return false;
      }
    }
    if (made.size() < recorded.size()) {
      const TaskEvent& next = recorded[made.size()];
      const core::Tick now = simulation_->tick();
      const bool due = next.tick < now || (commandsOfThisTickAreDone && next.tick == now &&
                                           next.source == EventSource::Command);
      if (due) {
        diverge(next.tick,
                "the recording has " + describe(next) + ", but the replay made nothing then");
        return false;
      }
    }
    return true;
  }

  void diverge(core::Tick tick, std::string what) {
    if (recording_.engineRevision != kEngineRevision) {
      what += " (the recording was made by engine revision " +
              std::to_string(recording_.engineRevision) + " and this is revision " +
              std::to_string(kEngineRevision) + ", so a difference is to be expected)";
    }
    result_.divergence = Divergence{tick, std::move(what)};
  }

  const Recording& recording_;
  const ReplayOptions& options_;
  ReplayResult result_;
  Simulation* simulation_ = nullptr;
  std::optional<RunJournal> journal_;
  std::optional<MetricsCollector> collector_;
  std::size_t nextCommand_ = 0;
  std::size_t nextCheckpoint_ = 0;
  std::size_t checkedEvents_ = 0;
};

}  // namespace

ReplayResult replay(const Recording& recording, const ReplayOptions& options) {
  return Replay(recording, options).run();
}

}  // namespace warehousesim::simulation
