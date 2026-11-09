#include "warehousesim/simulation/replay.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "recording_support.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/simulation/recording_file.hpp"

namespace {

using namespace warehousesim_test;
using warehousesim::core::Rng;
using warehousesim::core::Tick;
using warehousesim::simulation::Checkpoint;
using warehousesim::simulation::checksumText;
using warehousesim::simulation::commandName;
using warehousesim::simulation::EventSource;
using warehousesim::simulation::parseRecording;
using warehousesim::simulation::replay;
using warehousesim::simulation::ReplayOptions;
using warehousesim::simulation::ReplayResult;
using warehousesim::simulation::serializeRecording;
using warehousesim::simulation::TaskEvent;
using warehousesim::simulation::TaskEventKind;

bool mentions(const ReplayResult& result, const std::string& part) {
  return result.divergence && result.divergence->what.find(part) != std::string::npos;
}

// Everything a replay tells, as text, for saying that two replays told the same.
std::string whole(const ReplayResult& result) {
  std::string text = std::to_string(result.ticks) + "|" + std::to_string(result.checksum) + "|" +
                     std::to_string(result.checkpointsVerified) + "|" +
                     (result.endVerified ? "end" : "-") + "|" + metricsText(result.metrics) + "|";
  for (const TaskEvent& event : result.taskEvents) {
    text += std::to_string(event.tick) + std::string(toString(event.kind)) +
            std::to_string(event.task.value) + ";";
  }
  for (const Checkpoint& checkpoint : result.checkpoints) {
    text += std::to_string(checkpoint.tick) + ":" + std::to_string(checkpoint.checksum) + ";";
  }
  return text;
}

// ---- the replay of a run ends where the run did -------------------------------------------------

TEST(ReplayTest, TheReplayOfARunEndsInTheStateAndWithTheFiguresOfTheRun) {
  for (const std::string& name : bundledLayouts()) {
    for (const std::uint64_t seed : {1U, 4U, 9U}) {
      const LiveRun live = busyRun(name, seed, 160, 25);
      const ReplayResult result = replay(live.recording);
      ASSERT_TRUE(result.ok()) << name << " seed " << seed << ": " << result.divergence->what;
      EXPECT_EQ(result.ticks, 160U);
      EXPECT_TRUE(result.snapshot == live.snapshot) << name << " seed " << seed;
      EXPECT_EQ(result.checksum, live.checksum) << name << " seed " << seed;
      EXPECT_EQ(metricsText(result.metrics), metricsText(live.metrics)) << name << " seed " << seed;
      EXPECT_TRUE(result.taskEvents == live.recording.taskEvents) << name << " seed " << seed;
      EXPECT_EQ(result.checkpointsVerified, live.recording.checkpoints.size());
      EXPECT_TRUE(result.endVerified);
      EXPECT_EQ(result.metrics.ticks, 160U);
      EXPECT_TRUE(result.checkpoints == live.recording.checkpoints) << name << " seed " << seed;
    }
  }
}

TEST(ReplayTest, ReplayingTheSameRecordingAgainAndAgainGivesTheSameResultEveryTime) {
  for (const std::string& name : bundledLayouts()) {
    const Recording recording = busyRecording(name, 2, 140, 20);
    const std::string first = whole(replay(recording));
    for (int again = 0; again < 4; ++again) {
      EXPECT_EQ(whole(replay(recording)), first) << name << ", replay " << again + 2;
    }
  }
}

TEST(ReplayTest, ARecordingThatWentThroughAFileReplaysTheSame) {
  const LiveRun live = busyRun("fulfilment", 3, 140, 20);
  const auto reread = parseRecording(serializeRecording(live.recording));
  ASSERT_TRUE(reread.ok()) << reread.problem->text();
  const ReplayResult fromMemory = replay(live.recording);
  const ReplayResult fromFile = replay(*reread.recording);
  ASSERT_TRUE(fromFile.ok());
  EXPECT_EQ(whole(fromFile), whole(fromMemory));
  EXPECT_TRUE(fromFile.snapshot == live.snapshot);
  EXPECT_EQ(fromFile.checksum, live.checksum);
}

TEST(ReplayTest, ReplaysOnOtherThreadsAndInAnyOrderGiveTheSameResults) {
  const Recording a = busyRecording("demo", 5, 120, 30);
  const Recording b = busyRecording("compact", 6, 120, 30);
  const std::string wholeA = whole(replay(a));
  const std::string wholeB = whole(replay(b));
  std::vector<std::string> fromThreads(4);
  std::thread first([&] {
    fromThreads[0] = whole(replay(a));
    fromThreads[1] = whole(replay(b));
  });
  std::thread second([&] {
    fromThreads[2] = whole(replay(b));
    fromThreads[3] = whole(replay(a));
  });
  first.join();
  second.join();
  EXPECT_EQ(fromThreads[0], wholeA);
  EXPECT_EQ(fromThreads[1], wholeB);
  EXPECT_EQ(fromThreads[2], wholeB);
  EXPECT_EQ(fromThreads[3], wholeA);
}

TEST(ReplayTest, HowTheReplayIsMeasuredNeverChangesWhatItComputes) {
  const Recording recording = busyRecording("compact", 7, 120, 30);
  ReplayOptions slow;
  slow.metrics.secondsPerTick = 0.25;
  slow.metrics.sampleInterval = 7;
  const ReplayResult plain = replay(recording);
  const ReplayResult other = replay(recording, slow);
  ASSERT_TRUE(other.ok());
  EXPECT_EQ(other.checksum, plain.checksum);
  EXPECT_TRUE(other.snapshot == plain.snapshot);
  EXPECT_EQ(other.metrics.tasksCompleted, plain.metrics.tasksCompleted);
  EXPECT_EQ(other.metrics.cellsTravelled(), plain.metrics.cellsTravelled());
  EXPECT_NE(metricsText(other.metrics),
            metricsText(plain.metrics));  // but the figures are in other units
}

TEST(ReplayTest, AReplayIsTheRunItselfNotAnApproximationOfIt) {
  // Checked independently of the recorder: build the same simulation directly, apply the same
  // commands at the same ticks, and step.
  const Recording recording = busyRecording("compact", 8, 130, 30);
  Simulation by_hand(recording.scenario);
  std::size_t next = 0;
  for (Tick tick = 0;; ++tick) {
    while (next < recording.commands.size() && recording.commands[next].tick == tick) {
      (void)warehousesim::simulation::apply(by_hand, recording.commands[next].command);
      ++next;
    }
    if (tick == recording.ticks) {
      break;
    }
    by_hand.step();
  }
  const ReplayResult result = replay(recording);
  EXPECT_EQ(result.checksum, by_hand.stateChecksum());
  EXPECT_TRUE(result.snapshot == by_hand.snapshot());
}

// ---- a replay can stop early, play without comparing, or have nothing to compare ----------------

TEST(ReplayTest, AReplayCutShortStopsAtTheTickAskedForWithTheStateThere) {
  const Recording recording = busyRecording("demo", 2, 150, 5);
  ReplayOptions options;
  options.until = 65;
  const ReplayResult result = replay(recording, options);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.ticks, 65U);
  EXPECT_EQ(result.snapshot.tick, 65U);
  const auto at65 = std::find_if(recording.checkpoints.begin(), recording.checkpoints.end(),
                                 [](const Checkpoint& c) { return c.tick == 65; });
  ASSERT_TRUE(at65 != recording.checkpoints.end());
  EXPECT_EQ(result.checksum, at65->checksum);  // what the recording says the state was
  EXPECT_EQ(result.checkpointsVerified, 14U);  // ticks 0, 5, ... 65
  EXPECT_FALSE(result.endVerified);
  EXPECT_EQ(result.metrics.ticks, 65U);
}

TEST(ReplayTest, AnUntilBeyondTheEndIsTheWholeRunAndZeroIsJustTheStart) {
  const Recording recording = busyRecording("demo", 2, 100, 25);
  ReplayOptions beyond;
  beyond.until = 5000;
  const ReplayResult whole_run = replay(recording, beyond);
  ASSERT_TRUE(whole_run.ok());
  EXPECT_EQ(whole_run.ticks, 100U);
  EXPECT_TRUE(whole_run.endVerified);
  ReplayOptions none;
  none.until = 0;
  const ReplayResult start = replay(recording, none);
  ASSERT_TRUE(start.ok());
  EXPECT_EQ(start.ticks, 0U);
  EXPECT_EQ(start.checkpointsVerified, 1U);
  EXPECT_FALSE(start.endVerified);
}

TEST(ReplayTest, AReplayCutShortAtACommandsTickAppliesTheCommandsOfThatTick) {
  const Recording recording = busyRecording("compact", 3, 100, 25);
  ASSERT_FALSE(recording.commands.empty());
  const Tick tick = recording.commands[2].tick;  // some tick with a command
  ReplayOptions options;
  options.until = tick;
  const ReplayResult result = replay(recording, options);
  ASSERT_TRUE(result.ok()) << result.divergence->what;
  // Every command given at that tick has been applied, so what they made has been seen, and the
  // step that would begin at that tick has not been taken.
  std::vector<TaskEvent> expected;
  for (const TaskEvent& event : recording.taskEvents) {
    if (event.tick < tick || (event.tick == tick && event.source == EventSource::FromCommand)) {
      expected.push_back(event);
    }
  }
  ASSERT_GE(expected.size(), 2U);
  EXPECT_TRUE(result.taskEvents == expected);
}

TEST(ReplayTest, WithoutVerifyingTheReplayJustPlays) {
  Recording tampered = busyRecording("compact", 4, 120, 30);
  const std::uint64_t honest = replay(tampered).checksum;
  for (Checkpoint& checkpoint : tampered.checkpoints) {
    checkpoint.checksum ^= 1;
  }
  tampered.end->checksum ^= 1;
  tampered.commands[0].accepted = !tampered.commands[0].accepted;
  tampered.taskEvents.pop_back();
  ReplayOptions options;
  options.verify = false;
  const ReplayResult result = replay(tampered, options);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.checksum, honest);
  EXPECT_EQ(result.checkpointsVerified, 0U);
  EXPECT_FALSE(result.endVerified);
  EXPECT_FALSE(replay(tampered).ok());
}

TEST(ReplayTest, ARecordingWithNothingToCheckIsPlayedAndSaysNothingWasChecked) {
  Recording script = busyRecording("compact", 4, 120, 30);
  script.checkpoints.clear();
  script.end.reset();
  script.taskEvents.clear();
  script.taskEventsRecorded = false;  // as in a file that leaves them out
  const ReplayResult result = replay(script);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.checkpointsVerified, 0U);
  EXPECT_FALSE(result.endVerified);
  EXPECT_EQ(result.ticks, 120U);
  EXPECT_EQ(result.checksum, replay(busyRecording("compact", 4, 120, 30)).checksum);
}

// ---- finding where a replay goes wrong ----------------------------------------------------------

Recording honest() { return busyRecording("demo", 1, 150, 25); }

TEST(ReplayDivergenceTest, ARecordingThatIsHonestDoesNotDiverge) {
  const ReplayResult result = replay(honest());
  EXPECT_TRUE(result.ok());
  EXPECT_FALSE(result.divergence.has_value());
}

TEST(ReplayDivergenceTest, AChecksumThatDiffersIsFoundAtItsTickAndTheReplayStopsThere) {
  Recording recording = honest();
  auto checkpoint = std::find_if(recording.checkpoints.begin(), recording.checkpoints.end(),
                                 [](const Checkpoint& c) { return c.tick == 100; });
  ASSERT_TRUE(checkpoint != recording.checkpoints.end());
  const std::uint64_t truth = checkpoint->checksum;
  checkpoint->checksum ^= 0x10;
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, 100U);
  EXPECT_TRUE(mentions(result, "the state at tick 100 has checksum " + checksumText(truth)))
      << result.divergence->what;
  EXPECT_TRUE(mentions(result, "the recording says " + checksumText(truth ^ 0x10)))
      << result.divergence->what;
  EXPECT_EQ(result.ticks, 100U);
  EXPECT_EQ(result.checkpointsVerified, 4U);  // ticks 0, 25, 50, 75
  EXPECT_FALSE(result.endVerified);
  EXPECT_EQ(result.snapshot.tick, 100U);
}

TEST(ReplayDivergenceTest, ADifferentEndStateIsFoundAtTheEnd) {
  Recording recording = honest();
  recording.end->checksum += 1;
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, 150U);
  EXPECT_TRUE(mentions(result, "the state at the end of the run (tick 150)"))
      << result.divergence->what;
  EXPECT_EQ(result.checkpointsVerified, recording.checkpoints.size());
  EXPECT_FALSE(result.endVerified);
}

TEST(ReplayDivergenceTest, ADifferentSeedIsFoundAtTickZero) {
  Recording recording = honest();
  recording.scenario.config.seed += 1;
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, 0U);
  EXPECT_TRUE(mentions(result, "the state at tick 0 has checksum")) << result.divergence->what;
  EXPECT_EQ(result.ticks, 0U);
}

TEST(ReplayDivergenceTest, WithoutChecksumsTheTasksStillGiveItAway) {
  Recording recording = honest();
  recording.scenario.config.seed += 1;
  recording.checkpoints.clear();
  recording.end.reset();
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, 0U);  // the first task of the other seed is made at tick 0
  EXPECT_TRUE(mentions(result, "the recording has task #1 created at tick 0"))
      << result.divergence->what;
  EXPECT_TRUE(mentions(result, "but the replay made task #1 created at tick 0"))
      << result.divergence->what;
  EXPECT_LE(result.ticks, 1U);
}

TEST(ReplayDivergenceTest, ADifferentSettingIsFoundAtTheFirstCheckpointAfterItMatters) {
  Recording recording = honest();
  recording.scenario.config.robot.maxSpeed = 0.5;
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_GE(result.divergence->tick, 1U);  // not at the start: the speed is not in the state yet
  EXPECT_LE(result.divergence->tick, 25U);
  EXPECT_EQ(result.checkpointsVerified, 1U);  // only tick 0, the same whatever the speed
}

TEST(ReplayDivergenceTest, ACommandThatIsAcceptedInOneAndRefusedInTheOtherIsFoundAtItsTick) {
  Recording recording = honest();
  const auto refusedIndex = std::find_if(recording.commands.begin(), recording.commands.end(),
                                         [](const auto& entry) { return !entry.accepted; });
  ASSERT_TRUE(refusedIndex != recording.commands.end());
  const Tick tick = refusedIndex->tick;
  refusedIndex->accepted = true;  // the recording claims the engine took what it refused
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, tick);
  EXPECT_TRUE(mentions(result, "was accepted in the recording, but the replay refused it"))
      << result.divergence->what;
  EXPECT_TRUE(mentions(result, std::string("(") + commandName(refusedIndex->command) + ")"))
      << result.divergence->what;

  Recording other = honest();
  auto acceptedIndex = std::find_if(other.commands.begin(), other.commands.end(),
                                    [](const auto& entry) { return entry.accepted; });
  acceptedIndex->accepted = false;
  const ReplayResult second = replay(other);
  ASSERT_FALSE(second.ok());
  EXPECT_EQ(second.divergence->tick, acceptedIndex->tick);
  EXPECT_TRUE(mentions(second, "was refused in the recording, but the replay accepted it"))
      << second.divergence->what;
}

TEST(ReplayDivergenceTest, ACommandThatWasChangedShowsInTheTaskItMade) {
  Recording recording = honest();
  auto submit =
      std::find_if(recording.commands.begin(), recording.commands.end(), [](const auto& entry) {
        return std::holds_alternative<SubmitTask>(entry.command) && entry.accepted;
      });
  ASSERT_TRUE(submit != recording.commands.end());
  auto& command = std::get<SubmitTask>(submit->command);
  command.priority = command.priority == TaskPriority::Low ? TaskPriority::High : TaskPriority::Low;
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, submit->tick);
  EXPECT_EQ(result.ticks, submit->tick) << "found as soon as the command was applied";
  EXPECT_TRUE(mentions(result, "the recording has task #")) << result.divergence->what;
}

TEST(ReplayDivergenceTest, ACommandThatWasLeftOutIsFoundAsTheTaskThatIsMissing) {
  Recording recording = honest();
  const auto submits = [](const auto& entry) {
    return std::holds_alternative<SubmitTask>(entry.command);
  };
  const Tick tick =
      std::find_if(recording.commands.begin(), recording.commands.end(), submits)->tick;
  std::erase_if(recording.commands,
                [&](const auto& entry) { return entry.tick == tick && submits(entry); });
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, tick);
  EXPECT_EQ(result.ticks, tick) << "found as soon as the commands of that tick were applied";
  EXPECT_TRUE(mentions(result, "but the replay made nothing then")) << result.divergence->what;

  // Only some of them left out: the task that is made is not the one the recording says.
  Recording one = honest();
  const auto first = std::find_if(one.commands.begin(), one.commands.end(), [](const auto& entry) {
    return std::holds_alternative<SubmitTask>(entry.command) && entry.accepted;
  });
  one.commands.erase(first);
  const ReplayResult other = replay(one);
  ASSERT_FALSE(other.ok());
  EXPECT_TRUE(mentions(other, "but the replay made task #")) << other.divergence->what;
}

TEST(ReplayDivergenceTest, ATaskTheReplayDoesNotMakeIsFoundWhereItShouldHaveBeen) {
  Recording recording = honest();
  TaskEvent invented;
  invented.tick = 149;
  invented.kind = TaskEventKind::Created;
  invented.source = EventSource::Engine;
  // Numbered as the next task, so that the recording is consistent with itself.
  std::uint32_t created = 0;
  for (const TaskEvent& event : recording.taskEvents) {
    created += event.kind == TaskEventKind::Created ? 1U : 0U;
  }
  invented.task = TaskId{created + 1};
  invented.pickup = P(1, 1);
  invented.dropoff = P(2, 2);
  recording.taskEvents.push_back(invented);
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_TRUE(mentions(result, "but the replay made nothing then")) << result.divergence->what;
}

TEST(ReplayDivergenceTest, ATaskTheRecordingDoesNotHaveIsFound) {
  Recording recording = honest();
  recording.taskEvents.pop_back();
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_TRUE(mentions(result, "which the recording does not have")) << result.divergence->what;
}

TEST(ReplayDivergenceTest, ARecordingOfAnotherEngineSaysSoWhenItDiverges) {
  Recording recording = honest();
  recording.engineRevision = 0;
  recording.checkpoints[1].checksum ^= 1;
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_TRUE(mentions(
      result,
      "is from engine revision 0 and this is revision 2, so a difference is to be expected"))
      << result.divergence->what;
  // And one of another revision that still replays exactly is not accused of anything.
  Recording fine = honest();
  fine.engineRevision = 0;
  EXPECT_TRUE(replay(fine).ok());
}

TEST(ReplayDivergenceTest, ARecordingThatIsNotConsistentIsSaidToBeNotValidAndNotPlayed) {
  Recording recording = honest();
  recording.commands.back().tick = 9999;
  const ReplayResult result = replay(recording);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.divergence->tick, 0U);
  EXPECT_TRUE(mentions(result, "the recording is not valid: /commands/"))
      << result.divergence->what;
  EXPECT_EQ(result.ticks, 0U);
}

TEST(ReplayDivergenceTest, AScenarioThatCannotBeBuiltIsSaidSoNotThrown) {
  Recording recording = honest();
  recording.scenario.robotStarts[0] = P(-4, -4);
  ReplayResult result;
  ASSERT_NO_THROW(result = replay(recording));
  ASSERT_FALSE(result.ok());
  EXPECT_TRUE(mentions(result, "the scenario cannot be built: robot start cell is not walkable"))
      << result.divergence->what;
  recording = honest();
  recording.scenario.config.deadlockTicks = 0;
  ASSERT_NO_THROW(result = replay(recording));
  EXPECT_TRUE(mentions(result, "the scenario cannot be built:"));
}

// ---- a recording made earlier -------------------------------------------------------------------

// Recorded with this program (compact layout, seed 3, 90 ticks, commands of every kind) when replay
// was written, and kept as it was. It must go on replaying to the same checksums, and go on being
// read and written as the same text. If it stops, either the engine changed what a simulation does
// (then make a new one, raise kEngineRevision and update ThePinnedScenarioHasThePinnedChecksums in
// state_checksum_test.cpp, in the same change) or the format changed (then it needs a new version).
const char* const kEarlierRecording = R"({
  "format": "warehousesim-recording",
  "version": 1,
  "engine": 1,
  "scenario": {
    "seed": 3,
    "layout": [
      "############",
      "#C.......DD#",
      "#.SSSSSS...#",
      "#..........#",
      "#.SSSSSS...#",
      "#..........#",
      "#PP.......C#",
      "############"
    ],
    "robots": [[1,1],[10,6]],
    "config": {"abandonTicks":150,"cooperative":true,"deadlockTicks":6,"maxBlockedTicks":3,"maxDetourCells":4,"maxDodgeCells":12,"planningWorkPerSearch":1500,"planningWorkPerStep":4000,"priorityBoostTicks":40,"prolongedBlockTicks":20,"replanInterval":4,"reservationHorizon":12,"reservations":true,"robot":{"acceleration":0.5,"batteryCapacity":10000,"chargePerTick":250,"deceleration":1.0,"handlingDrainPerTick":2,"idleDrainPerTick":1,"initialCharge":null,"loadTicks":2,"lowBatteryLevel":3000,"maxSpeed":1.0,"moveCostPerCell":15,"turnRate":1.5707963267948966,"unloadTicks":2},"taskSpawnInterval":5,"varyTaskPriority":true}
  },
  "ticks": 90,
  "checkpointInterval": 30,
  "commands": [
    {"accepted":true,"command":"submitTask","dropoff":[9,1],"pickup":[2,1],"priority":"high","tick":10},
    {"accepted":true,"command":"submitTask","dropoff":[10,1],"pickup":[9,6],"priority":"low","tick":10},
    {"accepted":false,"command":"submitTask","dropoff":[9,1],"pickup":[-1,0],"priority":"normal","tick":10},
    {"accepted":true,"command":"setTaskPriority","priority":"urgent","task":3,"tick":20},
    {"accepted":true,"command":"cancelTask","task":4,"tick":20},
    {"accepted":false,"command":"cancelTask","task":100000,"tick":20},
    {"accepted":true,"command":"setRobotService","inService":false,"robot":1,"tick":30},
    {"accepted":true,"command":"setRobotService","inService":true,"robot":1,"tick":45},
    {"accepted":true,"cell":[2,1],"command":"placeObstacle","tick":50,"type":"wall"},
    {"accepted":true,"cell":[2,1],"command":"clearObstacle","tick":70}
  ],
  "taskEvents": [
    {"dropoff":[10,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":1,"tick":0},
    {"dropoff":[9,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":2,"tick":5},
    {"dropoff":[9,1],"event":"created","pickup":[2,1],"priority":"high","source":"command","task":3,"tick":10},
    {"dropoff":[10,1],"event":"created","pickup":[9,6],"priority":"low","source":"command","task":4,"tick":10},
    {"dropoff":[9,1],"event":"created","pickup":[1,6],"priority":"normal","source":"engine","task":5,"tick":10},
    {"dropoff":[9,1],"event":"created","pickup":[1,6],"priority":"low","source":"engine","task":6,"tick":15},
    {"event":"cancelled","source":"command","task":4,"tick":20},
    {"dropoff":[10,1],"event":"created","pickup":[1,6],"priority":"low","source":"engine","task":7,"tick":20},
    {"dropoff":[9,1],"event":"created","pickup":[1,6],"priority":"normal","source":"engine","task":8,"tick":25},
    {"dropoff":[10,1],"event":"created","pickup":[1,6],"priority":"normal","source":"engine","task":9,"tick":30},
    {"dropoff":[10,1],"event":"created","pickup":[1,6],"priority":"normal","source":"engine","task":10,"tick":35},
    {"dropoff":[10,1],"event":"created","pickup":[1,6],"priority":"low","source":"engine","task":11,"tick":40},
    {"dropoff":[10,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":12,"tick":45},
    {"event":"cancelled","source":"command","task":3,"tick":50},
    {"dropoff":[10,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":13,"tick":50},
    {"dropoff":[10,1],"event":"created","pickup":[1,6],"priority":"low","source":"engine","task":14,"tick":55},
    {"dropoff":[9,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":15,"tick":60},
    {"dropoff":[10,1],"event":"created","pickup":[1,6],"priority":"normal","source":"engine","task":16,"tick":65},
    {"dropoff":[9,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":17,"tick":70},
    {"dropoff":[10,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":18,"tick":75},
    {"dropoff":[9,1],"event":"created","pickup":[1,6],"priority":"normal","source":"engine","task":19,"tick":80},
    {"dropoff":[10,1],"event":"created","pickup":[2,6],"priority":"normal","source":"engine","task":20,"tick":85}
  ],
  "checkpoints": [
    {"checksum":"c0a6ccad6d1a8282","tick":0},
    {"checksum":"8a08f727977f2723","tick":30},
    {"checksum":"3b34ce571eddebe1","tick":60},
    {"checksum":"e4f3e1b2ed13e6ae","tick":90}
  ],
  "end": {"checksum":"e4f3e1b2ed13e6ae","tick":90}
}
)";

TEST(ReplayTest, ARecordingMadeEarlierStillReplaysToTheSameChecksums) {
  const auto load = parseRecording(kEarlierRecording);
  ASSERT_TRUE(load.ok()) << load.problem->text();
  const ReplayResult result = replay(*load.recording);
  ASSERT_TRUE(result.ok()) << result.divergence->what;
  EXPECT_EQ(result.ticks, 90U);
  EXPECT_EQ(result.checkpointsVerified, 4U);
  EXPECT_TRUE(result.endVerified);
  EXPECT_EQ(checksumText(result.checksum), "e4f3e1b2ed13e6ae");
  EXPECT_EQ(serializeRecording(*load.recording), kEarlierRecording);
}

// ---- the property: what is recorded replays, whatever was done ----------------------------------

void applyRandomCommand(Recorder& recorder, Rng& rng) {
  const Simulation& sim = recorder.simulation();
  const auto width = static_cast<std::uint64_t>(sim.warehouse().width());
  const auto height = static_cast<std::uint64_t>(sim.warehouse().height());
  const auto cell = [&] {  // now and then off the map
    return Point{static_cast<int>(rng.below(width + 2)) - 1,
                 static_cast<int>(rng.below(height + 2)) - 1};
  };
  const auto task = [&] {
    return TaskId{static_cast<std::uint32_t>(rng.below(sim.tasks().all().size() + 3))};
  };
  const auto priority = [&] { return static_cast<TaskPriority>(rng.below(4)); };
  switch (rng.below(6)) {
    case 0:
      (void)recorder.apply(SubmitTask{cell(), cell(), priority()});
      break;
    case 1:
      (void)recorder.apply(SetTaskPriority{task(), priority()});
      break;
    case 2:
      (void)recorder.apply(CancelTask{task()});
      break;
    case 3:
      (void)recorder.apply(
          SetRobotService{RobotId{static_cast<std::uint32_t>(rng.below(sim.robots().size() + 2))},
                          rng.below(2) == 0});
      break;
    case 4: {
      const CellType types[] = {CellType::Wall, CellType::Shelf, CellType::Pickup};
      (void)recorder.apply(PlaceObstacle{cell(), types[rng.below(3)]});
      break;
    }
    default:
      (void)recorder.apply(ClearObstacle{cell()});
      break;
  }
}

TEST(ReplayPropertyTest, AnyRunWithAnyCommandsReplaysExactly) {
  const std::vector<std::string> layouts = bundledLayouts();
  std::size_t accepted = 0;
  std::size_t refused = 0;
  for (std::uint64_t seed = 1; seed <= 36; ++seed) {
    Rng rng(seed * 7919);
    Recorder recorder(
        *warehousesim::simulation::makeLayoutSimulation(layouts[seed % layouts.size()], seed), 20);
    for (int tick = 0; tick < 140; ++tick) {
      const std::uint64_t commands = rng.below(6) == 0 ? 1 + rng.below(3) : 0;
      for (std::uint64_t i = 0; i < commands; ++i) {
        applyRandomCommand(recorder, rng);
      }
      recorder.step();
    }
    const Recording recording = recorder.recording();
    ASSERT_FALSE(warehousesim::simulation::validate(recording).has_value()) << "seed " << seed;
    for (const auto& entry : recording.commands) {
      (entry.accepted ? accepted : refused) += 1;
    }
    const ReplayResult result = replay(recording);
    ASSERT_TRUE(result.ok()) << "seed " << seed << ": " << result.divergence->what;
    EXPECT_EQ(result.checksum, recorder.simulation().stateChecksum()) << "seed " << seed;
    EXPECT_TRUE(result.snapshot == recorder.simulation().snapshot()) << "seed " << seed;
    EXPECT_TRUE(result.endVerified);
  }
  EXPECT_GT(accepted, 60U) << "the commands should do something";
  EXPECT_GT(refused, 60U) << "and some should be refused";
}

TEST(ReplayPropertyTest, AnyRecordingAtAllReplaysWithoutThrowingWhateverItSays) {
  Rng rng(2024);
  const std::vector<std::string> layouts = bundledLayouts();
  std::size_t built = 0;
  for (int round = 0; round < 120; ++round) {
    Recording recording = busyRecording(
        layouts[static_cast<std::size_t>(rng.below(layouts.size()))], 1 + rng.below(5), 90, 30);
    recording.ticks = 40 + rng.below(40);
    // Cut it short (still consistent), then garble it in ways the file reader would catch and
    // some it would not.
    const Tick ticks = recording.ticks;
    std::erase_if(recording.commands, [ticks](const auto& entry) { return entry.tick > ticks; });
    std::erase_if(recording.taskEvents,
                  [ticks](const TaskEvent& event) { return event.tick > ticks; });
    std::erase_if(recording.checkpoints, [ticks](const Checkpoint& c) { return c.tick > ticks; });
    recording.end->tick = ticks;
    const auto some = [&rng](auto& list) -> auto& {
      return list[static_cast<std::size_t>(rng.below(list.size()))];
    };
    switch (rng.below(7)) {
      case 0:
        some(recording.commands).tick = rng.below(recording.ticks + 5);
        break;
      case 1: {
        auto& entry = some(recording.commands);
        entry.accepted = !entry.accepted;
        break;
      }
      case 2:
        some(recording.checkpoints).tick = rng.below(100);
        break;
      case 3:
        recording.scenario.robotStarts.push_back(Point{static_cast<int>(rng.below(30)) - 5, 1});
        break;
      case 4:
        recording.scenario.config.reservationHorizon = rng.below(4);
        break;
      default:  // left as it is, but cut short
        break;
    }
    ReplayOptions options;
    options.verify = rng.below(2) == 0;
    ReplayResult result;
    ASSERT_NO_THROW(result = replay(recording, options)) << "round " << round;
    built += result.ticks > 0 ? 1U : 0U;
    if (!result.ok()) {
      EXPECT_FALSE(result.divergence->what.empty());
    }
  }
  EXPECT_GT(built, 20U) << "most should be able to run at all";
}

}  // namespace
