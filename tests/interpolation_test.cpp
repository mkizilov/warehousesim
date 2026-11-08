#include "warehousesim/view/interpolation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

#include "warehousesim/core/angle.hpp"
#include "warehousesim/core/fixed_timestep.hpp"
#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/vec2.hpp"
#include "warehousesim/robots/robot_state.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/simulation.hpp"

namespace {

using warehousesim::core::angleOf;
using warehousesim::core::Direction;
using warehousesim::core::FixedTimestep;
using warehousesim::core::kPi;
using warehousesim::core::Point;
using warehousesim::core::RobotId;
using warehousesim::core::Vec2;
using warehousesim::robots::RobotState;
using warehousesim::simulation::RobotPose;
using warehousesim::simulation::RobotSnapshot;
using warehousesim::simulation::Snapshot;
using warehousesim::view::blend;

Point P(int x, int y) { return Point{x, y}; }

RobotSnapshot robotAt(std::uint32_t id, Point cell, Vec2 center, double heading) {
  RobotSnapshot robot;
  robot.id = RobotId{id};
  robot.position = cell;
  robot.pose = RobotPose{center, heading};
  return robot;
}

Snapshot snapshotOf(warehousesim::core::Tick tick, std::vector<RobotSnapshot> robots) {
  Snapshot snapshot;
  snapshot.tick = tick;
  snapshot.robots = std::move(robots);
  return snapshot;
}

// ---- blend --------------------------------------------------------------------------------------

TEST(BlendTest, AlphaZeroShowsThePreviousPoseAndAlphaOneTheLatestExactly) {
  const Snapshot before = snapshotOf(4, {robotAt(1, P(0, 0), {0.5, 0.5}, 0.0)});
  const Snapshot after = snapshotOf(5, {robotAt(1, P(1, 0), {1.5, 0.5}, 0.0)});
  const Snapshot start = blend(before, after, 0.0);
  const Snapshot end = blend(before, after, 1.0);
  const Snapshot quarter = blend(before, after, 0.25);
  ASSERT_TRUE(start.robots[0].pose.has_value());
  EXPECT_EQ(start.robots[0].pose->center, (Vec2{0.5, 0.5}));
  EXPECT_EQ(end.robots[0].pose->center, (Vec2{1.5, 0.5}));
  EXPECT_NEAR(quarter.robots[0].pose->center.x, 0.75, 1e-12);
  EXPECT_NEAR(quarter.robots[0].pose->center.y, 0.5, 1e-12);
}

TEST(BlendTest, OnlyTheMotionIsInterpolatedEverythingElseIsTheLatestState) {
  RobotSnapshot a = robotAt(1, P(0, 0), {0.5, 0.5}, 0.0);
  a.state = RobotState::Idle;
  a.batteryLevel = 900;
  RobotSnapshot b = robotAt(1, P(1, 0), {1.5, 0.5}, 0.0);
  b.state = RobotState::Navigating;
  b.batteryLevel = 850;
  b.carrying = true;
  b.deliveries = 3;
  Snapshot before = snapshotOf(7, {a});
  Snapshot after = snapshotOf(8, {b});
  after.completedTasks = 12;
  after.cancelledTasks = 2;

  const Snapshot shown = blend(before, after, 0.5);
  EXPECT_EQ(shown.tick, 8U);
  EXPECT_EQ(shown.completedTasks, 12U);
  EXPECT_EQ(shown.cancelledTasks, 2U);
  EXPECT_EQ(shown.robots[0].position, P(1, 0));  // the logical cell is the latest one
  EXPECT_EQ(shown.robots[0].state, RobotState::Navigating);
  EXPECT_EQ(shown.robots[0].batteryLevel, 850U);
  EXPECT_TRUE(shown.robots[0].carrying);
  EXPECT_EQ(shown.robots[0].deliveries, 3U);
  EXPECT_NEAR(shown.robots[0].pose->center.x, 1.0, 1e-12);  // only the pose is half way
}

TEST(BlendTest, AHeadingTurnsTheShortWayRound) {
  const Snapshot before =
      snapshotOf(1, {robotAt(1, P(0, 0), {0.5, 0.5}, angleOf(Direction::West))});
  const Snapshot after =
      snapshotOf(2, {robotAt(1, P(0, 0), {0.5, 0.5}, angleOf(Direction::North))});
  const Snapshot shown = blend(before, after, 0.5);
  // Half way from west to north is north-west, not the long way through south and east.
  EXPECT_NEAR(shown.robots[0].pose->heading, -3.0 * kPi / 4.0, 1e-12);
  EXPECT_DOUBLE_EQ(blend(before, after, 1.0).robots[0].pose->heading, angleOf(Direction::North));
  EXPECT_DOUBLE_EQ(blend(before, after, 0.0).robots[0].pose->heading, angleOf(Direction::West));
}

TEST(BlendTest, AlphaIsClampedAndNonsenseMeansTheLatestState) {
  const Snapshot before = snapshotOf(1, {robotAt(1, P(0, 0), {0.5, 0.5}, 0.0)});
  const Snapshot after = snapshotOf(2, {robotAt(1, P(1, 0), {1.5, 0.5}, 0.0)});
  EXPECT_EQ(blend(before, after, -3.0).robots[0].pose->center, (Vec2{0.5, 0.5}));
  EXPECT_EQ(blend(before, after, 9.0).robots[0].pose->center, (Vec2{1.5, 0.5}));
  EXPECT_EQ(blend(before, after, std::numeric_limits<double>::quiet_NaN()).robots[0].pose->center,
            (Vec2{1.5, 0.5}));
}

TEST(BlendTest, ARobotThatWasMovedFarIsNotSweptThroughTheCellsBetween) {
  const Snapshot before = snapshotOf(1, {robotAt(1, P(0, 0), {0.5, 0.5}, 0.0)});
  const Snapshot after = snapshotOf(2, {robotAt(1, P(5, 0), {5.5, 0.5}, 0.0)});
  for (const double alpha : {0.0, 0.3, 0.9}) {
    EXPECT_EQ(blend(before, after, alpha).robots[0].pose->center, (Vec2{5.5, 0.5})) << alpha;
  }
  // One cell aside is a move the picture glides through.
  const Snapshot aside = snapshotOf(2, {robotAt(1, P(1, 0), {1.5, 0.5}, 0.0)});
  EXPECT_EQ(blend(before, aside, 0.0).robots[0].pose->center, (Vec2{0.5, 0.5}));
}

TEST(BlendTest, RobotsWithoutAPoseRestInTheirCellAndNewRobotsAppearWhereTheyAre) {
  RobotSnapshot resting;  // never given a pose: at rest in the middle of its cell
  resting.id = RobotId{1};
  resting.position = P(2, 1);
  RobotSnapshot newcomer = robotAt(2, P(4, 4), {4.5, 4.5}, 0.0);
  const Snapshot before = snapshotOf(1, {resting});
  const Snapshot after = snapshotOf(2, {resting, newcomer});
  const Snapshot shown = blend(before, after, 0.5);
  ASSERT_EQ(shown.robots.size(), 2U);
  EXPECT_EQ(shown.robots[0].pose->center, (Vec2{2.5, 1.5}));
  EXPECT_DOUBLE_EQ(shown.robots[0].pose->heading, angleOf(Direction::North));
  EXPECT_EQ(shown.robots[1].pose->center, (Vec2{4.5, 4.5}));  // nothing to interpolate from
}

TEST(BlendTest, TheFirstFrameBeforeAnyStepShowsTheStartState) {
  const Snapshot start = snapshotOf(0, {robotAt(1, P(3, 3), {3.5, 3.5}, 0.0)});
  for (const double alpha : {0.0, 0.5, 1.0}) {
    EXPECT_EQ(blend(start, start, alpha).robots[0].pose->center, (Vec2{3.5, 3.5}));
  }
}

// ---- the picture does not depend on the frame rate ------------------------------------

// The simulation state after every tick, 0..ticks.
std::vector<Snapshot> record(int ticks) {
  auto sim = warehousesim::simulation::makeDemoSimulation(1);
  std::vector<Snapshot> states = {sim.snapshot()};
  for (int i = 0; i < ticks; ++i) {
    sim.step();
    states.push_back(sim.snapshot());
  }
  return states;
}

// What the GUI does for a window that redraws every `frameSeconds`: the simulation owes whole
// steps for the elapsed time and the picture is the blend of the last two by how far into the
// next step the clock is.
struct Frame {
  double seconds;
  std::vector<Vec2> poses;
};
std::vector<Frame> playback(const std::vector<Snapshot>& states, double stepSeconds,
                            double frameSeconds, int frames) {
  FixedTimestep stepper(stepSeconds);
  std::size_t tick = 0;
  std::vector<Frame> shown;
  for (int i = 1; i <= frames; ++i) {
    tick += stepper.advance(frameSeconds);
    if (tick >= states.size()) {
      break;
    }
    const Snapshot& previous = states[tick == 0 ? 0 : tick - 1];
    const Snapshot picture = blend(previous, states[tick], tick == 0 ? 0.0 : stepper.alpha());
    Frame frame{frameSeconds * i, {}};
    for (const RobotSnapshot& robot : picture.robots) {
      frame.poses.push_back(robot.center());
    }
    shown.push_back(std::move(frame));
  }
  return shown;
}

// Where the picture should be at real time `seconds` however it is sampled: the simulation's own
// trajectory (poses at whole ticks, straight lines between them), one step behind real time.
std::vector<Vec2> trajectoryAt(const std::vector<Snapshot>& states, double stepSeconds,
                               double seconds) {
  const double shownTick = std::max(0.0, seconds / stepSeconds - 1.0);
  const auto index = static_cast<std::size_t>(std::floor(shownTick + 1e-9));
  const double fraction = std::clamp(shownTick - static_cast<double>(index), 0.0, 1.0);
  const Snapshot& from = states[index];
  const Snapshot& to = states[std::min(index + 1, states.size() - 1)];
  std::vector<Vec2> poses;
  for (std::size_t r = 0; r < from.robots.size(); ++r) {
    poses.push_back(
        warehousesim::core::lerp(from.robots[r].center(), to.robots[r].center(), fraction));
  }
  return poses;
}

TEST(FrameRateTest, ThePictureAtAGivenMomentIsTheSameAtEveryFrameRate) {
  const std::vector<Snapshot> states = record(160);
  constexpr double kStep = 0.1;  // seconds of real time per simulation step
  for (const double fps : {10.0, 24.0, 30.0, 60.0, 75.0, 144.0, 240.0}) {
    const double frame = 1.0 / fps;
    const std::vector<Frame> shown = playback(states, kStep, frame, static_cast<int>(12.0 * fps));
    ASSERT_GT(shown.size(), 100U) << fps;
    for (const Frame& f : shown) {
      const std::vector<Vec2> expected = trajectoryAt(states, kStep, f.seconds);
      ASSERT_EQ(f.poses.size(), expected.size());
      for (std::size_t r = 0; r < expected.size(); ++r) {
        ASSERT_NEAR(f.poses[r].x, expected[r].x, 1e-6) << "fps " << fps << " t " << f.seconds;
        ASSERT_NEAR(f.poses[r].y, expected[r].y, 1e-6) << "fps " << fps << " t " << f.seconds;
      }
    }
  }
}

TEST(FrameRateTest, MotionIsSmoothAtAnyFrameRateAndNeverFasterThanTheRobotCanGo) {
  const std::vector<Snapshot> states = record(160);
  constexpr double kStep = 0.1;
  std::vector<std::size_t> distinctPositions;
  for (const double fps : {10.0, 30.0, 60.0, 144.0}) {
    const double frame = 1.0 / fps;
    const std::vector<Frame> shown = playback(states, kStep, frame, static_cast<int>(12.0 * fps));
    const double maxSpeedCellsPerSecond = 1.0 / kStep;  // one cell per tick at most
    std::set<long long> distinctX;
    for (std::size_t i = 1; i < shown.size(); ++i) {
      for (std::size_t r = 0; r < shown[i].poses.size(); ++r) {
        const Vec2 a = shown[i - 1].poses[r];
        const Vec2 b = shown[i].poses[r];
        // Between two frames a robot moves at most as far as its top speed allows in that time.
        ASSERT_LE(std::hypot(b.x - a.x, b.y - a.y), maxSpeedCellsPerSecond * frame + 1e-6)
            << "fps " << fps;
        if (r == 2) {
          distinctX.insert(std::llround(b.x * 1e6));
        }
      }
    }
    distinctPositions.push_back(distinctX.size());
  }
  // More frames show more intermediate positions of the same journey: the picture is
  // oversampled, not jumpy, as the frame rate rises.
  EXPECT_LT(distinctPositions[0], distinctPositions[1]);
  EXPECT_LT(distinctPositions[1], distinctPositions[2]);
  EXPECT_LT(distinctPositions[2], distinctPositions[3]);
}

TEST(FrameRateTest, AllFrameRatesEndAtTheSameStateAfterTheSameTime) {
  const std::vector<Snapshot> states = record(160);
  constexpr double kStep = 0.1;
  std::vector<Vec2> reference;
  for (const double fps : {20.0, 50.0, 100.0, 200.0}) {  // all divide a step exactly
    const std::vector<Frame> shown =
        playback(states, kStep, 1.0 / fps, static_cast<int>(std::lround(9.3 * fps)));
    ASSERT_FALSE(shown.empty());
    if (reference.empty()) {
      reference = shown.back().poses;
    }
    ASSERT_EQ(shown.back().poses.size(), reference.size());
    for (std::size_t r = 0; r < reference.size(); ++r) {
      EXPECT_NEAR(shown.back().poses[r].x, reference[r].x, 1e-6) << fps;
      EXPECT_NEAR(shown.back().poses[r].y, reference[r].y, 1e-6) << fps;
    }
  }
}

}  // namespace
