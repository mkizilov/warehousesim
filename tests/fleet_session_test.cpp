#include "warehousesim/view/fleet_session.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "scenario_support.hpp"
#include "warehousesim/core/rng.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/scenarios.hpp"
#include "warehousesim/simulation/serialization.hpp"
#include "warehousesim/view/fleet_scene.hpp"

namespace {

using namespace warehousesim_test;
using std::chrono::milliseconds;
using warehousesim::simulation::apply;
using warehousesim::simulation::SetTaskPriority;
using warehousesim::simulation::SimulationRunner;
using warehousesim::simulation::SubmitTask;
using warehousesim::view::buildFleetScene;
using warehousesim::view::FleetController;
using warehousesim::view::FleetKey;
using warehousesim::view::FleetSession;
using warehousesim::view::InputEvents;
using warehousesim::view::PointerAction;
using warehousesim::view::PointerButton;
using warehousesim::view::ScreenPoint;
using warehousesim::view::ScreenRect;
using warehousesim::view::ScreenSize;
using warehousesim::view::Text;

constexpr milliseconds kTimeout{15000};
constexpr ScreenSize kWindow{1280, 720};

std::string json(const warehousesim::simulation::Snapshot& snapshot) {
  return warehousesim::simulation::toJson(snapshot).dump();
}

// The GUI without the window: a runner, the fleet controller and the session that joins them,
// driven with the same InputEvents the SDL window would produce.
struct Gui {
  SimulationRunner runner;
  FleetController controller;
  FleetSession session;
  warehousesim::simulation::Snapshot drawn;

  explicit Gui(std::uint64_t seed, double speed = 1.0)
      : runner(warehousesim::simulation::makeDemoSimulation(seed), milliseconds(1),
               *warehousesim::simulation::layoutScenario("demo")),
        controller(kWindow, runner.warehouse(), speed),
        session(runner, controller) {
    runner.setSpeed(controller.speed());
    frame({});
  }

  void frame(const InputEvents& input) { drawn = session.step(input, 0.016); }
  void key(FleetKey k) {
    InputEvents input;
    input.fleetKeys.push_back(k);
    frame(input);
  }
  ScreenPoint cellCentre(Point cell) const {
    const ScreenRect r = controller.camera().cellRect(cell);
    return {r.x + r.width / 2.0, r.y + r.height / 2.0};
  }
  void clickCell(Point cell) {
    const ScreenPoint at = cellCentre(cell);
    InputEvents input;
    input.pointerActions = {{PointerAction::Kind::Press, PointerButton::Primary, at},
                            {PointerAction::Kind::Release, PointerButton::Primary, at}};
    frame(input);
  }
  void pause() {
    InputEvents input;
    input.togglePause = true;
    frame(input);
  }
  // Frames until the runner's log has `count` records (the worker applies commands on its own).
  bool waitForLog(std::size_t count) {
    for (int i = 0; i < 5000; ++i) {
      if (runner.commandLog().size() >= count) {
        return true;
      }
      std::this_thread::sleep_for(milliseconds(1));
    }
    return false;
  }
  bool hasText(const std::string& part) const {
    for (const auto& command : buildFleetScene(runner.warehouse(), drawn, controller).commands) {
      if (const auto* text = std::get_if<Text>(&command)) {
        if (text->text.find(part) != std::string::npos) {
          return true;
        }
      }
    }
    return false;
  }
};

// ---- a session of use ---------------------------------------------------------------------------

TEST(FleetSessionTest, PausingFromTheKeyboardStopsTheRunnerAndResumingStartsItAgain) {
  Gui gui(3);
  gui.runner.start();
  gui.pause();
  EXPECT_TRUE(gui.controller.paused());
  std::this_thread::sleep_for(milliseconds(20));
  const auto tick = gui.runner.snapshot().tick;
  std::this_thread::sleep_for(milliseconds(40));
  EXPECT_EQ(gui.runner.snapshot().tick, tick);
  gui.pause();
  EXPECT_FALSE(gui.controller.paused());
  EXPECT_TRUE(gui.runner.waitForTick(tick + 3, kTimeout));
  gui.runner.stop();
}

TEST(FleetSessionTest, TheSpeedKeysReachTheRunner) {
  Gui gui(3);
  gui.runner.start();
  InputEvents faster;
  faster.speedSteps = 3;
  gui.frame(faster);
  EXPECT_DOUBLE_EQ(gui.runner.speed(), 8.0);
  InputEvents slower;
  slower.speedSteps = -5;
  gui.frame(slower);
  EXPECT_DOUBLE_EQ(gui.runner.speed(), 0.25);
  gui.runner.stop();
}

TEST(FleetSessionTest, TheStartingSpeedIsAppliedToTheRunner) {
  Gui gui(3, 4.0);
  EXPECT_DOUBLE_EQ(gui.runner.speed(), 4.0);
}

TEST(FleetSessionTest, ClickingOutANewTaskAddsItToTheSimulationAndTheListsAndSelectsIt) {
  Gui gui(3);
  gui.pause();  // before the start, so that not one step runs
  gui.runner.start();
  gui.key(FleetKey::NewTask);
  gui.key(FleetKey::Priority3);
  gui.clickCell(P(0, 5));
  gui.clickCell(P(0, 11));
  ASSERT_TRUE(gui.waitForLog(1));
  gui.frame({});  // the next frame hears what became of the command

  const auto log = gui.runner.commandLog();
  ASSERT_EQ(log.size(), 1U);
  EXPECT_TRUE(log[0].outcome.accepted);
  ASSERT_TRUE(log[0].outcome.task.has_value());
  EXPECT_EQ(std::get<SubmitTask>(log[0].command).priority, TaskPriority::High);

  const auto snapshot = gui.runner.snapshot();
  ASSERT_EQ(snapshot.activeTasks.size(), 1U);
  EXPECT_EQ(snapshot.activeTasks[0].pickup, P(0, 5));
  EXPECT_EQ(snapshot.activeTasks[0].dropoff, P(0, 11));
  EXPECT_EQ(snapshot.activeTasks[0].priority, TaskPriority::High);
  EXPECT_EQ(gui.controller.selectedTask(), log[0].outcome.task);
  EXPECT_TRUE(gui.hasText("#1   high")) << "the task is in the list";
  EXPECT_TRUE(gui.hasText("task #1 added")) << gui.controller.message();
  gui.runner.stop();
}

TEST(FleetSessionTest, WhatBecameOfACommandIsHeardOnceNotOnEveryFrame) {
  Gui gui(3);
  gui.pause();
  gui.runner.start();
  gui.key(FleetKey::NewTask);
  gui.clickCell(P(0, 5));
  gui.clickCell(P(0, 11));
  ASSERT_TRUE(gui.waitForLog(1));
  gui.frame({});
  ASSERT_TRUE(gui.controller.selectedTask().has_value());
  gui.key(FleetKey::Cancel);  // the user lets go of the task
  EXPECT_FALSE(gui.controller.selectedTask().has_value());
  gui.frame({});
  gui.frame({});
  EXPECT_FALSE(
      gui.controller.selectedTask().has_value());  // and the old news does not bring it back
  gui.runner.stop();
}

TEST(FleetSessionTest, ChangingThePriorityOfTheSelectedWaitingTaskReachesTheQueueAndTheList) {
  Gui gui(3);
  gui.pause();
  gui.runner.start();
  gui.key(FleetKey::NewTask);
  gui.clickCell(P(0, 5));
  gui.clickCell(P(0, 11));
  ASSERT_TRUE(gui.waitForLog(1));
  gui.frame({});
  ASSERT_TRUE(gui.controller.selectedTask().has_value());

  gui.key(FleetKey::Priority4);
  ASSERT_TRUE(gui.waitForLog(2));
  gui.frame({});
  EXPECT_EQ(gui.runner.snapshot().activeTasks[0].priority, TaskPriority::Urgent);
  EXPECT_TRUE(gui.hasText("#1   urgent")) << "the list shows the new priority";
  EXPECT_TRUE(gui.hasText("task #1 is now urgent"));
  gui.runner.stop();
}

TEST(FleetSessionTest, ARefusedPriorityChangeShowsTheReasonFromTheSimulation) {
  Gui gui(3);
  gui.runner.start(10);
  ASSERT_TRUE(gui.runner.waitForTick(10, kTimeout));
  gui.frame({});
  // Task 1 was made by the generator at tick 0 and a robot has it by now.
  ASSERT_FALSE(gui.drawn.activeTasks.empty());
  const auto id = gui.drawn.activeTasks[0].id;
  ASSERT_NE(gui.drawn.activeTasks[0].status, TaskStatus::Pending);
  ASSERT_TRUE(gui.runner.post(SetTaskPriority{id, TaskPriority::Urgent}));
  ASSERT_TRUE(gui.waitForLog(1));
  gui.frame({});
  EXPECT_TRUE(gui.controller.messageIsError());
  EXPECT_TRUE(gui.hasText("still waiting can change priority")) << gui.controller.message();
  gui.runner.stop();
}

TEST(FleetSessionTest, ResettingFromTheKeyboardRestartsTheRunAndDropsTheTaskSelection) {
  Gui gui(5);
  gui.runner.start();
  ASSERT_TRUE(gui.runner.waitForTick(30, kTimeout));
  gui.pause();
  gui.key(FleetKey::NewTask);
  gui.clickCell(P(0, 5));
  gui.clickCell(P(0, 11));
  ASSERT_TRUE(gui.waitForLog(1));
  gui.frame({});
  ASSERT_TRUE(gui.controller.selectedTask().has_value());

  gui.key(FleetKey::Reset);
  for (int i = 0; i < 5000 && gui.runner.frame().epoch == 0; ++i) {
    std::this_thread::sleep_for(milliseconds(1));
  }
  gui.frame({});
  gui.frame({});
  EXPECT_EQ(gui.runner.snapshot().tick, 0U);
  EXPECT_FALSE(gui.controller.selectedTask().has_value());
  EXPECT_EQ(json(gui.runner.snapshot()),
            json(warehousesim::simulation::makeDemoSimulation(5).snapshot()));
  EXPECT_TRUE(gui.hasText("scenario restarted (seed 5)")) << gui.controller.message();
  // The log is the new run's: just the restart.
  EXPECT_EQ(gui.runner.commandLog().size(), 1U);
  gui.runner.stop();
}

TEST(FleetSessionTest, AFullCommandQueueIsReportedNotSilentlyDropped) {
  Gui gui(3);  // not started: nothing drains the queue
  for (std::size_t i = 0; i < SimulationRunner::kMaxQueuedCommands; ++i) {
    ASSERT_TRUE(gui.runner.post(SubmitTask{P(0, 5), P(0, 11)}));
  }
  gui.key(FleetKey::Reset);
  EXPECT_TRUE(gui.controller.messageIsError());
  EXPECT_TRUE(gui.hasText("too many commands waiting"));
}

TEST(FleetSessionTest, SelectingRobotsAndTasksShowsThemInThePanel) {
  Gui gui(3);
  gui.runner.start(40);
  ASSERT_TRUE(gui.runner.waitForTick(40, kTimeout));
  gui.frame({});
  gui.key(FleetKey::NextRobot);
  EXPECT_EQ(gui.controller.selectedRobot()->value, 1U);
  EXPECT_TRUE(gui.hasText("robot 1  "));
  gui.key(FleetKey::NextRobot);
  EXPECT_TRUE(gui.hasText("robot 2  "));
  gui.key(FleetKey::NextTask);
  EXPECT_TRUE(gui.controller.selectedTask().has_value());
  gui.runner.stop();
}

// ---- what the interface cannot do to the simulation ---------------------------------------------

TEST(FleetDeterminismTest, ASessionOfOnlyLookingSelectingAndHoveringLeavesTheRunExactlyAsItWas) {
  warehousesim::core::Rng rng(31);
  Gui gui(4);
  gui.runner.start(120);
  while (gui.runner.snapshot().tick < 120) {
    InputEvents input;
    const ScreenPoint at{static_cast<double>(rng.below(1280)), static_cast<double>(rng.below(720))};
    input.pointerActions = {{PointerAction::Kind::Press, PointerButton::Primary, at},
                            {PointerAction::Kind::Release, PointerButton::Primary, at}};
    input.pointer = at;
    input.zoomSteps = static_cast<double>(rng.below(5)) - 2.0;
    input.dragX = static_cast<double>(rng.below(30)) - 15.0;
    input.togglePaths = rng.below(7) == 0;
    input.toggleOccupied = rng.below(7) == 0;
    if (rng.below(3) == 0) {
      const FleetKey looking[] = {FleetKey::NextRobot, FleetKey::PreviousRobot, FleetKey::NextTask,
                                  FleetKey::PreviousTask};
      input.fleetKeys.push_back(looking[rng.below(4)]);
    }
    gui.frame(input);
    (void)buildFleetScene(gui.runner.warehouse(), gui.drawn, gui.controller);
    std::this_thread::sleep_for(std::chrono::microseconds(300));
  }
  ASSERT_TRUE(gui.runner.waitForTick(120, kTimeout));
  auto reference = warehousesim::simulation::makeDemoSimulation(4);
  reference.run(120);
  EXPECT_EQ(json(gui.runner.snapshot()), json(reference.snapshot()));
  EXPECT_TRUE(gui.runner.commandLog().empty());
  gui.runner.stop();
}

TEST(FleetDeterminismTest, PausingAndChangingSpeedFromTheKeyboardAtRandomDoesNotChangeTheRun) {
  warehousesim::core::Rng rng(32);
  Gui gui(4);
  gui.runner.start(100);
  while (gui.runner.snapshot().tick < 100) {
    InputEvents input;
    input.togglePause = rng.below(6) == 0;
    input.speedSteps = rng.below(4) == 0 ? static_cast<int>(rng.below(5)) - 2 : 0;
    gui.frame(input);
    std::this_thread::sleep_for(std::chrono::microseconds(300));
  }
  InputEvents resume;
  while (gui.controller.paused()) {
    resume.togglePause = true;
    gui.frame(resume);
  }
  ASSERT_TRUE(gui.runner.waitForTick(100, kTimeout));
  auto reference = warehousesim::simulation::makeDemoSimulation(4);
  reference.run(100);
  EXPECT_EQ(json(gui.runner.snapshot()), json(reference.snapshot()));
  gui.runner.stop();
}

TEST(FleetDeterminismTest, ATaskAddedWhilePausedLeadsToTheSameRunAsAddingItBeforeStepOne) {
  // Paused, no step has run, so every clicked-out task is applied at tick 0, and the run that
  // follows must be exactly that of a simulation that had the same tasks submitted by hand.
  Gui gui(6);
  gui.pause();
  gui.runner.start(150);
  gui.key(FleetKey::NewTask);
  gui.key(FleetKey::Priority4);
  gui.clickCell(P(0, 5));
  gui.clickCell(P(3, 11));
  gui.key(FleetKey::NewTask);
  gui.key(FleetKey::Priority1);
  gui.clickCell(P(0, 8));
  gui.clickCell(P(0, 11));
  ASSERT_TRUE(gui.waitForLog(2));
  gui.frame({});
  ASSERT_EQ(gui.runner.snapshot().tick, 0U);
  gui.pause();
  ASSERT_TRUE(gui.runner.waitForTick(150, kTimeout));

  auto reference = warehousesim::simulation::makeDemoSimulation(6);
  EXPECT_TRUE(apply(reference, SubmitTask{P(0, 5), P(3, 11), TaskPriority::Urgent}).accepted);
  EXPECT_TRUE(apply(reference, SubmitTask{P(0, 8), P(0, 11), TaskPriority::Low}).accepted);
  reference.run(150);
  EXPECT_EQ(json(gui.runner.snapshot()), json(reference.snapshot()));
  gui.runner.stop();
}

TEST(FleetDeterminismTest, TheCommandsOfARandomSessionReplayToTheSameState) {
  for (std::uint64_t round = 1; round <= 4; ++round) {
    warehousesim::core::Rng rng(round + 100);
    Gui gui(2);
    gui.runner.start(150);
    while (gui.runner.snapshot().tick < 150) {
      InputEvents input;
      switch (rng.below(5)) {
        case 0:
          input.fleetKeys.push_back(FleetKey::NewTask);
          break;
        case 1:
          input.fleetKeys.push_back(static_cast<FleetKey>(5 + rng.below(4)));  // a priority key
          break;
        case 2:
        case 3: {
          const ScreenPoint at =
              gui.cellCentre(P(static_cast<int>(rng.below(20)), static_cast<int>(rng.below(12))));
          input.pointerActions = {{PointerAction::Kind::Press, PointerButton::Primary, at},
                                  {PointerAction::Kind::Release, PointerButton::Primary, at}};
          break;
        }
        default:
          input.fleetKeys.push_back(FleetKey::NextTask);
          break;
      }
      gui.frame(input);
      std::this_thread::sleep_for(std::chrono::microseconds(400));
    }
    ASSERT_TRUE(gui.runner.waitForTick(150, kTimeout));
    for (int i = 0; i < 100; ++i) {
      std::this_thread::sleep_for(milliseconds(1));
    }
    const auto log = gui.runner.commandLog();
    const auto finalState = gui.runner.snapshot();
    gui.runner.stop();

    auto replayed = warehousesim::simulation::makeDemoSimulation(2);
    std::size_t next = 0;
    while (true) {
      while (next < log.size() && log[next].tick == replayed.tick()) {
        if (const auto* submit = std::get_if<SubmitTask>(&log[next].command)) {
          (void)apply(replayed, *submit);
        } else if (const auto* change = std::get_if<SetTaskPriority>(&log[next].command)) {
          (void)apply(replayed, *change);
        }
        ++next;
      }
      if (replayed.tick() >= 150) {
        break;
      }
      replayed.step();
    }
    EXPECT_EQ(json(finalState), json(replayed.snapshot()))
        << "round " << round << ", " << log.size() << " commands";
  }
}

}  // namespace
