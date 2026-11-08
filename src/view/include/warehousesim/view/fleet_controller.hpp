#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "warehousesim/core/ids.hpp"
#include "warehousesim/core/point.hpp"
#include "warehousesim/core/vec2.hpp"
#include "warehousesim/simulation/commands.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/tasks/task.hpp"
#include "warehousesim/view/camera.hpp"
#include "warehousesim/view/fleet_layout.hpp"
#include "warehousesim/view/view_controller.hpp"
#include "warehousesim/world/warehouse.hpp"

namespace warehousesim::view {

// The tasks as the panel lists them. Active: waiting tasks in the order the dispatcher will take
// them (priority, then age, then id), then the ones a robot has claimed, by id. Finished: the
// Snapshot's recentTasks, newest first.
struct TaskLists {
  std::vector<tasks::Task> active;
  std::vector<tasks::Task> finished;
};

[[nodiscard]] TaskLists taskLists(const simulation::Snapshot& snapshot);

// What one frame of input asks for outside the controller. The controller never touches a
// simulation: it only describes what the interface wants, and whoever owns the simulation decides
// when and whether to do it.
struct FleetOutput {
  // For SimulationRunner::post(): the only things that can change the simulation's state.
  std::vector<simulation::Command> commands;
  // For SimulationRunner::setPaused() and setSpeed(): when steps happen, not what they compute.
  std::optional<bool> paused;
  std::optional<double> speed;
};

// Which cell of a new task the next click chooses.
enum class TaskEntry : std::uint8_t { None, Pickup, Dropoff };

// The fleet management interface, as logic: which robot and task are selected, the task being made,
// the camera, playback state and the status message. It turns plain InputEvents into those and into
// a FleetOutput; it has no SDL types and no drawing (buildFleetScene draws it).
//
//   click a robot, or Tab / Shift+Tab    select a robot and inspect it
//   click a row, or T / Shift+T          select a task (a waiting one can change priority: 1-4)
//   N, then click pickup, click dropoff  add a delivery task (1-4 first to choose its priority)
//   Space, [ ]                           pause / resume, slower / faster
//   R                                    start the scenario over
//   P, O, wheel, drag, WASD, 0           routes, held cells, zoom, pan, fit
//   Esc                                  cancel task entry, then clear the selection, then quit
class FleetController {
 public:
  // `window` is the whole window. The controller keeps its own copy of the layout, which is how it
  // knows whether a cell can be a pickup.
  FleetController(ScreenSize window, world::Warehouse warehouse, double initialSpeed = 1.0);

  void resize(ScreenSize window);
  // Applies one frame of input. `drawn` is the snapshot that is on screen (clicks are tested
  // against what the user sees), `epoch` the FrameState's: when it changes the scenario has been
  // restarted and the task selection and any task being made are dropped. Non-finite input values
  // are ignored.
  [[nodiscard]] FleetOutput apply(const InputEvents& input, double frameSeconds,
                                  const simulation::Snapshot& drawn, std::uint64_t epoch);
  // What became of a command this interface posted: shown as the message, and a task that was added
  // is selected so that its priority can be changed at once.
  void note(const simulation::CommandRecord& record);
  // Shows a message of the caller's own (for instance that the command queue was full).
  void report(std::string text, bool isError);

  [[nodiscard]] const Camera& camera() const noexcept { return view_.camera(); }
  [[nodiscard]] ScreenSize window() const noexcept { return window_; }
  [[nodiscard]] const FleetLayout& layout() const noexcept { return layout_; }
  [[nodiscard]] bool paused() const noexcept { return view_.paused(); }
  [[nodiscard]] double speed() const noexcept { return view_.speed(); }
  [[nodiscard]] bool showPaths() const noexcept { return view_.showPaths(); }
  [[nodiscard]] bool showOccupied() const noexcept { return view_.showOccupied(); }
  [[nodiscard]] std::optional<ScreenPoint> pointer() const noexcept { return view_.pointer(); }
  // The map cell under the pointer, if it is over the part of the map that can be clicked.
  [[nodiscard]] std::optional<core::Point> hoveredCell() const;

  [[nodiscard]] std::optional<core::RobotId> selectedRobot() const noexcept { return robot_; }
  [[nodiscard]] std::optional<core::TaskId> selectedTask() const noexcept { return task_; }
  [[nodiscard]] TaskEntry entry() const noexcept { return entry_; }
  [[nodiscard]] std::optional<core::Point> pickup() const noexcept { return pickup_; }
  [[nodiscard]] tasks::TaskPriority newTaskPriority() const noexcept { return newPriority_; }
  [[nodiscard]] const TaskLists& tasks() const noexcept { return lists_; }

  [[nodiscard]] const std::string& message() const noexcept { return message_; }
  [[nodiscard]] bool messageIsError() const noexcept { return messageIsError_; }
  [[nodiscard]] bool wantsQuit() const noexcept { return quit_; }

 private:
  struct RobotMark {
    core::RobotId id;
    core::Point cell;
    core::Vec2 center;
  };

  void handleKey(FleetKey key, FleetOutput& out);
  void choosePriority(tasks::TaskPriority priority, FleetOutput& out);
  void click(ScreenPoint at, FleetOutput& out);
  void clickPanel(ScreenPoint at);
  void clickCell(core::Point cell);
  void clickMap(core::Point cell, FleetOutput& out);
  void selectRobotAt(core::Point cell);
  void restarted();
  [[nodiscard]] std::optional<core::Point> cellAtCanvas(ScreenPoint at) const;
  [[nodiscard]] const tasks::Task* findTask(core::TaskId id) const;
  void setMessage(std::string text, bool isError);

  world::Warehouse warehouse_;
  ScreenSize window_;
  FleetLayout layout_;
  ViewController view_;
  std::vector<RobotMark> robots_;
  TaskLists lists_;
  std::optional<core::RobotId> robot_;
  std::optional<core::TaskId> task_;
  TaskEntry entry_ = TaskEntry::None;
  std::optional<core::Point> pickup_;
  tasks::TaskPriority newPriority_ = tasks::TaskPriority::Normal;
  std::optional<ScreenPoint> pressedAt_;
  std::uint64_t epoch_ = 0;
  std::string message_;
  bool messageIsError_ = false;
  bool quit_ = false;
};

}  // namespace warehousesim::view
