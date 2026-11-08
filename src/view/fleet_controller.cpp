#include "warehousesim/view/fleet_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "warehousesim/view/scene.hpp"
#include "warehousesim/view/scene_parts.hpp"

namespace warehousesim::view {

namespace {

using core::Point;
using tasks::Task;
using tasks::TaskPriority;
using tasks::TaskStatus;

// A press and release this close together are a click; further apart, the map was being dragged.
constexpr double kClickSlop = 6.0;

bool isFinite(ScreenPoint p) { return std::isfinite(p.x) && std::isfinite(p.y); }

std::string cellText(Point cell) {
  return "(" + std::to_string(cell.x) + ", " + std::to_string(cell.y) + ")";
}

std::string taskText(core::TaskId id) { return "task #" + std::to_string(id.value); }

// The id after (direction 1) or before (-1) `current` in `ids`, wrapping round; the first or last
// one if nothing is selected or the selection is not in the list.
template <class Id>
std::optional<Id> cycle(const std::vector<Id>& ids, const std::optional<Id>& current,
                        int direction) {
  if (ids.empty()) {
    return std::nullopt;
  }
  const auto found = current ? std::find(ids.begin(), ids.end(), *current) : ids.end();
  if (found == ids.end()) {
    return direction > 0 ? ids.front() : ids.back();
  }
  const auto count = static_cast<std::ptrdiff_t>(ids.size());
  const std::ptrdiff_t next =
      (static_cast<std::ptrdiff_t>(found - ids.begin()) + direction + count) % count;
  return ids[static_cast<std::size_t>(next)];
}

TaskPriority priorityOf(FleetKey key) {
  switch (key) {
    case FleetKey::Priority1:
      return TaskPriority::Low;
    case FleetKey::Priority2:
      return TaskPriority::Normal;
    case FleetKey::Priority3:
      return TaskPriority::High;
    default:
      return TaskPriority::Urgent;
  }
}

}  // namespace

TaskLists taskLists(const simulation::Snapshot& snapshot) {
  TaskLists lists;
  lists.active = snapshot.activeTasks;
  std::stable_sort(lists.active.begin(), lists.active.end(), [](const Task& a, const Task& b) {
    const bool aWaiting = a.status == TaskStatus::Pending;
    const bool bWaiting = b.status == TaskStatus::Pending;
    if (aWaiting != bWaiting) {
      return aWaiting;
    }
    if (aWaiting && a.priority != b.priority) {
      return a.priority > b.priority;
    }
    if (aWaiting && a.createdAt != b.createdAt) {
      return a.createdAt < b.createdAt;
    }
    return a.id < b.id;
  });
  lists.finished = snapshot.recentTasks;
  return lists;
}

FleetController::FleetController(ScreenSize window, world::Warehouse warehouse, double initialSpeed)
    : warehouse_(std::move(warehouse)),
      window_(window),
      layout_(computeLayout(window, {})),
      view_(layout_.canvas, warehouse_.size(), initialSpeed) {}

void FleetController::resize(ScreenSize window) {
  window_ = window;
  layout_ = computeLayout(window_, {robots_.size(), lists_.active.size(), lists_.finished.size()});
  view_.resize(layout_.canvas);
}

void FleetController::setMessage(std::string text, bool isError) {
  message_ = std::move(text);
  messageIsError_ = isError;
}

void FleetController::report(std::string text, bool isError) {
  setMessage(std::move(text), isError);
}

void FleetController::note(const simulation::CommandRecord& record) {
  setMessage(record.outcome.message, !record.outcome.accepted);
  if (record.outcome.accepted && record.outcome.task &&
      std::holds_alternative<simulation::SubmitTask>(record.command)) {
    task_ = record.outcome.task;  // so that its priority can be changed at once
  }
}

void FleetController::restarted() {
  task_.reset();
  entry_ = TaskEntry::None;
  pickup_.reset();
  pressedAt_.reset();
}

const Task* FleetController::findTask(core::TaskId id) const {
  for (const std::vector<Task>* list : {&lists_.active, &lists_.finished}) {
    const auto found =
        std::find_if(list->begin(), list->end(), [id](const Task& t) { return t.id == id; });
    if (found != list->end()) {
      return &*found;
    }
  }
  return nullptr;
}

std::optional<Point> FleetController::cellAtCanvas(ScreenPoint at) const {
  const ScreenSize canvas = layout_.canvas;
  if (!isFinite(at) || at.x < leftRulerWidth(warehouse_.size()) ||
      at.x >= static_cast<double>(canvas.width) || at.y < kRulerThickness ||
      at.y >= static_cast<double>(canvas.height) - kStatusBarHeight) {
    return std::nullopt;
  }
  return view_.camera().cellAt(at);
}

std::optional<Point> FleetController::hoveredCell() const {
  const auto at = view_.pointer();
  return at ? cellAtCanvas(*at) : std::nullopt;
}

void FleetController::choosePriority(TaskPriority priority, FleetOutput& out) {
  if (entry_ != TaskEntry::None) {
    newPriority_ = priority;
    setMessage(std::string("new tasks are ") + tasks::toString(priority), false);
    return;
  }
  if (!task_) {
    setMessage("select a waiting task first: click it in the list, or press T", true);
    return;
  }
  const Task* task = findTask(*task_);
  if (task == nullptr) {
    return;  // cannot happen: the selection is checked against the lists every frame
  }
  if (task->status != TaskStatus::Pending) {
    setMessage(taskText(task->id) + " is " + tasks::toString(task->status) +
                   "; only a waiting task can change priority",
               true);
    return;
  }
  if (task->priority == priority) {
    setMessage(taskText(task->id) + " is already " + tasks::toString(priority), false);
    return;
  }
  out.commands.push_back(simulation::SetTaskPriority{task->id, priority});
  setMessage(taskText(task->id) + ": asking for " + tasks::toString(priority), false);
}

void FleetController::handleKey(FleetKey key, FleetOutput& out) {
  switch (key) {
    case FleetKey::NextRobot:
    case FleetKey::PreviousRobot: {
      std::vector<core::RobotId> ids;
      for (const RobotMark& robot : robots_) {
        ids.push_back(robot.id);
      }
      robot_ = cycle(ids, robot_, key == FleetKey::NextRobot ? 1 : -1);
      break;
    }
    case FleetKey::NextTask:
    case FleetKey::PreviousTask: {
      std::vector<core::TaskId> ids;
      for (const Task& task : lists_.active) {
        ids.push_back(task.id);
      }
      for (const Task& task : lists_.finished) {
        ids.push_back(task.id);
      }
      task_ = cycle(ids, task_, key == FleetKey::NextTask ? 1 : -1);
      break;
    }
    case FleetKey::NewTask:
      entry_ = TaskEntry::Pickup;
      pickup_.reset();
      setMessage("new task: click a pickup cell (1-4 sets the priority, Esc cancels)", false);
      break;
    case FleetKey::Priority1:
    case FleetKey::Priority2:
    case FleetKey::Priority3:
    case FleetKey::Priority4:
      choosePriority(priorityOf(key), out);
      break;
    case FleetKey::Reset:
      out.commands.push_back(simulation::ResetScenario{});
      setMessage("restarting the scenario", false);
      break;
    case FleetKey::Cancel:
      if (entry_ != TaskEntry::None) {
        entry_ = TaskEntry::None;
        pickup_.reset();
        setMessage("new task cancelled", false);
      } else if (robot_ || task_) {
        robot_.reset();
        task_.reset();
        setMessage("", false);
      } else {
        quit_ = true;
      }
      break;
  }
}

void FleetController::selectRobotAt(Point cell) {
  robot_.reset();
  for (const RobotMark& robot : robots_) {
    const Point drawnCell{static_cast<std::int32_t>(std::floor(robot.center.x)),
                          static_cast<std::int32_t>(std::floor(robot.center.y))};
    if (drawnCell == cell) {
      robot_ = robot.id;
      return;
    }
  }
  for (const RobotMark& robot : robots_) {
    if (robot.cell == cell) {
      robot_ = robot.id;
      return;
    }
  }
}

void FleetController::clickMap(Point cell, FleetOutput& out) {
  if (entry_ == TaskEntry::None) {
    selectRobotAt(cell);
    return;
  }
  if (!warehouse_.isWalkable(cell)) {
    setMessage(
        cellText(cell) + " is a " + world::toString(warehouse_.at(cell)) + ": click open floor",
        true);
    return;
  }
  if (entry_ == TaskEntry::Pickup) {
    pickup_ = cell;
    entry_ = TaskEntry::Dropoff;
    setMessage("pickup " + cellText(cell) + ": now click a dropoff cell", false);
    return;
  }
  if (cell == *pickup_) {
    setMessage("the dropoff must be another cell than the pickup " + cellText(cell), true);
    return;
  }
  out.commands.push_back(simulation::SubmitTask{*pickup_, cell, newPriority_});
  setMessage("sending the task " + cellText(*pickup_) + " to " + cellText(cell), false);
  entry_ = TaskEntry::None;
  pickup_.reset();
}

void FleetController::clickPanel(ScreenPoint at) {
  const auto hit = hitTest(layout_, at);
  if (!hit) {
    return;
  }
  switch (hit->kind) {
    case PanelHit::Kind::Robot:
      if (hit->index < robots_.size()) {
        robot_ = robots_[hit->index].id;
      }
      break;
    case PanelHit::Kind::ActiveTask:
      if (hit->index < lists_.active.size()) {
        task_ = lists_.active[hit->index].id;
      }
      break;
    case PanelHit::Kind::FinishedTask:
      if (hit->index < lists_.finished.size()) {
        task_ = lists_.finished[hit->index].id;
      }
      break;
  }
}

void FleetController::click(ScreenPoint at, FleetOutput& out) {
  if (layout_.hasPanel() && at.x >= layout_.panel.x) {
    clickPanel(at);
    return;
  }
  if (const auto cell = cellAtCanvas(at)) {
    clickMap(*cell, out);
  }
}

FleetOutput FleetController::apply(const InputEvents& input, double frameSeconds,
                                   const simulation::Snapshot& drawn, std::uint64_t epoch) {
  FleetOutput out;
  if (epoch != epoch_) {
    epoch_ = epoch;
    restarted();
  }

  // What is on screen, which is what a click is tested against.
  robots_.clear();
  for (const simulation::RobotSnapshot& robot : drawn.robots) {
    robots_.push_back({robot.id, robot.position, robot.center()});
  }
  lists_ = taskLists(drawn);
  layout_ = computeLayout(window_, {robots_.size(), lists_.active.size(), lists_.finished.size()});
  if (robot_ && std::none_of(robots_.begin(), robots_.end(),
                             [this](const RobotMark& robot) { return robot.id == *robot_; })) {
    robot_.reset();
  }
  if (task_ && findTask(*task_) == nullptr) {
    task_.reset();
  }

  for (const FleetKey key : input.fleetKeys) {
    handleKey(key, out);
  }
  for (const PointerAction& action : input.pointerActions) {
    if (action.button != PointerButton::Primary) {
      continue;
    }
    if (action.kind == PointerAction::Kind::Press) {
      pressedAt_ = action.at;
    } else if (action.kind == PointerAction::Kind::Release && pressedAt_) {
      const ScreenPoint pressed = *pressedAt_;
      pressedAt_.reset();
      if (isFinite(action.at) && isFinite(pressed) &&
          std::hypot(action.at.x - pressed.x, action.at.y - pressed.y) <= kClickSlop) {
        click(action.at, out);
      }
    }
  }
  if (input.quit) {
    quit_ = true;
  }

  // Zoom, pan, fit, the overlays and the playback state belong to the plain view controller.
  InputEvents cameraInput = input;
  cameraInput.quit = false;
  cameraInput.commands.clear();
  cameraInput.pointerActions.clear();
  cameraInput.fleetKeys.clear();
  view_.apply(cameraInput, frameSeconds);
  if (input.togglePause) {
    out.paused = view_.paused();
  }
  if (input.speedSteps != 0) {
    out.speed = view_.speed();
  }
  return out;
}

}  // namespace warehousesim::view
