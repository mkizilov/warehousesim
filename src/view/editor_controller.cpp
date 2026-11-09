#include "warehousesim/view/editor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "warehousesim/editor/editor_files.hpp"
#include "warehousesim/view/scene.hpp"
#include "warehousesim/view/scene_parts.hpp"
#include "warehousesim/world/grid_size.hpp"
#include "warehousesim/world/layouts.hpp"

namespace warehousesim::view {

namespace {

using core::Point;
using editor::Tool;

// The camera looks at everything above the bar.
ScreenSize canvasOf(ScreenSize window) {
  return {std::max(window.width, 0),
          std::max(window.height - static_cast<int>(kEditorBarHeight), 0)};
}

bool isFinite(const ScreenPoint& p) { return std::isfinite(p.x) && std::isfinite(p.y); }

std::optional<Tool> toolOf(EditorCommand command) {
  switch (command) {
    case EditorCommand::ToolWall:
      return Tool::Wall;
    case EditorCommand::ToolShelf:
      return Tool::Shelf;
    case EditorCommand::ToolPickup:
      return Tool::Pickup;
    case EditorCommand::ToolDropoff:
      return Tool::Dropoff;
    case EditorCommand::ToolCharging:
      return Tool::Charging;
    case EditorCommand::ToolErase:
      return Tool::Erase;
    default:
      return std::nullopt;
  }
}

}  // namespace

EditorController::EditorController(ScreenSize window, editor::MapEditor editor,
                                   std::filesystem::path file)
    : editor_(std::move(editor)),
      file_(std::move(file)),
      window_(window),
      view_(canvasOf(window), editor_.layout().size()),
      issues_(editor_.issues()),
      issuesRevision_(editor_.revision()) {}

void EditorController::resize(ScreenSize window) {
  window_ = window;
  view_.resize(canvasOf(window));
}

ScreenRect EditorController::canvas() const {
  const ScreenSize screen = view_.camera().screen();
  const double left = leftRulerWidth(editor_.layout().size());
  return {left, kRulerThickness, std::max(0.0, static_cast<double>(screen.width) - left),
          std::max(0.0, static_cast<double>(screen.height) - kRulerThickness)};
}

std::optional<Point> EditorController::cellAtCanvas(ScreenPoint at) const {
  const ScreenRect area = canvas();
  if (!isFinite(at) || at.x < area.x || at.x >= area.right() || at.y < area.y ||
      at.y >= area.bottom()) {
    return std::nullopt;
  }
  return view_.camera().cellAt(at);
}

std::optional<Point> EditorController::hoveredCell() const {
  const auto at = view_.pointer();
  return at ? cellAtCanvas(*at) : std::nullopt;
}

void EditorController::setMessage(std::string text, bool isError) {
  message_ = std::move(text);
  messageIsError_ = isError;
}

void EditorController::fitView() {
  InputEvents fit;
  fit.resetView = true;
  view_.apply(fit, 0.0);
}

// Whether something that throws away the map as it is may go ahead. With unsaved changes the first
// request only asks the question; the same request again confirms it.
bool EditorController::mayDiscard(Pending what, const char* question) {
  if (!editor_.dirty() || pending_ == what) {
    pending_ = Pending::None;
    return true;
  }
  pending_ = what;
  setMessage(question, true);
  return false;
}

void EditorController::resizeBy(std::int32_t dx, std::int32_t dy) {
  const editor::ResizeResult result =
      editor_.resize(editor_.layout().width() + dx, editor_.layout().height() + dy);
  switch (result.status) {
    case editor::ResizeStatus::Applied: {
      std::string text = "the map is now " + std::to_string(editor_.layout().width()) + " x " +
                         std::to_string(editor_.layout().height());
      if (result.cellsLost > 0) {
        text += "; " + std::to_string(result.cellsLost) +
                " objects were cropped (Ctrl+Z brings them back)";
      }
      setMessage(std::move(text), false);
      break;
    }
    case editor::ResizeStatus::InvalidSize:
      setMessage("a map side must be from 1 to " + std::to_string(world::kMaxMapSide) + " cells",
                 true);
      break;
    case editor::ResizeStatus::Unchanged:
      break;
  }
}

void EditorController::run(EditorCommand command) {
  editor_.endStroke();
  drag_.reset();
  const Pending asks = command == EditorCommand::Load         ? Pending::Load
                       : command == EditorCommand::NextLayout ? Pending::NextLayout
                                                              : Pending::None;
  if (pending_ != asks) {
    pending_ = Pending::None;
  }

  if (const auto tool = toolOf(command)) {
    editor_.setTool(*tool);
    setMessage("", false);
    return;
  }
  switch (command) {
    case EditorCommand::Undo:
      if (!editor_.undo()) {
        setMessage("nothing to undo", false);
      }
      break;
    case EditorCommand::Redo:
      if (!editor_.redo()) {
        setMessage("nothing to redo", false);
      }
      break;
    case EditorCommand::Save: {
      const editor::FileOutcome outcome = editor::saveMap(editor_, file_);
      setMessage(outcome.message, !outcome.ok);
      break;
    }
    case EditorCommand::Load:
      if (mayDiscard(Pending::Load, "unsaved changes: press Ctrl+O again to discard them")) {
        const editor::FileOutcome outcome = editor::loadMap(editor_, file_);
        setMessage(outcome.message, !outcome.ok);
        if (outcome.ok) {
          view_.setGrid(editor_.layout().size());
          fitView();
        }
      }
      break;
    case EditorCommand::NextLayout:
      if (mayDiscard(Pending::NextLayout, "unsaved changes: press L again to discard them")) {
        const auto layouts = world::predefinedLayouts();
        const std::size_t index = nextLayout_ % layouts.size();
        const editor::FileOutcome outcome = editor::loadPredefined(editor_, layouts[index].name);
        setMessage(outcome.message, !outcome.ok);
        if (outcome.ok) {
          nextLayout_ = index + 1;
          view_.setGrid(editor_.layout().size());
          fitView();
        }
      }
      break;
    case EditorCommand::Clear:
      setMessage(editor_.clear().status == editor::EditStatus::Applied
                     ? "the map is empty (Ctrl+Z brings it back)"
                     : "the map is already empty",
                 false);
      break;
    case EditorCommand::Wider:
      resizeBy(1, 0);
      break;
    case EditorCommand::Narrower:
      resizeBy(-1, 0);
      break;
    case EditorCommand::Taller:
      resizeBy(0, 1);
      break;
    case EditorCommand::Shorter:
      resizeBy(0, -1);
      break;
    default:
      break;
  }
  if (view_.camera().grid() != editor_.layout().size()) {
    view_.setGrid(editor_.layout().size());
  }
}

void EditorController::pointerAction(const PointerAction& action) {
  switch (action.kind) {
    case PointerAction::Kind::Press: {
      pending_ = Pending::None;
      const auto cell = cellAtCanvas(action.at);
      if (drag_ || !cell) {
        return;  // a second button while one is down, or a click on a ruler or the bar
      }
      const Tool tool = action.button == PointerButton::Primary ? editor_.tool() : Tool::Erase;
      drag_ = Drag{action.button, tool, cell};
      setMessage("", false);
      editor_.beginStroke();
      editor_.paint(*cell, tool);
      break;
    }
    case PointerAction::Kind::Move: {
      if (!drag_) {
        return;
      }
      const auto cell = cellAtCanvas(action.at);
      if (!cell) {
        drag_->last.reset();  // off the canvas: do not join up with where it comes back
        return;
      }
      if (drag_->last) {
        editor_.paintLine(*drag_->last, *cell, drag_->tool);
      } else {
        editor_.paint(*cell, drag_->tool);
      }
      drag_->last = cell;
      break;
    }
    case PointerAction::Kind::Release:
      if (drag_ && drag_->button == action.button) {
        editor_.endStroke();
        drag_.reset();
      }
      break;
  }
}

void EditorController::apply(const InputEvents& input, double frameSeconds) {
  for (const EditorCommand command : input.commands) {
    run(command);
  }
  for (const PointerAction& action : input.pointerActions) {
    pointerAction(action);
  }
  if (input.quit &&
      mayDiscard(Pending::Quit, "unsaved changes: press Esc again to quit without saving")) {
    quit_ = true;
  }

  // The camera takes zoom, pan and "fit"; playback and overlays mean nothing here.
  InputEvents cameraInput = input;
  cameraInput.quit = false;
  cameraInput.togglePause = false;
  cameraInput.togglePaths = false;
  cameraInput.toggleOccupied = false;
  cameraInput.speedSteps = 0;
  cameraInput.commands.clear();
  cameraInput.pointerActions.clear();
  view_.apply(cameraInput, frameSeconds);

  if (issuesRevision_ != editor_.revision()) {
    issues_ = editor_.issues();
    issuesRevision_ = editor_.revision();
  }
}

}  // namespace warehousesim::view
