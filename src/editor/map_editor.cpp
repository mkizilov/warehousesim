#include "warehousesim/editor/map_editor.hpp"

#include <cstdlib>
#include <ranges>
#include <stdexcept>
#include <utility>

#include "warehousesim/world/grid_size.hpp"

namespace warehousesim::editor {

namespace {

using core::Point;
using world::CellType;

void requireValidSize(std::int32_t width, std::int32_t height) {
  if (!world::isValidMapSide(width) || !world::isValidMapSide(height)) {
    throw std::invalid_argument("a map side must be from 1 to " +
                                std::to_string(world::kMaxMapSide) + " cells, not " +
                                std::to_string(width) + " x " + std::to_string(height));
  }
}

// Bresenham: the cells a straight line from a to b passes through, both ends included. The line is
// always drawn from the smaller end (by x, then y), so that it covers the same cells whichever way
// round it is asked for.
std::vector<Point> lineCells(Point a, Point b) {
  if (b < a) {
    std::swap(a, b);
  }
  std::vector<Point> cells;
  const std::int32_t dx = std::abs(b.x - a.x);
  const std::int32_t dy = -std::abs(b.y - a.y);
  const std::int32_t stepX = a.x < b.x ? 1 : -1;
  const std::int32_t stepY = a.y < b.y ? 1 : -1;
  std::int32_t error = dx + dy;
  Point at = a;
  while (true) {
    cells.push_back(at);
    if (at == b) {
      break;
    }
    const std::int32_t doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      at.x += stepX;
    }
    if (doubled <= dx) {
      error += dx;
      at.y += stepY;
    }
  }
  return cells;
}

}  // namespace

CellType cellTypeOf(Tool tool) noexcept {
  switch (tool) {
    case Tool::Wall:
      return CellType::Wall;
    case Tool::Shelf:
      return CellType::Shelf;
    case Tool::Pickup:
      return CellType::Pickup;
    case Tool::Dropoff:
      return CellType::Dropoff;
    case Tool::Charging:
      return CellType::Charging;
    case Tool::Erase:
      return CellType::Free;
  }
  return CellType::Free;
}

const char* toString(Tool tool) noexcept {
  switch (tool) {
    case Tool::Wall:
      return "wall";
    case Tool::Shelf:
      return "shelf";
    case Tool::Pickup:
      return "pickup";
    case Tool::Dropoff:
      return "dropoff";
    case Tool::Charging:
      return "charging";
    case Tool::Erase:
      return "erase";
  }
  return "unknown";
}

MapEditor::MapEditor(world::Warehouse layout, std::string name)
    : layout_(std::move(layout)), name_(std::move(name)) {
  requireValidSize(layout_.width(), layout_.height());
}

MapEditor MapEditor::blank(std::int32_t width, std::int32_t height, std::string name) {
  requireValidSize(width, height);
  return MapEditor(world::Warehouse(width, height), std::move(name));
}

EditResult MapEditor::setCell(Point cell, CellType type) {
  const CellType before = layout_.at(cell);
  if (before == type) {
    return {EditStatus::Unchanged, 0};
  }
  layout_.set(cell, type);
  ++revision_;
  pending_.push_back({cell, before, type});
  return {EditStatus::Applied, 1};
}

void MapEditor::record(Step step) {
  undo_.push_back(std::move(step));
  if (undo_.size() > kHistoryLimit) {
    undo_.pop_front();
  }
  redo_.clear();
}

// Turns what has been painted since the last step into an undo step.
void MapEditor::commit() {
  if (pending_.empty()) {
    return;
  }
  Step step;
  step.changes = std::move(pending_);
  pending_.clear();
  step.stateBefore = state_;
  step.stateAfter = ++lastState_;
  state_ = step.stateAfter;
  record(std::move(step));
}

void MapEditor::endStroke() {
  stroke_ = false;
  commit();
}

EditResult MapEditor::paint(Point cell) { return paint(cell, tool_); }

EditResult MapEditor::paint(Point cell, Tool tool) {
  if (!layout_.inBounds(cell)) {
    return {EditStatus::OutOfBounds, 0};
  }
  const EditResult result = setCell(cell, cellTypeOf(tool));
  if (!stroke_) {
    commit();
  }
  return result;
}

EditResult MapEditor::paintLine(Point from, Point to, Tool tool) {
  if (!layout_.inBounds(from) || !layout_.inBounds(to)) {
    return {EditStatus::OutOfBounds, 0};
  }
  EditResult total;
  for (const Point cell : lineCells(from, to)) {
    total.changed += setCell(cell, cellTypeOf(tool)).changed;
  }
  total.status = total.changed > 0 ? EditStatus::Applied : EditStatus::Unchanged;
  if (!stroke_) {
    commit();
  }
  return total;
}

EditResult MapEditor::clear() {
  endStroke();
  EditResult total;
  for (std::size_t i = 0; i < layout_.size().cellCount(); ++i) {
    total.changed += setCell(layout_.size().pointAt(i), CellType::Free).changed;
  }
  total.status = total.changed > 0 ? EditStatus::Applied : EditStatus::Unchanged;
  commit();
  return total;
}

ResizeResult MapEditor::resize(std::int32_t width, std::int32_t height) {
  endStroke();
  if (!world::isValidMapSide(width) || !world::isValidMapSide(height)) {
    return {ResizeStatus::InvalidSize, 0};
  }
  if (width == layout_.width() && height == layout_.height()) {
    return {ResizeStatus::Unchanged, 0};
  }
  world::Warehouse resized(width, height);
  std::size_t lost = 0;
  for (std::size_t i = 0; i < layout_.size().cellCount(); ++i) {
    const Point cell = layout_.size().pointAt(i);
    if (resized.inBounds(cell)) {
      resized.set(cell, layout_.at(cell));
    } else if (layout_.at(cell) != CellType::Free) {
      ++lost;
    }
  }
  Step step;
  step.resized.emplace(layout_, resized);
  step.stateBefore = state_;
  step.stateAfter = ++lastState_;
  layout_ = std::move(resized);
  ++revision_;
  state_ = step.stateAfter;
  record(std::move(step));
  return {ResizeStatus::Applied, lost};
}

void MapEditor::replace(world::Warehouse layout, std::string name) {
  requireValidSize(layout.width(), layout.height());
  stroke_ = false;
  pending_.clear();
  undo_.clear();
  redo_.clear();
  layout_ = std::move(layout);
  ++revision_;
  name_ = std::move(name);
  state_ = ++lastState_;
  saved_ = state_;
}

bool MapEditor::undo() {
  endStroke();
  if (undo_.empty()) {
    return false;
  }
  Step step = std::move(undo_.back());
  undo_.pop_back();
  if (step.resized) {
    layout_ = step.resized->first;
  } else {
    for (const auto& change : std::views::reverse(step.changes)) {
      layout_.set(change.cell, change.before);
    }
  }
  ++revision_;
  state_ = step.stateBefore;
  redo_.push_back(std::move(step));
  return true;
}

bool MapEditor::redo() {
  endStroke();
  if (redo_.empty()) {
    return false;
  }
  Step step = std::move(redo_.back());
  redo_.pop_back();
  if (step.resized) {
    layout_ = step.resized->second;
  } else {
    for (const CellChange& change : step.changes) {
      layout_.set(change.cell, change.after);
    }
  }
  ++revision_;
  state_ = step.stateAfter;
  undo_.push_back(std::move(step));
  return true;
}

}  // namespace warehousesim::editor
