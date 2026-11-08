#include "warehousesim/view/camera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace warehousesim::view {

namespace {

ScreenSize sanitized(ScreenSize size) {
  return {std::max(size.width, 0), std::max(size.height, 0)};
}

}  // namespace

Camera::Camera(ScreenSize screen, world::GridSize grid) : screen_(sanitized(screen)), grid_(grid) {
  fit();
}

double Camera::clampZoom(double zoom) noexcept { return std::clamp(zoom, kMinZoom, kMaxZoom); }

ScreenPoint Camera::screenCenter() const noexcept {
  return {static_cast<double>(screen_.width) / 2.0, static_cast<double>(screen_.height) / 2.0};
}

ScreenPoint Camera::worldToScreen(WorldPoint p) const noexcept {
  const ScreenPoint middle = screenCenter();
  return {(p.x - center_.x) * zoom_ + middle.x, (p.y - center_.y) * zoom_ + middle.y};
}

WorldPoint Camera::screenToWorld(ScreenPoint p) const noexcept {
  const ScreenPoint middle = screenCenter();
  return {(p.x - middle.x) / zoom_ + center_.x, (p.y - middle.y) / zoom_ + center_.y};
}

ScreenRect Camera::cellRect(core::Point cell) const noexcept {
  // Both corners come from the same transform, so neighbouring cells share an exact edge.
  const ScreenPoint topLeft =
      worldToScreen({static_cast<double>(cell.x), static_cast<double>(cell.y)});
  const ScreenPoint bottomRight =
      worldToScreen({static_cast<double>(cell.x) + 1.0, static_cast<double>(cell.y) + 1.0});
  return {topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y};
}

std::optional<core::Point> Camera::cellAt(ScreenPoint p) const noexcept {
  const WorldPoint world = screenToWorld(p);
  if (!(world.x >= 0.0) || !(world.y >= 0.0) || world.x >= grid_.width() ||
      world.y >= grid_.height()) {
    return std::nullopt;
  }
  return core::Point{static_cast<std::int32_t>(std::floor(world.x)),
                     static_cast<std::int32_t>(std::floor(world.y))};
}

CellRange Camera::visibleCells() const noexcept {
  if (screen_.width <= 0 || screen_.height <= 0) {
    return {};
  }
  const ScreenPoint middle = screenCenter();
  const double halfWidth = middle.x / zoom_;
  const double halfHeight = middle.y / zoom_;
  const auto lower = [](double value, std::int32_t limit) {
    return static_cast<std::int32_t>(
        std::clamp(std::floor(value), 0.0, static_cast<double>(limit)));
  };
  const auto upper = [](double value, std::int32_t limit) {
    return static_cast<std::int32_t>(std::clamp(std::ceil(value), 0.0, static_cast<double>(limit)));
  };
  return {
      lower(center_.x - halfWidth, grid_.width()), lower(center_.y - halfHeight, grid_.height()),
      upper(center_.x + halfWidth, grid_.width()), upper(center_.y + halfHeight, grid_.height())};
}

void Camera::resize(ScreenSize screen) {
  screen_ = sanitized(screen);
  if (fitted_) {
    fit();
  }
}

void Camera::fit() {
  const double usableWidth = std::max(1.0, screen_.width - 2.0 * kFitMarginPixels);
  const double usableHeight = std::max(1.0, screen_.height - 2.0 * kFitMarginPixels);
  zoom_ = clampZoom(std::min(usableWidth / grid_.width(), usableHeight / grid_.height()));
  center_ = {grid_.width() / 2.0, grid_.height() / 2.0};
  fitted_ = true;
}

void Camera::panByPixels(double dx, double dy) {
  center_.x -= dx / zoom_;
  center_.y -= dy / zoom_;
  constrainCenter();
  fitted_ = false;
}

void Camera::centerOn(WorldPoint p) {
  center_ = p;
  constrainCenter();
  fitted_ = false;
}

void Camera::setZoom(double pixelsPerCell) {
  if (!std::isfinite(pixelsPerCell) || pixelsPerCell <= 0.0) {
    throw std::invalid_argument("zoom must be finite and positive");
  }
  zoom_ = clampZoom(pixelsPerCell);
  fitted_ = false;
}

void Camera::zoomBy(double factor, ScreenPoint anchor) {
  if (!std::isfinite(factor) || factor <= 0.0) {
    throw std::invalid_argument("zoom factor must be finite and positive");
  }
  const WorldPoint pivot = screenToWorld(anchor);
  zoom_ = clampZoom(zoom_ * factor);
  const ScreenPoint middle = screenCenter();
  center_ = {pivot.x - (anchor.x - middle.x) / zoom_, pivot.y - (anchor.y - middle.y) / zoom_};
  constrainCenter();
  fitted_ = false;
}

void Camera::zoomBySteps(double steps, ScreenPoint anchor) {
  zoomBy(std::pow(kZoomPerStep, steps), anchor);
}

void Camera::constrainCenter() noexcept {
  center_.x = std::clamp(center_.x, 0.0, static_cast<double>(grid_.width()));
  center_.y = std::clamp(center_.y, 0.0, static_cast<double>(grid_.height()));
}

}  // namespace warehousesim::view
