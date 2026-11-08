#include "warehousesim/rendering/sdl_window.hpp"

#include <SDL3/SDL.h>

#include <stdexcept>

#include "warehousesim/rendering/viewport.hpp"

namespace warehousesim::rendering {

namespace {

struct Color {
  Uint8 r;
  Uint8 g;
  Uint8 b;
};

constexpr Color kBackground{24, 26, 32};
constexpr Color kPendingTask{240, 210, 60};
constexpr Color kActiveTask{90, 200, 230};
constexpr Color kPauseIcon{255, 255, 255};

Color cellColor(world::CellType type) {
  switch (type) {
    case world::CellType::Floor:
      return {44, 48, 58};
    case world::CellType::Shelf:
      return {140, 100, 60};
    case world::CellType::Station:
      return {60, 160, 90};
    case world::CellType::Wall:
      return {100, 100, 110};
  }
  return kBackground;
}

Color robotColor(robots::RobotStatus status) {
  switch (status) {
    case robots::RobotStatus::Idle:
      return {80, 140, 240};
    case robots::RobotStatus::ToPickup:
      return {240, 150, 40};
    case robots::RobotStatus::ToDropoff:
      return {220, 70, 100};
  }
  return kBackground;
}

[[noreturn]] void throwSdlError(const char* what) {
  throw std::runtime_error(std::string(what) + ": " + SDL_GetError());
}

SDL_FRect toSdl(const Rect& rect) { return {rect.x, rect.y, rect.width, rect.height}; }

}  // namespace

struct SdlWindow::Impl {
  struct Session {
    Session() {
      if (!SDL_Init(SDL_INIT_VIDEO)) {
        throwSdlError("SDL_Init failed");
      }
    }
    ~Session() { SDL_Quit(); }
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
  };

  // Declaration order is destruction order in reverse: renderer, window, then SDL itself.
  Session session;
  std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window{nullptr, &SDL_DestroyWindow};
  std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer{nullptr,
                                                                         &SDL_DestroyRenderer};

  void setColor(Color color) const {
    if (!SDL_SetRenderDrawColor(renderer.get(), color.r, color.g, color.b, SDL_ALPHA_OPAQUE)) {
      throwSdlError("SDL_SetRenderDrawColor failed");
    }
  }

  void fill(Color color, const Rect& rect) const {
    setColor(color);
    const SDL_FRect sdlRect = toSdl(rect);
    if (!SDL_RenderFillRect(renderer.get(), &sdlRect)) {
      throwSdlError("SDL_RenderFillRect failed");
    }
  }
};

SdlWindow::SdlWindow(const std::string& title, int width, int height)
    : impl_(std::make_unique<Impl>()) {
  impl_->window.reset(SDL_CreateWindow(title.c_str(), width, height, SDL_WINDOW_RESIZABLE));
  if (!impl_->window) {
    throwSdlError("SDL_CreateWindow failed");
  }
  impl_->renderer.reset(SDL_CreateRenderer(impl_->window.get(), nullptr));
  if (!impl_->renderer) {
    throwSdlError("SDL_CreateRenderer failed");
  }
}

SdlWindow::~SdlWindow() = default;

InputEvents SdlWindow::pollEvents() {
  InputEvents result;
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_EVENT_QUIT) {
      result.quit = true;
    } else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
      if (event.key.key == SDLK_ESCAPE) {
        result.quit = true;
      } else if (event.key.key == SDLK_SPACE) {
        result.togglePause = !result.togglePause;
      }
    }
  }
  return result;
}

void SdlWindow::draw(const world::Warehouse& warehouse, const simulation::Snapshot& snapshot,
                     bool paused) {
  int outputWidth = 0;
  int outputHeight = 0;
  if (!SDL_GetCurrentRenderOutputSize(impl_->renderer.get(), &outputWidth, &outputHeight)) {
    throwSdlError("SDL_GetCurrentRenderOutputSize failed");
  }
  const Viewport viewport =
      Viewport::fit(outputWidth, outputHeight, warehouse.width(), warehouse.height());

  impl_->setColor(kBackground);
  if (!SDL_RenderClear(impl_->renderer.get())) {
    throwSdlError("SDL_RenderClear failed");
  }

  for (std::int32_t y = 0; y < warehouse.height(); ++y) {
    for (std::int32_t x = 0; x < warehouse.width(); ++x) {
      const core::Point cell{x, y};
      impl_->fill(cellColor(warehouse.at(cell)), viewport.insetCellRect(cell, 0.03F));
    }
  }
  for (const tasks::Task& task : snapshot.activeTasks) {
    const Color pickupColor =
        task.status == tasks::TaskStatus::Pending ? kPendingTask : kActiveTask;
    if (task.status != tasks::TaskStatus::InProgress) {
      impl_->fill(pickupColor, viewport.insetCellRect(task.pickup, 0.3F));
    }
    impl_->fill(kActiveTask, viewport.insetCellRect(task.dropoff, 0.4F));
  }
  for (const simulation::RobotSnapshot& robot : snapshot.robots) {
    impl_->fill(robotColor(robot.status), viewport.insetCellRect(robot.position, 0.15F));
  }
  if (paused) {
    impl_->fill(kPauseIcon, {8.0F, 8.0F, 6.0F, 20.0F});
    impl_->fill(kPauseIcon, {20.0F, 8.0F, 6.0F, 20.0F});
  }

  if (!SDL_RenderPresent(impl_->renderer.get())) {
    throwSdlError("SDL_RenderPresent failed");
  }
}

}  // namespace warehousesim::rendering
